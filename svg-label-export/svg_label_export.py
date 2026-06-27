#!/usr/bin/env python3
#
# This program source code file is part of a KiCad IPC API plugin.
#
# Copyright The KiCad Developers, see AUTHORS.txt for contributors.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the
# Free Software Foundation, either version 3 of the License, or (at your
# option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""
SVG Export with Labels
======================

A KiCad 10 IPC-API plugin (``kicad-python`` / ``kipy``) that behaves like the
built-in *Plot* feature for SVG output, but adds an option to overlay the
**pad numbers**, the **net names** of the pads, and/or the **component
reference designators** as text on top of the plotted board.

How it works
------------
1.  Connect to the running KiCad session over the IPC socket (``kipy``).
2.  Read the open board: the on-disk ``.kicad_pcb`` path, every pad
    (number, net name, absolute board position) and every footprint
    (reference designator + position).
3.  Render the chosen layers to a base SVG by invoking ``kicad-cli pcb
    export svg`` -- this reuses KiCad's own plotter so the artwork matches
    the Plot dialog exactly.
4.  Inject ``<text>`` elements into that SVG at the pad / footprint
    positions.  KiCad's SVG plotter emits a ``viewBox`` whose user units are
    millimetres with the page origin at (0, 0), so a board coordinate in
    nanometres maps to an SVG user coordinate simply by dividing by 1e6
    (see ``NM_PER_MM``).  This requires a full-page plot (``--page-size-mode
    0`` or ``1``); for "fit to board" the plugin subtracts the board
    bounding-box origin so labels stay aligned.

The plugin can be driven either through a small Tkinter dialog (default when
launched from KiCad) or fully from the command line / arguments declared in
``plugin.json`` (use ``--no-gui``).
"""

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# KiCad serialises all lengths in nanometres; the SVG plotter uses millimetres
# for its user-space coordinates (1 SVG user unit == 1 mm).
NM_PER_MM = 1_000_000.0

# A reasonable, commonly useful default layer selection.
DEFAULT_LAYERS = "F.Cu,Edge.Cuts"

# Layers offered as quick check-boxes in the dialog.  These are the canonical
# names accepted by ``kicad-cli pcb export svg --layers``.
COMMON_LAYERS = [
    "F.Cu",
    "B.Cu",
    "F.Silkscreen",
    "B.Silkscreen",
    "F.Mask",
    "B.Mask",
    "F.Paste",
    "B.Paste",
    "F.Fab",
    "B.Fab",
    "F.Courtyard",
    "B.Courtyard",
    "Edge.Cuts",
    "User.Drawings",
    "User.Comments",
]


# --------------------------------------------------------------------------- #
#  Options                                                                     #
# --------------------------------------------------------------------------- #
@dataclass
class ExportOptions:
    """Everything that controls a single export run."""

    layers: str = DEFAULT_LAYERS
    output: Optional[str] = None          # final SVG path; defaults next to board
    pad_numbers: bool = True              # overlay pad numbers
    net_names: bool = False               # overlay pad net names
    references: bool = False              # overlay component reference designators
    page_size_mode: int = 0               # 0 = frame+title, 1 = current page, 2 = board
    font_size_mm: float = 0.5
    pad_color: str = "#1414C8"            # pad-number text colour
    net_color: str = "#C81414"            # net-name text colour
    ref_color: str = "#149614"            # reference text colour
    halo: bool = True                     # draw a light halo behind text
    black_and_white: bool = False
    mirror: bool = False
    no_gui: bool = False
    extra_cli_args: List[str] = field(default_factory=list)


# --------------------------------------------------------------------------- #
#  Data gathered from the board                                                #
# --------------------------------------------------------------------------- #
@dataclass
class Label:
    """A single text label and where it should be placed (millimetres)."""

    text: str
    x_mm: float
    y_mm: float
    color: str


# --------------------------------------------------------------------------- #
#  KiCad / board access                                                        #
# --------------------------------------------------------------------------- #
def connect_kicad():
    """Connect to the running KiCad session, returning a ``kipy.KiCad``."""
    try:
        from kipy import KiCad
    except ImportError as exc:  # pragma: no cover - depends on runtime env
        raise SystemExit(
            "Could not import 'kipy' (kicad-python).\n"
            "Install it into the plugin's Python environment, e.g.:\n"
            "    pip install kicad-python\n"
            f"Underlying error: {exc}"
        )

    try:
        return KiCad()
    except BaseException as exc:  # kipy raises broad errors on connect failure
        raise SystemExit(
            "Could not connect to KiCad over the IPC API.\n"
            "Make sure KiCad is running and the API server is enabled in\n"
            "Preferences > Plugins.\n"
            f"Underlying error: {exc}"
        )


def resolve_board_path(kicad, board) -> str:
    """Return the absolute on-disk path of the open board."""
    from kipy.proto.common.types import DocumentType

    docs = kicad.get_open_documents(DocumentType.DOCTYPE_PCB)
    if not docs:
        raise SystemExit("No PCB document is open in KiCad.")

    spec = docs[0]
    filename = spec.board_filename
    project_path = spec.project.path if spec.HasField("project") else ""

    if filename and os.path.isabs(filename):
        return filename
    if project_path:
        return os.path.join(project_path, filename)

    # Last resort: ask the board to save and hope it has a usable name.
    raise SystemExit(
        "Could not determine the board's file path from the API.\n"
        "Please save the board to disk and try again."
    )


def gather_labels(board, opts: ExportOptions,
                  offset_mm: Tuple[float, float]) -> List[Label]:
    """Collect all requested labels from the board, in millimetres."""
    ox, oy = offset_mm
    labels: List[Label] = []

    if opts.pad_numbers or opts.net_names:
        # board.get_pads() returns every pad with ABSOLUTE board coordinates
        # (PAD::GetPosition applies the footprint transform) in nanometres.
        for pad in board.get_pads():
            try:
                px = pad.position.x / NM_PER_MM - ox
                py = pad.position.y / NM_PER_MM - oy
            except Exception:
                continue

            number = pad.number or ""
            net_name = ""
            try:
                net_name = pad.net.name or ""
            except Exception:
                net_name = ""

            # When both are requested, stack them: number above, net below.
            both = opts.pad_numbers and opts.net_names and number and net_name
            dy = opts.font_size_mm * 0.6 if both else 0.0

            if opts.pad_numbers and number:
                labels.append(Label(number, px, py - dy, opts.pad_color))
            if opts.net_names and net_name:
                labels.append(Label(net_name, px, py + dy, opts.net_color))

    if opts.references:
        for fp in board.get_footprints():
            try:
                ref_field = fp.reference_field
                text = ref_field.text.value
                pos = ref_field.text.position
                rx = pos.x / NM_PER_MM - ox
                ry = pos.y / NM_PER_MM - oy
            except Exception:
                # Fall back to the footprint origin if the field is unusable.
                try:
                    text = fp.reference_field.text.value
                    rx = fp.position.x / NM_PER_MM - ox
                    ry = fp.position.y / NM_PER_MM - oy
                except Exception:
                    continue
            if text:
                labels.append(Label(text, rx, ry, opts.ref_color))

    return labels


def board_offset_mm(board, opts: ExportOptions) -> Tuple[float, float]:
    """
    Offset (in mm) to subtract from label coordinates so they line up with the
    plotted SVG.  For full-page modes (0/1) the board sits at its real page
    coordinates and the offset is zero.  For "board area only" (mode 2) the
    plotter shifts the content to the board's bounding-box origin, so we
    subtract that origin if the API can provide it.
    """
    if opts.page_size_mode != 2:
        return (0.0, 0.0)

    getter = getattr(board, "get_bounding_box", None)
    if callable(getter):
        try:
            box = getter()
            return (box.pos.x / NM_PER_MM, box.pos.y / NM_PER_MM)
        except Exception:
            pass

    print(
        "Warning: 'fit to board' selected but the board bounding box could "
        "not be read from the API; labels may be offset. Consider page mode "
        "0 or 1.",
        file=sys.stderr,
    )
    return (0.0, 0.0)


# --------------------------------------------------------------------------- #
#  Base SVG via kicad-cli                                                       #
# --------------------------------------------------------------------------- #
def find_kicad_cli(kicad) -> str:
    """Locate the kicad-cli executable, preferring the API-reported path."""
    getter = getattr(kicad, "get_kicad_binary_path", None)
    if callable(getter):
        try:
            path = getter("kicad-cli")
            if path and os.path.exists(path):
                return path
        except Exception:
            pass

    found = shutil.which("kicad-cli") or shutil.which("kicad-cli.exe")
    if found:
        return found

    raise SystemExit(
        "Could not locate 'kicad-cli'. Make sure KiCad's command-line tools "
        "are installed and on PATH."
    )


def plot_base_svg(cli: str, board_path: str, out_svg: str,
                  opts: ExportOptions) -> None:
    """Render the selected layers to a single SVG using kicad-cli."""
    cmd = [
        cli, "pcb", "export", "svg",
        "--output", out_svg,
        "--layers", opts.layers,
        "--page-size-mode", str(opts.page_size_mode),
        "--mode-single",
    ]
    if opts.black_and_white:
        cmd.append("--black-and-white")
    if opts.mirror:
        cmd.append("--mirror")
    cmd.extend(opts.extra_cli_args)
    cmd.append(board_path)

    print("Running:", " ".join(cmd))
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        raise SystemExit(
            "kicad-cli failed to export the SVG:\n"
            f"{result.stdout}\n{result.stderr}"
        )
    if not os.path.exists(out_svg):
        raise SystemExit(
            "kicad-cli reported success but no SVG was produced at "
            f"{out_svg}.\n{result.stdout}\n{result.stderr}"
        )


# --------------------------------------------------------------------------- #
#  SVG overlay                                                                  #
# --------------------------------------------------------------------------- #
def _xml_escape(text: str) -> str:
    return (
        text.replace("&", "&amp;")
        .replace("<", "&lt;")
        .replace(">", "&gt;")
        .replace('"', "&quot;")
    )


def build_overlay_group(labels: List[Label], opts: ExportOptions) -> str:
    """Build an SVG <g> string containing all label <text> elements."""
    fs = opts.font_size_mm
    halo = ""
    if opts.halo:
        # paint-order:stroke draws the (light) stroke first, behind the fill,
        # giving each glyph a readable halo over busy artwork.
        halo = (
            f'paint-order:stroke;stroke:#ffffff;'
            f'stroke-width:{fs * 0.18:.4f};stroke-opacity:0.85;'
        )

    parts = [
        '<g inkscape:label="KiCad Labels" inkscape:groupmode="layer" '
        'id="kicad-labels" '
        f'style="font-family:sans-serif;font-size:{fs:.4f}px;'
        f'text-anchor:middle;{halo}stroke-linejoin:round;">'
    ]

    for lab in labels:
        # SVG <text> y is the baseline; nudge down by ~0.35em to vertically
        # centre the glyphs on the pad.
        y = lab.y_mm + fs * 0.35
        parts.append(
            f'<text x="{lab.x_mm:.4f}" y="{y:.4f}" '
            f'fill="{lab.color}" stroke-width="{(fs * 0.18) if opts.halo else 0:.4f}">'
            f'{_xml_escape(lab.text)}</text>'
        )

    parts.append("</g>")
    return "\n".join(parts)


def inject_overlay(svg_path: str, overlay: str) -> None:
    """Insert the overlay group just before the closing </svg> tag."""
    with open(svg_path, "r", encoding="utf-8") as fp:
        svg = fp.read()

    idx = svg.rfind("</svg>")
    if idx == -1:
        raise SystemExit(f"{svg_path} does not look like a valid SVG file.")

    new_svg = svg[:idx] + overlay + "\n" + svg[idx:]
    with open(svg_path, "w", encoding="utf-8") as fp:
        fp.write(new_svg)


# --------------------------------------------------------------------------- #
#  Orchestration                                                               #
# --------------------------------------------------------------------------- #
def default_output_path(board_path: str) -> str:
    base, _ = os.path.splitext(board_path)
    return base + "-labelled.svg"


def run_export(opts: ExportOptions) -> str:
    """Perform the full export and return the path of the written SVG."""
    kicad = connect_kicad()
    board = kicad.get_board()
    board_path = resolve_board_path(kicad, board)

    if not opts.output:
        opts.output = default_output_path(board_path)

    cli = find_kicad_cli(kicad)

    # Render the artwork into a temporary file, then overlay and move into
    # place so a failure never clobbers an existing output.
    tmp_dir = tempfile.mkdtemp(prefix="kicad-svg-labels-")
    tmp_svg = os.path.join(tmp_dir, "base.svg")
    try:
        plot_base_svg(cli, board_path, tmp_svg, opts)

        offset = board_offset_mm(board, opts)
        labels = gather_labels(board, opts, offset)
        print(f"Collected {len(labels)} label(s).")

        if labels:
            overlay = build_overlay_group(labels, opts)
            inject_overlay(tmp_svg, overlay)

        shutil.move(tmp_svg, opts.output)
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    print(f"Wrote {opts.output}")
    return opts.output


# --------------------------------------------------------------------------- #
#  GUI                                                                          #
# --------------------------------------------------------------------------- #
def run_gui(opts: ExportOptions) -> bool:
    """
    Show a Tkinter options dialog.  Returns True if the user pressed Export,
    False if they cancelled.  Mutates ``opts`` in place.  Raises ImportError
    if Tkinter is unavailable so the caller can fall back to a headless run.
    """
    import tkinter as tk
    from tkinter import filedialog, messagebox, ttk

    root = tk.Tk()
    root.title("Export SVG with Labels")
    root.resizable(False, False)

    state = {"ok": False}

    main = ttk.Frame(root, padding=12)
    main.grid(sticky="nsew")

    # --- Layers ----------------------------------------------------------- #
    ttk.Label(main, text="Layers to plot:").grid(
        row=0, column=0, sticky="w", pady=(0, 4))
    selected = set(s.strip() for s in opts.layers.split(",") if s.strip())
    layer_vars = {}
    layer_frame = ttk.Frame(main)
    layer_frame.grid(row=1, column=0, columnspan=2, sticky="w")
    for i, name in enumerate(COMMON_LAYERS):
        var = tk.BooleanVar(value=name in selected)
        layer_vars[name] = var
        ttk.Checkbutton(layer_frame, text=name, variable=var).grid(
            row=i // 3, column=i % 3, sticky="w", padx=4, pady=1)

    ttk.Label(main, text="Extra layers (comma separated):").grid(
        row=2, column=0, sticky="w", pady=(8, 0))
    extra_layers = tk.StringVar(
        value=",".join(sorted(selected - set(COMMON_LAYERS))))
    ttk.Entry(main, textvariable=extra_layers, width=40).grid(
        row=3, column=0, columnspan=2, sticky="we")

    # --- Label options ---------------------------------------------------- #
    ttk.Separator(main, orient="horizontal").grid(
        row=4, column=0, columnspan=2, sticky="we", pady=8)

    pad_var = tk.BooleanVar(value=opts.pad_numbers)
    net_var = tk.BooleanVar(value=opts.net_names)
    ref_var = tk.BooleanVar(value=opts.references)
    ttk.Checkbutton(main, text="Pad numbers", variable=pad_var).grid(
        row=5, column=0, sticky="w")
    ttk.Checkbutton(main, text="Net names", variable=net_var).grid(
        row=6, column=0, sticky="w")
    ttk.Checkbutton(main, text="Component references", variable=ref_var).grid(
        row=7, column=0, sticky="w")

    # --- Plot options ----------------------------------------------------- #
    ttk.Separator(main, orient="horizontal").grid(
        row=8, column=0, columnspan=2, sticky="we", pady=8)

    ttk.Label(main, text="Font size (mm):").grid(row=9, column=0, sticky="w")
    font_var = tk.StringVar(value=str(opts.font_size_mm))
    ttk.Entry(main, textvariable=font_var, width=8).grid(
        row=9, column=1, sticky="w")

    ttk.Label(main, text="Page mode:").grid(row=10, column=0, sticky="w")
    page_var = tk.StringVar(
        value={0: "Frame + title block", 1: "Current page size",
               2: "Board area only"}[opts.page_size_mode])
    ttk.Combobox(
        main, textvariable=page_var, state="readonly", width=24,
        values=["Frame + title block", "Current page size",
                "Board area only"],
    ).grid(row=10, column=1, sticky="w")

    bw_var = tk.BooleanVar(value=opts.black_and_white)
    mirror_var = tk.BooleanVar(value=opts.mirror)
    ttk.Checkbutton(main, text="Black and white", variable=bw_var).grid(
        row=11, column=0, sticky="w")
    ttk.Checkbutton(main, text="Mirror", variable=mirror_var).grid(
        row=11, column=1, sticky="w")

    # --- Output ----------------------------------------------------------- #
    ttk.Separator(main, orient="horizontal").grid(
        row=12, column=0, columnspan=2, sticky="we", pady=8)
    ttk.Label(main, text="Output SVG (blank = next to board):").grid(
        row=13, column=0, columnspan=2, sticky="w")
    out_var = tk.StringVar(value=opts.output or "")
    out_frame = ttk.Frame(main)
    out_frame.grid(row=14, column=0, columnspan=2, sticky="we")
    ttk.Entry(out_frame, textvariable=out_var, width=34).grid(
        row=0, column=0, sticky="we")

    def browse():
        path = filedialog.asksaveasfilename(
            defaultextension=".svg",
            filetypes=[("SVG files", "*.svg"), ("All files", "*.*")])
        if path:
            out_var.set(path)

    ttk.Button(out_frame, text="...", width=3, command=browse).grid(
        row=0, column=1, padx=(4, 0))

    # --- Buttons ---------------------------------------------------------- #
    def on_export():
        try:
            opts.font_size_mm = float(font_var.get())
        except ValueError:
            messagebox.showerror("Invalid input", "Font size must be a number.")
            return

        chosen = [n for n, v in layer_vars.items() if v.get()]
        extra = [s.strip() for s in extra_layers.get().split(",") if s.strip()]
        all_layers = chosen + [e for e in extra if e not in chosen]
        if not all_layers:
            messagebox.showerror("No layers", "Select at least one layer.")
            return

        opts.layers = ",".join(all_layers)
        opts.pad_numbers = pad_var.get()
        opts.net_names = net_var.get()
        opts.references = ref_var.get()
        opts.page_size_mode = {"Frame + title block": 0,
                               "Current page size": 1,
                               "Board area only": 2}[page_var.get()]
        opts.black_and_white = bw_var.get()
        opts.mirror = mirror_var.get()
        opts.output = out_var.get().strip() or None
        state["ok"] = True
        root.destroy()

    def on_cancel():
        root.destroy()

    btns = ttk.Frame(main)
    btns.grid(row=15, column=0, columnspan=2, sticky="e", pady=(12, 0))
    ttk.Button(btns, text="Cancel", command=on_cancel).grid(row=0, column=0,
                                                            padx=(0, 6))
    ttk.Button(btns, text="Export", command=on_export).grid(row=0, column=1)

    root.mainloop()
    return state["ok"]


# --------------------------------------------------------------------------- #
#  CLI                                                                          #
# --------------------------------------------------------------------------- #
def parse_args(argv: List[str]) -> ExportOptions:
    parser = argparse.ArgumentParser(
        description="Export selected KiCad board layers to SVG with pad "
                    "number / net name / reference labels.")
    parser.add_argument("--layers", default=DEFAULT_LAYERS,
                        help=f"Comma-separated layer list (default: {DEFAULT_LAYERS}).")
    parser.add_argument("--output", default=None,
                        help="Output SVG path (default: <board>-labelled.svg).")
    parser.add_argument("--pad-numbers", dest="pad_numbers",
                        action="store_true", default=True)
    parser.add_argument("--no-pad-numbers", dest="pad_numbers",
                        action="store_false")
    parser.add_argument("--net-names", dest="net_names", action="store_true",
                        default=False, help="Overlay pad net names.")
    parser.add_argument("--references", dest="references", action="store_true",
                        default=False, help="Overlay component references.")
    parser.add_argument("--page-size-mode", type=int, default=0, choices=[0, 1, 2],
                        help="0 = frame+title, 1 = current page, 2 = board area.")
    parser.add_argument("--font-size-mm", type=float, default=0.5)
    parser.add_argument("--black-and-white", action="store_true", default=False)
    parser.add_argument("--mirror", action="store_true", default=False)
    parser.add_argument("--no-halo", dest="halo", action="store_false",
                        default=True)
    parser.add_argument("--no-gui", dest="no_gui", action="store_true",
                        default=False, help="Skip the dialog; run with these args.")
    args = parser.parse_args(argv)

    return ExportOptions(
        layers=args.layers,
        output=args.output,
        pad_numbers=args.pad_numbers,
        net_names=args.net_names,
        references=args.references,
        page_size_mode=args.page_size_mode,
        font_size_mm=args.font_size_mm,
        black_and_white=args.black_and_white,
        mirror=args.mirror,
        halo=args.halo,
        no_gui=args.no_gui,
    )


def main(argv: Optional[List[str]] = None) -> int:
    opts = parse_args(argv if argv is not None else sys.argv[1:])

    if not opts.no_gui:
        try:
            if not run_gui(opts):
                print("Cancelled.")
                return 0
        except ImportError:
            print("Tkinter not available; running with command-line options.",
                  file=sys.stderr)

    run_export(opts)
    return 0


if __name__ == "__main__":
    sys.exit(main())

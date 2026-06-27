# SVG Export with Labels

A KiCad **10** plugin that works like the built-in **Plot → SVG** feature, but
adds an option to overlay **pad numbers**, **pad net names**, and/or
**component reference designators** as text on top of the plotted board.

It is an [IPC API](https://dev-docs.kicad.org/en/apis-and-binding/ipc-api/)
plugin written in Python using the official
[`kicad-python`](https://gitlab.com/kicad/code/kicad-python) (`kipy`) bindings.

> **Why not the old `pcbnew` Python module?**
> The legacy SWIG `pcbnew` module (and wrappers built on it, such as
> *kigadgets* / `atait/kicad-python`) has been **removed in KiCad 10**. The
> IPC API is the supported way to script KiCad 10, so this plugin targets it.

---

## What it does

1. Connects to the running KiCad session over the IPC socket.
2. Reads the open board: its file path, every pad (number, net name, absolute
   position) and every footprint (reference designator + position).
3. Plots the layers you select to a base SVG using `kicad-cli pcb export svg`
   — this reuses KiCad's own plotter, so the artwork matches the Plot dialog.
4. Injects `<text>` labels onto the pads / footprints at the correct
   coordinates and writes the final SVG.

### Coordinate mapping

KiCad's SVG plotter emits a `viewBox` whose **user units are millimetres** with
the page origin at `(0, 0)`. The API reports positions in **nanometres**, so a
board coordinate maps to an SVG coordinate by dividing by `1 000 000`. This holds
for the full-page plot modes (`Frame + title block`, `Current page size`). For
`Board area only` the plotter shifts the artwork to the board's bounding-box
origin, which the plugin compensates for when the API can report that box.

---

## Installation

1. Make sure the IPC API server is enabled: **Preferences → Plugins → "Enable
   the KiCad API"**.
2. Copy (or symlink) this `svg-label-export/` folder (the one containing
   `plugin.json`) into your KiCad **user plugins** directory, e.g.:
   - Linux: `~/.local/share/kicad/10.0/plugins/svg-label-export/`
   - Windows: `%APPDATA%\kicad\10.0\plugins\svg-label-export\`
   - macOS: `~/Library/Preferences/kicad/10.0/plugins/svg-label-export/`

   (The version segment matches your build — `10.0`, `10.99`, etc. KiCad also
   scans the Plugin & Content Manager path `…/10.0/3rdparty/plugins/`; the
   plain `plugins/` folder above is the simplest for a manual install.)
3. Ensure `kicad-python` is available in the plugin's Python environment.
   KiCad can manage this for you, or run `pip install -r requirements.txt`.
4. Restart KiCad / re-scan plugins. The action **“Export SVG with Labels…”**
   appears in the PCB Editor (toolbar button and Tools → External Plugins).

---

## Usage

### From the PCB Editor
Open a board, then run **Export SVG with Labels…**. A dialog lets you choose:

- **Layers** — quick check-boxes for common layers plus a free-text field for
  any others (inner copper, user layers, …).
- **Labels** — Pad numbers, Net names, Component references (any combination).
- **Font size**, **page mode**, **black & white**, **mirror**.
- **Output path** — defaults to `<board>-labelled.svg` next to the board.

### Headless / command line
The same script runs standalone (KiCad must still be open with the API server
enabled). Pass `--no-gui` to skip the dialog:

```bash
python svg_label_export.py --no-gui \
    --layers "F.Cu,F.Silkscreen,Edge.Cuts" \
    --net-names --pad-numbers \
    --font-size-mm 0.4 \
    --output /tmp/board-front.svg
```

Key options (`python svg_label_export.py --help` for the full list):

| Option | Meaning |
| ------ | ------- |
| `--layers` | Comma-separated canonical layer names (e.g. `F.Cu,B.Cu,Edge.Cuts`). |
| `--pad-numbers` / `--no-pad-numbers` | Overlay pad numbers (on by default). |
| `--net-names` | Overlay each pad's net name. |
| `--references` | Overlay component reference designators. |
| `--page-size-mode` | `0` frame+title, `1` current page, `2` board area only. |
| `--font-size-mm` | Label text height in millimetres. |
| `--black-and-white` / `--mirror` | Passed through to `kicad-cli`. |
| `--no-halo` | Disable the light halo drawn behind label text. |

---

## Notes & limitations

- **Mirror:** the base artwork is mirrored by `kicad-cli`, but the text overlay
  is not yet mirrored, so use mirror only when you don't need labels, or expect
  labels to read in normal orientation over a mirrored board.
- Labels are placed at pad / reference-field centres; very dense boards may
  overlap. Reduce `--font-size-mm` or plot fewer layers.
- Requires KiCad 10 (IPC API). It will not work on KiCad ≤ 9.

## Files

| File | Purpose |
| ---- | ------- |
| `plugin.json` | Plugin manifest read by KiCad (identifier, action, entrypoint). |
| `svg_label_export.py` | The plugin entry point (GUI + export logic). |
| `requirements.txt` | Python dependency (`kicad-python`). |

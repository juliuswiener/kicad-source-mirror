// bridge_smoketest.cpp
//
// Real link + runtime test of the KiCad-linked bridge. Loads a board with the
// minimal, kiface-free recipe (PCB_IO_KICAD_SEXPR + a hand-built DRC_ENGINE, as
// qa/tools/pns/pns_log_file.cpp does), attaches the bridge, extracts obstacles
// from the live PNS world, runs the gplan core over them, and exercises
// routeAndCheck. Proves the bridge links and runs against real KiCad/PNS.
//
// Built only when KICAD_BUILD_PNS_DEBUG_TOOL=ON (see qa/tools/pns/CMakeLists.txt).

#include <wx/init.h>
#include <wx/filename.h>
#include <cstdio>
#include <memory>
#include <map>
#include <vector>

#include <pad.h>
#include <footprint.h>
#include <pcb_track.h>

#include <pgm_base.h>
#include <board.h>
#include <board_design_settings.h>
#include <project.h>
#include <settings/settings_manager.h>
#include <pcbnew/pcb_io/kicad_sexpr/pcb_io_kicad_sexpr.h>
#include <pcbnew/drc/drc_engine.h>
#include <wildcards_and_files_ext.h>
#include <layer_ids.h>

#include "pns_bridge.h"
#include "planner_core.h"

// Minimal concrete PGM_BASE so Pgm() is valid in this standalone harness.
struct SMOKE_PGM : public PGM_BASE
{
    void MacOpenFile( const wxString& ) override {}
};
static SMOKE_PGM s_pgm;

static int run( int argc, char** argv )
{
    wxFileName fnPcb( ( argc > 1 ) ? wxString::FromUTF8( argv[1] )
                                   : wxString( "qa/data/pcbnew/complex_hierarchy.kicad_pcb" ) );
    fnPcb.MakeAbsolute();
    const wxString pcb = fnPcb.GetFullPath();

    // --- minimal, kiface-free board load (mirrors pns_log_file.cpp) ---------
    SETTINGS_MANAGER settingsMgr;
    wxFileName fnPro( pcb );
    fnPro.SetExt( wxT( "kicad_pro" ) );
    settingsMgr.LoadProject( fnPro.GetFullPath() );
    PROJECT* project = settingsMgr.GetProject( fnPro.GetFullPath() );

    std::unique_ptr<BOARD> board;
    try
    {
        PCB_IO_KICAD_SEXPR io;
        board.reset( io.LoadBoard( pcb, nullptr, nullptr ) );
    }
    catch( ... )
    {
        std::printf( "FAIL: could not load board '%s'\n", (const char*) pcb.utf8_str() );
        return 1;
    }
    if( !board )
    {
        std::printf( "FAIL: null board\n" );
        return 1;
    }

    board->SetProject( project );

    auto drcEngine = std::make_shared<DRC_ENGINE>();
    BOARD_DESIGN_SETTINGS& bds = board->GetDesignSettings();
    bds.m_DRCEngine = drcEngine;
    board->SynchronizeNetsAndNetClasses( true );
    drcEngine->SetBoard( board.get() );
    drcEngine->SetDesignSettings( &bds );

    wxFileName fnRules( pcb );
    fnRules.SetExt( FILEEXT::DesignRulesFileExtension );
    drcEngine->InitEngine( fnRules.FileExists() ? fnRules : wxFileName() );

    std::printf( "board loaded: %zu tracks, %zu footprints\n",
                 board->Tracks().size(), board->Footprints().size() );

    // --- bridge ------------------------------------------------------------
    gbridge::PnsBridge br;
    if( !br.attach( board.get() ) )
    {
        std::printf( "FAIL: bridge attach\n" );
        return 1;
    }
    std::printf( "bridge attached, PNS world synced\n" );

    // --- T-VIA: relocate a single via (stitching-via-blocks-escape scenario).
    // Run on the freshly-synced world, before any world-mutating test below can
    // shift/shove this via's position and cause a spurious QueryHoverItems miss.
    {
        PCB_VIA* via = nullptr;
        for( PCB_TRACK* t : board->Tracks() )
            if( t->Type() == PCB_VIA_T ) { via = static_cast<PCB_VIA*>( t ); break; }

        if( via )
        {
            VECTOR2I vp = via->GetPosition();
            std::printf( "T-VIA: via at (%d,%d)\n", vp.x, vp.y );

            gbridge::DragProbe pv = br.probeViaMove( vp.x, vp.y, vp.x + 200000, vp.y );
            std::printf( "probeViaMove(+0.2mm): clean=%d cost=%.0f shoved=%d\n",
                         pv.clean, pv.cost, pv.shoved );

            VECTOR2I cur = vp;   // tracks the via's actual current position
            gbridge::RouteChange mv = br.moveVia( cur.x, cur.y, cur.x + 200000, cur.y, false );
            std::printf( "moveVia(+0.2mm): ok=%d placed=%d modVias=%zu reason='%s'\n",
                         mv.ok, mv.placed, mv.modVias.size(), mv.reason.c_str() );
            std::printf( "VIA-MOVE ran (ok=%d, board-dependent)\n", mv.ok );
            if( mv.ok ) cur.x += 200000;

            // Lock the via -> moveVia must refuse (deterministic).
            via->SetLocked( true );
            gbridge::RouteChange mv2 = br.moveVia( cur.x, cur.y, cur.x + 200000, cur.y, false );
            std::printf( "moveVia on LOCKED via: ok=%d reason='%s'\n", mv2.ok, mv2.reason.c_str() );
            via->SetLocked( false );
            if( mv2.ok || mv2.reason != std::string( "via locked" ) )
            { std::printf( "VIA-LOCK FAIL: locked via was not refused\n" ); return 1; }
            std::printf( "VIA-LOCK OK (locked via refused)\n" );

            // shoveViaSearch: probe a few candidate offsets around the via's
            // CURRENT position, commit the cleanest/cheapest.
            std::vector<std::vector<double>> vcands = {
                { (double)( cur.x + 100000 ), (double) cur.y },
                { (double) cur.x, (double)( cur.y + 100000 ) },
                { (double)( cur.x - 100000 ), (double) cur.y } };
            gbridge::ShoveResult vsr = br.shoveViaSearch( cur.x, cur.y, vcands );
            std::printf( "shoveViaSearch: committed=%d chosen=(%.0f,%.0f) cost=%.0f\n",
                         vsr.committed, vsr.x, vsr.y, vsr.cost );
            std::printf( "VIA-SHOVE-SEARCH ran\n" );
        }
        else
        {
            std::printf( "T-VIA: no via on board; skipped\n" );
        }
    }

    std::vector<gplan::Obstacle> obs = br.getAllObstacles();
    size_t fixed = 0, movable = 0;
    for( const auto& o : obs ) ( o.fixed ? fixed : movable )++;
    std::printf( "obstacles extracted: %zu (fixed=%zu, movable=%zu)\n",
                 obs.size(), fixed, movable );
    if( obs.empty() )
    {
        std::printf( "FAIL: no obstacles extracted from a populated board\n" );
        return 1;
    }

    // --- core planner on F.Cu ---------------------------------------------
    gplan::PlannerParams p;
    p.trackWidth = 200000;   // 0.2 mm in nm
    p.clearance  = 200000;
    p.kPaths     = 3;
    int fcu = br.pnsLayer( F_Cu );
    std::vector<gplan::Obstacle> fcuObs;
    for( const auto& o : obs ) if( o.layer == fcu ) fcuObs.push_back( o );
    std::printf( "F.Cu (pns layer %d) obstacles: %zu\n", fcu, fcuObs.size() );

    gplan::Planner planner( fcuObs, p );

    // Pick a real net with >=2 pads on F.Cu and route between two of its pads,
    // so PNS actually has start/target items to work with.
    std::map<int, std::vector<VECTOR2I>> netPads;
    for( FOOTPRINT* fp : board->Footprints() )
        for( PAD* pad : fp->Pads() )
            if( pad->GetNetCode() > 0 && pad->IsOnLayer( F_Cu ) )
                netPads[pad->GetNetCode()].push_back( pad->GetPosition() );

    VECTOR2I pa, pb;
    bool havePair = false;
    for( auto& [net, pts] : netPads )
        if( pts.size() >= 2 )
        {
            pa = pts.front();
            pb = pts.back();
            havePair = true;
            std::printf( "routing net %d between (%d,%d) and (%d,%d)\n",
                         net, pa.x, pa.y, pb.x, pb.y );
            break;
        }
    if( !havePair )
    {
        std::printf( "no multi-pad F.Cu net found; skipping route\n" );
        std::printf( "SMOKETEST OK\n" );
        return 0;
    }

    gplan::Point a{ (double) pa.x, (double) pa.y };
    gplan::Point b{ (double) pb.x, (double) pb.y };

    // --- T-CORRIDOR: cascade-clear an escape from pa toward pb -------------
    {
        double dx = b.x - a.x, dy = b.y - a.y;
        double dlen = std::sqrt( dx * dx + dy * dy );
        double radius = std::min( dlen, 3000000.0 );   // cap at 3mm
        gbridge::EscapeClearResult ec =
            br.clearEscapeCorridor( a.x, a.y, fcu, dx, dy, radius );
        std::printf( "clearEscapeCorridor: ok=%d iterations=%d moves=%zu "
                     "blocking=(%.0f,%.0f)\n",
                     ec.ok, ec.iterations, ec.moves.size(), ec.blocking.x, ec.blocking.y );
        std::printf( "T-CORRIDOR ran (ok=%d, board-dependent)\n", ec.ok );
    }

    // --- T-CORRIDOR-2: same, but aimed at the nearest FOREIGN-net pad within
    // 2mm, to force an actual obstruction (not just an already-open probe).
    {
        VECTOR2I origin = pa;
        int originNet = -1;
        for( FOOTPRINT* fp : board->Footprints() )
            for( PAD* pad : fp->Pads() )
                if( pad->GetPosition() == origin && pad->IsOnLayer( F_Cu ) )
                { originNet = pad->GetNetCode(); break; }

        VECTOR2I nearest;
        double   nearestDist = 1e18;
        bool     haveTarget  = false;
        for( FOOTPRINT* fp : board->Footprints() )
            for( PAD* pad : fp->Pads() )
            {
                if( pad->GetNetCode() == originNet || !pad->IsOnLayer( F_Cu ) )
                    continue;
                VECTOR2I p = pad->GetPosition();
                double d = std::hypot( (double)( p.x - origin.x ), (double)( p.y - origin.y ) );
                if( d < 8000000.0 && d < nearestDist )
                { nearestDist = d; nearest = p; haveTarget = true; }
            }

        if( haveTarget )
        {
            double dx = nearest.x - origin.x, dy = nearest.y - origin.y;
            gbridge::EscapeClearResult ec2 = br.clearEscapeCorridor(
                origin.x, origin.y, fcu, dx, dy, nearestDist + 300000.0 );
            std::printf( "clearEscapeCorridor(adversarial, target net!=origin, "
                        "d=%.0f): ok=%d iterations=%d moves=%zu\n",
                        nearestDist, ec2.ok, ec2.iterations, ec2.moves.size() );
            std::printf( "T-CORRIDOR-2 ran (ok=%d, board-dependent)\n", ec2.ok );
        }
        else
            std::printf( "T-CORRIDOR-2: no nearby foreign-net pad within 8mm; skipped\n" );
    }

    auto paths = planner.plan( a, b );
    std::printf( "gplan candidates for the net: %zu\n", paths.size() );

    // --- exercise PNS route+verify path (must run, verdict not asserted) ----
    if( !paths.empty() )
    {
        gbridge::RouteResult r = br.routeAndCheck( paths.front().waypoints );
        std::printf( "routeAndCheck: ok=%d placed=%d vias=%d collided=%d reason='%s'\n",
                     r.ok, r.placed, r.vias, r.collided, r.reason.c_str() );
    }

    // --- forced multi-layer route: exercise the via path deterministically --
    int bcu = br.pnsLayer( B_Cu );
    VECTOR2I mid( ( pa.x + pb.x ) / 2, ( pa.y + pb.y ) / 2 );
    std::vector<gplan::Waypoint> ml = {
        { { (double) pa.x,  (double) pa.y  }, fcu },
        { { (double) mid.x, (double) mid.y }, fcu },
        { { (double) mid.x, (double) mid.y }, bcu },   // <- via here
        { { (double) pb.x,  (double) pb.y  }, bcu },
    };
    std::printf( "multilayer route F.Cu(%d)->B.Cu(%d), via at (%d,%d)\n",
                 fcu, bcu, mid.x, mid.y );
    gbridge::RouteResult mr = br.routeAndCheck( ml );
    std::printf( "multilayer routeAndCheck: ok=%d placed=%d vias=%d collided=%d reason='%s'\n",
                 mr.ok, mr.placed, mr.vias, mr.collided, mr.reason.c_str() );
    if( mr.placed && mr.vias >= 1 )
        std::printf( "VIA PATH OK (%d via placed)\n", mr.vias );
    else
        std::printf( "VIA PATH: no via placed (placed=%d) — see notes\n", mr.placed );

    // --- routeAndExtract: get the routed geometry back for the host to apply ---
    if( !paths.empty() )
    {
        gbridge::RouteGeom g = br.routeAndExtract( paths.front().waypoints );
        std::printf( "routeAndExtract: ok=%d placed=%d net=%d segs=%zu vias=%zu "
                     "removedSegs=%zu removedVias=%zu reason='%s'\n",
                     g.ok, g.placed, g.netcode, g.segs.size(), g.viaList.size(),
                     g.removedSegs.size(), g.removedVias.size(), g.reason.c_str() );
        if( !g.segs.empty() )
        {
            const auto& s = g.segs.front();   // {x1,y1,x2,y2,width,boardLayer}
            std::printf( "  first seg: (%.0f,%.0f)->(%.0f,%.0f) w=%.0f layer=%.0f\n",
                         s[0], s[1], s[2], s[3], s[4], s[5] );
        }
        if( g.placed && !g.segs.empty() )
            std::printf( "EXTRACT OK (%zu segs, %zu vias returned)\n",
                         g.segs.size(), g.viaList.size() );
        else
            std::printf( "EXTRACT: nothing returned (placed=%d)\n", g.placed );
    }

    // --- T12: route the same net under Walkaround as well as Shove ----------
    if( !paths.empty() )
    {
        br.setMode( gbridge::PnsBridge::RouteMode::Walkaround );
        gbridge::RouteResult wr = br.routeAndCheck( paths.front().waypoints );
        std::printf( "T12 walkaround routeAndCheck: ok=%d placed=%d collided=%d reason='%s'\n",
                     wr.ok, wr.placed, wr.collided, wr.reason.c_str() );
        br.setMode( gbridge::PnsBridge::RouteMode::Shove );   // restore default
        if( !wr.placed )
        { std::printf( "T12 FAIL: walkaround placed nothing\n" ); return 1; }
    }

    // --- T10: cleanup + re-attach to the same board + route again -----------
    if( !paths.empty() )
    {
        br.cleanup();
        if( !br.attach( board.get() ) )
        { std::printf( "T10 FAIL: re-attach\n" ); return 1; }
        gbridge::RouteResult rr = br.routeAndCheck( paths.front().waypoints );
        std::printf( "T10 re-attach routeAndCheck: ok=%d placed=%d\n", rr.ok, rr.placed );
        if( !rr.placed )
        { std::printf( "T10 FAIL: route after re-attach placed nothing\n" ); return 1; }
    }

    // --- routeAndCommit: lossless commit-to-world (PNS change stream) --------
    if( !paths.empty() )
    {
        gbridge::RouteChange c = br.routeAndCommit( paths.front().waypoints );
        std::printf( "routeAndCommit: ok=%d placed=%d reached=%d collided=%d net=%d "
                     "added=%zu mod=%zu removed=%zu vias=%d blocking=(%.0f,%.0f)\n",
                     c.ok, c.placed, c.reached, c.collided, c.netcode, c.addedSegs.size(),
                     c.modSegUuids.size(), c.removedUuids.size(), c.vias,
                     c.blocking.x, c.blocking.y );
        if( !c.ok )
        { std::printf( "COMMIT FAIL: did not reach target (honest)\n" ); return 1; }
        std::printf( "COMMIT OK (reached, %zu added segs, %zu modified neighbours by uuid)\n",
                     c.addedSegs.size(), c.modSegUuids.size() );

        // --- §4.6 optimizeRoute: post-route optimizer pass on the route we
        // just committed to the world. Probe the midpoint of a committed
        // segment; lengths before/after are PRINTED, not asserted (the
        // improvement is board-dependent — a short 2-segment route may already
        // be optimal). Assert only that the pass finds the copper and runs
        // without crash/collision.
        if( !c.addedSegs.empty() )
        {
            const auto& s = c.addedSegs.front();   // {x1,y1,x2,y2,width,boardLayer}
            double mx = ( s[0] + s[2] ) / 2.0, my = ( s[1] + s[3] ) / 2.0;
            int    ol = br.pnsLayer( (int) s[5] );

            gbridge::OptimizeResult o = br.optimizeRoute( mx, my, ol );
            std::printf( "optimizeRoute: ok=%d found=%d improved=%d "
                         "length %.0f -> %.0f nm, corners %d -> %d, "
                         "added=%zu removed=%zu reason='%s'\n",
                         o.ok, o.found, o.improved, o.lengthBefore, o.lengthAfter,
                         o.cornersBefore, o.cornersAfter,
                         o.change.addedSegs.size(), o.change.removedUuids.size(),
                         o.reason.c_str() );
            if( !o.found )
            { std::printf( "OPTIMIZE FAIL: no copper at committed-route midpoint\n" ); return 1; }
            if( !o.ok )
            { std::printf( "OPTIMIZE FAIL: pass did not run cleanly ('%s')\n",
                           o.reason.c_str() ); return 1; }
            std::printf( "OPTIMIZE OK (ran collision-free; length %.0f -> %.0f nm, "
                         "improvement board-dependent)\n", o.lengthBefore, o.lengthAfter );
        }
    }

    // --- probeTarget: seedable / congested flag for a target ----------------
    {
        int    bcu = br.pnsLayer( B_Cu );
        double clr = 200000;
        gbridge::TargetProbe t1 = br.probeTarget( pa.x, pa.y, fcu, 1, clr, bcu );
        std::printf( "probeTarget net-1 pad: seedable=%d congested=%d nearestForeign=%.0f "
                     "foreignNet=%d nearestOther=%.0f\n",
                     t1.seedable, t1.congested, t1.nearestForeign, t1.foreignNet, t1.nearestOther );
        gbridge::TargetProbe t2 = br.probeTarget( pa.x + 50000000, pa.y, fcu, 1, clr, bcu );
        std::printf( "probeTarget open point: seedable=%d congested=%d\n",
                     t2.seedable, t2.congested );
        if( t2.seedable )
        { std::printf( "PROBE FAIL: open point reported seedable\n" ); return 1; }
        std::printf( "PROBE OK\n" );
    }

    // --- dragComponent: shove a footprint + its tracks (connectivity-safe) ---
    {
        VECTOR2I seed; FOOTPRINT* seedFp = nullptr;
        for( FOOTPRINT* fp : board->Footprints() )      // seed from a real pad (has copper)
        {
            for( PAD* pad : fp->Pads() )
                if( pad->IsOnLayer( F_Cu ) ) { seed = pad->GetPosition(); seedFp = fp; break; }
            if( seedFp ) break;
        }
        if( seedFp )
        {
            VECTOR2I c = seed;
            gbridge::RouteChange d = br.dragComponent( c.x, c.y, c.x + 50000, c.y, false );
            std::printf( "dragComponent (+0.05mm): ok=%d placed=%d modSegs=%zu reason='%s'\n",
                         d.ok, d.placed, d.modSegs.size(), d.reason.c_str() );
            std::printf( "DRAG ran (clean=%d, board-dependent)\n", d.ok );

            // Lock the footprint -> dragComponent must refuse (deterministic).
            seedFp->SetLocked( true );
            gbridge::RouteChange d2 = br.dragComponent( c.x, c.y, c.x + 50000, c.y, false );
            std::printf( "dragComponent on LOCKED footprint: ok=%d reason='%s'\n",
                         d2.ok, d2.reason.c_str() );
            seedFp->SetLocked( false );
            if( d2.ok || d2.reason != std::string( "component locked" ) )
            { std::printf( "LOCK FAIL: locked component was not refused\n" ); return 1; }
            std::printf( "LOCK OK (locked component refused)\n" );

            // Speculative probe (no commit) — a zero move must be clean.
            gbridge::DragProbe pz = br.probeDrag( c.x, c.y, c.x, c.y );
            std::printf( "probeDrag(zero move): clean=%d cost=%.0f shoved=%d\n",
                         pz.clean, pz.cost, pz.shoved );
            if( !pz.clean )
            { std::printf( "PROBE-DRAG FAIL: zero move not clean\n" ); return 1; }

            // Router-driven placement search: probe candidates, commit the best.
            std::vector<std::vector<double>> cands = {
                { (double)( c.x + 30000 ), (double) c.y },
                { (double) c.x, (double)( c.y + 30000 ) },
                { (double)( c.x - 30000 ), (double) c.y } };
            gbridge::ShoveResult sr = br.shoveComponentSearch( c.x, c.y, cands );
            std::printf( "shoveComponentSearch: committed=%d chosen=(%.0f,%.0f) cost=%.0f\n",
                         sr.committed, sr.x, sr.y, sr.cost );
            std::printf( "SHOVE-SEARCH ran\n" );
        }
    }

    // --- routeLongHaul: checkpoint-retry driver -----------------------------
    if( !paths.empty() )
    {
        gbridge::RouteChange l = br.routeLongHaul( paths.front().waypoints, 4 );
        std::printf( "routeLongHaul: ok=%d reached=%d added=%zu blocking=(%.0f,%.0f)\n",
                     l.ok, l.reached, l.addedSegs.size(), l.blocking.x, l.blocking.y );
        if( !l.ok )
        { std::printf( "LONGHAUL FAIL: net 1 should reach\n" ); return 1; }
        std::printf( "LONGHAUL OK (reached)\n" );
    }

    // --- T-DP: differential pair routing. This board has no diff-pair-named
    // nets, so PNS's FindDpPrimitivePair is expected to fail to find a coupled
    // net — the point here is proving the code path runs (mode switch + honest
    // failure), not that this particular board has a pair to route.
    if( !paths.empty() )
    {
        gbridge::RouteChange dp = br.routeDiffPairAndCommit( paths.front().waypoints );
        std::printf( "routeDiffPairAndCommit: ok=%d placed=%d reached=%d reason='%s'\n",
                     dp.ok, dp.placed, dp.reached, dp.reason.c_str() );
        // Mode must be restored to single-route regardless of outcome — prove it
        // by running an ordinary route right after and confirming it still works.
        gbridge::RouteResult afterDp = br.routeAndCheck( paths.front().waypoints );
        std::printf( "post-DP routeAndCheck: ok=%d placed=%d\n", afterDp.ok, afterDp.placed );
        if( !afterDp.placed )
        { std::printf( "DP FAIL: router mode not restored after diff-pair attempt\n" ); return 1; }
        std::printf( "DIFFPAIR OK (mode restored, path ran without crash)\n" );
    }

    // --- T-TUNE: length-tune an EXISTING routed segment on net 1. -----------
    {
        PCB_TRACK* seg = nullptr;
        for( PCB_TRACK* t : board->Tracks() )
            if( t->GetNetCode() == 1 && t->Type() == PCB_TRACE_T )
            { seg = t; break; }

        if( seg )
        {
            VECTOR2I s = seg->GetStart(), e = seg->GetEnd();
            int      tl = br.pnsLayer( seg->GetLayer() );
            double   curLen = ( e - s ).EuclideanNorm();
            long long target = (long long) curLen + 500000;   // +0.5mm

            gbridge::PnsBridge::TuneResult tu = br.tuneLength(
                    s.x, s.y, e.x, e.y, tl, target );
            std::printf( "tuneLength: ok=%d placed=%d status=%d curLen=%lld target=%lld "
                         "added=%zu removed=%zu reason='%s'\n",
                         tu.change.ok, tu.change.placed, tu.status, tu.currentLength,
                         tu.targetLength, tu.change.addedSegs.size(),
                         tu.change.removedUuids.size(), tu.change.reason.c_str() );
            std::printf( "TUNE ran (status %s)\n",
                         tu.status == 2 ? "TUNED" : tu.status == 0 ? "TOO_SHORT"
                         : tu.status == 1 ? "TOO_LONG" : "unknown" );
        }
        else
        {
            std::printf( "TUNE: no net-1 segment found to tune; skipped\n" );
        }
    }

    // --- T-DRAG-PT (§4.7): general corner/segment drag on an EXISTING routed
    // segment on net 1 (mid-span point -> PNS auto-picks DM_SEGMENT). ---------
    {
        PCB_TRACK* seg = nullptr;
        for( PCB_TRACK* t : board->Tracks() )
            if( t->GetNetCode() == 1 && t->Type() == PCB_TRACE_T )
            { seg = t; break; }

        if( seg )
        {
            VECTOR2I s = seg->GetStart(), e = seg->GetEnd();
            VECTOR2I mid = ( s + e ) / 2;
            VECTOR2I perp( -( e.y - s.y ), e.x - s.x );   // sideways nudge
            double   plen = perp.EuclideanNorm();
            VECTOR2I nudge = ( plen > 0 )
                ? VECTOR2I( (int)( perp.x * 100000.0 / plen ), (int)( perp.y * 100000.0 / plen ) )
                : VECTOR2I( 0, 100000 );

            gbridge::DragProbe pd = br.probeTrackDrag( mid.x, mid.y,
                                                        mid.x + nudge.x, mid.y + nudge.y );
            std::printf( "probeTrackDrag(mid+0.1mm): clean=%d cost=%.0f shoved=%d\n",
                         pd.clean, pd.cost, pd.shoved );

            gbridge::RouteChange dt = br.dragTrackPoint( mid.x, mid.y,
                                                          mid.x + nudge.x, mid.y + nudge.y, false );
            std::printf( "dragTrackPoint(mid+0.1mm): ok=%d placed=%d modSegs=%zu reason='%s'\n",
                         dt.ok, dt.placed, dt.modSegs.size(), dt.reason.c_str() );
            std::printf( "DRAG-PT ran (ok=%d, board-dependent)\n", dt.ok );
        }
        else
        {
            std::printf( "DRAG-PT: no net-1 segment found to drag; skipped\n" );
        }
    }

    // --- T9: make net 1 unrouted (remove its tracks), find the ratsnest target
    //          via the bridge, and route the now-unrouted net. ----------------
    {
        std::vector<PCB_TRACK*> rm;
        for( PCB_TRACK* t : board->Tracks() )
            if( t->GetNetCode() == 1 && t->Type() != PCB_VIA_T )
                rm.push_back( t );
        for( PCB_TRACK* t : rm )
            board->Remove( t );
        board->BuildConnectivity();
        br.cleanup();
        br.attach( board.get() );
        std::printf( "T9: removed %zu net-1 tracks, re-synced\n", rm.size() );

        VECTOR2I padP; bool haveStart = false;
        for( FOOTPRINT* fp : board->Footprints() )
        {
            for( PAD* pad : fp->Pads() )
                if( pad->GetNetCode() == 1 && pad->IsOnLayer( F_Cu ) )
                { padP = pad->GetPosition(); haveStart = true; break; }
            if( haveStart ) break;
        }

        if( haveStart )
        {
            auto tgt = br.nearestUnconnected( padP.x, padP.y, fcu );
            if( !tgt )
            { std::printf( "T9 FAIL: no ratsnest target on the unrouted net\n" ); return 1; }
            std::printf( "T9 ratsnest target: (%.0f,%.0f) L%d\n",
                         tgt->p.x, tgt->p.y, tgt->layer );
            std::vector<gplan::Waypoint> wp = {
                { { (double) padP.x, (double) padP.y }, fcu }, *tgt };
            gbridge::RouteResult r = br.routeAndCheck( wp );
            std::printf( "T9 routeAndCheck on unrouted net: ok=%d placed=%d collided=%d\n",
                         r.ok, r.placed, r.collided );
            if( !r.placed )
            { std::printf( "T9 FAIL: could not route the now-unrouted net\n" ); return 1; }
        }
        else
            std::printf( "T9: no net-1 F.Cu pad found; skipping\n" );
    }

    // --- §4.8 routeWithStrategy: automated walkaround->shove fallback pass on
    // the still-unrouted net 1 left behind by T9 (routeAndCheck never commits).
    {
        VECTOR2I padP; bool haveStart = false;
        for( FOOTPRINT* fp : board->Footprints() )
        {
            for( PAD* pad : fp->Pads() )
                if( pad->GetNetCode() == 1 && pad->IsOnLayer( F_Cu ) )
                { padP = pad->GetPosition(); haveStart = true; break; }
            if( haveStart ) break;
        }

        if( haveStart )
        {
            auto tgt = br.nearestUnconnected( padP.x, padP.y, fcu );
            if( !tgt )
            { std::printf( "T-STRATEGY FAIL: no ratsnest target on unrouted net\n" ); return 1; }

            std::vector<gplan::Waypoint> wp = {
                { { (double) padP.x, (double) padP.y }, fcu }, *tgt };

            gbridge::RouteChange s = br.routeWithStrategy( wp );
            std::printf( "routeWithStrategy: ok=%d placed=%d reached=%d collided=%d net=%d "
                         "added=%zu mod=%zu removed=%zu vias=%d\n",
                         s.ok, s.placed, s.reached, s.collided, s.netcode, s.addedSegs.size(),
                         s.modSegUuids.size(), s.removedUuids.size(), s.vias );
            if( !s.ok )
            { std::printf( "T-STRATEGY FAIL: neither walkaround nor shove reached target\n" ); return 1; }
            std::printf( "T-STRATEGY OK (reached, %zu added segs, %zu vias)\n",
                         s.addedSegs.size(), s.vias );
        }
        else
            std::printf( "T-STRATEGY: no net-1 F.Cu pad found; skipping\n" );
    }

    // --- T10-RELOAD: PnsBridge::load() re-callable in the same process on a
    // DIFFERENT board (0.5 — verifies attach()'s cleanup()-first re-entrancy
    // actually holds at the load()-convenience-API level, not just attach()).
    {
        // NOTE: SETTINGS_MANAGER::LoadProject() (above, and inside br2.load())
        // calls wxSetWorkingDirectory() on the project's own directory as a
        // side effect, so resolve the second board relative to the FIRST
        // board's already-absolute directory rather than the process cwd.
        wxFileName fnPcb2( fnPcb.GetPath(), "issue7325.kicad_pcb" );
        const wxString pcb2 = fnPcb2.GetFullPath();

        gbridge::PnsBridge br2;
        if( !br2.load( std::string( pcb2.utf8_str() ) ) )
        { std::printf( "T10-RELOAD FAIL: load() #1 (%s)\n", (const char*) pcb2.utf8_str() ); return 1; }
        std::vector<gplan::Obstacle> obsA = br2.getAllObstacles();
        std::printf( "T10-RELOAD: load() #1 ok, %zu obstacles\n", obsA.size() );

        // Reload the SAME bridge instance with the ORIGINAL board — this is the
        // real 0.5 case: load->close->load in one process, one PnsBridge.
        if( !br2.load( std::string( pcb.utf8_str() ) ) )
        { std::printf( "T10-RELOAD FAIL: load() #2 (%s)\n", (const char*) pcb.utf8_str() ); return 1; }
        std::vector<gplan::Obstacle> obsB = br2.getAllObstacles();
        std::printf( "T10-RELOAD: load() #2 ok, %zu obstacles\n", obsB.size() );

        // The reloaded world must be the SECOND board's, not stale first-board
        // state (route a real net on it to prove the PNS world is live+correct).
        PCB_TRACK* seed2 = nullptr;
        for( PCB_TRACK* t : br2.board()->Tracks() )
            if( t->Type() == PCB_TRACE_T ) { seed2 = t; break; }
        if( seed2 )
        {
            VECTOR2I a = seed2->GetStart(), b = seed2->GetEnd();
            int lyr2 = br2.pnsLayer( seed2->GetLayer() );
            std::vector<gplan::Waypoint> wp2 = {
                { { (double) a.x, (double) a.y }, lyr2 }, { { (double) b.x, (double) b.y }, lyr2 } };
            gbridge::RouteResult rr2 = br2.routeAndCheck( wp2 );
            std::printf( "T10-RELOAD: route on reloaded-original board ok=%d placed=%d\n",
                         rr2.ok, rr2.placed );
            if( !rr2.placed )
            { std::printf( "T10-RELOAD FAIL: reloaded world not routable\n" ); return 1; }
        }
        else
            std::printf( "T10-RELOAD: no PCB_TRACE_T on reloaded board; skipped route check\n" );

        std::printf( "T10-RELOAD OK (load->close->load, same process, same PnsBridge)\n" );

        // --- T-ERR (0.6): load() error detail distinguishes failure modes ---
        std::string err;
        gbridge::PnsBridge br3;
        if( br3.load( "/nonexistent/path/does_not_exist.kicad_pcb", &err ) )
        { std::printf( "T-ERR FAIL: load() of a nonexistent path unexpectedly succeeded\n" ); return 1; }
        std::printf( "T-ERR: nonexistent path -> err='%s'\n", err.c_str() );
        if( err.empty() )
        { std::printf( "T-ERR FAIL: no error detail on load() failure\n" ); return 1; }
        std::printf( "T-ERR OK (load() failure surfaces a specific reason)\n" );
    }

    std::printf( "SMOKETEST OK\n" );
    return 0;
}

int main( int argc, char** argv )
{
    wxInitialize( argc, argv );
    wxDisableAsserts();   // headless: never block on a wx assert dialog
    SetPgm( &s_pgm );
    Pgm().InitPgm( /*aHeadless=*/ true, /*aIsUnitTest=*/ true );

    int rc = run( argc, argv );
    std::fflush( stdout );

    // KiCad's global/static teardown (Pgm, library-manager async loads, wx) is
    // unreliable in a bare headless harness and can hang/crash at exit. The work
    // is done and flushed, so hard-exit with the real status instead.
    std::_Exit( rc );
}

// pns_bridge.h
//
// KiCad-LINKED adapter that connects the standalone planner core (gplan) to
// KiCad's PNS detailed router. This is the half that CANNOT be standalone:
// it reads a real BOARD and drives PNS push-and-shove.
//
//   - getObstacles(layer): BOARD -> std::vector<gplan::Obstacle>
//   - routeAndCheck(path):  gplan waypoints -> PNS shove -> (ok, blocking point)
//
// Build: inside the KiCad source tree (links pcbnew/kicommon). It is NOT part of
// the dependency-free core and will not compile in isolation. Modelled on the
// verified headless harness qa/tools/pns/pns_log_player.cpp and the evaluate
// path in pcbnew/router/pns_router.cpp (markViolations / QueryColliding).

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "planner_core.h"

class BOARD;
class SETTINGS_MANAGER;
namespace PNS { class ROUTER; class ITEM; }
class PNS_KICAD_IFACE_BASE;

namespace gbridge {

struct RouteResult
{
    bool        ok        = false;   // placed and reached, no residual collisions
    bool        collided  = false;   // placed but obstacles remain
    bool        placed    = false;   // PNS produced any geometry at all
    gplan::Point blocking;           // where it got stuck (for bumpCongestion)
    std::string reason;              // router->FailureReason()
};

class PnsBridge
{
public:
    PnsBridge();
    ~PnsBridge();

    // Loads board + project (.kicad_pro) + custom rules (.kicad_dru) by the
    // usual filename convention, builds the DRC engine, and syncs the PNS world.
    // Returns false on failure.
    bool load( const std::string& pcbPath );

    // Map a KiCad copper layer (PCB_LAYER_ID int) to/from a PNS layer index.
    int  pnsLayer( int boardLayer ) const;

    // Extract obstacles on one PNS copper layer as gplan::Obstacle:
    //   fixed = pads / locked tracks / keepouts ; movable = unlocked copper.
    // Fixed shapes are emitted as (convex) bounding boxes for v1 robustness.
    std::vector<gplan::Obstacle> getObstacles( int pnsLayer ) const;

    // Convenience: obstacles across all copper layers (each tagged with its layer).
    std::vector<gplan::Obstacle> getAllObstacles() const;

    // Drive PNS along the planned waypoints (shove mode) and report the outcome.
    // waypoints carry layers; a layer change is realized as a via (see .cpp).
    RouteResult routeAndCheck( const std::vector<gplan::Waypoint>& waypoints );

    BOARD*       board() const { return m_board; }
    PNS::ROUTER* router() const { return m_router.get(); }

private:
    std::unique_ptr<SETTINGS_MANAGER>     m_settings;
    BOARD*                                m_board = nullptr;   // owned via m_boardHolder
    std::shared_ptr<BOARD>                m_boardHolder;
    std::unique_ptr<PNS_KICAD_IFACE_BASE> m_iface;
    std::unique_ptr<PNS::ROUTER>          m_router;
};

} // namespace gbridge

#pragma once
// online_jump_point_locator_prune.h
//
// A class wrapper around some code that finds, online, jump point
// successors of an arbitrary nodes in a uniform-cost grid map.
//
// For theoretical details see:
// [Harabor D. and Grastien A, 2011, 
// Online Graph Pruning Pathfinding on Grid Maps, AAAI]
//
// @author: dharabor
// @created: 03/09/2012
//

#include "jps.h"
#include "gridmap.h"
#include "search_node.h"
#include "online_jps_pruner.h"
#include <vector>

namespace warthog
{

class online_jump_point_locator_prune 
{
  public: 
    online_jps_pruner* jpruner;

    online_jump_point_locator_prune(warthog::gridmap* map);
    ~online_jump_point_locator_prune();
    bool gprune = false;
    bool jprune = false;
    bool verbose = false;

    void
    jump(warthog::jps::direction d, uint32_t node_id, uint32_t goalid, 
        uint32_t& jumpnode_id, warthog::cost_t& jumpcost);

    uint32_t 
    mem()
    {
      return sizeof(this) + rmap_->mem();
    }

    inline void set_verbose(bool verbose) {
      this->verbose = verbose;
      jpruner->verbose = verbose;
    }
  private:
    void
    jump_northwest(uint32_t node_id, uint32_t goal_id, 
        uint32_t& jumpnode_id, warthog::cost_t& jumpcost);
    void
    jump_northeast(uint32_t node_id, uint32_t goal_id, 
        uint32_t& jumpnode_id, warthog::cost_t& jumpcost);
    void
    jump_southwest(uint32_t node_id, uint32_t goal_id, 
        uint32_t& jumpnode_id, warthog::cost_t& jumpcost);
    void
    jump_southeast(uint32_t node_id, uint32_t goal_id, 
        uint32_t& jumpnode_id, warthog::cost_t& jumpcost);
    uint32_t
    jump_north(uint32_t node_id, uint32_t goal_id, 
        uint32_t& jumpnode_id, warthog::cost_t& jumpcost);
    uint32_t
    jump_south(uint32_t node_id, uint32_t goal_id, 
        uint32_t& jumpnode_id, warthog::cost_t& jumpcost);
    uint32_t
    jump_east(uint32_t node_id, uint32_t goal_id, 
        uint32_t& jumpnode_id, warthog::cost_t& jumpcost);
    uint32_t
    jump_west(uint32_t node_id, uint32_t goal_id, 
        uint32_t& jumpnode_id, warthog::cost_t& jumpcost);

    // these versions can be passed a map parameter to
    // use when jumping. they allow switching between
    // map_ and rmap_ (a rotated counterpart).
    uint32_t  
    __jump_east(uint32_t node_id, uint32_t goal_id, 
        uint32_t& jumpnode_id, warthog::cost_t& jumpcost, 
        warthog::gridmap* mymap);
    uint32_t  
    __jump_west(uint32_t node_id, uint32_t goal_id, 
        uint32_t& jumpnode_id, warthog::cost_t& jumpcost, 
        warthog::gridmap* mymap);
    uint32_t  
    __jump_north(uint32_t node_id, uint32_t goal_id, 
        uint32_t& jumpnode_id, warthog::cost_t& jumpcost,
        warthog::gridmap* mymap);
    uint32_t
    __jump_south(uint32_t node_id, uint32_t goal_id, 
        uint32_t& jumpnode_id, warthog::cost_t& jumpcost, 
        warthog::gridmap* mymap);

    inline uint32_t
    map_id_to_rmap_id(uint32_t mapid)
    {
      if(mapid == warthog::INF) { return mapid; }

      uint32_t x, y;
      uint32_t rx, ry;
      map_->to_unpadded_xy(mapid, x, y);
      ry = x;
      rx = map_->header_height() - y - 1;
      return rmap_->to_padded_id(rx, ry);
    }

    inline uint32_t
    rmap_id_to_map_id(uint32_t rmapid)
    {
      if(rmapid == warthog::INF) { return rmapid; }

      uint32_t x, y;
      uint32_t rx, ry;
      rmap_->to_unpadded_xy(rmapid, rx, ry);
      x = ry;
      y = rmap_->header_width() - rx - 1;
      return map_->to_padded_id(x, y);
    }

    warthog::gridmap*
    create_rmap();

    warthog::gridmap* map_;
    warthog::gridmap* rmap_;

    inline bool
    gValPruned(uint32_t jumpnode_id) {
      if (!gprune) return false;
      if (jpruner->rmapflag) {
        jumpnode_id = rmap_id_to_map_id(jumpnode_id);
      }
      return jpruner->gValPruned(jumpnode_id, jpruner->jumpdist * warthog::ONE + jpruner->curg);
    }

    inline void
    createConstraint(uint32_t jumpnode_id, warthog::cost_t c) {
      if (jpruner->rmapflag) {
        jumpnode_id = rmap_id_to_map_id(jumpnode_id);
      }
      jpruner->createConstraint(jumpnode_id, c);
    }

    inline uint32_t
    gVal(uint32_t jumpnode_id) {
      if (jpruner->rmapflag) {
        jumpnode_id = rmap_id_to_map_id(jumpnode_id);
      }
      return jpruner->gVal(jumpnode_id);
    }
};

}

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
#include <vector>

namespace warthog
{

class online_jump_point_locator_prune 
{
  public: 
    typedef std::pair<uint32_t, warthog::cost_t> pic;
    bool rmapflag;
    bool active_prune;
    uint32_t scan_cnt;
    warthog::search_node* cur;
    std::vector<pic> vis;

    online_jump_point_locator_prune(warthog::gridmap* map);
    ~online_jump_point_locator_prune();

    void
    jump(warthog::jps::direction d, uint32_t node_id, uint32_t goalid, 
        uint32_t& jumpnode_id, warthog::cost_t& jumpcost);

    uint32_t 
    mem()
    {
      return sizeof(this) + rmap_->mem();
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

    inline bool
    is_pruned(uint32_t jumpnode_id, uint32_t goal_id, warthog::cost_t c) {
      if (!active_prune) return false;

      if (this->rmapflag) jumpnode_id = rmap_id_to_map_id(jumpnode_id);
      if (vis[jumpnode_id].first != goal_id || vis[jumpnode_id].second == 0) {
        vis[jumpnode_id] = {goal_id, c + cur->get_g()};
        return false;
      }
      else {
        if (c + cur->get_g() >= vis[jumpnode_id].second) {
          return true;
        }
        vis[jumpnode_id] = {goal_id, c + cur->get_g()};
        return false;
      }
    }

    warthog::gridmap*
    create_rmap();

    warthog::gridmap* map_;
    warthog::gridmap* rmap_;
    uint32_t jumplimit_;
};

}

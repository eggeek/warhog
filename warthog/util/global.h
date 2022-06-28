#pragma once

#include <vector>
#include "constants.h"
#include "problem_instance.h"
#include "blocklist.h"
#include "gridmap.h"
#include "search_node.h"
using namespace std;
// set global variable that can be accessed everywhere
namespace global{

namespace statis {
  extern vector<warthog::cost_t> dist;
  extern uint32_t subopt_expd;
  extern uint32_t subopt_touch;
  extern uint32_t scan_cnt;
  extern uint32_t prunable;

  inline void update_subopt_expd(uint32_t id, warthog::cost_t gval) {
    assert(dist.empty() || id < dist.size());
    if (!dist.empty() && gval > dist[id]) subopt_expd++;
  }

  inline void update_pruneable(warthog::search_node* cur) {
    warthog::search_node* pa = cur->get_parent();
    // parent is subopt
    if (!dist.empty() && pa != nullptr && pa->get_g() > dist[pa->get_id()]) prunable++;
  }

  inline void update_subopt_touch(uint32_t id, warthog::cost_t gval) {
    assert(dist.empty() || id < dist.size());
    if (!dist.empty() && gval > dist[id]) subopt_touch++;
  }

  inline void clear() {
    dist.clear();
    subopt_expd = 0;
    subopt_touch = 0;
    scan_cnt = 0;
    prunable = 0;
  }

  inline void sanity_checking(uint32_t id, warthog::cost_t gval) {
    if (!dist.empty() && gval < dist[id]) {
      cerr << "invalid gval less than optimal: id=" << id << ", gval=" << gval << endl;
      assert(false);
      exit(1);
    }
  }
};

namespace query {
  extern warthog::blocklist* nodepool;
  extern uint32_t startid, goalid;
  extern warthog::cost_t cur_diag_gval;
  extern warthog::problem_instance* pi;
  extern warthog::gridmap *map, *rmap; // rmap is map rotated by 90 clockwise

  inline warthog::cost_t gval(uint32_t id) {
    warthog::search_node* s = nodepool->get(id);
    if (s != nullptr && s->get_searchid() == pi->get_searchid()) return s->get_g();
    else return warthog::INF;
  }

  inline warthog::gridmap*
  create_rmap(warthog::gridmap* map_)
  {
    uint32_t maph = map_->header_height();
    uint32_t mapw = map_->header_width();
    uint32_t rmaph = mapw;
    uint32_t rmapw = maph;
    warthog::gridmap* rmap = new warthog::gridmap(rmaph, rmapw);

    for(uint32_t x = 0; x < mapw; x++) 
    {
      for(uint32_t y = 0; y < maph; y++)
      {
        uint32_t label = map_->get_label(map_->to_padded_id(x, y));
        uint32_t rx = ((rmapw-1) - y);
        uint32_t ry = x;
        uint32_t rid = rmap->to_padded_id(rx, ry);
        rmap->set_label(rid, label);
      }
    }
    return rmap;
  }

  inline void clear() {
    if (map != nullptr) delete map;
    if (rmap != nullptr) delete rmap;
    map = rmap = nullptr;
  }
};

  inline void clear() {
    statis::clear();
    query::clear();
  }
}

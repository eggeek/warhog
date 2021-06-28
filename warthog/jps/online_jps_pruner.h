#pragma once
#include <vector>

#include "mapper.h"
#include "constants.h"
#include "helpers.h"
#include "problem_instance.h"
#include "search_node.h"
#include "blocklist.h"

namespace warthog {

using namespace std;

const static int ILLEGAL_LEN = -1;

struct Record {
  cost_t cost;
  uint32_t fromd_mask;
  uint32_t search_id;
};

struct Constraint {
  search_node* s;
  int d;
  int dg;
  int l; // 0<=l

  void setnull() { s = nullptr; }
  void calc_limit(int step) {
    if (s == nullptr) l = ILLEGAL_LEN;
    else {
      if (dg + d < step * (int)warthog::ROOT_TWO) l = 0;
      else {
        l = ((dg + d) - step * (int)warthog::ROOT_TWO) / 2;
        if (l + (int)warthog::ONE * step > d) {
          l = ILLEGAL_LEN;
          s = nullptr;
        }
        else {
          if (l % warthog::ONE == 0) l = l / warthog::ONE;
          else l = l / warthog::ONE + 1;
          l += 1; // scan one more step to check obstacle
        }
      }
    }
  }
};

class online_jps_pruner {

public:

  problem_instance* pi;
  Mapper* mapper;
  search_node* cur;
  Constraint north,south,east,west,v,h;
  bool t_labelv, t_labelh;
  int t_mapid, t_rmapid;

  vector<Record> rec;

  void init_constraint(Constraint& c, search_node* s, cost_t jcost) {
    if (s != nullptr && s->get_searchid() == pi->get_searchid()) c.s = s; 
    else c.s = nullptr;
    if (c.s != nullptr) {
      c.d = jcost;
      c.dg = s->get_g() - cur->get_g();
    }
  }

  void reset_constraints() {
    north.s = south.s = east.s = west.s = v.s = h.s = nullptr;
  }

  void init(uint32_t size) {
    rec.resize(size);
    for (size_t i=0; i<size; i++) rec[i] = {0, 0, warthog::INFID};
  }

  inline void update(uint32_t id, cost_t cost, uint32_t fromd) {
    if (rec[id].search_id == pi->get_searchid()) {
      if (rec[id].cost > cost) {
        rec[id] = {cost, fromd, pi->get_searchid()};
      }
      else if (rec[id].cost == cost) {
        rec[id].fromd_mask |= fromd;
      }
    }
    else {
      rec[id] = {cost, fromd, pi->get_searchid()};
    }
  }

  // inline void update_constraint(Constraint& c, uint32_t jump_id,
  // uint32_t dia_step, uint32_t cardinal_step, blocklist* nodepool) {
  //   c.s = nodepool->get(jump_id);
  //   if (c.s == nullptr) return;
  //   cost_t gval = cur->get_g() + dia_step * warthog::ROOT_TWO;
  //   c.dg = c.s->get_g() - gval;
  //   c.d = cardinal_step * warthog::ONE;
  // }

  inline uint32_t get_fromd_mask(uint32_t id, uint32_t cost) const {
    if (rec[id].search_id == pi->get_searchid()) {
      if (rec[id].cost == cost) return rec[id].fromd_mask;
      else if (rec[id].cost < cost) {
        return warthog::INVALID;
      } else return 0;
    }
    else return 0;
  }

  inline uint32_t get_pruned_suc(const search_node* cur, const uint32_t& c_tiles) const {
    uint32_t res = warthog::ALLMOVE;
    uint32_t cid = cur->get_id();
    uint32_t dmask = get_fromd_mask(cid, cur->get_g());
    if (dmask == warthog::INVALID) return 0;
    while (dmask && res) {
      uint32_t lb = warthog::helpers::LOWB(dmask);
      res &= mapper->get_successors(warthog::jps::d2i(lb), cid);
      // res &= warthog::jps::compute_successors((warthog::jps::direction)lb, c_tiles);
      dmask -= lb;
    }
    return res;
  }

  inline void before_scanv(gridmap* rmap, int rmapid, int direct) {
    if (v.l > 0) {
      t_rmapid = rmapid + direct * (v.l + 1);
      assert(t_rmapid >= 0);
      t_labelv = rmap->get_label(t_rmapid);
      rmap->set_label(t_rmapid, false);
    }
  }

  inline void before_scanh(gridmap* map, int mapid, int direct) {
    if (h.l > 0) {
      t_mapid = mapid + direct * (h.l + 1);
      assert(t_mapid >= 0);
      t_labelh = map->get_label(t_mapid);
      map->set_label(t_mapid, false);
    }
  }

  inline void after_scanv(gridmap* rmap, int cardinal_step) {
    if (v.l > 0) {
      rmap->set_label(t_rmapid, t_labelv);
      if (cardinal_step < v.l) {
        v.l = ILLEGAL_LEN;
        v.s = nullptr;
      }
    }
  }

  inline void after_scanh(gridmap* map, int cardinal_step) {
    if (h.l > 0) {
      map->set_label(t_mapid, t_labelh);
      // there is a jump point or dead-end before pruned by constraint
      // simply set the constraint be null
      // TODO: use the jump point/dead-end to create new constraint
      if (cardinal_step < h.l) {
        h.l = ILLEGAL_LEN;
        h.s = nullptr;
      }
    }
  }
};
}

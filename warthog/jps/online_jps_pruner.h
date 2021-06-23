#pragma once
#include <vector>

#include "mapper.h"
#include "constants.h"
#include "helpers.h"
#include "problem_instance.h"

namespace warthog {

using namespace std;
class online_jps_pruner {

public:
  struct Record {
    cost_t cost;
    uint32_t fromd_mask;
    uint32_t search_id;
  };

  problem_instance* pi;
  Mapper* mapper;

  vector<Record> rec;

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
};
}

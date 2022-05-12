#pragma once

#include <vector>
#include "constants.h"
#include "problem_instance.h"
#include "blocklist.h"
using namespace std;
// set global variable that can be accessed everywhere
namespace global{

namespace statis {
  extern vector<warthog::cost_t> dist;
  extern int subopt_expd;
  extern int subopt_touch;

  inline void update_subopt_expd(uint32_t id, warthog::cost_t gval) {
    if (!dist.empty() && gval > dist[id]) subopt_expd++;
  }

  inline void update_subopt_touch(uint32_t id, warthog::cost_t gval) {
    if (!dist.empty() && gval > dist[id]) subopt_touch++;
  }

  inline void clear() {
    dist.clear();
    subopt_expd = 0;
    subopt_touch= 0;
  }
};

namespace query {
  extern warthog::blocklist* nodepool;
  extern uint32_t startid, goalid;
  extern warthog::problem_instance* pi;

  inline warthog::cost_t gval(uint32_t id) {
    warthog::search_node* s = nodepool->get(id);
    if (s != nullptr && s->get_searchid() == pi->get_searchid()) return s->get_g();
    else return warthog::INF;
  }
};
}

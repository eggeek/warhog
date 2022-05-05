#pragma once

#include <vector>
#include "constants.h"
using namespace std;

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
}

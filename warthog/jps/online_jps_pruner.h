#pragma once

#include <vector>
#include "search_node.h"
#include "constants.h"

namespace warthog {

using namespace std;
class online_jps_pruner {
  public:
    typedef pair<uint32_t, warthog::cost_t> pic; // Pair of <Id, Cost>
    bool rmapflag;
    bool gvalue_prune;
    bool jlimit_prune;
    uint32_t scan_cnt;
    search_node* cur;
    vector<pic> vis;

    uint32_t jumplimit_;
    uint32_t jlimith, jlimitv;
    uint32_t jumpdist;
    enum EndType {
      forced,     // ended at a forced neighbour
      deadend,    // ended at a deadend
      pruned,     // ended by pruning strategy
      reached     // ended at target
    } etype;

    inline void init(uint32_t tot) {
      vis.resize(tot);
      fill(vis.begin(), vis.end(), pic{0, 0});
      scan_cnt = 0;
    }

    inline void set_forced() { this->etype = forced; }
    inline void set_pruned() { this->etype = pruned; }
    inline void set_deadend() { this->etype = deadend; }
    inline void set_reached() { this->etype = reached; }
    inline bool is_forced() { return this->etype == forced; }
    inline bool is_pruned() { return this->etype == pruned; }
    inline bool is_deadend() { return this->etype == deadend; }
    inline bool is_reached() { return this->etype == reached; }

    inline bool gValPruned(uint32_t jumpnode_id, uint32_t goal_id, warthog::cost_t c) {
      if (!gvalue_prune) return false;

      if (vis[jumpnode_id].first != goal_id || vis[jumpnode_id].second == 0) {
        vis[jumpnode_id] = {goal_id, c + cur->get_g()};
        return false;
      }
      else {
        if (c + cur->get_g() >= vis[jumpnode_id].second) {
          this->set_pruned();
          return true;
        }
        vis[jumpnode_id] = {goal_id, c + cur->get_g()};
        return false;
      }
    }

    inline bool jLmtPruned(uint32_t dist) {
      if (jlimit_prune && dist >= jumplimit_) {
        this->set_pruned();
        return true;
      }
      else return false;
    }

    inline void update_jlimtv() { assert(false); }

    inline void update_jlimth() { assert(false); }
};
}

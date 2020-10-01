#pragma once

#include <vector>
#include "search_node.h"
#include "constants.h"
#include "gridmap.h"

namespace warthog {

using namespace std;
class online_jps_pruner {
  public:
    typedef pair<uint32_t, warthog::cost_t> pic; // Pair of <Id, Cost>
    bool rmapflag;
    bool verbose;
    uint32_t scan_cnt;
    search_node* cur;
    gridmap* map;
    vector<pic> vis;

    warthog::cost_t curg;
    uint32_t jumplimit_;
    uint32_t jlimith, jlimitv;
    uint32_t jumpdist;
    uint32_t start_id;
    enum EndType {
      forced,     // ended at a forced neighbour
      deadend,    // ended at a deadend
      pruned,     // ended by pruning strategy
      reached     // ended at target
    } etype;

    inline void init(uint32_t tot) {
      vis.resize(tot);
      fill(vis.begin(), vis.end(), pic{warthog::INF, warthog::INF});
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

    inline bool gValPruned(const uint32_t& jumpnode_id, const warthog::cost_t& c) {

      if (vis[jumpnode_id].first != start_id || vis[jumpnode_id].second == warthog::INF) {
        vis[jumpnode_id] = {start_id, c};
        return false;
      }
      else {
        if (c > vis[jumpnode_id].second) {
          this->set_pruned();
          return true;
        }
        else {
          vis[jumpnode_id] = {start_id, c};
          return false;
        }
      }
    }

    inline uint32_t gVal(const uint32_t& jumpnode_id) {
      if (vis[jumpnode_id].first != start_id) return warthog::INF;
      else return vis[jumpnode_id].second;
    }

    inline void update_gval(const uint32_t& jumpnode_id, warthog::cost_t c) {
      #ifndef NDEBUG
      if (verbose) {
        uint32_t x, y;
        x = jumpnode_id % map->width();
        y = jumpnode_id / map->width();
        std::cerr << "update gvalue on (" << x << ", " << y << ")\t";
        warthog::cost_t f = this->curg + this->jumpdist * warthog::ONE;
        warthog::cost_t g = gVal(jumpnode_id);
        std::cerr << "[jump point, cost: " << f 
                  << "], [exist gVal: " << g << "]";
        if (f > g) std::cerr << " pruned";
        std::cerr << std::endl;
      }
      #endif
      if (vis[jumpnode_id].first != start_id || vis[jumpnode_id].second >= c) {
        vis[jumpnode_id] = {start_id, c};
      }
      else {
        this->set_pruned();
      }
    }

    inline void update_jlimtv(const uint32_t& jumpnode_id) { 
      #ifndef NDEBUG
      if (verbose) {
        uint32_t x, y;
        x = jumpnode_id % map->width();
        y = jumpnode_id / map->width();
        switch (this->etype) {
          case pruned:
            std::cerr << "update limit on (" << x << ", " << y << ")\t";
            std::cerr << "last is pruned, decrease to " 
              << (this->jumpdist? this->jumpdist-1: 0) << std::endl;
            break;
          case reached:
            std::cerr << "update limit on (" << x << ", " << y << ")\t";
            std::cerr << "last is reached, set to 0" << std::endl;
            break;
          case deadend:
            std::cerr << "last is deadend, set to INF" 
                      << ", jumpdist: " << this->jumpdist << std::endl;
            break;
          case forced:
            std::cerr << "last is forced, set to INF" 
                      << ", jumpdist:" << this->jumpdist << std::endl;
            break;
        }
      }
      #endif
      switch (this->etype) {
        case pruned:
          // this->jlimitv = this->jumpdist? this->jumpdist-1: 0; break;
          this->jlimitv = this->jumpdist; break;
        case reached:
          this->jlimitv = 0; break;
        case deadend:
          this->jlimitv = warthog::INF; break;
        case forced:
          this->jlimitv = warthog::INF; break;
      }
    }


    inline void update_jlimth(const uint32_t& jumpnode_id) {
      #ifndef NDEBUG
      if (verbose) {
        uint32_t x, y;
        x = jumpnode_id % map->width();
        y = jumpnode_id / map->width();
        switch (this->etype) {
          case pruned:
            std::cerr << "update limit on (" << x << ", " << y << ")\t";
            std::cerr << "last is pruned, decrease to " 
              << (this->jumpdist? this->jumpdist-1: 0) << std::endl;
            break;
          case reached:
            std::cerr << "update limit on (" << x << ", " << y << ")\t";
            std::cerr << "last is reached, set to 0" << std::endl;
            break;
          case deadend:
            std::cerr << "last is deadend, set to INF" 
                      << ", jumpdist: " << this->jumpdist << std::endl;
            break;
          case forced:
            std::cerr << "last is forced, set to INF" 
                      << ", jumpdist:" << this->jumpdist << std::endl;
            break;
        }
      }
      #endif
      switch (this->etype) {
        case pruned:
          // this->jlimith = this->jumpdist? this->jumpdist-1: 0; break;
          this->jlimith = this->jumpdist; break;
        case reached:
          this->jlimith = 0; break;
        case deadend:
          this->jlimith = warthog::INF; break;
        case forced:
          this->jlimith = warthog::INF; break;
      }
    }

    inline void startJump() {
      this->curg = cur->get_g();
    }

    inline void startExpand(const uint32_t& start_id, search_node* cur) {
      this->jumpdist = this->jlimith = this->jlimitv = warthog::INF;
      this->set_forced();
      this->start_id = start_id;
      this->rmapflag = false;
      this->cur = cur;
    }
};
}

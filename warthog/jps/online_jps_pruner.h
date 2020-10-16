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
    uint32_t jumpdist;
    uint32_t start_id;

    struct Constraint {
      /*
       *     b------*
       *    #|  |  /
       *    #| jp /
       *    #|  |/
       *    #d  i
       *    #| /
       *    #|/
       *     c
       *    /
       *   a
       *
       *    when expanding "a", we can create a constraint based on the scanned jump point "b"
       *    to avoid redundant scanning when move diagonal
       *
       *    Create constraint:
       *      when gc + d > gb
       *
       *    Apply constraint:
       *      When stepCnt + jumpdist == straightLen
       *
       *    Pruned:
       *      When apply constraint and gi + jp > gb + stepCnt
       *
       *    Update constraint:
       *      when pruned                       --> keep using this constraint in next iteration
       *      when get deadend before apply     --> straightLen = INF
       *      when get goal before apply        --> straightLen = 0, no further scanning
       *      when get jump point before apply  --> set new constraint when jump is better
       *                                        --> else straightLen = INF
       */
      uint32_t node_id,       // node id of "b"
               straightLen,   // num of straight steps from "a" to "b"
               stepCnt;       // num of diagonal moves made from creating "b"
      warthog::cost_t gVal;
    };
    Constraint constraints[4];
    Constraint constraintH, constraintV;

    inline void createConstraint(uint32_t node_id, warthog::cost_t cost) {
      if (vis[node_id].first != start_id || vis[node_id].second >= cost) {
        // current path is better
        vis[node_id] = {start_id, cost};
        set_forced();
      }
      else {
        // there is a better path, then update constraint
        Constraint& c = rmapflag? constraintV: constraintH;
        c.node_id = node_id;
        c.straightLen = jumpdist;
        c.stepCnt = 0;
        set_pruned();
      }
    }

    inline void updateConstraint() {
      Constraint& c = rmapflag? constraintV: constraintH;
      c.stepCnt++;
      switch (this->etype) {
        case pruned: break;
        case reached:
          c.straightLen = 0; break;
        case deadend:
          c.straightLen = warthog::INF; break;
        case forced:
          c.straightLen = warthog::INF; break;
      }
    }

    inline void set_north_constraint() { constraintV = constraints[0]; }
    inline void set_south_constraint() { constraintV = constraints[1]; }
    inline void set_east_constraint() { constraintH = constraints[2]; }
    inline void set_west_constraint() { constraintH = constraints[3]; }

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

    inline bool constraintPruned() {
      const Constraint& c = this->rmapflag? this->constraintV: this->constraintH;
      if (c.stepCnt + this->jumpdist >= c.straightLen &&
          gVal(c.node_id) + c.stepCnt * warthog::ONE < this->curg + this->jumpdist * warthog::ONE) {
        this->set_pruned();
        return true;
      }
      else
        return false;
    }

    inline uint32_t gVal(const uint32_t& jumpnode_id) {
      if (vis[jumpnode_id].first != start_id) return warthog::INF;
      else return vis[jumpnode_id].second;
    }

    inline void startJump() {
      this->curg = cur->get_g();
    }

    inline void startExpand(const uint32_t& start_id, search_node* cur) {
      this->set_forced();
      this->start_id = start_id;
      this->rmapflag = false;
      this->cur = cur;
      for (int i=0; i<4; i++) {
        this->constraints[i] = {warthog::INF, warthog::INF, 0};
      }
    }
};
}

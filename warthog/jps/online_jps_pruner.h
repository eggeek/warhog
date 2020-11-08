#pragma once

#include <vector>
#include "search_node.h"
#include "constants.h"
#include "gridmap.h"
#include "problem_instance.h"

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
    uint32_t search_id;

    enum EndType {
      forced,     // ended at a forced neighbour
      deadend,    // ended at a deadend
      pruned,     // ended by pruning strategy
      reached     // ended at target
    } etype;
    EndType etypeV, etypeH;

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

      inline void incStep(const EndType& etype) {
        stepCnt++;
        if (stepCnt >= straightLen) {
          straightLen = warthog::INF;
          gVal = warthog::INF;
          stepCnt = 0;
        }
        switch (etype) {
          case pruned: break;
          case reached:
            straightLen = 0; break;
          case deadend:
            straightLen = warthog::INF; break;
          case forced:
            straightLen = warthog::INF; break;
        }
      }
    };
    Constraint constraints[4];
    Constraint *constraintH, *constraintV, *curConstraint;

    inline void setCurConstraint() {
      if (rmapflag) curConstraint = constraintV;
      else curConstraint = constraintH;
    }

    inline void updateConstraint(uint32_t node_id, uint32_t dist) {
      warthog::cost_t d = dist * warthog::ONE;
      if (vis[node_id].first != search_id || vis[node_id].second >= d + curg) {
        // current path is better
        vis[node_id] = {search_id, d + curg};
        set_forced();
      }
      else if (vis[node_id].second + d < curg) {
        // there is a better path to current position
        set_deadend();
      }
      else {
        // there is a better path to scanned node, then update constraint
        curConstraint->node_id = node_id;
        curConstraint->straightLen = dist;
        curConstraint->stepCnt = 0;
        curConstraint->gVal = vis[node_id].second;
        set_pruned();
      }
    }

    inline void incStepV() {
      etypeV = this->etype;
      constraintV->incStep(etype);
    }

    inline void incStepH() {
      etypeH = this->etype;
      constraintH->incStep(etype);
    }

    inline uint32_t corner_pos(uint32_t (&neis)[3]) {
      uint32_t 
      corner_bits = (~neis[0] >> 1) & neis[0];
      corner_bits |= (~neis[2] >> 1) & neis[2];
      // detect any corner or dead-end tiles
      corner_bits |= ~neis[1];
      if (corner_bits) return __builtin_ffs(corner_bits) - 1;
      else return warthog::INF;
    }

    inline uint32_t corner_pos_upper(uint32_t (&neis)[3]) {
      uint32_t 
      corner_bits = (~neis[0] << 1) & neis[0];
      corner_bits |= (~neis[2] << 1) & neis[2];
      // detect any corner or dead-end tiles
      corner_bits |= ~neis[1];
      if (corner_bits) return __builtin_clz(corner_bits);
      else return warthog::INF;
    }

    inline void set_north_constraint() { constraintV = &(constraints[0]); }
    inline void set_south_constraint() { constraintV = &constraints[1]; }
    inline void set_east_constraint() { constraintH = &constraints[2]; }
    inline void set_west_constraint() { constraintH = &constraints[3]; }

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
    inline bool is_forcedH() { return this->etypeH == forced; }
    inline bool is_forcedV() { return this->etypeV == forced; }

    inline bool is_pruned() { return this->etype == pruned; }
    inline bool is_prunedH() { return this->etypeH == pruned; }
    inline bool is_prunedV() { return this->etypeV == pruned; }

    inline bool is_deadend() { return this->etype == deadend; }
    inline bool is_deadendH() { return this->etypeH == deadend; }
    inline bool is_deadendV() { return this->etypeV == deadend; }

    inline bool is_reached() { return this->etype == reached; }
    inline bool is_reachedH() { return this->etypeH == reached; }
    inline bool is_reachedV() { return this->etypeV == reached; }

    inline bool gValPruned(const uint32_t& jumpnode_id, const warthog::cost_t& c) {

      if (vis[jumpnode_id].first != search_id || vis[jumpnode_id].second == warthog::INF) {
        vis[jumpnode_id] = {search_id, c};
        return false;
      }
      else {
        if (c > vis[jumpnode_id].second) {
          this->set_pruned();
          return true;
        }
        else {
          vis[jumpnode_id] = {search_id, c};
          return false;
        }
      }
    }

    inline bool constraintPruned() {
      const Constraint& c = *curConstraint;
      if (c.stepCnt + this->jumpdist >= c.straightLen &&
          c.gVal + c.stepCnt * warthog::ONE < this->curg + (c.straightLen - c.stepCnt) * warthog::ONE) {
        this->set_pruned();
        return true;
      }
      else
        return false;
    }

    inline uint32_t gVal(const uint32_t& jumpnode_id) {
      if (vis[jumpnode_id].first != search_id) return warthog::INF;
      else return vis[jumpnode_id].second;
    }

    inline void startJump() {
      this->curg = cur->get_g();
    }

    inline void startExpand(problem_instance* instance, search_node* cur) {
      this->set_forced();
      this->search_id = instance->get_searchid();
      this->rmapflag = false;
      this->cur = cur;
      for (int i=0; i<4; i++) {
        this->constraints[i] = {warthog::INF, warthog::INF, 0};
      }
    }
};
}

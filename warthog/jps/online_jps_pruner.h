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
    vector<uint32_t> jlimit_arr;
    const uint32_t jlimit_mask = 1 << 5;

    warthog::cost_t curg;
    uint32_t curid;
    uint32_t jumpdist;
    uint32_t search_id;

    enum EndType {
      forced,     // ended at a forced neighbour
      deadend,    // ended at a deadend
      pruned,     // ended by pruning strategy
      terminated, // no longer jump in current direction
      reached     // ended at target
    } etype;
    EndType etypeV, etypeH;

    struct Constraint {
      /*
       *     b------*
       *    #|  |    /
       *    #d2 |   /
       *    #|  |  /
       *    #|  | /
       *    #m.........
       *    #|  /
       *    #d1/
       *    #|/
       *     c
       *    /
       *   a
       *    
       *    Fields:
       *      * d: |cb|
       *      * dm: |cm|
       *      * stepCnt: number of diagonal moves
       *
       *    when expanding "a", we can create a constraint based on the scanned jump point "b"
       *    to avoid redundant scanning when move diagonal
       *
       *    Create constraint:
       *      when gc + d > gb
       *
       *    Apply constraint:
       *      When:
       *        1. stepCnt + jumpdist >= d1;
       *           prune if gc + stepCnt * sqrt(2) + (d1 - stepCnt) > gb + d2 + stepCnt;
       *           stop scanning if gc + stepCnt * sqrt(2) > gb + d2 + stepCnt + (d1 - stepCnt)
       *
       *        2. stepCnt + jumpdist >= d1 + d2;
       *           prune if gc + stepCnt * sqrt(2) + (d1 + d2 - stepCnt) > gb + stepCnt;
       *           stop scanning if gc + stepCnt * sqrt(2) > gb + stepCnt + (d1 + d2 - stepCnt);
       *
       *      When apply constraint and gi + jp > gb + stepCnt
       *
       *    Update constraint:
       *      when pruned                       --> keep using this constraint in next iteration
       *      when get deadend before apply     --> d = INF
       *      when get goal before apply        --> d = 0, no further scanning
       *      when get jump point before apply  --> set new constraint when jump is better
       *                                        --> else d = INF
       */
      uint32_t node_id,       // node id of "b"
               d,             // distance from "c" to "b"
               dm,            // ceiled distance from "c" to "m"
               stepCnt,       // num of diagonal moves made from creating "b"
               jlimit;        // num of steps can jump
      warthog::cost_t gVal;

      inline void incStep(const EndType& etype) {
        stepCnt++;
        if (stepCnt >= d) {
          gVal = warthog::INF;
        }
        switch (etype) {
          case pruned: break;
          case reached:
            gVal = d = dm = 0; break;
          case deadend:
          case forced:
          case terminated:
            gVal = d = dm = warthog::INF; break;
        }
      }

      inline uint32_t calc_jlimit(const uint32_t& ga) {
        if (gVal == warthog::INF) { jlimit = warthog::INF; }
        else if (gVal + d * warthog::ONE < ga) { jlimit = 0; }
        else {
          jlimit = (gVal + d * warthog::ONE - ga - warthog::ROOT_TWO * stepCnt) / 2;
          if (jlimit > d) jlimit = warthog::INF;
        }
        return jlimit;
      }
    };
    Constraint constraints[4];
    Constraint constraintH, constraintV, *curConstraint;

    inline void setCurConstraint() {
      if (rmapflag) curConstraint = &constraintV;
      else curConstraint = &constraintH;
    }

    inline void updateConstraint(int direct, uint32_t dist) {
      uint32_t node_id = curid;
      if (rmapflag) node_id -= direct * (int)(dist * map->width());
      else node_id += direct * (int)dist;

      warthog::cost_t d = dist * warthog::ONE;
      if (vis[node_id].first != search_id || vis[node_id].second >= d + curg) {
        // current path is better
        vis[node_id] = {search_id, d + curg};
      }
      else
      {
        // there is a better path to scanned node, then update constraint
        curConstraint->node_id = node_id;
        curConstraint->gVal = vis[node_id].second;
        curConstraint->d = dist;
        curConstraint->dm = (curConstraint->gVal - curg + dist + 1) >> 1; // ceiling
        curConstraint->stepCnt = 0;
      }
    }

    inline void incStepV() {
      etypeV = this->etype;
      constraintV.incStep(etype);
    }

    inline void incStepH() {
      etypeH = this->etype;
      constraintH.incStep(etype);
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

    inline void set_jlimt_array(const uint32_t& limit) {
      if (limit != warthog::INF)
        jlimit_arr[limit >> 5] = 1 << (limit & this->jlimit_mask);
    }

    inline void unset_jlimit_array(const uint32_t& limit) {
      if (limit != warthog::INF)
        jlimit_arr[limit >> 5] = 0;
    }

    inline void set_jlimt_array_upper(const uint32_t& limit) {
      if (limit != warthog::INF)
        jlimit_arr[limit >> 5] = 1 << (31 - (limit & this->jlimit_mask));
    }

    inline void set_north_constraint() { constraintV = constraints[0]; }
    inline void set_south_constraint() { constraintV = constraints[1]; }
    inline void set_east_constraint() { constraintH = constraints[2]; }
    inline void set_west_constraint() { constraintH = constraints[3]; }

    inline void save_north_constraint() { constraints[0] = *curConstraint; }
    inline void save_south_constraint() { constraints[1] = *curConstraint; }
    inline void save_east_constraint() { constraints[2] = *curConstraint; }
    inline void save_west_constraint() { constraints[3] = *curConstraint; }
    

    inline void init(gridmap* map) {
      this->map = map;
      vis.resize(map->width() * map->height());
      fill(vis.begin(), vis.end(), pic{warthog::INF, warthog::INF});
      scan_cnt = 0;
      jlimit_arr.resize((max(map->width(), map->height())>>5) + 1);
      fill(jlimit_arr.begin(), jlimit_arr.end(), 0);
    }

    inline void set_forced() { this->etype = forced; }
    inline void set_pruned() { this->etype = pruned; }
    inline void set_deadend() { this->etype = deadend; }
    inline void set_reached() { this->etype = reached; }
    inline void set_terminated() {this->etype = terminated; }

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

    inline bool is_terminated() { return this->etype == terminated; }
    inline bool is_terminatedH() { return this->etypeH == terminated; }
    inline bool is_terminatedV() { return this->etypeV == terminated; }

    inline bool gValPruned(int direct, const uint32_t& dist) {
      uint32_t jumpnode_id = curid;
      if (rmapflag) jumpnode_id -= direct * (int)(dist * map->width());
      else jumpnode_id += direct * (int)dist;

      warthog::cost_t c = curg + dist * warthog::ONE;
      if (vis[jumpnode_id].first == search_id && c > vis[jumpnode_id].second) {
        this->set_deadend();
        return true;
      }
      else {
        vis[jumpnode_id] = {search_id, c};
        return false;
      }
    }

    inline bool constraintPruned() {
      const Constraint& c = *curConstraint;
      if (c.gVal == warthog::INF) return false; // the constraint is no longer valid

      uint32_t d2 = c.d - c.dm;
      if (curg > c.gVal + c.d * warthog::ONE) {
        set_terminated();
        return true;
      }
      if (c.stepCnt + jumpdist >= c.d) {
        if (curg + (c.d - c.stepCnt) * warthog::ONE > c.gVal + c.stepCnt * warthog::ONE) {
          set_pruned();
          return true;
        }
        else return false;
      }
      else if (c.stepCnt + jumpdist >= c.dm) {
        if (curg + (c.dm - c.stepCnt) * warthog::ONE > c.gVal + (d2 + c.stepCnt) * warthog::ONE) {
          set_pruned();
          return true;
        }
        else return false;
      }
      else return false;
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
      this->cur = cur;
      for (int i=0; i<4; i++) {
        this->constraints[i].gVal = warthog::INF;
      }
    }
};
}

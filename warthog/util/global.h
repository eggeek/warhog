#pragma once

#include <vector>
#include "constants.h"
#include "problem_instance.h"
#include "blocklist.h"
#include "gridmap.h"
#include "pqueue.h"
#include "search_node.h"
#include "pqueue.h"
using namespace std;
// set global variable that can be accessed everywhere
namespace global{

extern string alg;
// stores the gval on corner points
struct gvEntry {
  warthog::cost_t g;
  uint32_t searchid;
};
extern vector<gvEntry> corner_gv;

namespace statis {

  struct Log {
    uint32_t x, y, id, pid, padded_id, padded_pid, 
             sx, sy, tx, ty, sid, gid;
    string curalg, mapname;
    warthog::cost_t gval;
    int subopt;

    string to_str() {
      string res = "";
      res = mapname + "," + curalg + "," +
        to_string(padded_id) + "," +
        to_string(id) + "," + 
        to_string(x) + "," +
        to_string(y) + "," +
        to_string(sid) + "," + 
        to_string(sx) + "," +
        to_string(sy) + "," +
        to_string(gid) + "," + 
        to_string(tx) + "," +
        to_string(ty) + "," +
        to_string((double)(gval / warthog::ONE)) + "," +
        to_string(subopt);
      return res;
    }
  };

  extern vector<warthog::cost_t> dist;
  extern uint32_t subopt_expd;
  extern uint32_t subopt_touch;
  extern uint32_t scan_cnt;
  extern vector<Log> logs;

  Log gen(uint32_t id, warthog::cost_t gval, bool subopt);
  extern uint32_t prunable;
  extern vector<Log> logs;

  Log gen(uint32_t id, warthog::cost_t gval, bool subopt);

  inline void update_subopt_expd(uint32_t id, warthog::cost_t gval) {
    assert(dist.empty() || id < dist.size());
    if (!dist.empty() && gval > dist[id]) {
      subopt_expd++;
      // logs.push_back(gen(id, gval, 1));
    }
    // else {
    //   logs.push_back(gen(id, gval, 0));
    // }
  }

  inline void update_pruneable(warthog::search_node* cur) {
    warthog::search_node* pa = cur->get_parent();
    // parent is subopt
    if (!dist.empty() && pa != nullptr && pa->get_g() > dist[pa->get_id()]) prunable++;
  }

  inline void update_subopt_touch(uint32_t id, warthog::cost_t gval) {
    assert(dist.empty() || id < dist.size());
    if (!dist.empty() && gval > dist[id]) 
      subopt_touch++;
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

  inline void write_log(string fname) {
    std::ofstream fout(fname);
    string header = "map,alg,padded_id,id,x,y,sid,sx,sy,gid,tx,ty,gval,subopt";
    fout << header << endl;
    for (auto& log: logs) {
      fout << log.to_str() << endl;
    }
    fout.close();
  }
};

namespace query {
  extern warthog::blocklist* nodepool;
  extern uint32_t startid, goalid;
  extern warthog::cost_t cur_diag_gval;
  extern warthog::problem_instance* pi;
  extern warthog::gridmap *map;
  extern warthog::pqueue* open;

  inline warthog::cost_t gval(uint32_t id) {
    warthog::cost_t res = warthog::INF;
    assert(id < corner_gv.size());
    if (pi->get_searchid() == corner_gv[id].searchid)
      res = corner_gv[id].g;
    warthog::search_node* s = nodepool->get(id);
    if (s != nullptr && s->get_searchid() == pi->get_searchid()) 
      res = min(res, s->get_g());
    return res;
  }

  // set gvalue on corner point
  inline void set_corner_gv(uint32_t id, warthog::cost_t g) {
    assert(id < corner_gv.size());
    if (corner_gv[id].searchid != pi->get_searchid()) {
      corner_gv[id] = {g, pi->get_searchid()};
    }
    else if (corner_gv[id].g > g) {
      corner_gv[id].g = g;
    }
    warthog::search_node* n = nodepool->get(id);
    if (n != nullptr && open->contains(n) && g < n->get_g()) {
      n->relax(g, nullptr);
      open->decrease_key(n);
    }
  }

  inline void clear() {
    map = nullptr;
  }
};

  inline void clear() {
    statis::clear();
    query::clear();
  }
}

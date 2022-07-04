#include "cfg.h"
#include "flexible_astar.h"
#include "octile_heuristic.h"
#include "jps2_expansion_policy.h"
#include "jps2_expansion_policy_prune.h"
#include "jps2_expansion_policy_prune2.h"
#include "dijkstra.h"
#include "scenario_manager.h"
#include "global.h"
using namespace std;
namespace w = warthog;
namespace G = global;

struct ExpData {
  long long exp, gen, touch, scan, pruneable;
  // count subopt gval:
  long long subopt_expd, subopt_touch;
  double time;
  void reset() {
    exp = gen = touch = scan = pruneable = 0;
    subopt_expd = subopt_touch = 0;
  }
  template <typename H, typename E>
  void update(w::flexible_astar<H, E>* alg, long long cnts) {
    exp += alg->get_nodes_expanded();
    gen += alg->get_nodes_generated();
    touch += alg->get_nodes_touched();
    time += alg->get_search_time();
    scan += cnts;
  }

  void update_subopt() {
    subopt_expd += G::statis::subopt_expd;
    subopt_touch += G::statis::subopt_touch;
    pruneable += G::statis::prunable;
    scan += G::statis::scan_cnt;
  }

  string str() {
    string res = "";
    res += to_string(exp) + "\t" + 
           to_string(gen) + "\t" + 
           to_string(touch) + "\t" + 
           to_string((long long)time) + "\t" + to_string(scan);
    return res;
  }

  string subopt_str() {
    string res = "";
    res += to_string(subopt_touch) + "\t" +
           to_string(touch) + "\t" +
           to_string(subopt_expd) + "\t" +
           to_string(pruneable) + "\t" + 
           to_string(exp) + "\t" +
           to_string(scan);
    return res;
  }
};

void run(string mpath, string spath) {
  // string header = "map\tsubopt_expd\tpruneable\ttot\tscnt\talg";
  // cout << header << endl;
  warthog::gridmap* map = new warthog::gridmap(mpath.c_str());
  w::octile_heuristic heur(map->width(), map->height());

  /* JPS2 variants */
  w::jps2_expansion_policy ep2(map);
  w::jps2_expansion_policy_prune cep2(map);
  w::jps2_expansion_policy_prune2 c2ep2(map);
  w::flexible_astar<w::octile_heuristic, w::jps2_expansion_policy> jps2(&heur, &ep2);
  w::flexible_astar<w::octile_heuristic, w::jps2_expansion_policy_prune> cjps2(&heur, &cep2);
  w::flexible_astar<w::octile_heuristic, w::jps2_expansion_policy_prune2> c2jps2(&heur, &c2ep2);

  w::Dijkstra dij(mpath);
  w::scenario_manager smgr;
  smgr.load_scenario(spath.c_str());

  ExpData cnt_jps, cnt_cjps, cnt_c2jps,
          cnt_jps2, cnt_cjps2, cnt_c2jps2;

  ExpData* cnts[] = {&cnt_jps, &cnt_cjps, &cnt_c2jps, &cnt_jps2, &cnt_cjps2, &cnt_c2jps2};
  for (auto &i: cnts) i->reset();
  global::query::map = map;

  int fromidx = 0;
  int toindx = (int)smgr.num_experiments();
  for (int i=fromidx; i<toindx; i++) {
    w::experiment* exp = smgr.get_experiment(i);
    int sx = exp->startx(), sy = exp->starty();
    int tx = exp->goalx(), ty = exp->goaly();
    uint32_t sid = map->to_padded_id(sx, sy);
    uint32_t tid = map->to_padded_id(tx, ty);
    dij.run(sid);

    /* JPS2 variants */
    G::statis::clear();
    G::statis::dist = vector<warthog::cost_t>(dij.dist);
    G::query::nodepool = ep2.get_nodepool();
    // jps2.set_verbose(true);
    jps2.get_length(sid, tid);
    cnt_jps2.update_subopt();
    cnt_jps2.update(&jps2, 0);

    G::statis::clear();
    G::statis::dist = vector<warthog::cost_t>(dij.dist);
    G::query::nodepool = c2ep2.get_nodepool();
    c2jps2.get_length(sid, tid);
    cnt_c2jps2.update_subopt();
    cnt_c2jps2.update(&c2jps2, 0);
  }
  cout << mpath << "\t" << cnt_jps2.subopt_str() << "\tjps2" << endl;
  cout << mpath << "\t" << cnt_c2jps2.subopt_str() << "\tc2jps2" << endl;
  G::query::clear();
}

void run_perquery(string mpath, string spath) {
  // string header = "map\tsubopt_expd\tpruneable\ttot\tscnt\talg";
  // cout << header << endl;
  warthog::gridmap* map = new warthog::gridmap(mpath.c_str());
  w::octile_heuristic heur(map->width(), map->height());

  /* JPS2 variants */
  w::jps2_expansion_policy ep2(map);
  w::jps2_expansion_policy_prune cep2(map);
  w::jps2_expansion_policy_prune2 c2ep2(map);
  w::flexible_astar<w::octile_heuristic, w::jps2_expansion_policy> jps2(&heur, &ep2);
  w::flexible_astar<w::octile_heuristic, w::jps2_expansion_policy_prune> cjps2(&heur, &cep2);
  w::flexible_astar<w::octile_heuristic, w::jps2_expansion_policy_prune2> c2jps2(&heur, &c2ep2);

  w::Dijkstra dij(mpath);
  w::scenario_manager smgr;
  smgr.load_scenario(spath.c_str());

  ExpData cnt_jps, cnt_cjps, cnt_c2jps,
          cnt_jps2, cnt_cjps2, cnt_c2jps2;

  ExpData* cnts[] = {&cnt_jps, &cnt_cjps, &cnt_c2jps, &cnt_jps2, &cnt_cjps2, &cnt_c2jps2};
  string header = "map\tid\tsubopt_expd\ttot_touch\tsubopt_expd\tpruneable\ttot_expd\tscnt\talg";
  cout << header << endl;
  for (auto &i: cnts) i->reset();
  global::query::map = map;

  int fromidx = 0;
  int toindx = (int)smgr.num_experiments();
  for (int i=fromidx; i<toindx; i++) {
    w::experiment* exp = smgr.get_experiment(i);
    int sx = exp->startx(), sy = exp->starty();
    int tx = exp->goalx(), ty = exp->goaly();
    uint32_t sid = map->to_padded_id(sx, sy);
    uint32_t tid = map->to_padded_id(tx, ty);
    dij.run(sid);

    /* JPS2 variants */
    G::statis::clear();
    G::statis::dist = vector<warthog::cost_t>(dij.dist);
    G::query::nodepool = ep2.get_nodepool();
    // jps2.set_verbose(true);
    jps2.get_length(sid, tid);
    cnt_jps2.update_subopt();
    cnt_jps2.update(&jps2, 0);

    G::statis::clear();
    G::statis::dist = vector<warthog::cost_t>(dij.dist);
    G::query::nodepool = c2ep2.get_nodepool();
    c2jps2.get_length(sid, tid);
    cnt_c2jps2.update_subopt();
    cnt_c2jps2.update(&c2jps2, 0);
    cout << mpath << "\t" << i << "\t" << cnt_jps2.subopt_str() << "\tjps2" << endl;
    cout << mpath << "\t" << i << "\t" << cnt_c2jps2.subopt_str() << "\tc2jps2" << endl;

    for (auto &i: cnts) i->reset();
  }
  G::query::clear();
}

int main(int argc, char** argv) {
  // parse arguments
  int query = false;
  warthog::util::param valid_args[] = {
		{"scen",  required_argument, 0, 0},
    {"map", required_argument},
    {"query", no_argument, &query, 1}
  };
  warthog::util::cfg cfg;
  cfg.parse_args(argc, argv, valid_args);
  string sfile = cfg.get_param_value("scen");
  string mfile = cfg.get_param_value("map");
  if (!query)
    run(mfile, sfile);
  else
    run_perquery(mfile, sfile);
}

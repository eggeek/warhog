#include <vector>
#include <fstream>
#include <map>
#include <string>
#include <catch2/catch.hpp>

#include "experiment.h"
#include "gridmap.h"
#include "global.h"
#include "jps_expansion_policy_simple.h"
#include "jps_expansion_policy_prune.h"
#include "jps_expansion_policy_prune2.h"
#include "jps_expansion_policy.h"
#include "jps2_expansion_policy.h"
#include "jps2_expansion_policy_prune.h"
#include "jps2_expansion_policy_prune2.h"

#include "online_jump_point_locator_prune.h"
#include "online_jump_point_locator2_prune.h"
#include "online_jump_point_locator.h"
#include "flexible_astar.h"
#include "octile_heuristic.h"
#include "scenario_manager.h"
#include "jps_heuristic.h"
#include "neo_astar.h"
#include "dijkstra.h"
using namespace std;
namespace G = global;
namespace S = global::statis;
namespace TEST_JLIMIT {

string mapfile, scenfile;

struct node {
  int x, y;
};

const double eps = 1e-2;
namespace w=warthog;

struct ExpData {
  long long exp, gen, touch, scan;
  // count subopt gval:
  long long subopt_expd, subopt_touch;
  double time;
  void reset() {
    exp = gen = touch = scan = 0;
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
           to_string(subopt_expd) + "\t" +
           to_string(scan);
    return res;
  }
};

inline void print_query(int idx, string mapname, int mapw, int maph, int sx, int sy, int tx, int ty, 
double expect_len, double actual_len) {
  cerr << idx << "\t" << mapname << "\t" << mapw << "\t" << maph 
       << "\t" << sx << "\t" << sy 
       << "\t" << tx << "\t" << ty << "\t" << expect_len << endl;
  cerr << "expected: " << expect_len << ", actual: " << actual_len << endl;
}

void sanity_checking(int idx, w::gridmap* mptr, node s, node t, double expect, double actual) {
  if (fabs(expect - actual) > eps) {
    print_query(idx, mptr->filename(), mptr->header_width(), mptr->header_height(),
        s.x, s.y, t.x, t.y, expect, actual);
  }
  REQUIRE(fabs(expect - actual) < eps);
}

inline void run(w::gridmap& map, vector<node>& s, vector<node>& t, bool verbose=false) {

  /* Heuristic variants */
  w::octile_heuristic heuristic1(map.width(), map.height());
  // w::jps_heuristic jpsh(jps_prune.get_mapper());

  /* JPS variants */
  w::jps_expansion_policy_simple ep_jpss(&map);
  w::jps_expansion_policy ep_jps(&map);
  w::jps_expansion_policy_prune ep_cjps(&map);
  w::jps_expansion_policy_prune2 ep_c2jps(&map);

  w::flexible_astar<w::octile_heuristic,w::jps_expansion_policy> jps(&heuristic1, &ep_jps);
  w::flexible_astar<w::octile_heuristic, w::jps_expansion_policy_prune> cjps(&heuristic1, &ep_cjps);
  w::flexible_astar<w::octile_heuristic, w::jps_expansion_policy_prune2> c2jps(&heuristic1, &ep_c2jps);
  w::flexible_astar<w::octile_heuristic, w::jps_expansion_policy_simple> jpss(&heuristic1, &ep_jpss);
  jps.set_verbose(verbose);
  cjps.set_verbose(verbose);
  c2jps.set_verbose(verbose);
  jpss.set_verbose(verbose);

  /* JPS2 variants */
  w::jps2_expansion_policy ep_jps2(&map);
  w::jps2_expansion_policy_prune ep_cjps2(&map);
  w::jps2_expansion_policy_prune2 ep_c2jps2(&map);

  w::flexible_astar<w::octile_heuristic, w::jps2_expansion_policy> jps2(&heuristic1, &ep_jps2);
  w::flexible_astar<w::octile_heuristic, w::jps2_expansion_policy_prune> cjps2(&heuristic1, &ep_cjps2);
  w::flexible_astar<w::octile_heuristic, w::jps2_expansion_policy_prune2> c2jps2(&heuristic1, &ep_c2jps2);

  jps2.set_verbose(verbose);
  cjps2.set_verbose(verbose);
  c2jps2.set_verbose(verbose);

  ExpData cnt_jps, cnt_cjps, cnt_c2jps, cnt_jps2, cnt_cjps2, cnt_c2jps2;
  ExpData* cnts[] = {&cnt_jps, &cnt_cjps, &cnt_c2jps, &cnt_jps2, &cnt_cjps2, &cnt_c2jps2};
  for (auto &i: cnts) i->reset();

  for (int i=0; i<(int)s.size(); i++) {
    int sid = s[i].y * map.header_width() + s[i].x;
    int tid = t[i].y * map.header_width() + t[i].x;

    /* Run JPS variants */
    S::clear();
    G::query::nodepool = ep_jps.get_nodepool();
    double len_jps = jps.get_length(map.to_padded_id(sid), map.to_padded_id(tid));
    cnt_jps.update(&jps, S::scan_cnt);

    S::clear();
    G::query::nodepool = ep_cjps.get_nodepool();
    double len_cjps = cjps.get_length(map.to_padded_id(sid), map.to_padded_id(tid));
    cnt_cjps.update(&cjps, S::scan_cnt);
    sanity_checking(i, &map, s[i], t[i], len_jps, len_cjps);

    S::clear();
    G::query::nodepool = ep_c2jps.get_nodepool();
    double len_c2jps = c2jps.get_length(map.to_padded_id(sid), map.to_padded_id(tid));
    cnt_c2jps.update(&c2jps, S::scan_cnt);
    sanity_checking(i, &map, s[i], t[i], len_jps, len_c2jps);


    /* Run JPS2 variants */
    S::clear();
    G::query::nodepool = ep_jps2.get_nodepool();
    double len_jps2 = jps2.get_length(map.to_padded_id(sid), map.to_padded_id(tid));
    cnt_jps2.update(&jps2, S::scan_cnt);

    S::clear();
    G::query::nodepool = ep_cjps2.get_nodepool();
    double len_cjps2 = cjps2.get_length(map.to_padded_id(sid), map.to_padded_id(tid));
    cnt_cjps2.update(&cjps2, S::scan_cnt);
    sanity_checking(i, &map, s[i], t[i], len_jps2, len_cjps2);

    S::clear();
    G::query::nodepool = ep_c2jps2.get_nodepool();
    double len_c2jps2 = c2jps2.get_length(map.to_padded_id(sid), map.to_padded_id(tid));
    cnt_c2jps2.update(&c2jps2, S::scan_cnt);
    sanity_checking(i, &map, s[i], t[i], len_jps2, len_c2jps2);

  }
  cout << "--------------------------- " << map.filename() << endl;
  cout << "algo\texp\tgen\ttouch\ttime\tscan\n" << endl;
  cout << "jps\t" << cnt_jps.str() << endl;
  cout << "cjps\t" << cnt_cjps.str() << endl;
  cout << "c2jps\t" << cnt_c2jps.str() << endl;
  cout << "jps2\t" << cnt_jps2.str() << endl;
  cout << "cjps2\t" << cnt_cjps2.str() << endl;
  cout << "c2jps2\t" << cnt_c2jps2.str() << endl;
}

inline void run_scen(w::gridmap& gridmap, w::scenario_manager& scenmgr) {
  vector<node> s, t;
  for (int i=0; i<(int)scenmgr.num_experiments(); i++) {
    w::experiment* exp = scenmgr.get_experiment(i);
    s.push_back({(int)exp->startx(), (int)exp->starty()});
    t.push_back({(int)exp->goalx(), (int)exp->goaly()});
  }
  run(gridmap, s, t, false);
}

TEST_CASE("gval") {
  // count the number of suboptimal node touching/expansion of jps
  // need to modify the corresponding jps class to make this test work
  vector<vector<string>> cases = {
    {"./maps/dao/ost100d.map", "./scenarios/movingai/dao/ost100d.map.scen", "max"},
    {"./maps/dao/isound1.map", "./scenarios/movingai/dao/isound1.map.scen", "min"},
    {"./maps/bgmaps/AR0509SR.map", "./scenarios/movingai/bgmaps/AR0509SR.map.scen", "max"},
    {"./maps/bgmaps/AR0605SR.map", "./scenarios/movingai/bgmaps/AR0605SR.map.scen", "min"},
    {"./maps/starcraft/IceMountain.map", "./scenarios/movingai/starcraft/IceMountain.map.scen", "max"},
    {"./maps/starcraft/Caldera.map", "./scenarios/movingai/starcraft/Caldera.map.scen", "min"},
    {"./maps/iron/scene_sp_sax_04.map", "./scenarios/movingai/iron/scene_sp_sax_04.map.scen", "max"},
    {"./maps/iron/scene_mp_2p_02.map", "./scenarios/movingai/iron/scene_mp_2p_02.map.scen", "min"},
    {"./testcases/maps/diag-random-512.map", "./data/diag-random-512.scen", "synthetic"},
  };
  string header = "map\tsubopt_touch\tsubopt_expd\tscnt\talg";
  cout << header << endl;
  return;
  for (auto it: cases) {
    string mpath = it[0];
    string spath = it[1];
    string desc = it[2];

    warthog::gridmap* map = new warthog::gridmap(mpath.c_str());
    w::octile_heuristic heur(map->width(), map->height());
    /* JPS variants */
    w::jps_expansion_policy ep(map);
    w::jps_expansion_policy_prune cep(map);
    w::jps_expansion_policy_prune2 c2ep(map);
    w::flexible_astar<w::octile_heuristic, w::jps_expansion_policy> jps(&heur, &ep);
    w::flexible_astar<w::octile_heuristic, w::jps_expansion_policy_prune> cjps(&heur, &cep);
    w::flexible_astar<w::octile_heuristic, w::jps_expansion_policy_prune2> c2jps(&heur, &c2ep);

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

    int fromidx = (int)smgr.num_experiments()-50;
    // int fromidx = 0;
    int toindx = (int)smgr.num_experiments();
    for (int i=fromidx; i<toindx; i++) {
      w::experiment* exp = smgr.get_experiment(i);
      int sx = exp->startx(), sy = exp->starty();
      int tx = exp->goalx(), ty = exp->goaly();
      uint32_t sid = map->to_padded_id(sx, sy);
      uint32_t tid = map->to_padded_id(tx, ty);
      dij.run(sid);

      /* JPS variants */
      G::statis::clear();
      G::statis::dist = vector<warthog::cost_t>(dij.dist);
      G::query::nodepool = ep.get_nodepool();
      jps.get_length(sid, tid);
      cnt_jps.update_subopt();

      G::statis::clear();
      G::statis::dist = vector<warthog::cost_t>(dij.dist);
      G::query::nodepool = cep.get_nodepool();
      cjps.get_length(sid, tid);
      cnt_cjps.update_subopt();

      G::statis::clear();
      G::statis::dist = vector<warthog::cost_t>(dij.dist);
      G::query::nodepool = c2ep.get_nodepool();
      c2jps.get_length(sid, tid);
      cnt_c2jps.update_subopt();

      /* JPS2 variants */
      G::statis::clear();
      G::statis::dist = vector<warthog::cost_t>(dij.dist);
      G::query::nodepool = ep2.get_nodepool();
      jps2.get_length(sid, tid);
      cnt_jps2.update_subopt();

      G::statis::clear();
      G::statis::dist = vector<warthog::cost_t>(dij.dist);
      G::query::nodepool = cep2.get_nodepool();
      cjps2.get_length(sid, tid);
      cnt_cjps2.update_subopt();

      G::statis::clear();
      G::statis::dist = vector<warthog::cost_t>(dij.dist);
      G::query::nodepool = c2ep2.get_nodepool();
      c2jps2.get_length(sid, tid);
      cnt_c2jps2.update_subopt();
    }
    cout << mpath << "\t" << cnt_jps.subopt_str() << "\tjps" << endl;
    cout << mpath << "\t" << cnt_cjps.subopt_str() << "\tcjps" << endl;
    cout << mpath << "\t" << cnt_c2jps.subopt_str() << "\tc2jps" << endl;
    cout << mpath << "\t" << cnt_jps2.subopt_str() << "\tjps2" << endl;
    cout << mpath << "\t" << cnt_cjps2.subopt_str() << "\tcjps2" << endl;
    cout << mpath << "\t" << cnt_c2jps2.subopt_str() << "\tc2jps2" << endl;
  }
  G::query::clear();
}

TEST_CASE("jlimit-scen") {
  vector<pair<string, string>> cases = {
    {"./maps/dao/arena.map", "./scenarios/movingai/dao/arena.map.scen"},
    {"./maps/dao/den011d.map", "./scenarios/movingai/dao/den011d.map.scen"},
    {"./maps/starcraft/Archipelago.map", "./scenarios/movingai/starcraft/Archipelago.map.scen"},
    {"./maps/starcraft/ArcticStation.map", "./scenarios/movingai/starcraft/ArcticStation.map.scen"},
    {"./maps/starcraft/Aurora.map", "./scenarios/movingai/starcraft/Aurora.map.scen"},
    {"./maps/starcraft/CatwalkAlley.map", "./scenarios/movingai/starcraft/CatwalkAlley.map.scen"},
    {"./maps/starcraft/GhostTown.map", "./scenarios/movingai/starcraft/GhostTown.map.scen"},
    {"./maps/starcraft/GreenerPastures.map", "./scenarios/movingai/starcraft/GreenerPastures.map.scen"},
    {"./maps/starcraft/IceMountain.map", "./scenarios/movingai/starcraft/IceMountain.map.scen"},
    {"../maps/random10/random512-10-0.map", "../scenarios/movingai/random10/random512-10-0.map.scen"},
    {"../maps/random20/random512-20-0.map", "../scenarios/movingai/random20/random512-20-0.map.scen"},
    {"../maps/street/London_1_256.map", "../scenarios/movingai/street/London_1_256.map.scen"},
    {"../maps/mazes/maze512-1-0.map", "../scenarios/movingai/mazes/maze512-1-0.map.scen"},
    {"../maps/rooms/16room_000.map", "../scenarios/movingai/rooms/16room_000.map.scen"},
    {"../maps/street/Denver_1_1024.map", "../scenarios/movingai/street/Denver_1_1024.map.scen"},
    {"../maps/iron/scene_sp_endmaps.map", "../scenarios/movingai/iron/scene_sp_endmaps.map.scen"},
    {"../maps/iron/scene_mp_4p_01.map", "../scenarios/movingai/iron/scene_mp_4p_01.map.scen"},
  };
  w::scenario_manager scenmgr;
  for (const auto& c: cases) {
    string mpath = c.first;
    string spath = c.second;
    w::gridmap gridmap(mpath.c_str());
    scenmgr.load_scenario(spath.c_str());
    cerr << "map: " << mpath.c_str() << endl;
    run_scen(gridmap, scenmgr);
    scenmgr.clear();
    G::query::clear();
  }
}

TEST_CASE("jlimit-query") {
  vector<string> queries = {
    "./testcases/diag-random-256.query",
    "./testcases/diag-random-512.query",
    "./testcases/diag-random-1024.query",
    "./testcases/square-random-256.query",
    "./testcases/square-random-512.query",
    "./testcases/square-random-1024.query",
    "./testcases/prune.query",
    "./testcases/maze-random.query",
    "./testcases/Berlin.query"
  };
  vector<node> s, t;
  string mpath;
  int num;
  for (const auto& qpath: queries) {
    ifstream file(qpath);
    file >> mpath >> num;
    w::gridmap map(mpath.c_str());
    s.resize(num), t.resize(num);
    cerr << "\n{testing: " << qpath << endl;
    for (int i=0; i<num; i++) {
      int sx, sy, tx, ty;
      file >> sx >> sy >> tx >> ty;
      s[i] = node{sx, sy};
      t[i] = node{tx, ty};
    }
    run(map, s, t, false);
    cerr << "}" << endl;
  }
}

TEST_CASE("jlimit-maxscan") {
  vector<string> mpaths = {
    "./testcases/maps/maxscan-128.map",
    "./testcases/maps/maxscan-256.map",
    "./testcases/maps/maxscan-512.map",
    "./testcases/maps/maxscan-1024.map"
  };
  for (const auto& mpath: mpaths) {
    w::gridmap map(mpath.c_str());
    int l = map.header_height();
    vector<node> s = {{1, 1}};
    vector<node> t = {{l-1, l-1}};
    run(map, s, t);
  }
}

TEST_CASE("jlimit-empty") {
  vector<string> mpaths = {
    "./testcases/maps/empty-128.map",
    "./testcases/maps/empty-256.map",
    "./testcases/maps/empty-512.map",
    "./testcases/maps/empty-1024.map"
  };
  srand(time(NULL));
  const int num = 100;
  for (const auto& mpath: mpaths) {
    w::gridmap map(mpath.c_str());
    int l = map.header_height();
    vector<node> s, t;
    s.resize(num), t.resize(num);
    for (int i=0; i<num; i++) {
      s[i] = {rand() % (l-2), rand() % (l-2)};
      t[i] = {l-1, rand() % (l-1)};
    }
    run(map, s, t);
  }
}

TEST_CASE("jlimit-exp") {

  std::map< string, vector<tuple<node, node, double>> > cases = {
    {
      "../maps/starcraft/WatersEdge.map",
      {
        {{501, 97}, {138, 40}, 699.997}
      }
    },
    {
      "../maps/street/London_1_256.map",
      {
        {{113, 12}, {200, 176}, 212.521}
      }
    },
    {
      "./maps/random10/random512-10-0.map",
      {
        {{368, 110}, {358, 131}, 25.142}


      }
    },
    {
      "./maps/random20/random512-20-0.map",
      {
        {{274, 382}, {39, 398}, 256.74}
      }
    },
    {
      "./maps/random40/random512-40-0.map",
      {
        {{505, 246}, {414, 115}, 1045.1}
      }
    },
    {
      "./maps/starcraft/CatwalkAlley.map",
      {
        {{81, 343}, {20, 367}, 181.467}
      }
    },
    {
      "./maps/starcraft/ArcticStation.map",
      {
        {{643, 92}, {493, 259}, 423.515},
        {{767, 600}, {385, 407}, 534},
      }
    },
    {
      "./testcases/maps/dao/den011d.map",
      {
        {{102, 40}, {158, 51}, 198.296},
        {{10, 28}, {164, 65}, 281.266}
      }
    },
    {
      "./testcases/maps/dao/arena.map",
      {
        {{1, 3}, {3, 1}, 3.41421},
        {{1, 13}, {4, 12}, 3.41421},
      }
    },
    {
      "./testcases/maps/starcraft/GreenerPastures.map",
      {
        {{471, 292}, {526, 398}, 128.782},
        {{65, 454}, {572, 175}, 627.834},
        {{38, 489}, {572, 175}, 674.021},
        {{756, 460}, {746, 235}, 231.627},
        {{610, 163}, {375, 187}, 450.973},
      }
    },
    {
      "./testcases/maps/starcraft/GhostTown.map",
      {
        {{336, 186}, {1, 478}, 481.139}
      }
    }
  };


  for (const auto& it: cases) {
    const string& mpath = it.first;
    const auto& queries = it.second;
    w::gridmap map(mpath.c_str());

    // w::jps_expansion_policy_prune exp(&map);
    // w::jps_heuristic heuristic(exp.get_mapper());
    // w::neo_astar<w::jps_expansion_policy_prune> astar(&heuristic, &exp);

    w::jps_expansion_policy_prune jps1(&map);
    w::octile_heuristic heuristic1(map.width(), map.height());
    w::flexible_astar<
      w::octile_heuristic,
      w::jps_expansion_policy_prune> astar(&heuristic1, &jps1);

    astar.set_verbose(true);
    long long scnt = 0;

    for (const auto& q: queries) {

      S::clear();;
      node s = get<0>(q), t = get<1>(q);
      double ans = get<2>(q);

      int sid = s.y * map.header_width() + s.x;
      int gid = t.y * map.header_width() + t.x;
      double len = astar.get_length(map.to_padded_id(sid), map.to_padded_id(gid));
      REQUIRE(fabs(len - ans) <= eps);
      scnt += S::scan_cnt;
    }
    cerr << "scan: " << scnt << endl;
  }
}

}

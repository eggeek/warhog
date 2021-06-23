#include <vector>
#include <fstream>
#include <map>
#include <string>

#include "catch.hpp"
#include "experiment.h"
#include "gridmap.h"
#include "jps_expansion_policy_prune.h"
#include "jps_expansion_policy.h"
#include "online_jump_point_locator_prune.h"
#include "online_jump_point_locator.h"
#include "flexible_astar.h"
#include "octile_heuristic.h"
#include "scenario_manager.h"
#include "jps_heuristic.h"
#include "neo_astar.h"
using namespace std;

namespace TEST_JLIMIT {

struct node {
  int x, y;
};

const double eps = 1e-2;
namespace w=warthog;

inline void run(w::gridmap& map, vector<node>& s, vector<node>& t, bool verbose=false) {

  long long tot0, tot1, exp0, exp1, gen0, gen1, touch0, touch1;
  double time0, time1;
    w::jps_expansion_policy_prune jps0(&map);
    w::jps_heuristic jpsh(jps0.get_mapper());
    w::neo_astar<w::jps_expansion_policy_prune> neoAstar(&jpsh, &jps0);

    neoAstar.set_verbose(verbose);

    w::jps_expansion_policy jps1(&map);
    w::octile_heuristic heuristic1(map.width(), map.height());
    w::flexible_astar<
      w::octile_heuristic,
      w::jps_expansion_policy> astar1(&heuristic1, &jps1);

    astar1.set_verbose(verbose);

    tot0 = tot1 = exp0 = exp1 = gen0 = gen1 = touch0 = touch1 = 0;
    time0 = time1 = 0;
    for (int i=0; i<(int)s.size(); i++) {
      jps0.get_locator()->scan_cnt = 0;
      jps1.get_locator()->scan_cnt = 0;

      int sid = s[i].y * map.header_width() + s[i].x;
      int tid = t[i].y * map.header_width() + t[i].x;

      jpsh.set_target(map.to_padded_id(tid));
      double len_prune = neoAstar.get_length(map.to_padded_id(sid), map.to_padded_id(tid));
      exp0 += neoAstar.get_nodes_expanded();
      gen0 += neoAstar.get_nodes_generated();
      touch0 += neoAstar.get_nodes_touched();
      time0 += neoAstar.get_search_time();
      tot0 += jps0.get_locator()->scan_cnt;

      double len = astar1.get_length(map.to_padded_id(sid), map.to_padded_id(tid));
      exp1 += astar1.get_nodes_expanded();
      gen1 += astar1.get_nodes_generated();
      touch1 += astar1.get_nodes_touched();
      time1 += astar1.get_search_time();
      tot1 += jps1.get_locator()->scan_cnt;

      if (fabs(len_prune - len) > eps) {
        cerr << i << "\t" << map.filename() << "\t" << map.header_height()
             << "\t" << map.header_width() << "\t" << s[i].x << "\t" << s[i].y
             << "\t" << t[i].x << "\t" << t[i].y << "\t" << len << endl;
        cerr << "len_prune: " << len_prune << ", len: " << len << endl;
      }

      REQUIRE(fabs(len_prune - len) < eps);
    }
    cout << "--------------------------- " << map.filename() << endl;
    cout << "algo\texp\tgen\ttouch\ttime\tscan\n" << endl;
    cout << "jlimit\t" << exp0 << "\t" << gen0 << "\t" << touch0 << "\t" << time0 << "\t" << tot0 << endl;
    cout << "normal\t" << exp1 << "\t" << gen1 << "\t" << touch1 << "\t" << time1 << "\t" << tot1 << endl;

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
    {"../maps/random40/random512-40-0.map", "../scenarios/movingai/random40/random512-40-0.map.scen"},
    {"../maps/random30/random512-30-8.map", "../scenarios/movingai/random30/random512-30-8.map.scen"},
  };
  w::scenario_manager scenmgr;
  for (const auto& c: cases) {
    string mpath = c.first;
    string spath = c.second;
    w::gridmap gridmap(mpath.c_str());
    scenmgr.load_scenario(spath.c_str());
    run_scen(gridmap, scenmgr);
    scenmgr.clear();
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
      "./maps/starcraft/CatwalkAlley.map",
      {
        {{81, 343}, {20, 367}, 181.467}
      }
    },
    // {
    //   "./maps/starcraft/ArcticStation.map",
    //   {
    //     {{767, 600}, {385, 407}, 534}
    //   }
    // },
    // {
    //   "./testcases/maps/dao/den011d.map",
    //   {
    //     {{10, 28}, {164, 65}, 281.266}
    //   }
    // },
    // {
    //   "./testcases/maps/dao/arena.map",
    //   {
    //     {{1, 3}, {3, 1}, 3.41421},
    //     {{1, 13}, {4, 12}, 3.41421},
    //   }
    // },
    // {
    //   "./testcases/maps/starcraft/GreenerPastures.map",
    //   {
    //     {{471, 292}, {526, 398}, 128.782},
    //     {{65, 454}, {572, 175}, 627.834},
    //     {{38, 489}, {572, 175}, 674.021},
    //     {{756, 460}, {746, 235}, 231.627},
    //     {{610, 163}, {375, 187}, 450.973},
    //   }
    // },
    // {
    //   "./testcases/maps/starcraft/GhostTown.map",
    //   {
    //     {{336, 186}, {1, 478}, 481.139}
    //   }
    // }
  };


  for (const auto& it: cases) {
    const string& mpath = it.first;
    const auto& queries = it.second;
    w::gridmap map(mpath.c_str());
    w::jps_expansion_policy_prune exp(&map);

    w::jps_heuristic heuristic(exp.get_mapper());
    w::neo_astar<w::jps_expansion_policy_prune> astar(&heuristic, &exp);

    // w::jps_expansion_policy_prune jps1(&map);
    // w::octile_heuristic heuristic1(map.width(), map.height());
    // w::flexible_astar<
    //   w::octile_heuristic,
    //   w::jps_expansion_policy_prune> astar(&heuristic1, &jps1);

    astar.set_verbose(true);

    for (const auto& q: queries) {

      node s = get<0>(q), t = get<1>(q);
      double ans = get<2>(q);

      exp.get_locator()->scan_cnt = 0;

      int sid = s.y * map.header_width() + s.x;
      int gid = t.y * map.header_width() + t.x;
      double len = astar.get_length(map.to_padded_id(sid), map.to_padded_id(gid));
      REQUIRE(fabs(len - ans) <= eps);
    }
  }
}

}

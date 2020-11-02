#include <vector>
#include <fstream>
#include <map>
#include <string>

#include "catch.hpp"
#include "gridmap.h"
#include "jps_expansion_policy_prune.h"
#include "online_jump_point_locator_prune.h"
#include "flexible_astar.h"
#include "octile_heuristic.h"
#include "scenario_manager.h"
using namespace std;

namespace TEST_JLIMIT {

struct node {
  int x, y;
};

const double eps = 1e-2;
namespace w=warthog;

void run(w::gridmap& map, vector<node>& s, vector<node>& t, bool verbose=false) {

  long long tot0, tot1, exp0, exp1, gen0, gen1, touch0, touch1;
  double time0, time1;
    w::jps_expansion_policy_prune jps0(&map);
    w::octile_heuristic heuristic0(map.width(), map.height());
    w::flexible_astar<
      w::octile_heuristic,
      w::jps_expansion_policy_prune> astar0(&heuristic0, &jps0);

    jps0.get_locator()->jprune = true;
    jps0.get_locator()->gprune = false;
    astar0.set_verbose(verbose);

    w::jps_expansion_policy_prune jps1(&map);
    w::octile_heuristic heuristic1(map.width(), map.height());
    w::flexible_astar<
      w::octile_heuristic,
      w::jps_expansion_policy_prune> astar1(&heuristic1, &jps1);

    jps1.get_locator()->jprune = false;
    jps1.get_locator()->gprune = false;
    astar1.set_verbose(verbose);

    tot0 = tot1 = exp0 = exp1 = gen0 = gen1 = touch0 = touch1 = 0;
    time0 = time1 = 0;
    for (int i=0; i<(int)s.size(); i++) {
      jps0.get_locator()->jpruner->scan_cnt = 0;

      int sid = s[i].y * map.header_width() + s[i].x;
      int tid = t[i].y * map.header_width() + t[i].x;

      double len = astar0.get_length(map.to_padded_id(sid), map.to_padded_id(tid));
      exp0 += astar0.get_nodes_expanded();
      gen0 += astar0.get_nodes_generated();
      touch0 += astar0.get_nodes_touched();
      time0 += astar0.get_search_time();
      tot0 += jps0.get_locator()->jpruner->scan_cnt;

      int e0 = astar0.get_nodes_expanded();

      jps1.get_locator()->jpruner->scan_cnt = 0;

      double len2 = astar1.get_length(map.to_padded_id(sid), map.to_padded_id(tid));
      exp1 += astar1.get_nodes_expanded();
      gen1 += astar1.get_nodes_generated();
      touch1 += astar1.get_nodes_touched();
      time1 += astar1.get_search_time();
      tot1 += jps1.get_locator()->jpruner->scan_cnt;

      int e1 = astar1.get_nodes_expanded();

      if (fabs(len - len2) > eps || e0 > e1) {
        cerr << "1\t" << map.filename() << "\t" << map.header_height()
             << "\t" << map.header_width() << "\t" << s[i].x << "\t" << s[i].y
             << "\t" << t[i].x << "\t" << t[i].y << "\t" << len2 << endl;
        cerr << "len: " << len << ", len2: " << len2 << endl;
      }

      REQUIRE(e0 <= e1);
      REQUIRE(fabs(len - len2) < eps);
    }
    cout << "---------------------------" << endl;
    cout << "algo\texp\tgen\ttouch\ttime\tscan\n" << endl;
    cout << "jlimit\t" << exp0 << "\t" << gen0 << "\t" << touch0 << "\t" << time0 << "\t" << tot0 << endl;
    cout << "normal\t" << exp1 << "\t" << gen1 << "\t" << touch1 << "\t" << time1 << "\t" << tot1 << endl;

}

TEST_CASE("jlimit-random") {
  vector<string> queries = {
    "./testcases/random-256.query",
    "./testcases/random-512.query",
    "./testcases/random-1024.query",
  };
  vector<node> s, t;
  string mpath;
  int num;
  for (const auto& qpath: queries) {
    ifstream file(qpath);
    file >> mpath >> num;
    w::gridmap map(mpath.c_str());
    s.resize(num), t.resize(num);
    for (int i=0; i<num; i++) {
      int sx, sy, tx, ty;
      file >> sx >> sy >> tx >> ty;
      s[i] = node{sx, sy};
      t[i] = node{tx, ty};
    }
    run(map, s, t);
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
    vector<node> s = {{1, l / 2}};
    vector<node> t = {{l-2, l / 2}};
    run(map, s, t);
  }
}

TEST_CASE("jlimit-exp") {

  std::map< string, vector<tuple<node, node, double, bool, bool>> > cases = {
    {
      "./testcases/maps/dao/arena.map",
      {
        {{1, 13}, {4, 12}, 3.41421, false, true},
      }
    },
    {
      "./testcases/maps/starcraft/GreenerPastures.map",
      {
        {{471, 292}, {526, 398}, 128.782, false, true},
        {{65, 454}, {572, 175}, 627.834, false, true},
        {{38, 489}, {572, 175}, 674.021, false, true},
        {{756, 460}, {746, 235}, 231.627, false, true},
        {{610, 163}, {375, 187}, 450.973, false, true},
      }
    },
    {
      "./testcases/maps/starcraft/GhostTown.map",
      {
        {{336, 186}, {1, 478}, 481.139, false, true}
      }
    }
  };


  for (const auto& it: cases) {
    const string& mpath = it.first;
    const auto& queries = it.second;
    w::gridmap map(mpath.c_str());
    w::jps_expansion_policy_prune exp(&map);
    w::octile_heuristic heuristic(map.width(), map.height());
    w::flexible_astar<
      w::octile_heuristic, 
      w::jps_expansion_policy_prune> astar(&heuristic, &exp);
    astar.set_verbose(true);

    for (const auto& q: queries) {

      node s = get<0>(q), t = get<1>(q);
      double ans = get<2>(q);
      bool gprune = get<3>(q), jlimit = get<4>(q);

      exp.get_locator()->gprune = gprune;
      exp.get_locator()->jprune = jlimit;
      exp.get_locator()->jpruner->scan_cnt = 0;

      int sid = s.y * map.header_width() + s.x;
      int gid = t.y * map.header_width() + t.x;
      double len = astar.get_length(map.to_padded_id(sid), map.to_padded_id(gid));
      REQUIRE(fabs(len - ans) <= eps);
    }
  }
}

}

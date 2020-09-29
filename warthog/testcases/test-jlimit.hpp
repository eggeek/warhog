#include <vector>
#include <fstream>


#include "catch.hpp"
#include "gridmap.h"
#include "jps_expansion_policy_prune.h"
#include "online_jump_point_locator_prune.h"
#include "flexible_astar.h"
#include "octile_heuristic.h"
using namespace std;

namespace TEST_JLIMIT {

struct node {
  int x, y;
};

const double eps = 1e-3;
namespace w=warthog;

TEST_CASE("jlimit-8") {
  string mpath = "./testcases/maps/square-8.map";
  w::gridmap map(mpath.c_str());
  w::jps_expansion_policy_prune exp(&map);
  w::octile_heuristic heuristic(map.width(), map.height());
  w::flexible_astar<
    w::octile_heuristic, 
    w::jps_expansion_policy_prune> astar(&heuristic, &exp);
  astar.set_verbose(false);


  vector<tuple<node, node, double, bool, bool>> queries = {
    {{1, 3}, {9, 5}, 11.414, false, false},
    {{1, 3}, {9, 5}, 11.414, false, true},
  };


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
    REQUIRE(fabs(len - ans) < eps);
    cout << exp.get_locator()->jpruner->scan_cnt << endl;
  }
}

}

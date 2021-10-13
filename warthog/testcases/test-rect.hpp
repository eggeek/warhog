#include <fstream>
#include <string>
#include <utility>
#include <vector>
#include "catch.hpp"
#include "constants.h"
#include "gridmap.h"
#include "jps.h"
#include "octile_heuristic.h"
#include "rect_jump_point_locator.h"
#include "online_jump_point_locator2.h"
#include "rectmap.h"
#include "scenario_manager.h"
#include "jps2_expansion_policy.h"
#include "rect_expansion_policy.h"
#include "flexible_astar.h"

namespace TEST_RECT {
  using namespace warthog::rectscan;
  using namespace std;
  const vector<string> desc = {
    "NORTH", "SOUTH", "EAST", "WEST",
    "NORTHEAST", "NORTHWEST", "SOUTHEAST", "SOUTHWEST"
  };

  TEST_CASE("gen") {
    vector<pair<string, string>> cases = {
      {"./maps/dao/arena.map", "./testcases/rects/arena.rect"},
      {"../maps/rooms/64room_000.map", "./testcases/rects/64room.rect"},
    };
    RectMap rectmap;
    for (auto& each: cases) {
      string mapfile = each.first;
      string writeto = each.second;
      rectmap.init(mapfile.c_str()); 
      ofstream out;
      out.open(writeto.c_str());
      rectmap.print(out);
      out.close();
    }
  }

  TEST_CASE("adj") {
    vector<pair<string, string>> cases = {
      {"./testcases/rects/simple0.rect", "./testcases/rects/simple0.adj"},
    };

    RectMap rectmap;
    for (auto &each: cases) {
      string rectfile = each.first;
      string adjfile = each.second;
      rectmap.init(rectfile.c_str(), false);
      ifstream in;
      in.open(adjfile.c_str());
      vector<Rect> rects;
      rects.resize(rectmap.rects.size());
      for (int i=0; i < (int)rects.size(); i++) rects[i].rid = -1;
      for (auto& r: rectmap.rects) {
        rects[r.rid].read(in);
      }
      for (int i=0; i < (int)rectmap.rects.size(); i++) {
        Rect rect = rectmap.rects[i];
        REQUIRE(rects[i].equal(rect));
      }
    }
  }


  void cmp_jpts(vector<uint32_t> &jpts, vector<uint32_t> &jpts2, warthog::gridmap* gmap, RectMap* rmap) { 

    REQUIRE(jpts.size() == jpts2.size());
    const uint32_t id_mask = (1 << 24) - 1;
    set<int> m1, m2;
    for (int i=0; i<(int)jpts.size(); i++) {
      int x, y;
      uint32_t x2, y2;
      rmap->to_xy(jpts[i] & id_mask, x, y);
      gmap->to_unpadded_xy(jpts2[i] & id_mask, x2, y2);
      
      m1.insert(rmap->to_id(x, y));
      m2.insert(y * gmap->header_width() + x);
    }
    for (auto &it: m1) {
      REQUIRE(m2.count(it) == 1);
    }
    for (auto &it: m2) {
      REQUIRE(m1.count(it) == 1);
    }
  }


  TEST_CASE("jump") {
    vector<string> cases = {
      "./testcases/rects/simple0.rect",
      "./testcases/rects/arena.rect",
      "../maps/rooms/64room_000.map",
      "../maps/starcraft/CatwalkAlley.map"
    };
    for (auto &each: cases) {
      string rectfile = each;
      cout << "map: " << rectfile << endl;
      bool flag = rectfile.back() == 'p';
      RectMap rectmap(rectfile.c_str(), flag);
      warthog::gridmap* gmap = new warthog::gridmap(rectfile.c_str());
      warthog::online_jump_point_locator2* jpl2 = 
        new warthog::online_jump_point_locator2(gmap);
      rect_jump_point_locator jpl = rect_jump_point_locator(&rectmap); 

      vector<uint32_t> jpts; 
      vector<uint32_t> jpts2;
      vector<warthog::cost_t> costs2;
      for (int d=0; d<8; d++)
      for (int x=0; x<rectmap.mapw; x++) {
        for (int y=0; y<rectmap.maph; y++) {
          warthog::jps::direction dir = (warthog::jps::direction)(1<<d);
          int padded_nid = gmap->to_padded_id(x, y);
          if (gmap->get_label(padded_nid) == 0) continue;
          Rect* r = rectmap.get_rect(x, y);

          // cout << "id: " << r->rid << ", x: " << x << ", y: " << y
          //      << " d: " << d << " " << desc[d] << endl;

          jpts2.clear(); costs2.clear();
          jpl2->jump(dir, padded_nid, warthog::INF, jpts2, costs2);

          jpl.get_jpts().clear();
          jpl.get_costs().clear();
          jpl.jump(dir, y*rectmap.mapw+x, INF, r);
          jpts = jpl.get_jpts();
          cmp_jpts(jpts, jpts2, gmap, &rectmap);
        }
      }
      delete gmap;
      delete jpl2;
    }
  }

  TEST_CASE("intervalScan") {
    vector<string> cases = {
      "./testcases/rects/simple0.rect",
      "./testcases/rects/arena.rect",
      "../maps/rooms/64room_000.map",
      "../maps/starcraft/CatwalkAlley.map"
    };
    for (auto &each: cases) {
      string rectfile = each;
      cout << "map:" << rectfile << endl; 
      bool flag = rectfile.back() == 'p';
      RectMap rectmap(rectfile.c_str(), flag);
      warthog::gridmap* gmap = new warthog::gridmap(rectfile.c_str());
      warthog::online_jump_point_locator2* jpl2 = 
        new warthog::online_jump_point_locator2(gmap);
      rect_jump_point_locator jpl = rect_jump_point_locator(&rectmap); 

      vector<uint32_t> jpts; 
      vector<uint32_t> jpts2;
      vector<warthog::cost_t> costs, costs2;
      for (auto& it: rectmap.rects) {
        for (int d=0; d<4; d++) {
          warthog::jps::direction dir = (warthog::jps::direction)(1<<d);
          int dx = warthog::dx[d];
          int dy = warthog::dy[d];
          int lb=0, ub=0, ax;
          eposition cure = r2e.at({dx, dy, rdirect::B});
          it.get_range(cure, lb, ub);
          ax = it.axis(cure);
          jpts2.clear(); costs2.clear();
          // cout << "id: " << it.rid << ", lb: " << lb << ", ub: " << ub
          //      << ", ax: " << ax << ", d: " << desc[d] << endl;
          for (int i=lb; i<=ub; i++) {
            int padded_nid = gmap->to_padded_id(dx?ax: i, dx?i: ax);
            jpl2->jump(dir, padded_nid, warthog::INF, jpts2, costs2);
          }

          jpl.get_jpts().clear(); jpl.get_costs().clear();
          jpl.scanInterval(lb, ub, &(it), dx, dy);
          jpts = jpl.get_jpts();
          cmp_jpts(jpts, jpts2, gmap, &rectmap);
        }
      }
      delete gmap;
      delete jpl2;
    }
  }

  void run_scen(warthog::scenario_manager& scenmgr, bool verbose=false) {

    warthog::gridmap map(scenmgr.mapfile.c_str());
    warthog::jps2_expansion_policy expander(&map);
    warthog::octile_heuristic heuristic(map.width(), map.height());
    warthog::flexible_astar<
      warthog::octile_heuristic,
      warthog::jps2_expansion_policy> a2(&heuristic, &expander);
    a2.set_verbose(verbose);

    bool flag = scenmgr.mapfile.back() == 'p';
    RectMap rmap(scenmgr.mapfile.c_str(), flag);
    rect_expansion_policy rexpan(&rmap);
    warthog::octile_heuristic rheur(rmap.mapw, rmap.maph);
    warthog::flexible_astar<
      warthog::octile_heuristic,
      rect_expansion_policy> ra(&rheur, &rexpan);
    ra.set_verbose(verbose);

    for (int i=0; i<(int)scenmgr.num_experiments(); i++) {
      warthog::experiment* exp = scenmgr.get_experiment(i);
      int sid = exp->starty() * exp->mapwidth() + exp->startx();
      int tid = exp->goaly() * exp->mapwidth() + exp->goalx();
      double len2 = a2.get_length(map.to_padded_id(sid), map.to_padded_id(tid));
      double len = ra.get_length(sid, tid);
      REQUIRE(abs(len - len2) == 0);
    }
  }

  TEST_CASE("query") {
    vector<pair<string, string>> cases = {
      {"../maps/dao/arena.map", "../scenarios/movingai/dao/arena.map.scen"},
      {"../maps/dao/brc202d.map", "../scenarios/movingai/dao/brc202d.map.scen"},
      {"../maps/starcraft/CatwalkAlley.map", "../scenarios/movingai/starcraft/CatwalkAlley.map.scen"},
    };
    for (auto& it: cases) {
      string mpath = it.first;
      string spath = it.second;
      warthog::scenario_manager* scenmgr = new warthog::scenario_manager();
      scenmgr->load_scenario(spath.c_str());
      scenmgr->mapfile = mpath;
      cerr << "Run map: " << mpath << endl;
      run_scen(*scenmgr, false);
      delete scenmgr;
    }
  }
}

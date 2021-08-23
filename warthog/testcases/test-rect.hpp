#include <fstream>
#include <string>
#include <utility>
#include <vector>
#include "catch.hpp"
#include "constants.h"
#include "gridmap.h"
#include "jps.h"
#include "rect_jump_point_locator.h"
#include "online_jump_point_locator2.h"
#include "rectmap.h"

namespace TEST_RECT {
  using namespace warthog::rectscan;
  using namespace std;

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

  void cmp_jpts(vector<int> &jpts, vector<warthog::cost_t>& costs, 
      vector<uint32_t> &jpts2, vector<warthog::cost_t> &costs2, warthog::gridmap* gmap) {
    REQUIRE(jpts.size() == jpts2.size());

    const uint32_t id_mask = (1 << 24) - 1;
    map<int, warthog::cost_t> m1, m2;
    for (int i=0; i<(int)jpts.size(); i++) {
      uint32_t x, y;
      gmap->to_unpadded_xy(jpts2[i] & id_mask, x, y);
      m1[jpts[i]] = costs[i];
      m2[y * gmap->header_width() + x] = costs[i];
    }
    for (auto &it: m1) {
      REQUIRE(m2.find(it.first) != m2.end());
      REQUIRE(it.second == m2.at(it.first));
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

      vector<int> jpts; 
      vector<uint32_t> jpts2;
      vector<warthog::cost_t> costs, costs2;
      for (int d=0; d<4; d++)
      for (int x=0; x<rectmap.mapw; x++) {
        for (int y=0; y<rectmap.maph; y++) {
          warthog::jps::direction dir = (warthog::jps::direction)(1<<d);
          // cout << "x: " << x << ", y: " << y << " d: " << dir << endl;
          int padded_nid = gmap->to_padded_id(x, y);
          if (gmap->get_label(padded_nid) == 0) continue;
          Rect* r = rectmap.get_rect(x, y);
          jpts.clear(); jpts2.clear();
          costs.clear(); costs2.clear();
          jpl2->jump(dir, padded_nid, warthog::INF, jpts2, costs2);
          jpl.jump(dir, y*rectmap.mapw+x, warthog::INF, r, jpts, costs);
          cmp_jpts(jpts, costs, jpts2, costs2, gmap);
        }
      }
      delete gmap;
      delete jpl2;
    }
  }
}

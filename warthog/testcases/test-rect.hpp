#include <fstream>
#include <string>
#include <utility>
#include <vector>
#include "catch.hpp"
#include "grid2rect.h"
#include "rectmap.h"

namespace TEST_RECT {
  using namespace rectscan;
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
}

#pragma once

/*
 * decompose grid map into rectangles, 
 * all grids in each rectangle are traversable,
 * so that recursive diagonal scanning can be faster
 */
#include <cstdlib>
#include <fstream>
#include <istream>
#include <map>
#include <vector>
#include <set>

#include "gm_parser.h"
#include "gridmap.h"
#include "grid2rect.h"

namespace rectscan {

using namespace std;
using namespace warthog;

/*
 *    ...
 *   |    |  |    ||     |
 *   +----+  +----++-----+ neighbor rectangles
 *      +--------------+
 *    y↓|x→  edge0     |
 *      |              |
 * edge3|              | edge1
 *      |  rectangle   |
 *      |              |
 *      |              |
 *      +--------------+
 *           edge2
 */

// the id of oppsite edge: {0, 2}, {1, 3}
static const int nxtId[4] = {2, 3, 0, 1};

class Rect {
  public:
  int rid,            // id of this rectangle
      x, y,           // top-left corner of the rect
      h, w;           // shape of the rect
  vector<int> adj[4]; // adj[i] stores adjacent rects of edge i

  void read(istream& in) {
    in >> rid >> x >> y >> h >> w;
    for (int i=0; i<4; i++) {
      int size;
      in >> size;
      adj[i].resize(size);
      for (int j=0; j<size; j++) in >> adj[i][j];
    }
  }
  
  bool operator== (const Rect& other) const {
    return equal(other);
  }

  bool equal(const Rect& other) const {
    if (rid != other.rid) return false;
    if (x != other.x || y != other.y || h != other.h || w != other.w) return false;
    for (int i=0; i < 4; i++) {
      if (adj[i].size() != other.adj[i].size()) return false;
      for (int j=0; j < (int)adj[i].size(); j++) 
        if (adj[i][j] != other.adj[i][j]) return false;
    }
    return true;
  }
};


class RectMap {
  public:
  vector<Rect> rects;
  int maph, mapw;

  RectMap() {};
  
  RectMap(const char* mapfile, bool make=true) {
    init(mapfile, make);
  };

  ~RectMap() {
    rects.clear();
    rects.shrink_to_fit();
  }

  void init(const char* mapfile, bool make=true) {
    // generate rectangle from original map
    if (make) {
      ifstream fin;
      fin.open(mapfile);
      rectgen::read_map(fin);
      rectgen::make_rectangles();
      maph = rectgen::map_height;
      mapw = rectgen::map_width;
    }
    else {
    // or load rectangles from a user specified rectangles
    // each rectangle is labelled by 0~9 a~z A~Z
    // only work if #rect is not large than 10+26*2 
      make_rectangles_from_file(mapfile);
    }
    init_rects();
  }

  void make_rectangles_from_file(const char* rectfile) {
    gm_parser parser(rectfile);
    gm_header header = parser.get_header();
    maph = header.height_;
    mapw = header.width_;
    map<char, rectgen::FinalRect> rs;

    for (int i=0; i<(int)parser.get_num_tiles(); i++) {
      char c = parser.get_tile_at(i);
      if (
        ('0' <= c && c <= '9') || 
        ('a' <= c && c <= 'z') ||
        ('A' <= c && c <= 'Z') 
      ) {
        int x = i % mapw;
        int y = i / mapw;
        if (rs.find(c) == rs.end()) {
          rs[c].x = x;
          rs[c].y = y;
          rs[c].height = y;
          rs[c].width = x;
        }
        else {
          rs[c].x = min(rs[c].x, x);
          rs[c].y = min(rs[c].y, y);
          rs[c].height = max(rs[c].height, y);
          rs[c].width = max(rs[c].width, x);
        }
      }
    }
    rectgen::final_rectangles.clear();
    for (auto& it: rs) {
      it.second.height -= it.second.y - 1;
      it.second.width -= it.second.x - 1;
      rectgen::final_rectangles.push_back(it.second);
    }
  }

  void print(ostream& out) {
    out << "type "<< "octile" << std::endl;
    out << "height "<< maph << std::endl;
    out << "width "<< mapw << std::endl;
    out << "map" << std::endl;

    auto get_label = [&](int id) {
      if (id == -1) return '@';
      id %= 10 + 26 * 2;
      if (id<10) return char('0' + id);
      id -= 10;
      if (id<26) return char('a' + id);
      id -= 26;
      return char('A' + id);
    };


    vector<int> idmap;
    idmap.resize(mapw*maph);
    fill(idmap.begin(), idmap.end(), -1);
    for (int i=0; i<(int)rects.size(); i++) {
      for (int x=rects[i].x; x<rects[i].x+rects[i].w; x++)
      for (int y=rects[i].y; y<rects[i].y+rects[i].h; y++) {
        idmap[y*mapw+x] = rects[i].rid;
      }
    }
    for (int y=0; y<maph; y++)
    {
      for (int x=0; x<mapw; x++) {
        int pos = y * mapw + x;
        char c = get_label(idmap[pos]);
        out << c;
      }
      out << endl;
    }
  }

  private:

  void init_rects() {

    const vector<rectgen::FinalRect>& frects = rectgen::final_rectangles;
    rects.resize(frects.size());

    vector<int> idmap;
    idmap.resize(maph * mapw);

    // idmap stores the rect id of each traversable tile
    // so that we can compute the neighbor rects of each edge
    fill(idmap.begin(), idmap.end(), -1);
    for (int i=0; i<(int)frects.size(); i++) {
      for (int x=frects[i].x; x<frects[i].x + frects[i].width; x++) {
        for (int y=frects[i].y; y<frects[i].y + frects[i].height; y++) {
          idmap[y * mapw + x] = i;
        }
      }
    }

    auto calc_adj = [&](
        int xl, int xr, int yl, int yr) {
        
      set<int> adj_set;
      vector<int> adj;
      adj.clear();
      // adjacent rects are in order left-right, top-down
      for (int x=xl; x<=xr; x++)
      for (int y=yl; y<=yr; y++) 
      if (x >= 0 && x < mapw && y >= 0 && y < maph) {
        // ignore adjacent obstacles
        if (idmap[y * mapw + x] == -1) continue;
        if (adj_set.find(idmap[y * mapw + x]) == adj_set.end()) {
          adj_set.insert(idmap[y * mapw + x]);
          adj.push_back(idmap[y * mapw + x]);
        }
      }
      return adj;
    };

    for (int i=0; i<(int)rects.size(); i++) {
      Rect& r = rects[i];
      rectgen::FinalRect fr = rectgen::final_rectangles[i];
      r.rid = i;
      r.x = fr.x;
      r.y = fr.y;
      r.h = fr.height;
      r.w = fr.width;

      for (int j=0; j<4; j++) r.adj[j].clear();
      r.adj[0] = calc_adj(r.x, r.x+r.w-1, r.y-1, r.y-1);
      r.adj[1] = calc_adj(r.x+r.w, r.x+r.w, r.y, r.y+r.h-1);
      r.adj[2] = calc_adj(r.x, r.x+r.w-1, r.y+r.h, r.y+r.h);
      r.adj[3] = calc_adj(r.x-1, r.x-1, r.y, r.y+r.h-1);
    }

  }
};

}

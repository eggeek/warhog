#pragma once

/*
 * decompose grid map into rectangles, 
 * all grids in each rectangle are traversable,
 * so that recursive diagonal scanning can be faster
 */
#include <cassert>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <istream>
#include <map>
#include <vector>
#include <set>
#include <map>
#include <tuple>

#include "gm_parser.h"
#include "gridmap.h"
namespace warthog {
namespace rectscan {

using namespace std;

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

enum eposition {
  N = 0,
  E = 1,
  S = 2,
  W = 3,
  I = 4,
};

/* relative direction
 *       Forward 2
 *      +--------+
 *      |        |
 *  Left|        |Right
 *    1 |        | 3
 *      +--------+
 *        Back 0
 *
 */
enum rdirect {
  B = 0,
  L = 1,
  R = 2,
  F = 3,
};

// given move direction, return the edge id in the relative direction
const map<tuple<int, int, rdirect>, eposition> r2e = {
  // north move
  {{0, -1, rdirect::B}, eposition::S},
  {{0, -1, rdirect::L}, eposition::W},
  {{0, -1, rdirect::F}, eposition::N},
  {{0, -1, rdirect::R}, eposition::E},
  // south move
  {{0, 1,  rdirect::B}, eposition::N},
  {{0, 1,  rdirect::L}, eposition::E},
  {{0, 1,  rdirect::F}, eposition::S},
  {{0, 1,  rdirect::R}, eposition::W},
  // east move
  {{1, 0,  rdirect::B}, eposition::W},
  {{1, 0,  rdirect::L}, eposition::N},
  {{1, 0,  rdirect::F}, eposition::E},
  {{1, 0,  rdirect::R}, eposition::S},
  // west move
  {{-1, 0, rdirect::B}, eposition::E},
  {{-1, 0, rdirect::L}, eposition::S},
  {{-1, 0, rdirect::F}, eposition::W},
  {{-1, 0, rdirect::R}, eposition::N},
};

// given move direction, return the edge id in the relative direction
const map<tuple<int, int, eposition>, rdirect> e2r = {
  // north move
  {{0, -1, eposition::N}, rdirect::F},
  {{0, -1, eposition::E}, rdirect::R},
  {{0, -1, eposition::S}, rdirect::B},
  {{0, -1, eposition::W}, rdirect::L},
  // south move
  {{0, 1,  eposition::N}, rdirect::B},
  {{0, 1,  eposition::E}, rdirect::L},
  {{0, 1,  eposition::S}, rdirect::F},
  {{0, 1,  eposition::W}, rdirect::R},
  // east move
  {{1, 0,  eposition::N}, rdirect::L},
  {{1, 0,  eposition::E}, rdirect::F},
  {{1, 0,  eposition::S}, rdirect::R},
  {{1, 0,  eposition::W}, rdirect::B},
  // west move
  {{-1, 0,  eposition::N}, rdirect::R},
  {{-1, 0,  eposition::E}, rdirect::B},
  {{-1, 0,  eposition::S}, rdirect::L},
  {{-1, 0,  eposition::W}, rdirect::F},
};

class Rect {
  public:
  int rid,              // id of this rectangle
      x, y,             // top-left corner of the rect
      h, w;             // shape of the rect
  vector<int> adj[4];   // adj rects ids of edge i, in top-down-left-right (increase) order
  vector<int> jptf[4];  // jump points in "forward" direction (top-down, left-right)
  vector<int> jptr[4];  // jump points in "reverse" direction (down-top, right-left)
  vector<int> mark[4];  // mark nodes on the border of rect

  inline eposition pos(const int& px, const int& py) const {
    if (py == y) return eposition::N;
    if (px == x+w-1) return eposition::E;
    if (py == y+h-1) return eposition::S;
    if (px == x) return eposition::W;
    return eposition::I;
  }

  inline int get_mark(int pos, eposition cure) {
    switch (cure) {
      case eposition::N:
      case eposition::S:
        pos -= x;
        break;
      case eposition::W:
      case eposition::E:
        pos -= y;
        break;
      default:
        return -1;
    }
    if (pos >= 0 && pos < (int)mark[(int)cure].size()) 
      return mark[(int)cure][pos];
    else
      return -1;
  }

  inline void set_mark(int pos, eposition cure, int v) {
    if (pos < 0) return;
    switch (cure) {
      case eposition::N:
      case eposition::S:
        pos -= x;
        break;
      case eposition::W:
      case eposition::E:
        pos -= y;
        break;
      default:
        break;
    }
    if (pos < (int)mark[(int)cure].size()) {
      // cerr << "mark rect: " << this->rid << ", e: " << cure
      //      << ", at: " << ((cure&1)?pos+y:pos+x) << " value: " << v << endl;
      mark[(int)cure][pos] = v;
    }
  }

  // return axis of an edge
  // vertical return x, horizontal return y
  inline int axis(eposition p) const {
    switch (p) {
      case eposition::N: return y;
      case eposition::E: return x+w-1;
      case eposition::S: return y+h-1;
      case eposition::W: return x;
      default:
        assert(false);
        return -1;
    }
  }

  inline int len(eposition p) const {
    switch (p) {
      case eposition::N:
      case eposition::S:
        return w;
      case eposition::E:
      case eposition::W:
        return h;
      default:
        assert(false);
        return -1;
    }
  }

  // distance to border F
  // return >= 0
  inline int disF(int dx, int dy, int curx, int cury) const {
    int ax = axis(r2e.at({dx, dy, rdirect::F}));
    return dx * (ax - curx) + dy * (ax - cury);
  }

  // distance to border L or R
  // L: <0
  // R: >0
  inline int disLR(rdirect p, int dx, int dy, int curx, int cury) const {
    int ax = axis(r2e.at({dx, dy, p}));
    return dy * (ax - curx) + dx * (ax - cury);
  }

  inline void get_range(const int& eid, int& lb, int& ub) const {
    switch (eid) {
      case 0: 
        lb=x, ub=x+w-1; break;
      case 1: 
        lb=y, ub=y+h-1; break;
      case 2:
        lb=x, ub=x+w-1; break;
      case 3:
        lb=y, ub=y+h-1; break;
      default:
        assert(false);
        break;
    }
  }

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
  gridmap gmap;
  // isjptr[pre_mask][cur_mask]
  bool isjptr[4][4];
  vector<int> idmap;

  RectMap(): gmap(0, 0) { };
  RectMap(const char* mapfile, bool make=true);
  ~RectMap() {
    rects.clear();
    rects.shrink_to_fit();
  }

  void init(const char* mapfile, bool make=true);
  void make_rectangles_from_file(const char* mapfile);

  inline const char* filename() { return this->_filename.c_str();}

  inline void get_neighbours(uint32_t id, uint8_t tiles[3]) {
    gmap.get_neighbours(gmap.to_padded_id(id), tiles);
  }

  inline int get_adj_rect(const Rect* r, const int& eid, const int& pos) const {
    // when edge is horizontal, pos is x axis, otherwise pos is y axis
    int lb, ub;
    for (int i: r->adj[eid]) {
      rects[i].get_range(eid^2, lb, ub);
      if (lb <= pos && pos <= ub) return i;
    }
    return -1;
  }

  // return the mask horizontal, i.e. left,right
  inline int get_maskh(const int& x, const int& y) {
    int padid = gmap.to_padded_id(x, y);
    uint32_t px, py;
    gmap.to_padded_xy(padid, px, py);
    int res = 3;
    if (gmap.get_label(px-1, py)) res ^= 2;
    if (gmap.get_label(px+1, py)) res ^= 1;
    return res;
  }

  // return the mask vertical, i.e. up,down
  inline int get_maskw(const int& x, const int& y) {
    int padid = gmap.to_padded_id(x, y);
    uint32_t px, py;
    gmap.to_padded_xy(padid, px, py);
    int res = 3;
    if (gmap.get_label(px, py-1)) res ^= 2;
    if (gmap.get_label(px, py+1)) res ^= 1;
    return res;
  }

  int mem() { return sizeof(*this); }

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

  inline void to_xy(int id, int& x, int& y) {
    y = id / mapw;
    x = id % mapw;
  }

  inline int to_id(int x, int y) const {
    return y * mapw + x;
  }

  inline int get_rid(int x, int y) {
    if (x < 0 || x >= mapw || y < 0 || y >= maph) return -1;
    return idmap[y * mapw + x];
  }

  inline Rect* get_rect(int x, int y) {
    return &(rects[idmap[y * mapw + x]]);
  }

  inline Rect* get_rect(int id) {
    return &(rects[idmap[id]]);
  }

  private:

  string _filename;

  void init_rects(); 
};

}};

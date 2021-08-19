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

// the id of oppsite edge: {0, 2}, {1, 3}
static const int nxtId[4] = {2, 3, 0, 1};
enum position {
  N = 0,
  E = 1,
  S = 2,
  W = 3,
  I = 4,
};

class Rect {
  public:
  int rid,              // id of this rectangle
      x, y,             // top-left corner of the rect
      h, w;             // shape of the rect
  vector<int> adj[4];   // adj[i] stores adjacent rects of edge i
  vector<int> jptf[4];  // jump points in "forward" direction (top-down, left-right)
  vector<int> jptr[4];  // jump points in "reverse" direction (down-top, right-left)

  inline int pos(const int& px, const int& py) const {
    if (py == y) return position::N;
    if (px == x+w-1) return position::E;
    if (py == y+h-1) return position::S;
    if (px == x) return position::W;
    return position::I;
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

  inline int get_adj_rect(const Rect* r, const int& eid, const int& pos) {
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

  uint32_t mem() { return sizeof(*this); }

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

  inline void to_xy(uint32_t id, uint32_t& x, uint32_t& y) {
    y = id / mapw;
    x = id % mapw;
  }

  inline uint32_t to_id(uint32_t x, uint32_t y) {
    return y * mapw + x;
  }

  inline int get_rid(int x, int y) {
    return idmap[y * mapw + x];
  }

  inline Rect* get_rect(int x, int y) {
    return &(rects[idmap[y * mapw + x]]);
  }

  private:

  string _filename;

  void init_rects(); 
};

}};

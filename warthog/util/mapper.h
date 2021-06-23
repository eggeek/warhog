#pragma once
#include <vector>
#include "gridmap.h"
#include "jps.h"

namespace warthog {
using namespace std;

// return the 8bit mask of node v
// 012
// 3v4
// 567
// 0: traversable, 1: obstacle
inline uint8_t get_neib_mask(gridmap* map, int cid) {
  uint8_t res = 0;
  uint32_t x, y;
  uint32_t w = map->width(), h = map->height();
  map->to_padded_xy(cid, x, y);

  if (y<1 || x<1 || (!map->get_label(x-1, y-1)))      res |= 1;
  if (y<1 || (!map->get_label(x, y-1)))               res |= 1<<1;
  if (y<1 || x+1>=w || (!map->get_label(x+1, y-1)))   res |= 1<<2;

  if (x<1 || (!map->get_label(x-1, y)))               res |= 1<<3;
  if (x+1>=w || (!map->get_label(x+1, y)))            res |= 1<<4;

  if (y+1>=h || x<1 || (!map->get_label(x-1, y+1)))   res |= 1<<5;
  if (y+1>=h || (!map->get_label(x,y+1)))             res |= 1<<6;
  if (y+1>=h || x+1>=w || (!map->get_label(x+1,y+1))) res |= 1<<7;

  return res;
}

class Mapper {

public:

  struct xyLoc {
    uint32_t x, y;
  };

  gridmap* map;
  vector<xyLoc> loc;
  vector<uint32_t> tiles; 
  vector<uint8_t> neib_mask;

  uint8_t suc[1<<11], start_suc[1<<8];
  unsigned int mapw, maph;

  Mapper(gridmap* map_): map(map_) {
    int num = map->height() * map->width();
    mapw = map->width(); maph = map->height();
    loc.resize(num);
    tiles.resize(num);
    neib_mask.resize(num);
    for (int i=0; i<num; i++) {
      map->to_padded_xy(i, loc[i].x, loc[i].y);
      if (map->get_label(loc[i].x, loc[i].y)) {
        map->get_neighbours(i, (uint8_t*)&(tiles[i]));
      } else tiles[i] = 0x7FFF;
    }

    for (int i=0; i<num; i++) if (map->get_label(loc[i].x, loc[i].y)) {
      neib_mask[i] = get_neib_mask(map, i);
    }
    fill(suc, suc+(1<<11), 0);
    for (int i=0; i<num; i++) if (map->get_label(loc[i].x, loc[i].y)) {
      for (int d=0; d<8; d++) {
        int idx = (neib_mask[i]<<3) | d;
        suc[idx] = jps::compute_successors((jps::direction)(1<<d), tiles[i]);
      }
      start_suc[neib_mask[i]] = jps::compute_successors(jps::direction::NONE, tiles[i]);
    }
  };

  void inline 
  xy(const uint32_t& padded_id, uint32_t& px, uint32_t& py) const {
    px = loc[padded_id].x, py = loc[padded_id].y;
  }

  inline void 
  get_neighbours(uint32_t cid, uint32_t& ctiles) const { ctiles = tiles[cid]; }
  
  inline uint8_t
  get_successors(const uint8_t& d, const uint32_t& id) {
    switch(d) {
      case 8: return start_suc[neib_mask[id]];
      default:
        return suc[neib_mask[id]<<3|d];
    }
  }


  vector<string> tiles2str(uint32_t tiles) {
    vector<string> g = {
      "...",
      ".x.",
      "..."
    };
    for (int i=0; i<3; i++, tiles >>= 8) {
      for (int j=0; j<3; j++) {
        if (tiles & (1 << j)) g[i][j] = '.';
        else g[i][j] = '#';
      }
    }
    g[1][1] = 'x';
    return g;
  }

  vector<string> id2tiles(uint32_t id) {
    uint32_t mask;
    map->get_neighbours(id, (uint8_t*)&(mask));
    return tiles2str(mask);
  }
};
}

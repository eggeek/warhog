#include "rectmap.h"
#include "grid2rect.h"
#include <vector>

typedef warthog::rectscan::RectMap rmap;

rmap::RectMap(const char* mapfile, bool make): gmap(mapfile) {
    _filename = string(mapfile);
    init(mapfile, make);
}

void rmap::init(const char* mapfile, bool make) {
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
  // init isjptr
  for (int pre=0; pre<4; pre++)
  for (int cur=0; cur<4; cur++) {
    if (((pre & 2) && (cur & 2) == 0) ||
        ((pre & 1) && (cur & 1) == 0)
       ) isjptr[pre][cur] = true;
    else isjptr[pre][cur] = false;
  }
}

void rmap::make_rectangles_from_file(const char* rectfile) {
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
  
void rmap::init_rects() {

  const vector<rectgen::FinalRect>& frects = rectgen::final_rectangles;
  rects.resize(frects.size());

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

  auto calc_jptsf = [&] (int xl, int xr, int yl, int yr, int dx, int dy) {
    int pre = 0, cur;
    vector<int> res;
    for (int x=xl; x<=xr; x++)
    for (int y=yl; y<=yr; y++)
    if (x >= 0 && x < mapw && y >= 0 && y < maph) {
      cur = idmap[y * mapw + x];
      // from obstacle to traversable
      // (x-dx, y-dy) is on border
      // (x, y) is the adjacent
      if (pre < 0 && cur >= 0) res.push_back((y - dy) * mapw + (x - dx));
      pre = cur;
    }
    return res;
  };
  
  auto calc_jptsr = [&] (int xl, int xr, int yl, int yr, int dx, int dy) {
    int pre = 0, cur;
    vector<int> res;
    for (int x=xr; x>=xl; x--)
    for (int y=yr; y>=yl; y--)
    if (x >= 0 && x < mapw && y >= 0 && y < maph) {
      cur = idmap[y * mapw + x];
      // from obstacle to traversable
      // (x-dx, y-dy) is on border
      // (x, y) is the adjacent
      if (pre < 0 && cur >= 0) res.push_back((y - dy) * mapw + (x - dx));
      pre = cur;
    }
    return res;
  };

  for (int i=0; i<(int)rects.size(); i++) {
    Rect& r = rects[i];
    rectgen::FinalRect fr = rectgen::final_rectangles[i];
    r.rid = i;
    r.x = fr.x;
    r.y = fr.y;
    r.h = fr.height;
    r.w = fr.width;

    r.mark[(int)eposition::N] = vector<int>(r.w, -1);
    r.mark[(int)eposition::S] = vector<int>(r.w, -1);
    r.mark[(int)eposition::W] = vector<int>(r.h, -1);
    r.mark[(int)eposition::E] = vector<int>(r.h, -1);

    for (int j=0; j<4; j++) {
      r.adj[j].clear();
      r.jptf[j].clear();
      r.jptr[j].clear();
    }
    r.adj[0]  = calc_adj(r.x, r.x+r.w-1, r.y-1, r.y-1);
    r.jptf[0] = calc_jptsf(r.x, r.x+r.w-1, r.y-1, r.y-1, 0, -1);
    r.jptr[0] = calc_jptsr(r.x, r.x+r.w-1, r.y-1, r.y-1, 0, -1);

    r.adj[1]  = calc_adj(r.x+r.w, r.x+r.w, r.y, r.y+r.h-1);
    r.jptf[1] = calc_jptsf(r.x+r.w, r.x+r.w, r.y, r.y+r.h-1, 1, 0);
    r.jptr[1] = calc_jptsr(r.x+r.w, r.x+r.w, r.y, r.y+r.h-1, 1, 0);

    r.adj[2]  = calc_adj(r.x, r.x+r.w-1, r.y+r.h, r.y+r.h);
    r.jptf[2] = calc_jptsf(r.x, r.x+r.w-1, r.y+r.h, r.y+r.h, 0, 1);
    r.jptr[2] = calc_jptsr(r.x, r.x+r.w-1, r.y+r.h, r.y+r.h, 0, 1);

    r.adj[3]  = calc_adj(r.x-1, r.x-1, r.y, r.y+r.h-1);
    r.jptf[3] = calc_jptsf(r.x-1, r.x-1, r.y, r.y+r.h-1, -1, 0);
    r.jptr[3] = calc_jptsr(r.x-1, r.x-1, r.y, r.y+r.h-1, -1, 0);
  }
}

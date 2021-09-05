#include "rect_jump_point_locator.h"
#include "constants.h"
#include <algorithm>
#include <cstdint>
#include <vector>

typedef warthog::rectscan::rect_jump_point_locator rectlocator;
using namespace std;

void rectlocator::_scan(int node_id, Rect* cur_rect, int dx, int dy) {

  rdirect curp;
  eposition cure;
  int curx, cury;
  map_->to_xy(node_id, curx, cury);
  cure = cur_rect->pos(curx, cury);

  int jpid;
  bool onL = false, onR = false;

  onL = cur_rect->disLR(rdirect::L, dx, dy, curx, cury) == 0;
  onR = cur_rect->disLR(rdirect::R, dx, dy, curx, cury) == 0;

  auto move_fwd = [&]() {
    cure = r2e.at({dx, dy, rdirect::F});
    int d2F = cur_rect->disF(dx, dy, curx, cury);
    switch (dx) {
      case 0:
        cury += dy * d2F;
        break;
      default:
        curx += dx * d2F;
        break;
    }
  };
  
  // inside, then move to the forward edge
  if (cure == eposition::I) {
    move_fwd();
  }

  // we need to explicitly check jump points if on border L/R
  if (onL)
    cure = r2e.at({dx, dy, rdirect::L});
  else if (onR)
    cure = r2e.at({dx, dy, rdirect::R});

  assert(e2r.find({dx, dy, cure}) != e2r.end());
  curp = e2r.at({dx, dy, cure});

  while (true) {
    // when the cur rect contains the goal
    if (cur_rect->rid == _goal_rid) {
      jpts_.push_back(_encode_pdir(map_->to_id(curx, cury), dx, dy));
      break;
    }
    switch (curp) {
      // base case
      // on verticle border
      case rdirect::L:
      case rdirect::R:
      {
        bool res = _find_jpt(cur_rect, cure, curx, cury, dx, dy, jpid);
        if (res) {
          jpts_.push_back(_encode_pdir((uint32_t)jpid, dx, dy));
          return;
        }
      }
      // cross the rect
      case rdirect::B:
      {
        // move to the end of the border in this direction
        // and going to move to adjacent rect
        move_fwd();
        curp = rdirect::F;
      }
      // move to adjacent rect
      case rdirect::F:
      {
        int rid = map_->get_rid(curx+dx, cury+dy);
        if (rid == -1)  // no adjacent, dead end
          return;

        // move to adjacent rect in (dx, dy)
        curx += dx, cury += dy;
        cure = r2e.at({dx, dy, rdirect::B});
        cur_rect = &(map_->rects[rid]);
        onL = cur_rect->disLR(rdirect::L, dx, dy, curx, cury) == 0;
        onR = cur_rect->disLR(rdirect::R, dx, dy, curx, cury) == 0;

        // we need to explicitly check jump points if on border L/R
        if (onL)
          cure = r2e.at({dx, dy, rdirect::L});
        else if (onR)
          cure = r2e.at({dx, dy, rdirect::R});
        curp = e2r.at({dx, dy, cure});
      } break;
    }
  }
}

bool rectlocator::_find_jpt(Rect* cur_rect, eposition cure, 
    int curx, int cury, int dx, int dy, int& node_id) {
  int x, y;
  bool res = false;
  vector<int> *jpts;
  vector<int>::iterator it;
  cost_t cost = INF;
  node_id = INF;

  int curid = map_->to_id(curx, cury);

  auto find = [&](eposition e) {
    if (dx + dy > 0) {
      // find min jpt in jptf > node id
      jpts = &(cur_rect->jptf[e]);
      // jpts in jptf stores in ascend order
      it = std::upper_bound(jpts->begin(), jpts->end(), curid);
    }
    else {
      // find max jpt in jptr < node id
      jpts = &(cur_rect->jptr[e]);
      // jpts in jptr stores in descend order
      it = std::upper_bound(jpts->begin(), jpts->end(), curid, greater<int>());
    }
  };

  find(cure);
  if (it != jpts->end()) {
    node_id = *it;
    map_->to_xy(node_id, x, y);
    cost = octile_dist(curx, cury, x, y);
    res = true;
  }
  // if the rect is a line (or dot)
  if ((dx?cur_rect->h: cur_rect->w) == 1) {
    // we also need to check another side
    find(eposition(cure^2));
    if (it != jpts->end()) {
      map_->to_xy(*it, x, y);
      cost_t new_cost = octile_dist(curx, cury, x, y);
      if (new_cost < cost) {
        node_id = *it;
        cost = new_cost;
        res = true;
      }
    }
  }
  // move to adjacent rect
  if (!res) {
    int d2F = cur_rect->disF(dx, dy, curx, cury);
    int cur_mask, nxt_mask;
    curx += dx * d2F;
    cury += dy * d2F;
    if (map_->get_rid(curx+dx, cury+dy) != -1) {
      if (dx) {
        cur_mask = map_->get_maskw(curx, cury);
        nxt_mask = map_->get_maskw(curx+dx, cury+dy);
      }
      else {
        cur_mask = map_->get_maskh(curx, cury);
        nxt_mask = map_->get_maskh(curx+dx, cury+dy);
      }
      if (map_->isjptr[cur_mask][nxt_mask]) {
        node_id = map_->to_id(curx+dx, cury+dy);
        res = true;
      }
    }
  } 
  return res;
}

void rectlocator::_scanDiag(
  int node_id, Rect* rect, int dx, int dy) {
  int curx, cury;
  map_->to_xy(node_id, curx, cury);

  int vertD, horiD, d;

  while (true) {
    // move one step
    if (map_->get_rid(curx+dx, cury) != -1 &&
        map_->get_rid(curx, cury+dy) != -1 &&
        map_->get_rid(curx+dx, cury+dy) != -1) {
      curx += dx, cury += dy;
      node_id += map_->mapw * dy + dx;
      rect = map_->get_rect(curx, cury);
    }
    else break;

    // if reach the rect that contains the goal
    if (rect->rid == _goal_rid) {
      jpts_.push_back(_encode_pdir(map_->to_id(curx, cury), dx, dy));
      break;
    }

    vertD = rect->disF(0, dy, curx, cury);
    horiD = rect->disF(dx, 0, curx, cury);
    d  = min(vertD, horiD);

    int xlb=curx, xub=curx+d*dx, ylb=cury, yub=cury+d*dy;
    if (dx<0) swap(xlb, xub);
    if (dy<0) swap(ylb, yub);
    // try to move interval (xlb, xub) to the front, 
    // find jpt if the path is on L/R border
    if (_scanLR(rect, curx, cury, 0, dy))
      dx>0?xlb++: xub--;
    // try to move interval (ylb, yub) to the front
    // find jpt if the path is on L/R border
    if (_scanLR(rect, curx, cury, dx, 0))
      dy>0?ylb++: yub--;

    curx += dx*d;
    cury += dy*d;
    node_id += map_->mapw * d*dy + d*dx;

    // scan on border
    if (d > 0) {
      if (vertD >= horiD) {
        int node_id = INF;
        if (_find_jpt(rect, dx>0?eposition::E: eposition::W, 
                      curx, cury, 0, dy, node_id)) {
          jpts_.push_back(_encode_pdir((uint32_t)node_id, 0, dy));
          dx>0?xub--: xlb++;
        }
      }
      if (horiD >= vertD) {
        int node_id = INF;
        if (_find_jpt(rect, dy>0?eposition::S: eposition::N, 
                      curx, cury, dx, 0, node_id)) {
          jpts_.push_back(_encode_pdir((uint32_t)node_id, dx, 0));
          dy>0?yub--: ylb++;
        }
      }
    }
    
    if (xlb <= xub) {
      assert(intervals.size() == 0);
      _pushIntervalF(rect, xlb, xub, 0, dy);
      _pushInterval(0, dy);
    }

    if (ylb <= yub) {
      assert(intervals.size() == 0);
      _pushIntervalF(rect, ylb, yub, dx, 0);
      _pushInterval(dx, 0);
    }
  }
}

inline bool rectlocator::_scanLR(Rect* r, int curx, int cury, int dx, int dy) {
    int node_id = INF;
    bool onL = r->disLR(rdirect::L, dx, dy, curx, cury) == 0;
    bool onR = r->disLR(rdirect::R, dx, dy, curx, cury) == 0;
    if ( onL || onR) {
      if (_find_jpt(r, r2e.at({dx, dy, onL?rdirect::L: rdirect::R}), curx, cury, dx, dy, node_id)) {
        jpts_.push_back(_encode_pdir((uint32_t)node_id, dx, dy));
        return true;
      }
    }
    return false;
  };

inline void rectlocator::_pushIntervalF(Rect* curr, int lb, int ub, int dx, int dy) {
    // interval is on F border now
    eposition cure = r2e.at({dx, dy, rdirect::F});
    for (int rid: curr->adj[cure]) {
      // the adjacent rect contains the goal
      Rect* r = &(map_->rects[rid]);

      int rL=0, rR=0, hori, vertL, vertR;
      r->get_range(cure, rL, rR);
      if (rL > ub || rR < lb) continue;
      rL = max(rL, lb); 
      rR = min(rR, ub);

      if (rid == _goal_rid) {
        rL = max(rL, lb);
        rR = min(rR, ub);
        int ax = r->axis(r2e.at({dx, dy, rdirect::B}));
        int x, y;
        if (dx == 0) {
          y = ax;
          if (rL <= _goalx && _goalx <= rR) x=_goalx;
          else if (rL > _goalx) x=rL;
          else x=rR;
        }
        else {
          x = ax;
          if (rL <= _goaly && _goaly <= rR) y=_goaly;
          else if (rL > _goaly) y=rL;
          else y=rR;
        }
        jpts_.push_back(_encode_pdir(map_->to_id(x, y), dx, dy));
        continue;
      }

      hori = r->axis(r2e.at({dx, dy, rdirect::B}));
      vertL = r->axis(dx?eposition::N: eposition::W);
      if (lb <= vertL && vertL <= ub) {
        if (_scanLR(r, dx?hori: vertL, dx?vertL: hori, dx, dy))
          rL++;
      }
      vertR = r->axis(dx?eposition::S: eposition::E);
      if (vertL != vertR && lb <= vertR && vertR <= ub) {
        if (_scanLR(r, dx?hori:vertR, dx?vertR:hori, dx, dy))
          rR--;
      }
      if (rL <= rR)
        intervals.push({rL, rR, r});
    }
}

// interval [lb, ub] in rectangle at B border
void rectlocator::_pushInterval(int dx, int dy) {
  int ax, lb, ub;

  while (!intervals.empty()) {
    Interval c = intervals.front(); intervals.pop();
    if (c.lb > c.ub) continue;
    lb = c.lb, ub = c.ub;
    eposition cure = r2e.at({dx, dy, rdirect::B});
    ax = c.r->axis(cure);
    // is lb on L/R border
    if (_scanLR(c.r, dx?ax: lb, dx?lb: ax, dx, dy)) 
      lb++;
    if (c.lb != c.ub && _scanLR(c.r, dx?ax: ub, dx?ub: ax, dx, dy))
      ub--;
    if (lb <= ub)
      _pushIntervalF(c.r, lb, ub, dx, dy);
  }
}

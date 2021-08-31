#include "rect_jump_point_locator.h"
#include "constants.h"
#include <algorithm>
#include <cstdint>
#include <vector>

typedef warthog::rectscan::rect_jump_point_locator rectlocator;
using namespace std;

inline int manhatan_dis(int x0, int y0, int x1, int y1) {
  return max(x0, x1) - min(x0, x1) + max(y0, y1) - min(y0, y1);
}

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
    switch (curp) {
      // base case
      // on verticle border
      case rdirect::L:
      case rdirect::R:
      {
        bool res = _find_jpt(cur_rect, cure, curx, cury, dx, dy, jpid);
        if (res) {
          jpts_.push_back(jpid);
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
    cost = manhatan_dis(curx, cury, x, y) * ONE;
    res = true;
  }
  // if the rect is a line (or dot)
  if ((dx?cur_rect->h: cur_rect->w) == 1) {
    // we also need to check another side
    find(eposition(cure^2));
    if (it != jpts->end()) {
      map_->to_xy(*it, x, y);
      cost_t new_cost = manhatan_dis(curx, cury, x, y) * ONE;
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

// interval [lb, ub], in rectangle
void rectlocator::_pushInterval(int dx, int dy) {
  int ax, curx, cury, lb, ub;
  eposition cure;

  auto pushLR= [&](Rect*r, int bf, int lr) {
    curx = dx? bf: lr, cury = dx? lr: bf;
    int node_id = INF;
    bool onL = r->disLR(rdirect::L, dx, dy, curx, cury) == 0;
    bool onR = r->disLR(rdirect::R, dx, dy, curx, cury) == 0;
    if ( onL || onR) {
      if (_find_jpt(r, r2e.at({dx, dy, onL?rdirect::L: rdirect::R}), curx, cury, dx, dy, node_id)) {
        jpts_.push_back(node_id);
        return true;
      }
    }
    return false;
  };

  while (!intervals.empty()) {
    Interval c = intervals.front(); intervals.pop();
    if (c.lb > c.ub) continue;
    lb = c.lb, ub = c.ub;
    cure = r2e.at({dx, dy, rdirect::B});
    ax = c.r->axis(cure);
    // is lb on L/R border
    if (pushLR(c.r, ax, lb)) 
      lb++;
    if (c.lb != c.ub && pushLR(c.r, ax, ub))
      ub--;

    // interval is on F border now
    cure = r2e.at({dx, dy, rdirect::F});
    for (int rid: c.r->adj[cure]) {
      Rect* r = &(map_->rects[rid]);
      int rL, rR, hori, vertL, vertR;
      r->get_range(cure, rL, rR);
      if (rL > ub || rR < lb) continue;
      hori = r->axis(r2e.at({dx, dy, rdirect::B}));
      vertL = r->axis(dx?eposition::N: eposition::W);
      if (lb <= vertL && vertL <= ub) {
        if (pushLR(r, hori, vertL))
          rL++;
      }
      vertR = r->axis(dx?eposition::S: eposition::E);
      if (vertL != vertR && lb <= vertR && vertR <= ub) {
        if (pushLR(r, hori, vertR))
          rR--;
      }
      intervals.push({max(rL, lb), min(rR, ub), r});
    }
  }
}

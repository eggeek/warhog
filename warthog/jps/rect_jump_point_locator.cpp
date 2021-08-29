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
  cur_node_id_ = node_id;
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
    cur_node_id_ = map_->to_id(curx, cury);
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
        int rid = map_->get_adj_rect(cur_rect, cure, (cure & 1 ? cury: curx));
        if (rid == -1)  // no adjacent, dead end
          return;

        // if cur node is on L/R border,
        // we need to check whether the next node can be a jump pont
        if (onL || onR) {
          int cur_mask, nxt_mask;
          if (dx) {
            cur_mask = map_->get_maskw(curx, cury);
            nxt_mask = map_->get_maskw(curx+dx, cury+dy);
          }
          else {
            cur_mask = map_->get_maskh(curx, cury);
            nxt_mask = map_->get_maskh(curx+dx, cury+dy);
          }
          if (map_->isjptr[cur_mask][nxt_mask]) {
            jpts_.push_back(map_->to_id(curx+dx, cury+dy));
            return;
          }
        }
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
        // update cur node id
        cur_node_id_ = map_->to_id(curx, cury);
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

  auto find = [&](eposition e) {
    if (dx + dy > 0) {
      // find min jpt in jptf > node id
      jpts = &(cur_rect->jptf[e]);
      // jpts in jptf stores in ascend order
      it = std::upper_bound(jpts->begin(), jpts->end(), cur_node_id_);
    }
    else {
      // find max jpt in jptr < node id
      jpts = &(cur_rect->jptr[e]);
      // jpts in jptr stores in descend order
      it = std::upper_bound(jpts->begin(), jpts->end(), cur_node_id_, greater<int>());
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
  return res;
}

void rectlocator::_scanDiag(
  int& curx, int& cury, Rect* rect, int dx, int dy) {

  int vertD, horiD, d;

  while (true) {
    // vertical scan if possible
    if (rect->disLR(rdirect::L, 0, dy, curx, cury) ||
        rect->disLR(rdirect::R, 0, dy, curx, cury))
      // _scan(curx, cury, rect, jpts, costs, dx, dy);
    vertD = rect->disF(0, dy, curx, cury);
    horiD = rect->disF(dx, 0, curx, cury);
    d  = min(vertD, horiD);
    _scanInterval(curx, curx+dx*d, rect, dx, 0);
    _scanInterval(cury, cury+dy*d, rect, 0, dy);
    curx += dx*d;
    cury += dy*d;

  }
}

// push interval [lb, ub] in a cardinal direction (dx, dy)
// lb and ub are guaranteed not overlap with L/R border of the current rect
// thus no need to check jump point when moving from current rect to the adjcent
// we always push the interval from the F border of cur to B border of the next
void rectlocator::_scanInterval(int lb, int ub, Rect* cur_rect, int dx, int dy) {

  // interval is empty
  if (lb > ub) return;
  eposition cure = r2e.at({dx, dy, rdirect::F});

  int rL, rR, vertL, vertR, hori;

  for (int rid: cur_rect->adj[cure]) {
    Rect* r = &(map_->rects[rid]);
    r->get_range(cure, rL , rR);
    // no overlap
    if (rL > ub || rR < lb) continue;
    hori = r->axis(r2e.at({dx, dy, rdirect::B}));

    // base case 1:
    vertL = r->axis(r2e.at({dx, dy, rdirect::L})); 
    if (lb <= vertL && vertL <= ub) {
      // dx!=0 is a horizontal move
      _scan(map_->to_id(dx?hori:vertL, dx?vertL:hori), r, dx, dy);
    }

    // base case 2:
    vertR = r->axis(r2e.at({dx, dy, rdirect::R}));
    if (vertR != vertL && lb <= vertR && vertR <= ub) {
      _scan(map_->to_id(dx?hori:vertR, dx?vertR:hori), r, dx, dy);
    }
    // remove L/R borders
    rL++, rR--;
    // if they still overlap, push the interval to the next rect
    _scanInterval(max(rL, lb), min(rR, ub), r, dx, dy);
  }
}

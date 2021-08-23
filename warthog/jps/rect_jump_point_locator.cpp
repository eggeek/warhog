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

void rectlocator::_scan(
    int curx, int cury, Rect* cur_rect,
    vector<int> &jpts, vector<cost_t> &costs, int dx, int dy) {

  rdirect curp;
  eposition cure;
  cure = cur_rect->pos(curx, cury);

  int jpid, startx = curx, starty = cury;
  bool onL = false, onR = false;
  cost_t cost;

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
        bool res = _find_jpt(cur_rect, cure, curx, cury, dx, dy, jpid, cost);
        if (res) {
          jpts.push_back(jpid);
          costs.push_back(cost + manhatan_dis(startx, starty, curx, cury) * ONE);
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
            jpts.push_back(map_->to_id(curx+dx, cury+dy));
            costs.push_back(manhatan_dis(startx, starty, curx+dx, cury+dy) * ONE);
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
    int curx, int cury,
    int dx, int dy, int& node_id, cost_t& cost) {
  int x, y;
  bool res = false;
  vector<int> *jpts;
  vector<int>::iterator it;

  node_id = INF, cost = INF;

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

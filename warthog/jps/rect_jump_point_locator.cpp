#include "rect_jump_point_locator.h"
#include "constants.h"
#include <cstdint>

typedef warthog::rectscan::rect_jump_point_locator rectlocator;

void rectlocator::jump_north(vector<uint32_t> &jpts, vector<cost_t> &costs) {
  
  uint32_t jpid, starty = cury;
  cost_t cost;

  // adjust edge id
  if ((int)curx == cur_rect->x) curp = 3;
  else if ((int)curx == cur_rect->x+cur_rect->w-1) curp = 1;

  while (true) {
    switch (curp) {
      // base case
      // on verticle border
      case position::E:
      case position::W:
      {
        bool res = _jump_north(jpid, cost);
        if (res) {
          jpts.push_back(jpid);
          costs.push_back(cost + (starty - cury) * ONE);
          return;
        }
      }
      case position::I:
      // cross the rect
      case position::S:
      {
        // move to the end of the border in this direction
        // and going to move to adjacent rect
        cury = cur_rect->y;
        curp = position::N;
      }
      // move to adjacent rect
      case position::N:
      {
        int rid = map_->get_adj_rect(cur_rect, curp, curx);
        if (rid == -1)  // no adjacent, dead end
          return;
        // move to next rect in north
        cury -= 1;
        curp ^= 2;
        cur_rect = &(map_->rects[rid]);
        // adjust edge id
        if ((int)curx == cur_rect->x) curp = 3;
        else if ((int)curx == cur_rect->x+cur_rect->w-1) curp = 1;
        
        int cur_mask = map_->get_maskh(curx, cury);
        int pre_mask = map_->get_maskh(curx, cury+1);
        if (map_->isjptr[pre_mask][cur_mask]) {
          jpts.push_back(map_->to_id(curx, cury));
          costs.push_back((starty - cury) * ONE);
          return;
        }
      } break;
    }
  }
}

bool rectlocator::_jump_north(uint32_t& node_id, cost_t& cost) {
  uint32_t x, y;
  bool res = false;

  node_id = INF, cost = INF;
  for (int nid: cur_rect->jptr[curp]) {
    map_->to_xy(nid, x, y);
    if (y < cury) {
      node_id = nid;
      cost = (cury - y) * ONE;
      res = true;
      break;
    }
  }
  // rect is a line
  // we also need to check another side
  if (cur_rect->w == 1) {
    for (int nid: cur_rect->jptr[curp^2]) {
      map_->to_xy(nid, x, y);
      cost_t new_cost = (cury - y) * ONE;
      if (y < cury && (new_cost < cost)) {
        node_id = nid;
        cost = new_cost;
        res = true;
        break;
      }
    }
  }
  return res;
};

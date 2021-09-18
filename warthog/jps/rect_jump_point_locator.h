#pragma once

// rect_jump_point_locator.h
//
// @author: shizhe
// @created: 14/09/2021
//

#include "constants.h"
#include "jps.h"
#include "rectmap.h"
#include <cstdint>
#include <limits>
#include <queue>
#include <vector>

namespace warthog {
namespace rectscan {
using namespace std;

inline int octile_dist(int x0, int y0, int x1, int y1) {
  int dx = abs(x0 - x1);
  int dy = abs(y0 - y1);
  int diag = min(dx, dy);
  return diag * warthog::ROOT_TWO + (dx + dy - (diag<<1)) * warthog::ONE;
}

const int INF = numeric_limits<int>::max();

class rect_jump_point_locator
{
	public: 

		rect_jump_point_locator(rectscan::RectMap* map): map_(map) {
      cur_node_id_ = INF;
      cur_goal_id_ = INF;
      _curx = _cury = _goal_rid = INF;
      jpts_.reserve(1<<7);
      costs_.reserve(1<<7);
    };
    int scan_cnt = 0;

		void
		jump(jps::direction d, int node_id, int goal_id, Rect* rect) {

      assert(intervals_h.empty());
      assert(intervals_v.empty());
      cur_node_id_ = node_id;
      map_->to_xy(cur_node_id_, _curx, _cury);
      if (cur_goal_id_ != goal_id) {
        cur_goal_id_ = goal_id;
        map_->to_xy(cur_goal_id_, _goalx, _goaly);
        _goal_rid = map_->get_rid(_goalx, _goaly);
      }
      if (rect->rid == _goal_rid) {
        jpts_.push_back(_encode_pdir(cur_goal_id_, 0, 0));
      }
      else {
        switch(d) {
          case jps::NORTH:
            _scan(node_id, rect, 0, -1);
            break;
          case jps::SOUTH:
            _scan(node_id, rect, 0, 1);
            break;
          case jps::EAST:
            _scan(node_id, rect, 1, 0);
            break;
          case jps::WEST:
            _scan(node_id, rect, -1, 0);
            break;
          case jps::NORTHEAST:
            _scanDiag(node_id, rect, 1, -1);
            break;
          case jps::NORTHWEST:
            _scanDiag(node_id, rect, -1, -1);
            break;
          case jps::SOUTHEAST:
            _scanDiag(node_id, rect, 1, 1);
            break;
          case jps::SOUTHWEST:
            _scanDiag(node_id, rect, -1, 1);
            break;
          default:
            break;
        }
      }
      costs_.resize(jpts_.size());
      for (int i=0; i<(int)jpts_.size(); i++) {
        int x, y;
        map_->to_xy(jpts_[i] & IDMASK, x, y);
        costs_[i] = octile_dist(x, y, _curx, _cury);
      }
      assert(intervals_h.empty());
      assert(intervals_v.empty());
    }

		int mem() { return sizeof(this); }

    void scanInterval(int lb, int ub, Rect* cur_rect, int dx, int dy) {

      queue<Interval> &intervals = dx? intervals_v: intervals_h;
      cur_rect->set_mark(lb, r2e.at({dx, dy, rdirect::B}), ub);
      cur_rect->set_mark(ub, r2e.at({dx, dy, rdirect::B}), lb);
      intervals.push({lb, ub, cur_rect});
      _pushInterval(intervals, dx, dy);
    }

    vector<uint32_t>& get_jpts() { return jpts_; }
    void set_jpts(vector<uint32_t> vi) { jpts_ = vector<uint32_t>(vi.begin(), vi.end()); }

    vector<cost_t>& get_costs() { return costs_; }
    void set_costs(vector<cost_t> vc) { costs_ = vector<cost_t>(vc.begin(), vc.end());}

    inline void reset() {
      jpts_.clear();
      costs_.clear();
    }

	private:
		int cur_goal_id_;
		int cur_node_id_;
    int _curx, _cury, _goalx, _goaly, _goal_rid;
    RectMap* map_;
    vector<uint32_t> jpts_;
    vector<cost_t> costs_;

    struct Interval {
      int lb, ub;
      Rect* r;
    };
    queue<Interval> intervals_h, intervals_v;

    bool _find_jpt(Rect* cur_rect, eposition cure, int curx, int cury, 
        int dx, int dy, int& node_id);

    void _scanDiag(int node_id, Rect* cur_rect, int dx, int dy); 

    bool _scanLR(Rect* r, int curx, int cury, int dx, int dy);

    void _pushIntervalF(queue<Interval>& intervals, Rect* r, Rect* nxt, int& lb, int& ub, int dx, int dy);
    void _pushInterval(queue<Interval>& intervals, int dx, int dy);

    void _scan(int node_id, Rect* cur_rect, int dx, int dy);

    inline uint32_t _encode_pdir(uint32_t node_id, int dx, int dy) {
      jps::direction d = jps::direction::NONE;
      switch (dx) {
        case -1: d = jps::direction::WEST; break;
        case 1: d = jps::direction::EAST; break;
        default:
          switch (dy) {
            case -1: d = jps::direction::NORTH; break;
            case 1: d = jps::direction::SOUTH; break;
            default:
              break;
          }
      }
		  *(((uint8_t*)&node_id)+3) = d;
      return node_id;
    }
};

}}


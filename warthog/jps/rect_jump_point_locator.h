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
#include <queue>
#include <vector>

namespace warthog {
namespace rectscan {
using namespace std;

class rect_jump_point_locator
{
	public: 

		rect_jump_point_locator(rectscan::RectMap* map): map_(map) {
      cur_node_id_ = warthog::INF;
      cur_goal_id_ = warthog::INF;
    };
    int scan_cnt;

		void
		jump(jps::direction d, int node_id, int goal_id, Rect* rect) {

      switch(d) {
        case jps::NORTH:
          _scan(node_id, rect, 0, -1);
          // jump_north(jpts, costs);
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

		int mem() { return sizeof(this); }

    void scanInterval(int lb, int ub, Rect* cur_rect, int dx, int dy) {

      // eposition cure = r2e.at({dx, dy, rdirect::F});
      // int ax = cur_rect->axis(cure);
      // _scan(map_->to_id(dx?ax: lb, dx?lb: ax), cur_rect, dx, dy);
      // if (ub != lb) {
      //   _scan(map_->to_id(dx?ax: ub, dx?ub: ax), cur_rect, dx, dy);
      //   _scanInterval(lb+1, ub-1, cur_rect, dx, dy);
      // }
      while (!intervals.empty()) intervals.pop();
      intervals.push({lb, ub, cur_rect});
      _pushInterval(dx, dy);
    }

    vector<int>& get_jpts() { return jpts_; }
    void set_jpts(vector<int> vi) { jpts_ = vector<int>(vi.begin(), vi.end()); }

    vector<cost_t>& get_costs() { return costs_; }
    void set_costs(vector<cost_t> vc) { costs_ = vector<cost_t>(vc.begin(), vc.end());}

	private:
		int cur_goal_id_;
		int cur_node_id_;
    RectMap* map_;
    vector<int> jpts_;
    vector<cost_t> costs_;

    struct Interval {
      int lb, ub;
      Rect* r;
    };
    queue<Interval> intervals;

    bool _find_jpt(Rect* cur_rect, eposition cure, int curx, int cury, 
        int dx, int dy, int& node_id);

    void _scanDiag(int node_id, Rect* cur_rect, int dx, int dy); 

    bool _scanLR(Rect* r, int curx, int cury, int dx, int dy);

    void _pushIntervalF(Rect* r, int lb, int ub, int dx, int dy);

    void _pushInterval(int dx, int dy);

    void _scan(int node_id, Rect* cur_rect, int dx, int dy);

};

}}


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
		jump(jps::direction d, int node_id, int goal_id, Rect* rect,
				vector<int>& jpts,
				vector<cost_t>& costs) {

	    if(goal_id != cur_goal_id_) 
        cur_goal_id_ = goal_id;
      if (node_id != cur_node_id_)
        cur_node_id_ = node_id;
      int curx, cury;
      map_->to_xy(cur_node_id_, curx, cury);

      switch(d) {
        case jps::NORTH:
          _scan(curx, cury, rect, jpts, costs, 0, -1);
          // jump_north(jpts, costs);
          break;
        case jps::SOUTH:
          _scan(curx, cury, rect, jpts, costs, 0, 1);
          break;
        case jps::EAST:
          _scan(curx, cury, rect, jpts, costs, 1, 0);
          break;
        case jps::WEST:
          _scan(curx, cury, rect, jpts, costs, -1, 0);
          break;
        // case jps::NORTHEAST:
        //   scanDiag(jpts, costs, 1, -1);
        //   break;
        // case jps::NORTHWEST:
        //   scanDiag(jpts, costs, -1, -1);
        //   break;
        // case jps::SOUTHEAST:
        //   scanDiag(jpts, costs, 1, 1);
        //   break;
        // case jps::SOUTHWEST:
        //   scanDiag(jpts, costs, -1, 1);
        //   break;
        default:
          break;
      }
    }

		int mem() { return sizeof(this); }


	private:
		int cur_goal_id_;
		int cur_node_id_;
    RectMap* map_;

    void _scan(
        int curx, int cury, Rect* cur_rect,
        vector<int> &jpts, vector<cost_t> &costs, int dx, int dy);

    bool _find_jpt(
        Rect* cur_rect, eposition cure, int curx, int cury, 
        int dx, int dy, int& node_id, cost_t& cost);
};

}}


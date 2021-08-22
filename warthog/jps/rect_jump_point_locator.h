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
      map_->to_xy(cur_node_id_, curx, cury);
      cur_rect = rect;
      cure = cur_rect->pos(curx, cury);

      switch(d) {
        case jps::NORTH:
          scan(jpts, costs, 0, -1);
          // jump_north(jpts, costs);
          break;
        case jps::SOUTH:
          scan(jpts, costs, 0, 1);
          break;
        case jps::EAST:
          scan(jpts, costs, 1, 0);
          break;
        case jps::WEST:
          scan(jpts, costs, -1, 0);
          break;
        // case jps::NORTHEAST:
        //   jump_northeast(jpts, costs);
        //   break;
        // case jps::NORTHWEST:
        //   jump_northwest(jpts, costs);
        //   break;
        // case jps::SOUTHEAST:
        //   jump_southeast(jpts, costs);
        //   break;
        // case jps::SOUTHWEST:
        //   jump_southwest(jpts, costs);
        //   break;
        default:
          break;
      }
    }

		int mem() { return sizeof(this); }

		void jump_north( vector<int>& jpts, vector<cost_t>& costs);
    void scan(vector<int> &jpts, vector<cost_t> &costs, int dx, int dy);
		// void jump_south( vector<int>& jpts, vector<cost_t>& costs);
		// void jump_east( vector<int>& jpts, vector<cost_t>& costs);
		// void jump_west( vector<int>& jpts, vector<cost_t>& costs);
		// void jump_northeast( vector<int>& jpts, vector<cost_t>& costs);
		// void jump_northwest( vector<int>& jpts, vector<cost_t>& costs);
		// void jump_southeast( vector<int>& jpts, vector<cost_t>& costs);
		// void jump_southwest( vector<int>& jpts, vector<cost_t>& costs);

	private:
		int cur_goal_id_;
		int cur_node_id_;
		int curx, cury; 
    eposition cure;
    rdirect curp;
    RectMap* map_;
    Rect* cur_rect;

    bool _jump_north(int& node_id, cost_t& cost);
    bool _find_jpt(int dx, int dy, int& node_id, cost_t& cost);
};

}}


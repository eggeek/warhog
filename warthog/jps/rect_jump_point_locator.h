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
    uint32_t scan_cnt;

		void
		jump(jps::direction d, uint32_t node_id, uint32_t goal_id, Rect* rect,
				vector<uint32_t>& jpts,
				vector<cost_t>& costs) {

	    if(goal_id != cur_goal_id_) 
        cur_goal_id_ = goal_id;
      if (node_id != cur_node_id_)
        cur_node_id_ = node_id;
      map_->to_xy(cur_node_id_, curx, cury);
      cur_rect = rect;
      curp = cur_rect->pos(curx, cury);

      switch(d) {
        case jps::NORTH:
          jump_north(jpts, costs);
          break;
        // case jps::SOUTH:
        //   jump_south(jpts, costs);
        //   break;
        // case jps::EAST:
        //   jump_east(jpts, costs);
        //   break;
        // case jps::WEST:
        //   jump_west(jpts, costs);
        //   break;
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

		uint32_t mem() { return sizeof(this); }

		void jump_north( vector<uint32_t>& jpts, vector<cost_t>& costs);
		// void jump_south( vector<uint32_t>& jpts, vector<cost_t>& costs);
		// void jump_east( vector<uint32_t>& jpts, vector<cost_t>& costs);
		// void jump_west( vector<uint32_t>& jpts, vector<cost_t>& costs);
		// void jump_northeast( vector<uint32_t>& jpts, vector<cost_t>& costs);
		// void jump_northwest( vector<uint32_t>& jpts, vector<cost_t>& costs);
		// void jump_southeast( vector<uint32_t>& jpts, vector<cost_t>& costs);
		// void jump_southwest( vector<uint32_t>& jpts, vector<cost_t>& costs);

	private:
		uint32_t cur_goal_id_;
		uint32_t cur_node_id_;
		uint32_t curx, cury, curp;
    RectMap* map_;
    Rect* cur_rect;

    bool _jump_north(uint32_t& node_id, cost_t& cost);
};

}}


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
		jump(jps::direction d, int node_id, int goal_id, Rect* rect,
				vector<int>& jpts,
				vector<cost_t>& costs) {

      switch(d) {
        case jps::NORTH:
          _scan(node_id, rect, jpts, costs, 0, -1);
          // jump_north(jpts, costs);
          break;
        case jps::SOUTH:
          _scan(node_id, rect, jpts, costs, 0, 1);
          break;
        case jps::EAST:
          _scan(node_id, rect, jpts, costs, 1, 0);
          break;
        case jps::WEST:
          _scan(node_id, rect, jpts, costs, -1, 0);
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

    void scanInterval(int lb, int ub, Rect* cur_rect, 
        vector<int>& jpts, vector<cost_t>& costs, int dx, int dy) {

      eposition cure = r2e.at({dx, dy, rdirect::F});
      int ax = cur_rect->axis(cure);
      _scan(map_->to_id(dx?ax: lb, dx?lb: ax), cur_rect, jpts, costs, dx, dy);
      if (ub != lb) {
        _scan(map_->to_id(dx?ax: ub, dx?ub: ax), cur_rect, jpts, costs, dx, dy);
        _scanInterval(lb+1, ub-1, cur_rect, jpts, costs, dx, dy);
      }
    }

	private:
		int cur_goal_id_;
		int cur_node_id_;
    RectMap* map_;

    bool _find_jpt(
        Rect* cur_rect, eposition cure, int curx, int cury, 
        int dx, int dy, int& node_id, cost_t& cost);

    void _scanDiag(
        int& curx, int& cury, Rect* cur_rect, 
        vector<int> &jpts, vector<cost_t> &costs, int dx, int dy); 

    void _scanInterval(
        int lb, int ub, Rect* cur_rect,
        vector<int> &jpts, vector<cost_t> &costs, int dx, int dy);

    void _scan(
        int node_id, Rect* cur_rect,
        vector<int> &jpts, vector<cost_t> &costs, int dx, int dy);


};

}}


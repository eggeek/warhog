#pragma once
// jps_heuristic.h
//
// @author: shizhe
// @created: 11/06/2021
//

#include "constants.h"
#include "helpers.h"
#include "mapper.h"

#include <cassert>
#include <cstdlib>

namespace warthog
{

class jps_heuristic
{
  typedef jps::direction dir;

	public:
		jps_heuristic(Mapper* mapper_) 
        : mapper(mapper_) { }
		~jps_heuristic() { }

		inline warthog::cost_t
		h(int32_t x, int32_t y, int32_t x2, int32_t y2)
		{
            // NB: precision loss when warthog::cost_t is an integer
			double dx = abs(x-x2);
			double dy = abs(y-y2);
			if(dx < dy)
			{
				return dx * warthog::ROOT_TWO + (dy - dx) * warthog::ONE;
			}
			return dy * warthog::ROOT_TWO + (dx - dy) * warthog::ONE;
		}

		inline warthog::cost_t
		h(unsigned int id, unsigned int id2)
		{
			unsigned int x, x2;
			unsigned int y, y2;
      warthog::helpers::index_to_xy(id, mapper->mapw, x, y);
      warthog::helpers::index_to_xy(id2, mapper->mapw, x2, y2);
      // mapper->xy(id, x, y);
      // mapper->xy(id, x2, y2);
			return this->h(x, y, x2, y2);
		}

    inline warthog::cost_t
    h(unsigned int id) {
      unsigned int x, y;
      warthog::helpers::index_to_xy(id, mapper->mapw, x, y);
      return this->h(x, y, tx, ty);
    }

    inline warthog::cost_t
    octile_dist(int dX, int dY) {
      double dx = abs(dX), dy = abs(dY);
      if (dx < dy) return dx * warthog::ROOT_TWO + (dy - dx) * warthog::ONE;
      else return dy * warthog::ROOT_TWO + (dx - dy) * warthog::ONE;
    }

    inline warthog::cost_t
    h_(int d, unsigned int x, unsigned int y) {
      int vx = warthog::dx[d], vy = warthog::dy[d];
      int dx = tx - x, dy = ty - y;
      
      // cardinal
      if (vx*vy == 0) {
        // swap coordinate when dir is vertical
        if (vx == 0) { swap(dx, dy); swap(vx, vy); }
        // flip dir vector to positive
        if (vx < 0) { vx *= -1; dx *= -1; }

        // target on axis
        if (dx*dy == 0) {
          /*
          *        |
          *        3
          *        |
          *    -2--c→-1-→
          *        |    x
          *        4
          *        |
          *       y↓
          *
          * Assume there an obstacle near 'c' to form jump points 
          *
          * case 2: from 'c' to '*' then using octile distance
          *        *←←
          *         #↑
          *      .t.c→
          *
          * case 3/4: from 'c' to '*' then using octile distance
          *        .
          *        t
          *        .*
          *        #↑
          *        c→
          */

          if (dx > 0) { return dx*ONE; }                          // case 1
          if (dx < 0) { return 5*ONE + octile_dist(dx+1, dy+2); } // case 2
          if (dy > 0) { return 3*ONE + octile_dist(dx+1, dy-2); } // case 3
          if (dy < 0) { return 3*ONE + octile_dist(dx+1, dy+2); } // case 4
          return 0;
        }
        // target in quadrants
        else {
          /*
           *         |
           *    t4  +←← t1
           *    t4'  #↑      x
           *    -----c*------→
           *    t3'  #↓
           *    t3  +←← t2
           *         |
           *        y↓
           *
           * case 1/2: from c to *, then to t1/t2 using octile distance
           * case 3/4: from c to +, then to t3/t4 using octile distance
           */
          if (dx > 0) { return 1*ONE + octile_dist(dx-1, dy);}         // case 1/2
          else {
            if (dy < -1) return 3*ONE + octile_dist(dx-1, dy+2);       // case 3
            else if (dy == -1) return 5*ONE + octile_dist(dx+1, dy+2); // case 3'
            else if (dy > 1) return 3*ONE + octile_dist(dx-1, dy-2);   // case 4
            else return 5*ONE + octile_dist(dx+1, dy-2);               // case 4'
          } 
        }
      }
      // diagonal
      else {
        /*       |  1
         *       |🡥
         *   ----c----
         *       |
         *       |
         *
         * the only case is (vx,vy) and t are in same quadrant
         * otherwise we need a jump point by cardinal move which 
         * has been covered by the previous case
         */ 
        if (dx*vx>0 && dy*vy>0) return octile_dist(dx, dy);
        else return warthog::INF;
        return 0;
      }
    }

    inline warthog::cost_t
    h(dir fromd, unsigned int id) {
      if (id == tid) return 0;
      unsigned int x, y;
      warthog::helpers::index_to_xy(id, mapper->mapw, x, y);
      uint32_t suc = mapper->get_successors(warthog::jps::d2i(fromd), id);
      cost_t res = mapper->mapw * mapper->maph * ONE;

      while (suc) {
        uint32_t lb = warthog::helpers::LOWB(suc);
        res = min(res, h_(warthog::jps::d2i(lb), x, y));
        suc -= lb;
      }
      return res;
    }

    void set_target(unsigned int tid) {
      this->tid = tid;
      warthog::helpers::index_to_xy(tid, mapper->mapw, tx, ty);
    }

	private:
    Mapper* mapper;
    unsigned int tid, tx, ty;
};

}

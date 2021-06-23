#ifndef WARTHOG_HELPERS_H
#define WARTHOG_HELPERS_H

// helpers.h
//
// Helper functions that don't fit anywhere else.
//
// @author: dharabor
// @created: 21/08/2012
//

#include <cstddef>
#include <cstdint>
namespace warthog
{
namespace helpers
{

// convert id into x/y coordinates on a grid of width 'mapwidth'
inline void
index_to_xy(unsigned int id, unsigned int mapwidth, 
		unsigned int& x, unsigned int& y)
{	
	y = id / mapwidth;
	x = id % mapwidth;
//	x = id;
//	y = 0;
//	for( ; x >= mapwidth ; x -= mapwidth)
//	{
//		y++; 
//	}
}


inline uint32_t LOWB(const uint32_t& mask) { return mask & (-mask); };

template<typename T> size_t sgn(const T& v1, const T& v2) {
  if (v1 < v2) return -1;
  else if (v1 == v2) return 0;
  else return 1;
}

}
}

#endif



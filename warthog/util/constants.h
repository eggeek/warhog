#ifndef WARTHOG_CONSTANTS_H
#define WARTHOG_CONSTANTS_H

// constants.h
//
// @author: dharabor
// @created: 01/08/2012
//

#include <cfloat>
#include <cmath>
#include <climits>
#include <limits>
#include <stdint.h>

namespace warthog
{
	// each node in a weighted grid map uses sizeof(dbword) memory.
	// in a uniform-cost grid map each dbword is a contiguous set
	// of nodes s.t. every bit represents a node.
	typedef unsigned char dbword;
	typedef uint32_t cost_t;

	// gridmap constants
	static const uint32_t DBWORD_BITS = sizeof(warthog::dbword)*8;
	static const uint32_t DBWORD_BITS_MASK = (warthog::DBWORD_BITS-1);
	static const uint32_t LOG2_DBWORD_BITS = ceil(log10(warthog::DBWORD_BITS) / log10(2));

	// search and sort constants
	static const double DBL_ONE = 1.0f;
	static const double DBL_TWO = 2.0f;
	static const double DBL_ROOT_TWO = 1.414213562f;
	static const double DBL_ONE_OVER_TWO = 0.5;
	static const double DBL_ONE_OVER_ROOT_TWO = 1.0/DBL_ROOT_TWO;//0.707106781f;
	static const double DBL_ROOT_TWO_OVER_FOUR = DBL_ROOT_TWO*0.25;
	static const warthog::cost_t ONE = 10000;
	static const warthog::cost_t TWO = 20000;
	static const warthog::cost_t ROOT_TWO = DBL_ROOT_TWO * ONE;
	static const warthog::cost_t ONE_OVER_TWO = DBL_ONE_OVER_TWO * ONE;
	static const warthog::cost_t ONE_OVER_ROOT_TWO = DBL_ONE_OVER_ROOT_TWO * ONE;
	static const warthog::cost_t ROOT_TWO_OVER_FOUR = DBL_ROOT_TWO * ONE;
	static const warthog::cost_t INF = 0xffffffff;
  static const uint32_t INFID = std::numeric_limits<uint32_t>::max();
  static const uint32_t ALLMOVE = 0x7FFF;
  static const uint32_t INVALID = 1<<15;
	static const uint32_t IDMASK  = (1 << 24)-1;
  static const int MAXSIDE = 1<<15;

	// hashing constants
	static const uint32_t FNV32_offset_basis = 2166136261;
	static const uint32_t FNV32_prime = 16777619;

  // graph mapper              0, 1, 2, 3,  4,  5, 6, 7
  static const int16_t dx[] = {0, 0, 1, -1, 1, -1, 1, -1};
  static const int16_t dy[] = {-1, 1, 0, 0, -1, -1, 1, 1};
}

#endif



#ifndef WARTHOG_JPS_H
#define WARTHOG_JPS_H

// jps.h
//
// This file contains the namespace for common definitions
// required by the various classes that use Jump Points.
// Note that the operations defined here assume corner
// cutting is not allowed. This change requires some slight 
// modification to the basic Jump Point Search method. 
// For details see:
// [D Harabor and A Grastien, The JPS+ Pathfinding System, SoCS, 2012]
//
// @author: dharabor
// @created: 04/09/2012
//

#include "stdint.h"

namespace warthog
{

namespace jps
{

typedef enum
{
	NONE = 0,
	NORTH = 1,
	SOUTH = 2,
	EAST = 4,
	WEST = 8,
	NORTHEAST = 16,
	NORTHWEST = 32, 
	SOUTHEAST = 64,
	SOUTHWEST = 128
} direction;

inline uint8_t d2i(const direction& d) {
  switch (d) {
    case NORTH: return 0;
    case SOUTH: return 1;
    case EAST: return 2;
    case WEST: return 3;
    case NORTHEAST: return 4;
    case NORTHWEST: return 5;
    case SOUTHEAST: return 6;
    case SOUTHWEST: return 7;
    default: return 8;
  };
}

inline uint8_t d2i(const uint32_t& d) {
  switch (d) {
    case 1: return 0;
    case 2: return 1;
    case 4: return 2;
    case 8: return 3;
    case 16: return 4;
    case 32: return 5;
    case 64: return 6;
    case 128: return 7;
    default: return 8;
  };
}

// Computes the set of "forced" directions in which to search for jump points
// from a given location (x, y). 
// A neighbour is forced if it cannot be proven that there is at least one 
// alternative optimal path that does not pass through the node (x, y).
uint32_t
compute_forced(warthog::jps::direction d, uint32_t tiles);

// Computes the set of "natural" neighbours for a given location
// (x, y).
uint32_t 
compute_natural(warthog::jps::direction d, uint32_t tiles);

// Computes all successors (forced \union natural) of a node (x, y).
// This function is specialised for uniform cost grid maps.
//
// @param d: the direction of travel used to reach (x, y)
// @param tiles: the 3x3 square of cells having (x, y) at its centre.
//
// @return an integer representing the set of forced directions.
// Each of the first 8 bits of the returned value, when set, correspond to a
// direction, as defined in warthog::jps::direction
inline uint32_t
compute_successors(warthog::jps::direction d, uint32_t tiles)
{
	return warthog::jps::compute_forced(d, tiles) |
	   	warthog::jps::compute_natural(d, tiles);
}


}
}

#endif


#include "jps_expansion_policy_prune.h"
#include "constants.h"
#include <cassert>

warthog::jps_expansion_policy_prune::jps_expansion_policy_prune(warthog::gridmap* map)
{
	map_ = map;
	nodepool_ = new warthog::blocklist(map->height(), map->width());
	jpl_ = new warthog::online_jump_point_locator_prune(map);
  jpruner.init(map_->height() * map_->width());
  jpl_->jpruner = &jpruner;
	reset();
}

warthog::jps_expansion_policy_prune::~jps_expansion_policy_prune()
{
	delete jpl_;
	delete nodepool_;
}

void 
warthog::jps_expansion_policy_prune::expand(
		warthog::search_node* current, warthog::problem_instance* problem)
{
	reset();

	// compute the direction of travel used to reach the current node.
	warthog::jps::direction dir_c =
	   	this->compute_direction(current->get_parent(), current);

	// get the tiles around the current node c
	uint32_t c_tiles;
	uint32_t current_id = current->get_id();
	map_->get_neighbours(current_id, (uint8_t*)&c_tiles);

	// look for jump points in the direction of each natural 
	// and forced neighbour
	uint32_t succ_dirs = warthog::jps::compute_successors(dir_c, c_tiles);
	uint32_t goal_id = problem->get_goal();

  jpruner.startExpand();
	for(uint32_t i = 0; i < 8; i++)
	{
		warthog::jps::direction d = (warthog::jps::direction) (1 << i);
		if(succ_dirs & d)
		{
			warthog::cost_t jumpcost;
			uint32_t succ_id;
      jpruner.startJump(current);
			jpl_->jump(d, current_id, goal_id, succ_id, jumpcost);

      assert(&jpruner == jpl_->jpruner);
      assert(jpruner.is_forced() || jpruner.is_pruned() || jpruner.is_deadend());
      if (jpruner.is_deadend()) assert(succ_id == warthog::INF);
      if (succ_id != warthog::INF) {
        assert(jpruner.is_forced() || jpruner.is_pruned());
      }
      if (succ_id != warthog::INF && jpruner.is_pruned()) {
        assert(i <= 3);
      }

			if(succ_id != warthog::INF && jpruner.is_forced())
			{
				neighbours_[num_neighbours_] = nodepool_->generate(succ_id);
				costs_[num_neighbours_] = jumpcost;
				// move terminator character as we go
				neighbours_[++num_neighbours_] = 0;
			}
		}
	}
}


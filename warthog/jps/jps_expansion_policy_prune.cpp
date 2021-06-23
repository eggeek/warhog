#include "jps_expansion_policy_prune.h"

warthog::jps_expansion_policy_prune::jps_expansion_policy_prune(warthog::gridmap* map)
{
	map_ = map;
	nodepool_ = new warthog::blocklist(map->height(), map->width());
	jpl_ = new warthog::online_jump_point_locator_prune(map);
  mapper = new warthog::Mapper(map_);
  jpruner.init(map->height() * map->width());
  jpruner.mapper = mapper;
	reset();
}

warthog::jps_expansion_policy_prune::~jps_expansion_policy_prune()
{
	delete jpl_;
	delete nodepool_;
  delete mapper;
}

void 
warthog::jps_expansion_policy_prune::expand(
		warthog::search_node* current, warthog::problem_instance* problem)
{
	reset();

  jpruner.pi = problem;
  uint32_t id_mask = (1 << 24)-1;
  uint32_t searchid = problem->get_searchid();
	// compute the direction of travel used to reach the current node.
	warthog::jps::direction dir_c =
	   	this->compute_direction(current->get_parent(), current);


	// get the tiles around the current node c
	uint32_t c_tiles;
	uint32_t current_id = current->get_id();
  map_->get_neighbours(current_id, (uint8_t*)&c_tiles);
  // mapper->get_neighbours(current_id, c_tiles);

  // pruning
  // uint8_t dint = warthog::jps::d2i(dir_c);
  // uint32_t succ_dirs = jpruner.get_pruned_suc(current, c_tiles);
  // succ_dirs &= mapper->get_successors(dint, current_id);
	// look for jump points in the direction of each natural 
	// and forced neighbour
  // uint32_t succ_dirs = mapper->get_successors(dint, current_id);
  uint32_t succ_dirs = jps::compute_successors(dir_c, c_tiles);
	uint32_t goal_id = problem->get_goal();
	for(uint32_t i = 0; i < 8; i++)
	{
		warthog::jps::direction d = (warthog::jps::direction) (1 << i);
		if(succ_dirs & d)
		{
			warthog::cost_t jumpcost;
			uint32_t succ_id;
			jpl_->jump(d, current_id, goal_id, succ_id, jumpcost);

			if(succ_id != warthog::INF)
			{
		    warthog::search_node* mynode = nodepool_->generate(succ_id & id_mask);
        if (mynode->get_searchid() != searchid) {
          mynode->reset(searchid);
        }
        mynode->set_pdir(d);
				neighbours_[num_neighbours_] = mynode;
				costs_[num_neighbours_] = jumpcost;
        // jpruner.update(succ_id, jumpcost + current->get_g(), d);
				// move terminator character as we go
				neighbours_[++num_neighbours_] = 0;
			}
		}
	}
}


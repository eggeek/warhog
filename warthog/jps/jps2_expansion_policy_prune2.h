#pragma once
// jps2_expansion_policy_prune2.h

// @author: shizhe
// @created: 30/06/2021

#include "blocklist.h"
#include "gridmap.h"
#include "helpers.h"
#include "jps.h"
#include "online_jump_point_locator2_prune2.h"
#include "problem_instance.h"
#include "search_node.h"
#include "online_jps_pruner2.h"
#include "mapper.h"

#include "stdint.h"

namespace warthog
{

class jps2_expansion_policy_prune2
{
	public:
		jps2_expansion_policy_prune2(warthog::gridmap* map);
		~jps2_expansion_policy_prune2();

		// create a warthog::search_node object from a state description
		// (in this case, an id)
		inline warthog::search_node*
		generate(uint32_t node_id)
		{
			return nodepool_->generate(node_id);
		}


		// reset the policy and discard all generated nodes
		inline void
		clear()
		{
			reset();
			nodepool_->clear();
		}

    inline blocklist* get_nodepool() { return nodepool_; }

		void 
		expand(warthog::search_node*, warthog::problem_instance*);

		inline void
		first(warthog::search_node*& ret, warthog::cost_t& cost)
		{
			which_ = 0;
			ret = neighbours_[which_];
			cost = costs_[which_];
		}

		inline bool
		has_next()
		{
			if((which_+1) < num_neighbours_) { return true; }
			return false;
		}

		inline void
		n(warthog::search_node*& ret, warthog::cost_t& cost)
		{
			ret = neighbours_[which_];
			cost = costs_[which_];
		}

		inline void
		next(warthog::search_node*& ret, warthog::cost_t& cost)
		{
			if(which_ < num_neighbours_)
			{
				which_++;
			}
			ret = neighbours_[which_];
			cost = costs_[which_];
		}

		inline uint32_t
		mem()
		{
			return sizeof(*this) + map_->mem() + nodepool_->mem() + jpl_->mem();
		}

		uint32_t 
		mapwidth()
		{
			return map_->width();
		}

    warthog::online_jump_point_locator2_prune2* get_locator() {
      return this->jpl_;
    }

    void init_tables() {
      this->jpl_->init_tables();
    }

	private:
		warthog::gridmap* map_;
		warthog::blocklist* nodepool_;
    warthog::Mapper* mapper;
		online_jump_point_locator2_prune2* jpl_;
		uint32_t which_;
		uint32_t num_neighbours_;
		std::vector<warthog::search_node*> neighbours_;
		std::vector<warthog::cost_t> costs_;
		std::vector<uint32_t> jp_ids_;
    online_jps_pruner2 jpruner;

		inline void
		reset()
		{
			which_ = 0;
			num_neighbours_ = 0;
			neighbours_.clear();
			costs_.clear();
			jp_ids_.clear();
		}

};

}

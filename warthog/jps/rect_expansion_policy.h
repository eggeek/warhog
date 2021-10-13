#pragma once
// rect_expansion_policy.h
//
// similar to jps2, but scanning is based on rectangle mesh
//
// @author: shizhe
// @created: 14/08/2021

#include "rectmap.h"
#include "blocklist.h"
#include "jps.h"
#include "problem_instance.h"
#include "rect_jump_point_locator.h"
#include "search_node.h"

#include "stdint.h"

namespace warthog
{
namespace rectscan {

class rect_expansion_policy 
{
	public:
		rect_expansion_policy(RectMap* map);
		~rect_expansion_policy();

		// create a search_node object from a state description
		// (in this case, an id)
		inline search_node*
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


		void 
		expand(search_node*, problem_instance*);

		inline void
		first(search_node*& ret, cost_t& cost)
		{
			which_ = 0;
			ret = neighbours_[which_];
			cost = jpl_->get_costs()[which_];
		}

		inline bool
		has_next()
		{
			if((which_+1) < num_neighbours_) { return true; }
			return false;
		}

		inline void
		n(search_node*& ret, cost_t& cost)
		{
			ret = neighbours_[which_];
			cost = jpl_->get_costs()[which_];
		}

		inline void
		next(search_node*& ret, cost_t& cost)
		{
			if(which_ < num_neighbours_)
			{
				which_++;
			}
			ret = neighbours_[which_];
			cost = jpl_->get_costs()[which_];
		}

		inline uint32_t
		mem()
		{
			return sizeof(*this) + map_->mem() + nodepool_->mem() + jpl_->mem();
		}

		uint32_t 
		mapwidth()
		{
			return map_->mapw;
		}

    rect_jump_point_locator* get_locator() {
      return this->jpl_;
    }

	private:
		rectscan::RectMap* map_;
		blocklist* nodepool_;
		rect_jump_point_locator* jpl_;
		uint32_t which_;
		uint32_t num_neighbours_;
		vector<search_node*> neighbours_;

		inline void
		reset()
		{
			which_ = 0;
			num_neighbours_ = 0;
			neighbours_.clear();
      jpl_->reset();
		}

};

}}

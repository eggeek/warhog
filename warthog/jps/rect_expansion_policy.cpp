#include "rect_expansion_policy.h"
#include "jps.h"
#include "rect_jump_point_locator.h"

typedef warthog::rectscan::rect_expansion_policy rexpand;

rexpand::rect_expansion_policy(RectMap* map) {
  map_ = map;
  nodepool_ = new warthog::blocklist(map_->maph, map_->mapw);
  jpl_ = new warthog::rectscan::rect_jump_point_locator(map);
  reset();

  neighbours_.reserve(1<<7);
}

rexpand::~rect_expansion_policy() {
  delete jpl_;
  delete nodepool_;
}

void rexpand::expand(
    warthog::search_node* cur, warthog::problem_instance* prob) {
  reset();
  jps::direction dir = cur->get_pdir();
  uint32_t c_tiles;
  uint32_t cur_id = cur->get_id();
  map_->get_neighbours(cur_id, (uint8_t*)&c_tiles);
  uint32_t succ_dirs = warthog::jps::compute_successors(dir, c_tiles);
  uint32_t goal_id = prob->get_goal();


  Rect* curr = map_->get_rect(cur_id);
  uint32_t searchid = prob->get_searchid();
  uint32_t id_mask = (1 << 24)-1;
  vector<uint32_t> &jpts = jpl_->get_jpts();
  vector<cost_t> &costs = jpl_->get_costs();

  if (curr->rid == map_->get_rect(goal_id)->rid) {
    jpts.push_back(goal_id);
    int curx, cury, gx, gy;
    map_->to_xy(cur_id, curx, cury);
    map_->to_xy(goal_id, gx, gy);
    costs.push_back(octile_dist(curx, cury, gx, gy));
  }
  else {
    for (int i=0; i<8; i++) {
      if (succ_dirs & (1<<i)) {
        jpl_->jump((jps::direction)(1<<i), cur_id, goal_id, curr);
      }
    }
  }

  for (int i=0; i<(int)jpts.size(); i++) {
    uint32_t jp_id = jpts.at(i);
    jps::direction pdir = (jps::direction)*(((uint8_t*)(&jp_id))+3);
    search_node* mynode = nodepool_->generate(jp_id & id_mask);
    neighbours_.push_back(mynode);
    if (mynode->get_searchid() != searchid) { mynode->reset(searchid); }

    if ((cur->get_g() + costs.at(i)) < mynode->get_g()) {
      mynode->set_pdir(pdir);
    }
  }
  if (prob->get_start() == cur_id && neighbours_.size() == 0) {
    cerr << "stop plz ..." << endl;
  }
  num_neighbours_ = neighbours_.size();
  neighbours_.push_back(0);
  costs.push_back(0);
}

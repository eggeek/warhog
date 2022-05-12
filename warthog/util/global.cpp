#include "global.h"
using namespace global;
int statis::subopt_expd = 0;
int statis::subopt_touch = 0;
vector<warthog::cost_t> statis::dist = vector<warthog::cost_t>();
warthog::problem_instance* query::pi = nullptr;
warthog::blocklist* query::nodepool = nullptr;
uint32_t query::startid = warthog::INF;
uint32_t query::goalid = warthog::INF;

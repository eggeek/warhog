#include "global.h"
using namespace global;
uint32_t statis::subopt_expd = 0;
uint32_t statis::subopt_touch = 0;
uint32_t statis::scan_cnt = 0;
vector<warthog::cost_t> statis::dist = vector<warthog::cost_t>();
warthog::problem_instance* query::pi = nullptr;
warthog::blocklist* query::nodepool = nullptr;
uint32_t query::startid = warthog::INF;
uint32_t query::goalid = warthog::INF;

#include "gridmap.h"
#include <string>
#include <queue>
#include <vector>

using namespace std;

namespace warthog {
class Dijkstra {
  public:

  struct node {
    int id;
    cost_t g;
    bool operator<(const node& rhs) const {
      return g > rhs.g;
    }
  };
  priority_queue<node, vector<node>, less<node>> q;
  gridmap* gmap;
  vector<cost_t> dist;
  Dijkstra(string mpath) {
    gmap = new gridmap(mpath.c_str());
    dist.resize(gmap->width()*gmap->height());
  }
  ~Dijkstra() {
    delete gmap;
  }

  void run(int sid) {
    fill(dist.begin(), dist.end(), INF);
    dist[sid] = 0;
    q.push({sid, 0});
    while (!q.empty()) {
      node c = q.top(); q.pop();
      uint32_t cx, cy;
      gmap->to_padded_xy(c.id, cx, cy);
      if (c.g != dist[c.id]) continue;
      for (int i=0; i<8; i++) {
        int nx = cx + dx[i];
        int ny = cy + dy[i];
        if (nx < 0 || ny < 0) continue;
        if (!gmap->get_label(nx, ny) ||
            !gmap->get_label(cx, ny) ||
            !gmap->get_label(nx, cy)) continue;
        int nid = ny*gmap->width()+nx;
        cost_t w = i<4?warthog::ONE: warthog::ROOT_TWO;
        if (dist[nid] > dist[c.id] + w) {
          dist[nid] = dist[c.id] + w;
          q.push({nid, dist[nid]});
        }
      }
    }
  }
  
};
};

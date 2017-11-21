
#include <ros/ros.h>
#include <limits>
#include <cstdlib>
#include <cmath>
#include "a_star.h"

bool a_star(node *start, node *end, path &out_path) {
  if (start == NULL || end == NULL) {
    return false;
  }
  std::set<path*> open_set;
  std::set<path*> closed_set;

  path *init_path = new path();
  init_path->nodes.push_back(start);
  init_path->cost = DISTANCE(start,end);

  open_set.insert(init_path);
  bool path_found = false;
  while (!open_set.empty()) {
    // best current path
    path *cur_path = NULL;
    float cur_cost = std::numeric_limits<float>::infinity();

    for (path *p : open_set) {
      if (p->cost < cur_cost) {
        cur_path = p;
        cur_cost = p->cost;
      }
    }
    open_set.erase(cur_path);
    closed_set.insert(cur_path);
    if (cur_path->nodes.back() == end) {
      // best current path is path to end
      // finished!
ROS_INFO("PATH FOUND");
      path_found = true;
      out_path.nodes = cur_path->nodes;
      out_path.cost = cur_cost;
      break;
    }
    if (cur_path->nodes.back()->adj.empty()) {
      // this should not happen
ROS_INFO("ENCOUNTERED NODE WITH EMPTY ADJACENCY SET");
      break;
    }

    // for each neighbour
    for (node *n : cur_path->nodes.back()->adj) {
      // check if neighbour is in closed set
      bool found_in_closed = false;
      for (path *cs_path : closed_set) {
        if (n == cs_path->nodes.back()) {
          found_in_closed = true;
          break;
        }
      }
      if (found_in_closed) {
        continue;
      }
      // check if neighbours in open set
      path *n_path = NULL;
      float n_cost = cur_cost
                     - DISTANCE(cur_path->nodes.back(),end)
                     + DISTANCE(cur_path->nodes.back(),n)
                     + DISTANCE(n,end);
      for (path *p : open_set) {
        if (n == p->nodes.back()) {
          // found in open set
          n_path = p;
          // update cost only if new cost is lower
          if (n_cost < p->cost) {
            p->cost = n_cost;
          }
          break;
        }
      }
      // not found in open set
      if (n_path == NULL) {
        n_path = new path();
        n_path->nodes = cur_path->nodes;
        n_path->nodes.push_back(n);
        n_path->cost = n_cost;
        open_set.insert(n_path);
      }
    }
  }
  for (path *p : open_set) {
    delete(p);
  }
  for (path *p : closed_set) {
    delete(p);
  }
  return path_found;
}
#ifndef A_STAR_H
#define A_STAR_H

#include <vector>

typedef struct node_t {
  int x;
  int y;
  std::vector<node_t*> adj;
} node;

typedef struct {
  std::vector<node*> nodes;
  float cost; // estimate
} path;

#endif
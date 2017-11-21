#ifndef A_STAR_H
#define A_STAR_H

#include <vector>

#define SET_EDGE(M,N) (M)->adj.push_back(N); (N)->adj.push_back(M);

#define DISTANCE(X,Y) (sqrt((X->x - Y->x)*(X->x - Y->x) \
                          + (X->y - Y->y)*(X->y - Y->y)))

typedef struct node_t {
  int x;
  int y;
  std::vector<node_t*> adj;
} node;

typedef struct {
  std::vector<node*> nodes;
  float cost; // estimate
} path;

bool a_star(node *start, node *end, path &out_path);

#endif
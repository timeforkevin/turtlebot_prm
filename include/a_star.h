#ifndef A_STAR_H
#define A_STAR_H

#include <vector>

#define SET_EDGE(M,N) (M)->adj.push_back(N); (N)->adj.push_back(M);

#define DISTANCE(X1,X2,Y1,Y2) (sqrt(((X1)-(X2))*((X1)-(X2))\
                                  + ((Y1)-(Y2))*((Y1)-(Y2))))
#define DISTANCE_NODES(X,Y) (DISTANCE((X)->x,(Y)->x,(X)->y,(Y)->y))

typedef struct node_t {
  float x;
  float y;
  float yaw;
  std::vector<node_t*> adj;
} node;

typedef struct {
  std::vector<node*> nodes;
  float cost; // estimate
} path;

bool a_star(node *start, node *end, path &out_path);

#endif
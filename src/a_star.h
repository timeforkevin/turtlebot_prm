#ifndef A_STAR_H
#define A_STAR_H

typedef struct node_t {
  int x;
  int y;
  std::vector<node_t> adj;
} node;

#endif
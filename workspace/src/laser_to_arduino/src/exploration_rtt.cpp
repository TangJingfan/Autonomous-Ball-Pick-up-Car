#include "include/rtt.h"
#include <algorithm>
#include <iostream>
#include <math.h>
#include <vector>

std::vector<point> rrt_explore(std::vector<std::vector<int>> &grid_map,
                               point start, int max_iter, int step_size) {
  std::vector<point> nodes = {start};
  for (int i = 0; i < max_iter; ++i) {
    // randomly pick element
    int rand_x = rand() % grid_map.size();
    int rand_y = rand() % grid_map[0].size();
    point rand_point = {rand_x, rand_y};

    // test whether the robot is approaching the edge
    if (!is_near_unknown(grid_map, rand_point.row_x, rand_point.col_y)) {
      continue;
    }

    // find nearest node to that random node
    auto nearest_node =
        *std::min_element(nodes.begin(), nodes.end(),
                          [&rand_point](const std::pair<int, int> &a,
                                        const std::pair<int, int> &b) {
                            return std::hypot(a.first - rand_point.row_x,
                                              a.second - rand_point.col_y) <
                                   std::hypot(b.first - rand_point.row_x,
                                              b.second - rand_point.col_y);
                          });

    // calculate new node an the way
    double theta = atan2(rand_point.col_y - nearest_node.col_y,
                         rand_point.row_x - nearest_node.row_x);
    point new_node = {
        static_cast<int>(nearest_node.row_x + step_size * cos(theta)),
        static_cast<int>(nearest_node.col_y + step_size * sin(theta))};

    // collision test
    if (grid_map[new_node.row_x][new_node.col_y] == 1) {
      continue;
    }

    // add new node
    nodes.push_back(new_node);

    // if we are getting unknown places
    if (is_near_unknown(grid_map, new_node.row_x, new_node.col_y)) {
      std::cout << "Reached unknown area!" << std::endl;
      break;
    }
  }
  return nodes;
}

bool is_near_unknown(const std::vector<std::vector<int>> &grid_map, int x,
                     int y) {
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      int nx = x + dx, ny = y + dy;
      if (nx >= 0 && nx < grid_map.size() && ny >= 0 &&
          ny <= grid_map[0].size()) {
        if (grid_map[nx][ny] == -1) {
          return true;
        }
      }
    }
  }
  return false;
}
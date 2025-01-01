#include <vector>

// point struct
struct point {
  int row_x;
  int col_y;
};

// map size
const int map_size_x = 100;
const int map_size_y = 100;
// resolution 0.1 meter in real world
const double resolution = 0.1;
// step size
int step_size = 5;
// max iteration times
int max_iter = 1000;

// a mappin function from real world to grid map
point world_to_grid(double x, double y) {
  return {static_cast<int>(x / resolution), static_cast<int>(y / resolution)};
}

// a function to test whether the robot is approaching the edge of known areas
bool is_near_unknown(const std::vector<std::vector<int>> &grid_map, int x,
                     int y);

std::vector<std::vector<int>> grid_map(map_size_x,
                                       std::vector<int>(map_size_y, -1));

// init map
// -1: unknown area
// 0: free area
// 1: obstacle
std::vector<std::vector<int>> grid_map(map_size_x,
                                       std::vector<int>(map_size_y, -1));

// init starting point
point start = {50, 50};

std::vector<point> rrt_explore(std::vector<std::vector<int>> &grid_map,
                               point start, int max_iter, int step_size);
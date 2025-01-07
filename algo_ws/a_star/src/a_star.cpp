#include <cmath>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace cv;
using namespace std;

// Load the PGM map
vector<vector<int>> loadPGM(const string &filename) {
  Mat img = imread(filename, IMREAD_GRAYSCALE);
  if (img.empty()) {
    cerr << "Fail to load picture!" << endl;
    exit(1);
  }

  vector<vector<int>> map(img.rows, vector<int>(img.cols));
  for (int i = 0; i < img.rows; ++i) {
    for (int j = 0; j < img.cols; ++j) {
      map[i][j] = img.at<uchar>(i, j);
    }
  }

  return map;
}

struct a_star_node {
  pair<int, int> point;
  int f, g, h;
  shared_ptr<a_star_node> parent;

  a_star_node(pair<int, int> point_, int g_cost, int h_cost,
              shared_ptr<a_star_node> parent_ = nullptr)
      : point(point_), f(g_cost + h_cost), g(g_cost), h(h_cost),
        parent(parent_) {}

  bool operator>(const a_star_node &other) const { return f > other.f; }
};

int manhattan_distance(pair<int, int> first, pair<int, int> second) {
  return abs(first.first - second.first) + abs(first.second - second.second);
}

double euclidean_distance(pair<int, int> first, pair<int, int> second) {
  return sqrt(pow(first.first - second.first, 2) +
              pow(first.second - second.second, 2));
}

vector<pair<int, int>> find_neighbor(pair<int, int> parent,
                                     const vector<vector<int>> &map) {
  vector<pair<int, int>> neighbor;
  const vector<pair<int, int>> direction = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
                                            {0, 1},   {1, -1}, {1, 0},  {1, 1}};
  for (const auto &dir : direction) {
    int nx = parent.first + dir.first;
    int ny = parent.second + dir.second;
    if (nx >= 0 && nx < map.size() && ny >= 0 && ny < map[0].size() &&
        map[nx][ny] == 254) { // Check for walkable terrain
      neighbor.push_back({nx, ny});
    }
  }

  return neighbor;
}

vector<pair<int, int>> reconstruct_path(shared_ptr<a_star_node> node) {
  vector<pair<int, int>> path;
  while (node != nullptr) {
    path.push_back(node->point);
    node = node->parent;
  }
  reverse(path.begin(), path.end());
  return path;
}

struct pair_hash {
  template <typename T1, typename T2>
  size_t operator()(const pair<T1, T2> &p) const {
    auto h1 = hash<T1>{}(p.first);
    auto h2 = hash<T2>{}(p.second);
    return h1 ^ (h2 << 1);
  }
};

vector<pair<int, int>> a_star_search(pair<int, int> start, pair<int, int> end,
                                     const vector<vector<int>> &map) {
  auto cmp = [](shared_ptr<a_star_node> a, shared_ptr<a_star_node> b) {
    return a->f > b->f;
  };
  priority_queue<shared_ptr<a_star_node>, vector<shared_ptr<a_star_node>>,
                 decltype(cmp)>
      open_list(cmp);
  unordered_map<pair<int, int>, bool, pair_hash> closed_list;

  shared_ptr<a_star_node> start_node =
      make_shared<a_star_node>(start, 0, euclidean_distance(start, end));
  open_list.push(start_node);

  while (!open_list.empty()) {
    shared_ptr<a_star_node> current = open_list.top();
    open_list.pop();

    if (current->point == end) {
      return reconstruct_path(current);
    }

    closed_list[current->point] = true;

    for (const auto &neighbor_pos : find_neighbor(current->point, map)) {
      if (closed_list[neighbor_pos])
        continue;

      int g_new = current->g + euclidean_distance(neighbor_pos, current->point);
      int h_new = euclidean_distance(neighbor_pos, end);

      shared_ptr<a_star_node> neighbor =
          make_shared<a_star_node>(neighbor_pos, g_new, h_new, current);

      open_list.push(neighbor);
    }
  }

  return {};
}

void arrayToImage(const vector<vector<int>> &array, int row_start, int row_end,
                  int col_start, int col_end, const string &filename) {
  int rows = array.size();
  int cols = array[0].size();

  // create mat object
  Mat img(rows, cols, CV_8U);

  for (int i = row_start; i < row_end; ++i) {
    for (int j = col_start; j < col_end; ++j) {
      img.at<uchar>(i, j) = static_cast<uchar>(array[i][j]);
    }
  }

  // save image
  imwrite(filename, img);
}

int main() {
  string pgmFile = "../map/map.pgm"; // Adjust path as needed
  vector<vector<int>> map = loadPGM(pgmFile);

  pair<int, int> start = {1064, 1032};
  pair<int, int> end = {1041, 1045};

  pair<int, int> row = {1000, 1090};
  pair<int, int> col = {1000, 1070};

  vector<pair<int, int>> path = a_star_search(start, end, map);

  if (!path.empty()) {
    cout << "Path found:\n";
    for (const auto &p : path) {
      cout << "(" << p.first << ", " << p.second << ") ";
      map[p.first][p.second] = 6;
    }
    cout << endl;
  } else {
    cout << "No path found.\n";
    return 0;
  }

  arrayToImage(map, row.first, row.second, col.first, col.second, "route.png");

  return 0;
}
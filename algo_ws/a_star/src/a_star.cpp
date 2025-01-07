#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

// handle map
vector<vector<int>> loadPGM(const string &filename) {
  // upload image
  Mat img = imread(filename, IMREAD_GRAYSCALE);
  if (img.empty()) {
    cerr << "Fail to load picture!" << endl;
    exit(1);
  }

  // convert map into vector
  vector<vector<int>> map(img.rows, vector<int>(img.cols));
  for (int i = 0; i < img.rows; ++i) {
    for (int j = 0; j < img.cols; ++j) {
      map[i][j] = img.at<uchar>(i, j);
    }
  }

  return map;
}

struct a_star_node {
  // member
  pair<int, int> point;
  int f;
  int g;
  int h;
  a_star_node *parent;

  // constructor
  a_star_node(pair<int, int> point_, int g_cost, int h_cost,
              a_star_node *parent_ = nullptr)
      : point(point_), f(g_cost + h_cost), g(g_cost), h(h_cost),
        parent(parent_) {}

  // compare operator
  bool operator>(const a_star_node &other) const { return f > other.f; }
};

int manhattan_distance(pair<int, int> first, pair<int, int> second) {
  return abs(first.first - second.first) + abs(first.second - second.second);
}

double euclidean_distance(pair<int, int> first, pair<int, int> second) {
  return sqrt(pow(first.first - second.first, 2) +
              pow(first.second - second.second, 2));
}

// we use euclidean distance as our cost function
vector<pair<int, int>> find_neighbor(pair<int, int> parent,
                                     vector<vector<int>> map) {
  vector<pair<int, int>> neighbor;
  const vector<pair<int, int>> direction = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
                                            {0, 1},   {1, -1}, {1, 0},  {1, 1}};
  for (const auto &dir : direction) {
    int nx = parent.first + dir.first;
    int ny = parent.second + dir.second;
    if (nx >= 0 && nx <= map.size() && ny >= 0 && ny <= map[0].size() &&
        map[nx][ny] == 254) {
      neighbor.push_back({nx, ny});
    }
  }
}

void a_star_search(pair<int, int> start, pair<int, int> end,
                   vector<vector<int>> map) {}

int main() {
  string pgmFile = "../map/map.pgm";
  vector<vector<int>> map = loadPGM(pgmFile);

  // print part of the map
  for (int i = 1000; i < 1090; ++i) {
    for (int j = 1000; j < 1070; ++j) {
      if (map[i][j] == 205) {
        map[i][j] = 100;
      }

      cout << map[i][j] << " ";
    }
    cout << endl;
  }

  return 0;
}
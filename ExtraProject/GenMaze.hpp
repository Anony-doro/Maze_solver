#ifndef GENMAZE_HPP

#define GENMAZE_HPP

#include <random>
#include <vector>
#include <opencv2/opencv.hpp>

class MazeGenerator {
private:
  std::vector<std::vector<int>> maze;
  int size;
  std::random_device rd;
  std::mt19937 gen;

  // Cell types
  static constexpr int FREE = 0;
  static constexpr int WALL = 1;
  static constexpr int START = 2;
  static constexpr int GOAL = 3;
public:
  MazeGenerator(int mazeSize);
  void generateRandomMaze(double obstacleRatio = 0.2);
  void setRandomStartGoal();
  void printMaze();
  void printStatistics();
  std::vector<std::vector<int>> getMaze() const;
  int getSize() const;
  cv::Mat visualize(int cellPixels = 10) const;
};

#endif //GENMAZE_HPP
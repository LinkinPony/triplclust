//
// Created on 24-10-17.
//

#include <open3d_wrapper/triplclust_wrapper.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <iostream>
#include <random>

std::shared_ptr<open3d::geometry::PointCloud> ReadPointCloudFromFile(const std::filesystem::path &path) {

  auto point_cloud = std::make_shared<open3d::geometry::PointCloud>();

  std::ifstream infile(path);
  if (!infile.is_open()) {
    std::cerr << "Failed to open file: " << path << std::endl;
    return nullptr;
  }

  std::string line;
  while (std::getline(infile, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::istringstream iss(line);
    float x, y, z;
    if (iss >> x >> y >> z) {
      point_cloud->points_.emplace_back(x, y, z);
    }
  }
  infile.close();
  return point_cloud;

}

int main() {

  std::string filename = "attpc.dat";
  std::filesystem::path path = std::filesystem::current_path().parent_path().parent_path() / "data";
  auto pcd = ReadPointCloudFromFile(path / filename);

  auto param = Opt();
  auto result = triplclust::TriplclustWrapper::Run(pcd, param);
  std::cout << result->cluster_index_.size();

  std::vector<Eigen::Vector3d> random_color(result->points_.size());
  std::random_device random_device{};
  std::mt19937 gen(random_device());
  std::uniform_real_distribution<double> dist{0, 1};
  for (auto &color : random_color) {
    color = {dist(gen), dist(gen), dist(gen)};
  }
  result->colors_.resize(result->points_.size());
  for (size_t i = 0; i < result->colors_.size(); i++) {
    if (result->cluster_index_[i].empty()) {
      continue;
    }
    auto index = result->cluster_index_[i].back();
    std::cout << index << std::endl;
    result->colors_[i] = random_color[index];
  }

  open3d::visualization::Visualizer visualizer;
  visualizer.CreateVisualizerWindow("triplclust", 1920, 1080);
  visualizer.AddGeometry(result);
  visualizer.Run();
  visualizer.DestroyVisualizerWindow();

  return 0;
}
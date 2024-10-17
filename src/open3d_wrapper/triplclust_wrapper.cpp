//
// Created on 24-10-17.
//

#include "triplclust_wrapper.h"

namespace triplclust {

ClusterPointCloud &ClusterPointCloud::Clear() {
  PointCloud::Clear();
  cluster_index_.clear();
  return *this;
}
bool ClusterPointCloud::IsEmpty() const {
  return PointCloud::IsEmpty() && cluster_index_.empty();
}
std::shared_ptr<ClusterPointCloud> TriplclustWrapper::Run(std::shared_ptr<open3d::geometry::PointCloud> &point_cloud,
                                                          Opt &option) {
  
  return std::shared_ptr<ClusterPointCloud>();
}
} // triplclust
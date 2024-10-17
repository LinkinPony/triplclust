//
// Created on 24-10-17.
//

#ifndef TRIPLCLUST_SRC_OPEN3D_WRAPPER_TRIPLCLUST_WRAPPER_H_
#define TRIPLCLUST_SRC_OPEN3D_WRAPPER_TRIPLCLUST_WRAPPER_H_


//from triplclust
#include <option.h>

#include <open3d/Open3D.h>

namespace triplclust {

class ClusterPointCloud : public open3d::geometry::PointCloud {

 public:
  ClusterPointCloud() = default;
  const size_t NOISE_INDEX = std::numeric_limits<size_t>::max();

 public:
  ClusterPointCloud &Clear() override;
  [[nodiscard]] bool IsEmpty() const override;

 public://members
  std::vector<size_t> cluster_index_;
};

class TriplclustWrapper {

 public:
  //TODO: refactor option
  std::shared_ptr<ClusterPointCloud> Run(std::shared_ptr<open3d::geometry::PointCloud> &point_cloud, Opt &option);
};

} // triplclust

#endif //TRIPLCLUST_SRC_OPEN3D_WRAPPER_TRIPLCLUST_WRAPPER_H_

//
// Created on 24-10-17.
//

#include "triplclust_wrapper.h"

#include "cluster.h"
#include "dnn.h"
#include "graph.h"
#include "option.h"
#include "output.h"
#include "pointcloud.h"

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
                                                          Opt &opt_params) {
  int opt_verbose = opt_params.get_verbosity();
  auto cloud_xyz = ConvertOpen3dPCDToTriplclust_(point_cloud);
  // compute characteristic length dnn if needed
  if (opt_params.needs_dnn()) {
    double dnn = std::sqrt(first_quartile(cloud_xyz));
    if (opt_verbose > 0) {
      std::cout << "[Info] computed dnn: " << dnn << std::endl;
    }
    opt_params.set_dnn(dnn);
    if (dnn == 0.0) {
      std::cerr << "[Error] dnn computed as zero. "
                << "Suggestion: remove doublets, e.g. with 'sort -u'"
                << std::endl;
      return {};
    }
  }

  // Step 1) smoothing by position averaging of neighboring points
  PointCloud cloud_xyz_smooth;
  smoothen_cloud(cloud_xyz, cloud_xyz_smooth, opt_params.get_r());

  if (opt_verbose > 1) {
    bool rc;
    rc = cloud_to_csv(cloud_xyz_smooth);
    if (!rc)
      std::cerr << "[Error] can't write debug_smoothed.csv" << std::endl;
    rc = debug_gnuplot(cloud_xyz, cloud_xyz_smooth);
    if (!rc)
      std::cerr << "[Error] can't write debug_smoothed.gnuplot" << std::endl;
  }

  // Step 2) finding triplets of approximately collinear points
  std::vector<triplet> triplets;
  generate_triplets(cloud_xyz_smooth, triplets, opt_params.get_k(),
                    opt_params.get_n(), opt_params.get_a());

  // Step 3) single link hierarchical clustering of the triplets
  cluster_group cl_group;
  compute_hc(cloud_xyz_smooth, cl_group, triplets, opt_params.get_s(),
             opt_params.get_t(), opt_params.is_tauto(), opt_params.get_dmax(),
             opt_params.is_dmax(), opt_params.get_linkage(), opt_verbose);

  // Step 4) pruning by removal of small clusters ...
  cleanup_cluster_group(cl_group, opt_params.get_m(), opt_verbose);
  cluster_triplets_to_points(triplets, cl_group);
  // .. and (optionally) by splitting up clusters at gaps > dmax
  if (opt_params.is_dmax()) {
    cluster_group cleaned_up_cluster_group;
    for (auto &cl : cl_group) {
      max_step(cleaned_up_cluster_group, cl, cloud_xyz, opt_params.get_dmax(),
               opt_params.get_m() + 2);
    }
    cl_group = cleaned_up_cluster_group;
  }

  // store cluster labels in points
  add_clusters(cloud_xyz, cl_group, opt_params.is_gnuplot());

  auto result = std::make_shared<ClusterPointCloud>();
  //TODO: pre allocate
  for (auto &point : cloud_xyz) {
    result->points_.emplace_back(point.x, point.y, point.z);
    result->cluster_index_.emplace_back();
    for (auto id : point.cluster_ids) {
      result->cluster_index_.back().emplace_back(id);
    }
  }

  return result;
}
TriplclustWrapper::TriplclustPointCloud TriplclustWrapper::ConvertOpen3dPCDToTriplclust_(
    std::shared_ptr<open3d::geometry::PointCloud> &point_cloud) {
  TriplclustPointCloud result;
  for (auto &point : point_cloud->points_) {
    result.emplace_back(point.x(), point.y(), point.z());
  }
  return result;
}
} // triplclust
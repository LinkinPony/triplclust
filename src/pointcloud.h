//
// pointcloud.h
//     Classes and functions for 3D points and clouds thereof.
//
// Author:  Jens Wilberg, Lukas Aymans, Christoph Dalitz
// Date:    2024-02-02
// License: see ../LICENSE
//

#ifndef POINTCLOUD_H
#define POINTCLOUD_H
#include <cstddef>
#include <fstream>
#include <iostream>
#include <ostream>
#include <set>
#include <vector>

// 3D point class.
class Point {
 public:
  double x;
  double y;
  double z;
  std::set<size_t> cluster_ids;
  size_t index;    // only used for chronological order

  Point(){};
  Point(const std::vector<double>& point);
  Point(const std::vector<double>& point, const std::set<size_t>& cluster_ids);
  Point(double x, double y, double z);
  Point(double x, double y, double z, const std::set<size_t>& cluster_ids);
  Point(double x, double y, double z, size_t index);

  // representation of 3D point as std::vector
  std::vector<double> as_vector() const;
  // Euclidean norm
  double norm() const;
  // squared norm
  double squared_norm() const;

  friend std::ostream& operator<<(std::ostream& os, const Point& p);
  bool operator==(const Point& p) const;
  Point& operator=(const Point& other);
  // vector addition
  Point operator+(const Point& p) const;
  // vector subtraction
  Point operator-(const Point& p) const;
  // scalar product
  double operator*(const Point& p) const;
  // scalar division
  Point operator/(double c) const;
};

// scalar multiplication
Point operator*(Point x, double c);
Point operator*(double c, Point x);

// The Pointcloud is a vector of points
class PointCloud {
 private:
  bool points2d;
  bool ordered;   //!
  std::vector<Point> points_;

 public:
  bool is2d() const;
  void set2d(bool is2d);
  bool isOrdered() const;   //!
  void setOrdered(bool isOrdered);  //!
  [[nodiscard]] const std::vector<Point> &getPoints() const { return points_; }

  //TODO: test
 public:
  PointCloud() : points2d(false), ordered(false) {}
  PointCloud(bool is2d, bool isOrdered) : points2d(is2d), ordered(isOrdered) {}
  PointCloud(PointCloud &&other) noexcept
      : points2d(other.points2d),
        ordered(other.ordered),
        points_(std::move(other.points_)) {}

  PointCloud &operator=(PointCloud &&other) noexcept {
    if (this != &other) {
      points2d = other.points2d;
      ordered = other.ordered;
      points_ = std::move(other.points_);
    }
    return *this;
  }
  PointCloud(const PointCloud &) = default;
  PointCloud &operator=(const PointCloud &) = default;
 public:
  using iterator = std::vector<Point>::iterator;
  std::vector<Point>::iterator begin() { return points_.begin(); }
  [[nodiscard]] std::vector<Point>::const_iterator begin() const { return points_.begin(); }
  std::vector<Point>::iterator end() { return points_.end(); }
  [[nodiscard]] std::vector<Point>::const_iterator end() const { return points_.end(); }

  [[nodiscard]] std::vector<Point>::const_iterator cbegin() const { return points_.cbegin(); }
  [[nodiscard]] std::vector<Point>::const_iterator cend() const { return points_.cend(); }

  std::vector<Point>::reverse_iterator rbegin() { return points_.rbegin(); }
  [[nodiscard]] std::vector<Point>::const_reverse_iterator rbegin() const { return points_.rbegin(); }
  std::vector<Point>::reverse_iterator rend() { return points_.rend(); }
  [[nodiscard]] std::vector<Point>::const_reverse_iterator rend() const { return points_.rend(); }

  size_t size() const { return points_.size(); }
  bool empty() const { return points_.empty(); }
  size_t capacity() const { return points_.capacity(); }
  void reserve(size_t n) { points_.reserve(n); }
  void shrink_to_fit() { points_.shrink_to_fit(); }
  void push_back(const Point &p) { points_.push_back(p); }
  void push_back(Point &&p) { points_.push_back(std::move(p)); }
  template<typename... Args>
  void emplace_back(Args &&... args) { points_.emplace_back(std::forward<Args>(args)...); }

  void pop_back() { points_.pop_back(); }
  void clear() { points_.clear(); }

  iterator insert(iterator pos, const Point &value) { return points_.insert(pos, value); }
  iterator insert(iterator pos, Point &&value) { return points_.insert(pos, std::move(value)); }
  template<typename InputIt>
  void insert(iterator pos, InputIt first, InputIt last) { points_.insert(pos, first, last); }

  iterator erase(iterator pos) { return points_.erase(pos); }
  iterator erase(iterator first, iterator last) { return points_.erase(first, last); }

  void resize(size_t count) { points_.resize(count); }
  void resize(size_t count, const Point &value) { points_.resize(count, value); }

  Point &operator[](size_t index) { return points_[index]; }
  const Point &operator[](size_t index) const { return points_[index]; }
};


// Load csv file.
void load_csv_file(const char* fname, PointCloud& cloud, const char delimiter,
                   size_t skip = 0);
// Smoothing of the PointCloud *cloud*. The result is returned in *result_cloud*
void smoothen_cloud(const PointCloud& cloud, PointCloud& result_cloud,
                    double radius);

#endif

#pragma once
#include <eigen3/Eigen/Dense>
#include <hippo_common/tf2_utils.hpp>
#include <numeric>

template <typename T>
T WrapFloating(T x, T low, T high) {
  if (low <= x && x < high) {
    return x;
  }
  const auto range = high - low;
  const auto inv_range = T(1) / range;
  const auto num_wraps = floor((x - low) * inv_range);
  return x - range * num_wraps;
}

template <typename T>
T WrapPi(T x) {
  return WrapFloating(x, T(-M_PI), T(M_PI));
}

class ViewpointProgress {
 public:
  ViewpointProgress(int threshold) : progress_counter_(0) {
    threshold_ = threshold;
    Reset();
  }

  void Update(bool in_tolerance) {
    if (completed_) {
      return;
    }
    progress_counter_ += in_tolerance ? 1 : -1;
    if (progress_counter_ >= threshold_) {
      completed_ = true;
    }
    progress_counter_ = std::clamp(progress_counter_, 0, threshold_);
    progress_ = static_cast<double>(progress_counter_) / threshold_;
  }

  void Reset() {
    progress_counter_ = 0;
    progress_ = 0.0;
    completed_ = false;
  }

  double Progress() const { return progress_; }
  bool Completed() const { return completed_; }

 private:
  int progress_counter_;
  bool completed_{false};
  double progress_;
  int threshold_;
};

class Viewpoint {
 public:
  Viewpoint(const Eigen::Vector3d &_position,
            const Eigen::Quaterniond &_orientation, double _position_tolerance,
            double _angle_tolerance, int _threshold = 80)
      : position(_position),
        orientation(_orientation),
        progress_(_threshold),
        position_tolerance_(_position_tolerance),
        angle_tolerance_(_angle_tolerance) {}

  bool IsInTolerance() const { return in_tolerance_; }

  double PositionError(const Eigen::Vector3d &_position) {
    Eigen::Vector3d d_vec = _position - position;
    // we ignore z because we deal with a 2d problem
    d_vec.z() = 0.0;
    return d_vec.norm();
  }

  double AngleError(const Eigen::Quaterniond &_orientation) {
    Eigen::Vector3d target_heading = orientation * Eigen::Vector3d::UnitX();
    target_heading.z() = 0.0;
    target_heading.normalize();
    Eigen::Vector3d current_heading = _orientation * Eigen::Vector3d::UnitX();
    current_heading.z() = 0.0;
    current_heading.normalize();
    const double angle_error = std::acos(target_heading.dot(current_heading));
    return std::abs(WrapPi(angle_error));
  }

  bool PeekIsInTolerance(const Eigen::Vector3d &_position,
                         const Eigen::Quaterniond &_orientation) {
    Eigen::Vector3d d_vec = _position - position;
    // we ignore z because we deal with a 2d problem
    d_vec.z() = 0.0;
    const bool in_position_tolerance{d_vec.squaredNorm() <=
                                     position_tolerance_ * position_tolerance_};
    const double angle_error = AngleError(_orientation);
    const bool in_angle_tolerance{angle_error <= angle_tolerance_};
    return in_angle_tolerance && in_position_tolerance;
  }

  bool IsCompleted() const { return progress_.Completed(); }

  double Progress() const { return progress_.Progress(); }

  void Update(bool in_tolerance) {
    in_tolerance_ = in_tolerance;
    progress_.Update(in_tolerance_);
  }

  void Update(const Eigen::Vector3d &_position,
              const Eigen::Quaterniond &_orientation) {
    UpdateInTolerance(_position, _orientation);
    UpdateProgress();
  }

  void Reset() {
    in_tolerance_ = false;
    progress_.Reset();
  }
  void UpdateInTolerance(const Eigen::Vector3d &_position,
                         const Eigen::Quaterniond &_orientation) {
    in_tolerance_ = PeekIsInTolerance(_position, _orientation);
  }
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;

 private:
  void UpdateProgress() { progress_.Update(in_tolerance_); }
  ViewpointProgress progress_;
  double position_tolerance_;
  double angle_tolerance_;
  bool in_tolerance_;
};

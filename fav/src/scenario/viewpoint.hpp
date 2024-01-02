#pragma once
#include <algorithm>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <hippo_common/tf2_utils.hpp>
#include <numeric>
#include <queue>

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
  ViewpointProgress(int size, double threshold) {
    size_ = size;
    threshold_ = threshold;
    Reset();
  }

  void Update(bool in_tolerance) {
    if (completed_) {
      return;
    }
    progress_buffer_.pop_front();
    progress_buffer_.push_back(in_tolerance);
    const double sum =
        std::accumulate(progress_buffer_.begin(), progress_buffer_.end(), 0.0);
    if (progress_buffer_.size() > 0) {
      progress_ = sum / progress_buffer_.size() / threshold_;
      if (progress_ >= 1.0) {
        completed_ = true;
      }
      progress_ = std::clamp(progress_, 0.0, 1.0);
    }
  }

  void Reset() {
    progress_buffer_.clear();
    for (int i = 0; i < size_; ++i) {
      progress_buffer_.push_back(false);
    }
    progress_ = 0.0;
    completed_ = false;
  }

  double Progress() const { return progress_; }
  bool Completed() const { return completed_; }

 private:
  std::deque<bool> progress_buffer_;
  int size_;
  bool completed_{false};
  double progress_;
  double threshold_;
};

class Viewpoint {
 public:
  Viewpoint(const Eigen::Vector3d &_position,
            const Eigen::Quaterniond &_orientation, double _position_tolerance,
            double _angle_tolerance, int _buffer_size = 100,
            double _threshold = 0.8)
      : position(_position),
        orientation(_orientation),
        progress_(_buffer_size, _threshold),
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

#include <fast_tf/fast_tf.hpp>
#include <benchmark/benchmark.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <Eigen/Geometry>

using benchmark::State;

static void
eigen_isometry_prod(State& _state) {
  const Eigen::Isometry3d tf1(Eigen::Translation3d(1, 2, 3) *
                              Eigen::Quaterniond(-0.002, -0.678, 0.226, 0.699));

  const Eigen::Isometry3d tf2(Eigen::Translation3d(3, 4, 1) *
                              Eigen::Quaterniond(-0.003, -0.945, 0.315, 0.091));

  for (auto _ : _state)
    benchmark::DoNotOptimize(tf1 * tf2);
}

static const Eigen::Vector3d
quat_prod(const Eigen::Quaterniond& _q, const Eigen::Vector3d& _v) {
    return _q * _v;
}

static void
eigen_manual_prod(State& _state) {
  Eigen::Vector3d v1(1, 2, 3);
  Eigen::Quaterniond q1(-0.002, -0.678, 0.226, 0.699);

  Eigen::Vector3d v2(3, 4, 1);
  Eigen::Quaterniond q2(-0.003, -0.945, 0.315, 0.091);

  for (auto _ : _state) {
    benchmark::DoNotOptimize(quat_prod(q2, v1) + v2);
    benchmark::DoNotOptimize(q2 * q1);
  }
}

static void
tf_transform_accum_prod(State& _state) {
  tf2::Quaternion q1(-0.002, -0.678, 0.226, 0.699);
  tf2::Vector3 v1(1, 2, 3);

  tf2::Quaternion q2(-0.003, -0.945, 0.315, 0.091);
  tf2::Vector3 v2(3, 4, 1);

  for (auto _ : _state) {
    benchmark::DoNotOptimize(tf2::quatRotate(q2, v1) + v2);
    benchmark::DoNotOptimize(q2 * q1);
  }
}

BENCHMARK(eigen_isometry_prod);
BENCHMARK(eigen_manual_prod);
BENCHMARK(tf_transform_accum_prod);
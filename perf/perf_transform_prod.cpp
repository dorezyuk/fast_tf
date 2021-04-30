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

struct tf_transform {
  tf2::Vector3 v;
  tf2::Quaternion q;
};

// without this the compiler will still remove the call to the product. this can
// be seen when running the benchmark below with valgrind --tool=callgrind.
__attribute__((noinline)) tf_transform
tf2_product(const tf_transform& _l, const tf_transform& _r) {
  return {tf2::quatRotate(_l.q, _r.v) + _l.v, _l.q * _r.q};
}

static void
tf_transform_accum_prod(State& _state) {
  tf2::Quaternion q1(-0.002, -0.678, 0.226, 0.699);
  tf2::Vector3 v1(1, 2, 3);

  tf2::Quaternion q2(-0.003, -0.945, 0.315, 0.091);
  tf2::Vector3 v2(3, 4, 1);

  tf_transform tf1{v1, q1};
  tf_transform tf2{v2, q2};
  for (auto _ : _state)
    benchmark::DoNotOptimize(tf2_product(tf1, tf2));
}

// benchmarks comparing the tf product of transform with the eigen-based.
BENCHMARK(eigen_isometry_prod);
BENCHMARK(tf_transform_accum_prod);
#pragma once
/**
 * Cap TBB / std::execution::par* parallelism like BIEVR's max_num_threads.
 *
 *   max_num_threads > 0  → use that many workers
 *   max_num_threads <= 0 → hardware concurrency (TBB default)
 *
 * Keep the returned global_control alive for the process lifetime.
 */
#include <tbb/global_control.h>
#include <tbb/task_arena.h>

#include <memory>

inline int resolveTbbMaxThreads(int max_num_threads) {
  if (max_num_threads > 0) return max_num_threads;
  return tbb::this_task_arena::max_concurrency();
}

inline std::unique_ptr<tbb::global_control> makeTbbThreadLimit(int max_num_threads) {
  const int n = resolveTbbMaxThreads(max_num_threads);
  return std::make_unique<tbb::global_control>(tbb::global_control::max_allowed_parallelism, n);
}

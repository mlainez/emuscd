/* SPDX-License-Identifier: CC0-1.0 */
#include "engines/xp/common/worker_pool.h"

#include <cassert>
#ifdef NDEBUG
#error "this test is assertion-driven; NDEBUG compiles it away"
#endif
#include <atomic>
#include <cstdio>
#include <cstring>
#include <thread>

using EmuSC::Xp::WorkerPool;

namespace {

struct Counts {
  std::atomic<unsigned> hits[257];
  std::atomic<unsigned> slow;
};

void count_job(void *context, unsigned index)
{
  struct Counts *counts = (struct Counts *)context;
  counts->hits[index].fetch_add(1u, std::memory_order_relaxed);
}

/* Every index of every run is called exactly once, and its effect is
   visible to the caller when run() returns - across runs that follow each
   other with no pause, which is where one generation's workers could reach
   into the next one's counter. */
void check_exactly_once(WorkerPool *pool)
{
  static struct Counts counts;
  for (unsigned round = 0; round < 3000; ++round) {
    unsigned count = (round * 37u) % 258u;
    for (unsigned i = 0; i < 257; ++i)
      counts.hits[i].store(0u, std::memory_order_relaxed);
    pool->run(count, count_job, &counts);
    for (unsigned i = 0; i < 257; ++i)
      assert(counts.hits[i].load(std::memory_order_relaxed) ==
             (i < count ? 1u : 0u));
  }
}

struct Seen {
  std::thread::id ids[64];
};

void slow_job(void *context, unsigned index)
{
  struct Seen *seen = (struct Seen *)context;
  seen->ids[index] = std::this_thread::get_id();
  /* Long enough that the caller cannot drain all 64 before a worker wakes. */
  volatile unsigned sink = 0;
  for (unsigned i = 0; i < 200000u; ++i)
    sink = sink + i;
}

/* A pool of more than one thread really runs jobs on more than one. */
void check_spreads(WorkerPool *pool)
{
  struct Seen seen;
  pool->run(64, slow_job, &seen);
  unsigned distinct = 0;
  for (unsigned i = 0; i < 64; ++i) {
    bool first = true;
    for (unsigned j = 0; j < i; ++j)
      if (seen.ids[j] == seen.ids[i])
        first = false;
    distinct += first;
  }
  assert(distinct >= 2u && distinct <= pool->threads());
}

}  // namespace

int main(void)
{
  assert(WorkerPool::available_cpus() >= 1u);
  {
    WorkerPool one(1);
    assert(one.threads() == 1u);
    check_exactly_once(&one);
  }
  {
    WorkerPool many(WorkerPool::kMaxThreads + 5u);
    assert(many.threads() <= WorkerPool::kMaxThreads);
  }
  {
    WorkerPool four(4);
    assert(four.threads() == 1u || four.threads() == 4u);
    check_exactly_once(&four);
    if (four.threads() > 1u)
      check_spreads(&four);
    else
      std::printf("xp_worker_pool_test: built without render threads\n");
  }
  return 0;
}

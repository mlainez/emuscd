/* SPDX-License-Identifier: CC0-1.0 */
/*
 *  A fixed set of render threads that a voice loop hands its independent
 *  voices to, block by block.
 *
 *  The pool decides only WHERE a job runs, never what it computes or in
 *  what order its results are combined: every caller renders each voice
 *  into storage of its own and sums those afterwards, on one thread, in
 *  the order the serial loop would have. Output is therefore the same
 *  bytes at every thread count, and a one-thread pool is the serial engine.
 *
 *  Built without EMUSC_RENDER_THREADS (the 32-bit Windows target, whose
 *  floor predates the OS calls std::thread links against) the pool never
 *  starts a thread and run() calls every job in index order on the
 *  caller's. The class layout is the same either way, so code that only
 *  includes this header does not depend on how worker_pool.cc was built.
 */
#ifndef EMUSC_XP_COMMON_WORKER_POOL_H
#define EMUSC_XP_COMMON_WORKER_POOL_H

#include <stdint.h>

namespace EmuSC { namespace Xp {

class WorkerPool {
public:
  typedef void (*Job)(void *context, unsigned index);

  /* `threads` counts the caller: a pool of N runs jobs on N - 1 threads of
     its own plus the one that calls run(). 0 asks for one per CPU this
     process may run on. Clamped to what this build supports - always 1
     without EMUSC_RENDER_THREADS - and to kMaxThreads. Worker i is pinned
     to workerCpus[i] where one is given, on Linux; the scheduler never
     moves a thread onto a CPU taken out of balancing (isolcpus), so pinning
     is the only way a worker gets one. Elsewhere the list is ignored. */
  explicit WorkerPool(unsigned threads, const unsigned *workerCpus = nullptr,
                      unsigned workerCpuCount = 0);
  ~WorkerPool();
  WorkerPool(const WorkerPool &) = delete;
  WorkerPool &operator=(const WorkerPool &) = delete;

  unsigned threads() const { return _threads; }

  /* Calls job(context, i) once for every i in [0, count), spread across the
     pool, and returns once all of them have returned. Jobs may run in any
     order and concurrently with each other, so job i may touch only what
     belongs to index i. Not reentrant: one run() at a time per pool. */
  void run(unsigned count, Job job, void *context);

  /* The CPUs this process may run on, at least 1. */
  static unsigned available_cpus();

  static const unsigned kMaxThreads = 8;

private:
  struct Shared;
  Shared *_shared;
  unsigned _threads;
};

/* One thread at idle priority that works through a queue of small fixed
   items, for work that may as well happen early but must not hold up a
   render: decoding a wave a loaded patch will need before its first note
   asks for it. Built without EMUSC_RENDER_THREADS there is no thread and
   every post is dropped, which leaves the work to whoever needs it first.
   The output never depends on whether an item ran. */
class BackgroundWorker {
public:
  struct Item {
    uint32_t a, b, c;
  };
  typedef void (*Task)(void *context, const Item &item);

  BackgroundWorker(Task task, void *context);
  ~BackgroundWorker();
  BackgroundWorker(const BackgroundWorker &) = delete;
  BackgroundWorker &operator=(const BackgroundWorker &) = delete;

  /* Queues `item`; false, dropping it, when the queue is full or there is
     no thread. */
  bool post(const Item &item);

  /* A lock the task and the code that shares its data hold around that
     data; it does nothing where there is no thread. */
  void lock();
  void unlock();

  static const unsigned kQueueItems = 2048;

private:
  struct Shared;
  Shared *_shared;
};

}}  // namespace EmuSC::Xp

#endif

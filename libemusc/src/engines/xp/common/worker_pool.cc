/* SPDX-License-Identifier: CC0-1.0 */
#include "worker_pool.h"

#ifdef EMUSC_RENDER_THREADS
#include <atomic>
#include <cfenv>
#include <condition_variable>
#include <mutex>
#include <thread>
#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#endif
#if defined(__SSE__)
#include <xmmintrin.h>
#endif
#endif

namespace EmuSC { namespace Xp {

#ifdef EMUSC_RENDER_THREADS

/* One run() is a generation. A worker takes part in a generation only while
   it is open, and run() closes it and waits for every worker that joined to
   leave before returning, so no worker can still be drawing indices from one
   generation's counter when the next generation resets it. */
struct WorkerPool::Shared {
  std::mutex mutex;
  std::condition_variable wake;
  std::thread workers[kMaxThreads];
  unsigned workerCount = 0;
  bool quit = false;
  bool open = false;
  std::atomic<unsigned> generation{0};
  Job job = nullptr;
  void *context = nullptr;
  unsigned count = 0;
  std::atomic<unsigned> next{0};
  std::atomic<unsigned> remaining{0};
  std::atomic<unsigned> busy{0};
  /* The floating-point environment of the thread that built the pool. A job
     computes on a worker exactly as it would on that thread only if the
     worker runs under the same rounding and flush-to-zero modes, so each
     worker takes it over before its first job rather than relying on the
     OS to have copied it. MXCSR is copied whole: fesetenv need not carry
     its flush-to-zero bit. */
  std::fenv_t fenv;
#if defined(__SSE__)
  unsigned mxcsr = 0;
#endif

  void work(Job j, void *c, unsigned n)
  {
    for (;;) {
      unsigned i = next.fetch_add(1u, std::memory_order_relaxed);
      if (i >= n)
        return;
      j(c, i);
      remaining.fetch_sub(1u, std::memory_order_release);
    }
  }

  void loop()
  {
    std::fesetenv(&fenv);
#if defined(__SSE__)
    _mm_setcsr(mxcsr);
#endif
    unsigned seen = 0;
    for (;;) {
      /* A block's jobs follow each other within microseconds, so a worker
         looks for the next generation for a short while before it sleeps;
         a condition-variable wake costs tens of microseconds on the small
         ARM cores this pool exists for. */
      for (unsigned spin = 0; spin < 4096u; ++spin) {
        if (generation.load(std::memory_order_acquire) != seen)
          break;
        std::this_thread::yield();
      }
      Job j;
      void *c;
      unsigned n;
      {
        std::unique_lock<std::mutex> lock(mutex);
        wake.wait(lock, [&] {
          return quit || generation.load(std::memory_order_relaxed) != seen;
        });
        if (quit)
          return;
        seen = generation.load(std::memory_order_relaxed);
        if (!open)
          continue;
        busy.fetch_add(1u, std::memory_order_relaxed);
        j = job;
        c = context;
        n = count;
      }
      work(j, c, n);
      busy.fetch_sub(1u, std::memory_order_release);
    }
  }
};

unsigned WorkerPool::available_cpus()
{
#if defined(__linux__)
  /* The affinity mask rather than the CPUs online: a core the kernel keeps
     for something else (isolcpus) is not one this process will be given. */
  cpu_set_t set;
  CPU_ZERO(&set);
  if (sched_getaffinity(0, sizeof set, &set) == 0) {
    int n = CPU_COUNT(&set);
    if (n > 0)
      return (unsigned)n;
  }
#endif
  unsigned n = std::thread::hardware_concurrency();
  return n ? n : 1u;
}

WorkerPool::WorkerPool(unsigned threads, const unsigned *workerCpus,
                       unsigned workerCpuCount)
  : _shared(nullptr), _threads(1)
{
  if (!threads)
    threads = available_cpus();
  if (threads > kMaxThreads)
    threads = kMaxThreads;
  if (threads <= 1)
    return;
  _shared = new Shared;
  std::fegetenv(&_shared->fenv);
#if defined(__SSE__)
  _shared->mxcsr = _mm_getcsr();
#endif
  for (unsigned i = 0; i + 1 < threads; ++i) {
    try {
      _shared->workers[i] = std::thread([s = _shared] { s->loop(); });
    } catch (...) {
      break;
    }
#if defined(__linux__)
    if (workerCpus && i < workerCpuCount && workerCpus[i] < CPU_SETSIZE) {
      cpu_set_t set;
      CPU_ZERO(&set);
      CPU_SET(workerCpus[i], &set);
      pthread_setaffinity_np(_shared->workers[i].native_handle(), sizeof set,
                             &set);
    }
#endif
    ++_shared->workerCount;
  }
  _threads = _shared->workerCount + 1u;
}

WorkerPool::~WorkerPool()
{
  if (!_shared)
    return;
  {
    std::lock_guard<std::mutex> lock(_shared->mutex);
    _shared->quit = true;
  }
  _shared->wake.notify_all();
  for (unsigned i = 0; i < _shared->workerCount; ++i)
    _shared->workers[i].join();
  delete _shared;
}

void WorkerPool::run(unsigned count, Job job, void *context)
{
  if (!_shared || count <= 1u) {
    for (unsigned i = 0; i < count; ++i)
      job(context, i);
    return;
  }
  Shared *s = _shared;
  {
    std::lock_guard<std::mutex> lock(s->mutex);
    s->job = job;
    s->context = context;
    s->count = count;
    s->next.store(0u, std::memory_order_relaxed);
    s->remaining.store(count, std::memory_order_relaxed);
    s->open = true;
    s->generation.fetch_add(1u, std::memory_order_release);
  }
  s->wake.notify_all();
  s->work(job, context, count);
  while (s->remaining.load(std::memory_order_acquire))
    std::this_thread::yield();
  {
    std::lock_guard<std::mutex> lock(s->mutex);
    s->open = false;
  }
  while (s->busy.load(std::memory_order_acquire))
    std::this_thread::yield();
}


/* The worker drains a ring of items under the mutex, running each task with
   it released. It drops to SCHED_IDLE, so it only ever runs on time no
   render thread wants, and widens its affinity back to every CPU, since it
   inherits a render thread's pinning from its creator. */
struct BackgroundWorker::Shared {
  std::mutex mutex;
  std::condition_variable wake;
  std::thread thread;
  bool quit = false;
  Task task;
  void *context;
  Item ring[kQueueItems];
  unsigned head = 0;   /* next to run */
  unsigned count = 0;
  /* The lock callers share with the task; separate from `mutex` so that
     posting never waits on a task in progress. */
  std::mutex data;

  void loop()
  {
#if defined(__linux__)
    sched_param param{};
    pthread_setschedparam(pthread_self(), SCHED_IDLE, &param);
    cpu_set_t set;
    CPU_ZERO(&set);
    long cpus = sysconf(_SC_NPROCESSORS_CONF);
    for (long c = 0; c < cpus && c < CPU_SETSIZE; ++c)
      CPU_SET(c, &set);
    pthread_setaffinity_np(pthread_self(), sizeof set, &set);
#endif
    for (;;) {
      Item item;
      {
        std::unique_lock<std::mutex> lock(mutex);
        wake.wait(lock, [&] { return quit || count; });
        if (quit)
          return;
        item = ring[head];
        head = (head + 1u) % kQueueItems;
        --count;
      }
      task(context, item);
    }
  }
};

BackgroundWorker::BackgroundWorker(Task task, void *context)
  : _shared(new Shared)
{
  _shared->task = task;
  _shared->context = context;
  try {
    _shared->thread = std::thread([s = _shared] { s->loop(); });
  } catch (...) {
    delete _shared;
    _shared = nullptr;
  }
}

BackgroundWorker::~BackgroundWorker()
{
  if (!_shared)
    return;
  {
    std::lock_guard<std::mutex> lock(_shared->mutex);
    _shared->quit = true;
  }
  _shared->wake.notify_one();
  _shared->thread.join();
  delete _shared;
}

bool BackgroundWorker::post(const Item &item)
{
  if (!_shared)
    return false;
  {
    std::lock_guard<std::mutex> lock(_shared->mutex);
    if (_shared->count == kQueueItems)
      return false;
    _shared->ring[(_shared->head + _shared->count) % kQueueItems] = item;
    ++_shared->count;
  }
  _shared->wake.notify_one();
  return true;
}

void BackgroundWorker::lock()
{
  if (_shared)
    _shared->data.lock();
}

void BackgroundWorker::unlock()
{
  if (_shared)
    _shared->data.unlock();
}

#else

struct WorkerPool::Shared {};

unsigned WorkerPool::available_cpus()
{
  return 1u;
}

WorkerPool::WorkerPool(unsigned, const unsigned *, unsigned)
  : _shared(nullptr), _threads(1)
{
}

WorkerPool::~WorkerPool()
{
}

void WorkerPool::run(unsigned count, Job job, void *context)
{
  for (unsigned i = 0; i < count; ++i)
    job(context, i);
}


struct BackgroundWorker::Shared {};

BackgroundWorker::BackgroundWorker(Task, void *)
  : _shared(nullptr)
{
}

BackgroundWorker::~BackgroundWorker()
{
}

bool BackgroundWorker::post(const Item &)
{
  return false;
}

void BackgroundWorker::lock()
{
}

void BackgroundWorker::unlock()
{
}

#endif

}}  // namespace EmuSC::Xp

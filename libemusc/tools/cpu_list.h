/*
 * SPDX-License-Identifier: CC0-1.0
 *
 * This file consists entirely of AI-generated code without direct human
 * authorship and is dedicated to the public domain under CC0 1.0.
 *
 * --cpus for the host tools that render (emuscd, emusc-render): which CPUs
 * the render threads run on. The calling thread takes the first CPU listed
 * and the render pool's threads the rest, from the last down, so a core the
 * kernel keeps out of scheduling (isolcpus) - which no thread is ever moved
 * onto - still gets a worker when it is listed. Linux only; elsewhere the
 * list parses and the pinning does nothing.
 */
#ifndef EMUSC_TOOLS_CPU_LIST_H
#define EMUSC_TOOLS_CPU_LIST_H

#include <exception>
#include <string>
#include <vector>

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#endif

namespace emusc_tools {

// A CPU list as taskset writes one: "0-3", "0,2,3", "1-2,3".
inline bool parse_cpu_list(const std::string &text,
                           std::vector<unsigned> &cpus)
{
  size_t i = 0;
  while (i < text.size()) {
    size_t comma = text.find(',', i);
    std::string item = text.substr(i, comma == std::string::npos
                                          ? std::string::npos : comma - i);
    size_t dash = item.find('-');
    try {
      unsigned first = (unsigned) std::stoul(item.substr(0, dash));
      unsigned last = dash == std::string::npos
        ? first : (unsigned) std::stoul(item.substr(dash + 1));
      if (last < first || last > 1023) return false;
      for (unsigned c = first; c <= last; c++) cpus.push_back(c);
    } catch (const std::exception &) {
      return false;
    }
    if (comma == std::string::npos) break;
    i = comma + 1;
  }
  return !cpus.empty();
}

// The pool's CPUs for a list: every CPU but the first, from the last down.
inline std::vector<unsigned> worker_cpus(const std::vector<unsigned> &cpus)
{
  std::vector<unsigned> workers;
  for (size_t i = cpus.size(); i-- > 1;) workers.push_back(cpus[i]);
  return workers;
}

// Pins the calling thread to `cpu`; false where that is refused or
// unsupported.
inline bool pin_this_thread(unsigned cpu)
{
#if defined(__linux__)
  if (cpu >= CPU_SETSIZE) return false;
  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  return pthread_setaffinity_np(pthread_self(), sizeof set, &set) == 0;
#else
  (void) cpu;
  return false;
#endif
}

}  // namespace emusc_tools

#endif

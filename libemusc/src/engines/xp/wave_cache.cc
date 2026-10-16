/* SPDX-License-Identifier: CC0-1.0 */
#include "wave_cache.h"

#include "common/worker_pool.h"

#include <cassert>
#include <cstdlib>
#include <cstring>

namespace EmuSC { namespace Xp {

namespace {

/* The preconditions fce_decode_storage and fce_decoder_read would refuse,
   checked before the cache is consulted so a descriptor the decoder
   rejects is rejected on a hit too. */
bool decodable(const struct XpDeviceProfile *profile, const uint8_t *bank,
               size_t bankSize, const struct xp_wave_descriptor *desc)
{
  return profile && bank && desc && bankSize >= profile->waveBankSize &&
    desc->address_a <= desc->address_c &&
    desc->address_c < profile->waveBankSize;
}

int32_t *decodeNew(const struct XpDeviceProfile *profile, const uint8_t *bank,
                   size_t bankSize, const struct xp_wave_descriptor *desc,
                   uint32_t *base, size_t *count)
{
  uint32_t first = desc->address_a & ~UINT32_C(0x0f);
  size_t capacity = (size_t)(desc->address_c - first) + 1;
  if (capacity > SIZE_MAX / sizeof(int32_t))
    return nullptr;
  int32_t *pcm = (int32_t *)std::malloc(capacity * sizeof *pcm);
  if (!pcm)
    return nullptr;
  if (!fce_decode_storage(profile, bank, bankSize, desc, pcm, capacity,
                           base, count)) {
    std::free(pcm);
    return nullptr;
  }
  return pcm;
}

/* Holds the cache's lock for one call, where it has one. */
struct Guard {
  EmuSC::Xp::BackgroundWorker *worker;
  explicit Guard(const struct xp_wave_cache *cache)
    : worker(cache ? cache->guard : nullptr)
  {
    if (worker)
      worker->lock();
  }
  ~Guard()
  {
    if (worker)
      worker->unlock();
  }
};

struct xp_wave_cache_entry *findEntry(struct xp_wave_cache *cache,
                                      const uint8_t *bank, uint32_t first,
                                      uint32_t last)
{
  for (unsigned i = 0; i < XP_WAVE_CACHE_ENTRIES; ++i) {
    struct xp_wave_cache_entry *entry = cache->entries + i;
    if (entry->pcm && entry->bank == bank && entry->base == first &&
        entry->last == last)
      return entry;
  }
  return nullptr;
}

/* Frees unreferenced entries, least recently used first, until the cache
   is within its byte budget or nothing unreferenced is left. */
void trimToBudget(struct xp_wave_cache *cache)
{
  while (cache->bytes > XP_WAVE_CACHE_BYTES) {
    struct xp_wave_cache_entry *oldest = nullptr;
    for (unsigned i = 0; i < XP_WAVE_CACHE_ENTRIES; ++i) {
      struct xp_wave_cache_entry *entry = cache->entries + i;
      if (entry->pcm && entry->refs == 0 &&
          (!oldest || entry->last_use < oldest->last_use))
        oldest = entry;
    }
    if (!oldest)
      return;
    cache->bytes -= oldest->count * sizeof *oldest->pcm;
    std::free(oldest->pcm);
    std::memset(oldest, 0, sizeof *oldest);
  }
}

}  // namespace

bool wave_cache_acquire(struct xp_wave_cache *cache,
                         const struct XpDeviceProfile *profile,
                         const uint8_t *bank, size_t bankSize,
                         const struct xp_wave_descriptor *desc,
                         const int32_t **pcm, uint32_t *base,
                         size_t *count)
{
  if (!pcm || !base || !count)
    return false;
  *pcm = nullptr;
  *base = 0;
  *count = 0;
  if (!decodable(profile, bank, bankSize, desc))
    return false;
  if (!cache) {
    *pcm = decodeNew(profile, bank, bankSize, desc, base, count);
    return *pcm != nullptr;
  }

  Guard guard(cache);
  const uint32_t first = desc->address_a & ~UINT32_C(0x0f);
  struct xp_wave_cache_entry *victim = nullptr;
  for (unsigned i = 0; i < XP_WAVE_CACHE_ENTRIES; ++i) {
    struct xp_wave_cache_entry *entry = cache->entries + i;
    if (!entry->pcm) {
      if (!victim || victim->pcm)
        victim = entry;
      continue;
    }
    if (entry->bank == bank && entry->base == first &&
        entry->last == desc->address_c) {
      ++entry->refs;
      entry->last_use = ++cache->clock;
      *pcm = entry->pcm;
      *base = entry->base;
      *count = entry->count;
      return true;
    }
    /* LIFETIME INVARIANT: an entry with live references is never chosen
       for replacement. A voice reads its PCM until it releases it, and
       several voices can hold the same entry at once, so replacing a
       referenced entry frees memory a sounding voice is still reading.
       Only an unreferenced entry - the least recently used one - may go;
       an empty entry is preferred over any occupied one. */
    if (entry->refs == 0 &&
        (!victim || (victim->pcm && entry->last_use < victim->last_use)))
      victim = entry;
  }

  uint32_t decodedBase;
  size_t decodedCount;
  int32_t *fresh = decodeNew(profile, bank, bankSize, desc, &decodedBase,
                             &decodedCount);
  if (!fresh)
    return false;
  if (victim) {
    assert(victim->refs == 0);
    if (victim->pcm)
      cache->bytes -= victim->count * sizeof *victim->pcm;
    std::free(victim->pcm);
    cache->bytes += decodedCount * sizeof *fresh;
    victim->pcm = fresh;
    victim->bank = bank;
    victim->base = decodedBase;
    victim->last = desc->address_c;
    victim->count = decodedCount;
    victim->refs = 1;
    victim->last_use = ++cache->clock;
    trimToBudget(cache);
  }
  /* With no victim every entry is referenced: `fresh` stays private to
     this caller and wave_cache_release frees it, since no entry holds
     it. */
  *pcm = fresh;
  *base = decodedBase;
  *count = decodedCount;
  return true;
}

bool wave_cache_prefetch(struct xp_wave_cache *cache,
                          const struct XpDeviceProfile *profile,
                          const uint8_t *bank, size_t bankSize,
                          const struct xp_wave_descriptor *desc)
{
  if (!cache || !decodable(profile, bank, bankSize, desc))
    return false;
  const uint32_t first = desc->address_a & ~UINT32_C(0x0f);
  {
    Guard guard(cache);
    if (findEntry(cache, bank, first, desc->address_c))
      return false;
  }
  uint32_t decodedBase;
  size_t decodedCount;
  int32_t *fresh = decodeNew(profile, bank, bankSize, desc, &decodedBase,
                             &decodedCount);
  if (!fresh)
    return false;
  Guard guard(cache);
  /* A note may have asked for it while it was decoding. */
  struct xp_wave_cache_entry *victim = nullptr;
  bool present = findEntry(cache, bank, first, desc->address_c) != nullptr;
  if (!present) {
    for (unsigned i = 0; i < XP_WAVE_CACHE_ENTRIES; ++i) {
      struct xp_wave_cache_entry *entry = cache->entries + i;
      if (!entry->pcm) {
        victim = entry;
        break;
      }
      if (entry->refs == 0 &&
          (!victim || entry->last_use < victim->last_use))
        victim = entry;
    }
  }
  if (!victim) {
    std::free(fresh);
    return false;
  }
  if (victim->pcm)
    cache->bytes -= victim->count * sizeof *victim->pcm;
  std::free(victim->pcm);
  cache->bytes += decodedCount * sizeof *fresh;
  victim->pcm = fresh;
  victim->bank = bank;
  victim->base = decodedBase;
  victim->last = desc->address_c;
  victim->count = decodedCount;
  victim->refs = 0;
  victim->last_use = ++cache->clock;
  trimToBudget(cache);
  return true;
}

void wave_cache_release(struct xp_wave_cache *cache, const int32_t *pcm)
{
  if (!pcm)
    return;
  Guard guard(cache);
  if (cache) {
    for (unsigned i = 0; i < XP_WAVE_CACHE_ENTRIES; ++i) {
      struct xp_wave_cache_entry *entry = cache->entries + i;
      if (entry->pcm == pcm) {
        assert(entry->refs > 0);
        if (entry->refs > 0)
          --entry->refs;
        if (entry->refs == 0)
          trimToBudget(cache);
        return;
      }
    }
  }
  std::free(const_cast<int32_t *>(pcm));
}

void wave_cache_clear(struct xp_wave_cache *cache)
{
  if (!cache)
    return;
  EmuSC::Xp::BackgroundWorker *guard = cache->guard;
  if (guard)
    guard->lock();
  for (unsigned i = 0; i < XP_WAVE_CACHE_ENTRIES; ++i) {
    assert(cache->entries[i].refs == 0);
    std::free(cache->entries[i].pcm);
  }
  std::memset(cache, 0, sizeof *cache);
  cache->guard = guard;
  if (guard)
    guard->unlock();
}

}}  // namespace EmuSC::Xp

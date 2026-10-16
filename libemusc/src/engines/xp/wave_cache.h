/* SPDX-License-Identifier: CC0-1.0 */
#ifndef EMUSC_XP_WAVE_CACHE_H
#define EMUSC_XP_WAVE_CACHE_H

#include "wave.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
namespace EmuSC { namespace Xp { class BackgroundWorker; } }

extern "C" {
#endif

/* Bounded by entry count and by bytes. A song plays a few hundred distinct
   waves - the JV-1080's first factory song 190 - and a decode evicted
   between two of its notes is decoded again on the next, inside the block
   that note starts in, so the table holds a whole song's worth. The byte
   budget bounds what that keeps resident: past it, unreferenced entries go,
   least recently used first. A referenced entry is never displaced, so a
   miss with every entry referenced decodes into a private buffer instead
   of growing the table. */
#define XP_WAVE_CACHE_ENTRIES 512
#define XP_WAVE_CACHE_BYTES ((size_t)64 << 20)

struct xp_wave_cache_entry {
  /* Null marks an empty entry. */
  int32_t *pcm;
  /* The key: which bank's bytes, and the exact address extent
     fce_decode_storage decodes - from the 16-sample block holding
     address_a through address_c inclusive. The output depends on nothing
     else, so two descriptors with the same key decode identically. */
  const uint8_t *bank;
  uint32_t base;
  uint32_t last;
  size_t count;
  /* Live references handed out by wave_cache_acquire and not yet given
     back through wave_cache_release. */
  unsigned refs;
  uint64_t last_use;
};

struct xp_wave_cache {
  struct xp_wave_cache_entry entries[XP_WAVE_CACHE_ENTRIES];
  uint64_t clock;
  /* Decoded bytes the entries hold. */
  size_t bytes;
  /* Where the cache is shared with a prefetching thread, the lock every
     call here takes; null where one thread owns the cache. Not owned. */
  EmuSC::Xp::BackgroundWorker *guard;
};

#ifdef __cplusplus
}

namespace EmuSC { namespace Xp {

// Decoded-PCM cache for the XP voice path. FCE decoding is a pure function
// of the bank's bytes and the decoded extent, so a cached decode is the
// same samples a fresh one would produce. One cache belongs to one
// renderer; a prefetching thread may share it once `guard` is set, and
// then every call takes guard's lock.

/* Hand out decoded PCM for `desc`, from the cache or freshly decoded.
 * On success `*pcm` holds `*count` samples starting at wave address
 * `*base`, and stays valid and unchanged until exactly one matching
 * wave_cache_release(cache, *pcm). A null `cache` decodes into a private
 * buffer every time, which release then frees. */
bool wave_cache_acquire(struct xp_wave_cache *cache,
                         const struct XpDeviceProfile *profile,
                         const uint8_t *bank, size_t bankSize,
                         const struct xp_wave_descriptor *desc,
                         const int32_t **pcm, uint32_t *base,
                         size_t *count);

/* Decode `desc` into the cache ahead of a note that will need it, as an
 * unreferenced entry. Decodes outside the lock and takes only an empty
 * entry or the least recently used unreferenced one, so it never displaces
 * a sounding voice's PCM. False if it was already cached or could not be
 * kept; either way the next acquire is answered with the same samples. */
bool wave_cache_prefetch(struct xp_wave_cache *cache,
                          const struct XpDeviceProfile *profile,
                          const uint8_t *bank, size_t bankSize,
                          const struct xp_wave_descriptor *desc);

/* Give back one reference from wave_cache_acquire. Null is a no-op. */
void wave_cache_release(struct xp_wave_cache *cache, const int32_t *pcm);

/* Free every entry. Only valid once every acquired reference has been
 * released; the cache is empty and reusable afterwards. */
void wave_cache_clear(struct xp_wave_cache *cache);

}}  // namespace EmuSC::Xp
#endif

#endif

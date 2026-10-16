/* SPDX-License-Identifier: CC0-1.0 */
#include "engine.h"

#include "common/constants.h"
#include "common/worker_pool.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <new>

namespace EmuSC { namespace Xp {

namespace {

void queueNote(struct xp_engine *engine, uint8_t note)
{
  engine->note_next_free[note] = XP_ENGINE_NONE;
  if (engine->free_note_tail == XP_ENGINE_NONE)
    engine->free_note_head = note;
  else
    engine->note_next_free[engine->free_note_tail] = note;
  engine->free_note_tail = note;
}

uint8_t popNote(struct xp_engine *engine)
{
  uint8_t note = engine->free_note_head;
  if (note == XP_ENGINE_NONE)
    return note;
  engine->free_note_head = engine->note_next_free[note];
  if (engine->free_note_head == XP_ENGINE_NONE)
    engine->free_note_tail = XP_ENGINE_NONE;
  engine->note_next_free[note] = XP_ENGINE_NONE;
  return note;
}

void queueSlot(struct xp_engine *engine, uint8_t slot)
{
  engine->slots[slot].next_free = XP_ENGINE_NONE;
  if (engine->free_slot_tail == XP_ENGINE_NONE)
    engine->free_slot_head = slot;
  else
    engine->slots[engine->free_slot_tail].next_free = slot;
  engine->free_slot_tail = slot;
  ++engine->free_slot_count;
}

uint8_t popSlot(struct xp_engine *engine)
{
  uint8_t slot = engine->free_slot_head;
  if (slot == XP_ENGINE_NONE)
    return slot;
  engine->free_slot_head = engine->slots[slot].next_free;
  if (engine->free_slot_head == XP_ENGINE_NONE)
    engine->free_slot_tail = XP_ENGINE_NONE;
  engine->slots[slot].next_free = XP_ENGINE_NONE;
  --engine->free_slot_count;
  return slot;
}

/* The oscillator's pitch contribution, in pitch-word units.
 *
 * Exact: the waveform, the rate (`increment * 124.987501249875 / 65536` Hz)
 * and the delay/fade ramp, whose fade multiplies the depth. Exact too is
 * the depth word the controller matrix forms, `(depth * value) >> 2`
 * summed over its sources.
 *
 * Calibrated, not decoded: what that word is worth in cents. The manual
 * publishes one figure - the initial modulation-to-LFO1-pitch depth of
 * `0x0a` is 47 cents - and at full modulation that depth gives the word
 * `(10 * 127) >> 2 = 317`, so 317 stands for 47 cents. The scaling of
 * these words into XP pitch state is listed as still being decoded in
 * `07_synthesis/lfo.md`, so this is the anchor available (`M-019`).
 */
int32_t lfoPitchOffset(const struct xp_engine *engine,
                        const struct xp_engine_slot *slot, uint8_t part)
{
  double cents = 0.0;
  uint16_t matrix = engine->parts[part].lfo1_pitch_depth;
  int16_t local = slot->component.lfo2_pitch_depth;
  int16_t common = slot->component.lfo1_pitch_depth;
  if (matrix)
    cents += 47.0 / 317.0 * (double)matrix *
      ((double)slot->component.lfo1.ramp.fade / 65535.0) *
      ((double)slot->component.lfo1.output / 32767.0);
  /* The tone-common oscillator's vibrato is READ but not applied.

     Its depth now reaches the voice correctly resolved - component byte
     `+17` through the curve at `0x78304`, as `05_data_model` and
     `07_synthesis/pitch.md` specify - and every property of that curve
     verifies against the ROM. What is not established is the unit of the
     curve's OUTPUT. Sharing the local field's +/-4032 bound does not
     prove it shares the local field's unit, and applying the manual's
     anchor to it measures worse at every magnitude tried: the attack
     excursion over the seven hardware recordings is 0.67 with this term
     absent, 0.67 at a quarter of the anchor, and 0.37 at the anchor
     itself. A term that only ever subtracts is not merely mis-scaled.

     The likeliest reason is one this code does not model: `lfo.md`
     records the tone-common oscillator as SHARED between voices, and
     detached when its owner is released, so "an implementation cannot
     give every voice an independent phase unconditionally". Ours does.
     Independent phases make an ensemble's vibrato incoherent, which
     smears exactly what this metric measures. */
  /* Instrumentation only: apply it at a settable fraction of the local
     field's anchor, so the unit can be calibrated against a direct
     observable instead of a whole-song proxy. Off unless asked. */
  if (common) {
    const char *scale = std::getenv("XP_COMMON_VIBRATO");
    if (scale) {
      double k = std::atof(scale);
      if (k != 0.0)
        cents += k * 47.0 / 317.0 * (double)common *
          ((double)slot->component.lfo1.ramp.fade / 65535.0) *
          ((double)slot->component.lfo1.output / 32767.0);
    }
  }
  /* The local oscillator's vibrato. Its field shares the matrix's units -
     both saturate at exactly 4032, which is `(127 * 127) >> 2` - so the
     same 47-cents-per-317 anchor applies (`M-020`). */
  if (local)
    cents += 47.0 / 317.0 * (double)local *
      ((double)slot->component.lfo2.ramp.fade / 65535.0) *
      ((double)slot->component.lfo2.output / 32767.0);
  if (cents == 0.0)
    return 0;
  /* 0x4000 pitch-word units to the octave */
  return (int32_t)(cents * 16384.0 / 1200.0);
}

/* One oscillator's contribution, as its depth scaled by the waveform and
   the fade. Both oscillators sum, which is what `07_synthesis/lfo.md`
   describes: two waveforms, each multiplied by its own depth term and its
   own fade. */
double lfoTerm(const struct xp_lfo *lfo, int16_t depth)
{
  if (!depth)
    return 0.0;
  return (double)depth * ((double)lfo->ramp.fade / 65535.0) *
    ((double)lfo->output / 32767.0);
}

/* The amplitude modulation, as a gain. The depths are attenuation-word
   units and the level tables run at about -5.26 dB per 0x1000, so one unit
   is 0.001284 dB. */
float lfoAmplitude(const struct xp_engine_slot *slot)
{
  double attenuation =
    lfoTerm(&slot->component.lfo1, slot->component.lfo1_tva_depth) +
    lfoTerm(&slot->component.lfo2, slot->component.lfo2_tva_depth);
  if (attenuation == 0.0)
    return 1.0f;
  return (float)std::pow(10.0, -attenuation * 0.001284 / 20.0);
}

/* One oscillator's filter term. The fade reaches this path as the high
   word of the tone's depth times the ramp (`0x6b11`, `0x6be9`), not as
   the fraction the pitch and amplitude paths take, and the clamp, scale
   and waveform multiply that follow are `tvf_lfo_filter_term`. The
   part-level depth the controller matrix adds between the two
   (`0x6b23`, `0x6c01`) is not wired. */
int16_t lfoFilterTerm(const struct xp_lfo *lfo, int16_t depth)
{
  if (!depth)
    return 0;
  int32_t faded = (int32_t)depth * (int32_t)lfo->ramp.fade;
  faded = faded >= 0 ? faded / 65536
                     : -(int32_t)(((uint32_t)(-faded) + 65535u) >> 16);
  return tvf_lfo_filter_term((int16_t)faded, lfo->output);
}

/* The filter modulation, in the pre-base accumulator's units. Both terms
   are added into the word at RAM 30da beside the key and controller
   terms (`0x6bd7`, `0x6cbb`) with plain 16-bit adds, so the sum wraps -
   the accumulator is the same register either term alone would reach. */
int16_t lfoFilter(const struct xp_engine_slot *slot)
{
  return (int16_t)((uint16_t)lfoFilterTerm(&slot->component.lfo1,
                                            slot->component.lfo1_tvf_depth) +
                   (uint16_t)lfoFilterTerm(&slot->component.lfo2,
                                            slot->component.lfo2_tvf_depth));
}

int sharedLfoSlot(struct xp_engine *engine, uint32_t tone, uint32_t comp,
                   uint8_t which);
void sharedLfoJoin(struct xp_engine *engine, uint32_t tone, uint32_t comp,
                    uint8_t which, struct xp_lfo *lfo);

void updateSlotPitch(struct xp_engine *engine,
                      struct xp_engine_slot *slot)
{
  if (!engine || !slot || !slot->allocated ||
      slot->note >= XP_ENGINE_NOTE_COUNT)
    return;
  const struct xp_engine_note *note = engine->notes + slot->note;
  uint32_t base = slot->component.static_pitch_word;
  /* While the glide runs the key itself is moving, so the static word is
     recomposed from it rather than offset - SC88-CTL 0x6063 and 0x6077 do
     exactly that every control period. The glide ends on the target key,
     where the recomposition returns the note-on word unchanged. */
  if (slot->component.portamento.active &&
      !renderer_pitch_word_at(&engine->renderer->rom,
                              &slot->component.portamento,
                              slot->component.portamento.current, &base))
    base = slot->component.static_pitch_word;
  uint32_t word = pitch_current_word(
    base,
    engine->parts[note->part].pitch_offset +
      lfoPitchOffset(engine, slot, note->part),
    pitch_envelope_sum(&slot->component.pitch_envelope,
                        &slot->component.pitch_release));
  oscillator_set_step(&slot->component.oscillator,
                       pitch_word_rate(word, engine->renderer->output_rate));
}

void freeNoteIfEmpty(struct xp_engine *engine, uint8_t noteIndex)
{
  struct xp_engine_note *note = engine->notes + noteIndex;
  if (!note->allocated || note->slot_count)
    return;
  std::memset(note, 0, sizeof *note);
  queueNote(engine, noteIndex);
}

void freeSlot(struct xp_engine *engine, uint8_t slotIndex, bool prepend)
{
  struct xp_engine_slot *slot = engine->slots + slotIndex;
  if (!slot->allocated)
    return;
  for (unsigned k = 0; k < engine->active_slot_count; ++k) {
    if (engine->active_slots[k] == slotIndex) {
      --engine->active_slot_count;
      for (unsigned m = k; m < engine->active_slot_count; ++m)
        engine->active_slots[m] = engine->active_slots[m + 1];
      break;
    }
  }
  uint8_t noteIndex = slot->note;
  if (noteIndex < XP_ENGINE_NOTE_COUNT) {
    struct xp_engine_note *note = engine->notes + noteIndex;
    for (unsigned i = 0; i < XP_MAX_TONE_COMPONENTS; ++i) {
      if (note->slots[i] == slotIndex) {
        note->slots[i] = XP_ENGINE_NONE;
        --note->slot_count;
        break;
      }
    }
  }
  renderer_component_release(engine->renderer, &slot->component);
  std::memset(slot, 0, sizeof *slot);
  slot->note = XP_ENGINE_NONE;
  if (prepend) {
    slot->next_free = engine->free_slot_head;
    engine->free_slot_head = slotIndex;
    if (engine->free_slot_tail == XP_ENGINE_NONE)
      engine->free_slot_tail = slotIndex;
    ++engine->free_slot_count;
  } else {
    queueSlot(engine, slotIndex);
  }
  if (noteIndex < XP_ENGINE_NOTE_COUNT)
    freeNoteIfEmpty(engine, noteIndex);
}

void startRelease(struct xp_engine *engine, uint8_t noteIndex)
{
  struct xp_engine_note *note = engine->notes + noteIndex;
  if (!note->allocated || note->key_down || note->hold_retained ||
      note->sostenuto_retained)
    return;
  for (unsigned i = 0; i < XP_MAX_TONE_COMPONENTS; ++i) {
    uint8_t slotIndex = note->slots[i];
    if (slotIndex == XP_ENGINE_NONE)
      continue;
    struct xp_render_component *component = &engine->slots[slotIndex].component;
    if (!component->release.active) {
      /* `58b7`, `58bb`, `58bf`: the three release ramps go up for every
         tone, whatever follows. */
      (void)tva_release_set_pedal(
        &engine->renderer->rom, engine->parts[note->part].hold_value,
        component->continuous_hold_release,
        component->keep_release_scale_at_zero, false,
        &component->release);
      (void)tvf_release_set_pedal(
        &engine->renderer->rom, engine->parts[note->part].hold_value,
        component->continuous_hold_release,
        component->keep_release_scale_at_zero, false,
        &component->tvf_release);
      (void)pitch_release_activate(
        &engine->renderer->rom, engine->parts[note->part].hold_value,
        component->continuous_hold_release,
        component->keep_release_scale_at_zero, false,
        &component->pitch_release);
      /* `58c3`, the guard on everything from `58c9`. Tone-common `+0x15`
         raises bit 1 of the voice's flag byte at `0x1ef4` (`58a4..58a9`),
         and a voice carrying that bit branches straight to the tail: the
         envelope stages are NOT forced to 4 and the level deltas below are
         not taken, so the envelopes keep running underneath the release
         ramp instead of stopping where the key left them. 23 of the ROM's
         922 tones set the byte and all but two are piano or electric
         piano; on those, the tail sits up to 1.3 dB lower than it did.

         `58f5..58f7` guards the TVA arm a second time, on Hold 1 being
         up. It is unreachable from here: a note released under a held
         pedal is `hold_retained` and this function has returned above. */
      if (!component->continuous_hold_release) {
        /* `58c9`, and `58ce..58d6`: the word that carried the release
           destination becomes the distance the ramp has left to cover. */
        component->pitch_envelope.stage = 4;
        component->pitch_envelope.active = false;
        component->pitch_release.delta = (int16_t)(
          (uint16_t)component->pitch_release.destination -
          (uint16_t)component->pitch_envelope.current);
        /* `58da`, and `58df..58e7`: the same for the filter. Stage 4 makes
           `6e6b` return before the envelope clock, so `0x335a` holds where
           the key left it, and the release ramp `0x36da` - which `6967`
           adds beside it - covers the distance from there to the release
           level. The composed offset therefore ends on the release level
           itself. A tone with envelope depth 0 never runs `6e6b` and has
           both words at zero, which the zero target below reproduces. */
        component->tvf_envelope.stage = 4;
        component->tvf_envelope.active = false;
        component->tvf_release.target = (int16_t)(
          (uint16_t)component->tvf_release.target -
          (uint16_t)component->tvf_envelope.current);
        /* `58f9`: TVA stage 4, which is this envelope standing still. */
        tva_envelope_freeze(
          &engine->renderer->rom, &component->envelope,
          engine->scheduler_clocks / kXpControlPeriodClocks);
      }
    }
  }
}

uint8_t oldestSlot(const struct xp_engine *engine, bool releasedOnly)
{
  uint8_t candidate = XP_ENGINE_NONE;
  uint64_t serial = UINT64_MAX;
  for (unsigned i = 0; i < XP_ENGINE_SLOT_COUNT; ++i) {
    const struct xp_engine_slot *slot = engine->slots + i;
    if (!slot->allocated || slot->note >= XP_ENGINE_NOTE_COUNT)
      continue;
    const struct xp_engine_note *note = engine->notes + slot->note;
    if (releasedOnly && note->key_down)
      continue;
    if (slot->serial < serial) {
      candidate = (uint8_t)i;
      serial = slot->serial;
    }
  }
  return candidate;
}

void stopVoice(struct xp_engine *engine, uint8_t slotIndex);

void reclaimSlots(struct xp_engine *engine, unsigned count)
{
  while (count--) {
    uint8_t slot = oldestSlot(engine, true);
    if (slot == XP_ENGINE_NONE)
      slot = oldestSlot(engine, false);
    if (slot == XP_ENGINE_NONE)
      return;
    /* A stolen voice is still sounding when the CPU takes its slot back
       for a different note - the same circumstance same-note recycle
       hands to the stop ramp below, and the same traced routine (0x4c60)
       serves both (P-0358). free_slot's memset would end it here, the
       hard cut TASK-189 AC#1 is about; run it down instead. */
    stopVoice(engine, slot);
    freeSlot(engine, slot, false);
  }
}

bool noteMatches(const struct xp_engine_note *note, uint8_t part,
                  uint8_t key, uint32_t toneOffset, uint8_t context)
{
  return note->allocated && note->part == part && note->key == key &&
    note->tone_offset == toneOffset && note->context == context;
}

/* Hand a still-sounding voice to the chip's stop ramp, so that taking its
   slot back does not end the waveform where it stands.

   The register is picked up where it stands this instant and given the
   target zero; everything else the CPU composed is frozen with it, so the
   first ramped sample continues the last serviced one. See
   `xp_engine_stopping` for what is traced here and what is not: the slot
   accounting is, the stop's effect on the sound is not. */
void stopVoice(struct xp_engine *engine, uint8_t slotIndex)
{
  struct xp_engine_slot *slot = engine->slots + slotIndex;
  double fraction = engine->scheduler_clocks / kXpControlPeriodClocks;
  if (!slot->allocated || !slot->component.active ||
      slot->note >= XP_ENGINE_NOTE_COUNT)
    return;
  uint32_t current = xp_render_static_gain_q17(&slot->component, fraction);
  if (!current)
    return;
  for (unsigned i = 0; i < XP_ENGINE_STOPPING_COUNT; ++i) {
    struct xp_engine_stopping *stop = engine->stopping + i;
    if (stop->active)
      continue;
    stop->gain =
      (tva_envelope_linear_q17(&engine->renderer->rom,
                                &slot->component.envelope, fraction) /
       131072.0f) *
      lfoAmplitude(slot) *
      engine->notes[slot->note].provisional_gain;
    stop->part = engine->notes[slot->note].part;
    stop->periods = 0.0;
    stop->component = slot->component;
    stop->component.static_gain_current_q17 = current;
    stop->component.static_gain_q17 = 0;
    /* The wave reference moves with the voice; the slot must not release
       the buffer the ramp is still reading. */
    slot->component.pcm24 = nullptr;
    stop->active = true;
    return;
  }
  /* Nowhere to put it: the voice ends where it stands. */
}

void recycleNote(struct xp_engine *engine, uint8_t noteIndex)
{
  uint8_t slots[XP_MAX_TONE_COMPONENTS];
  std::memcpy(slots, engine->notes[noteIndex].slots, sizeof slots);
  for (unsigned i = 0; i < XP_MAX_TONE_COMPONENTS; ++i) {
    uint8_t slot = slots[XP_MAX_TONE_COMPONENTS - 1 - i];
    if (slot == XP_ENGINE_NONE)
      continue;
    /* The slot goes back on the free list this instant and at its head, as
       `0x2333` does; the sound of the voice it held runs down separately. */
    stopVoice(engine, slot);
    freeSlot(engine, slot, true);
  }
}

void applySameNoteMode(struct xp_engine *engine,
                        const struct xp_render_voice *voice, uint8_t part,
                        uint8_t key, uint8_t context,
                        enum xp_same_note_mode mode)
{
  uint8_t oldest = XP_ENGINE_NONE;
  uint64_t oldestSerial = UINT64_MAX;
  unsigned matches = 0;
  if (mode == XP_SAME_NOTE_FULL_MULTI)
    return;
  for (unsigned i = 0; i < XP_ENGINE_NOTE_COUNT; ++i) {
    const struct xp_engine_note *note = engine->notes + i;
    if (!noteMatches(note, part, key, voice->tone_offset, context))
      continue;
    ++matches;
    if (note->serial < oldestSerial) {
      oldest = (uint8_t)i;
      oldestSerial = note->serial;
    }
  }
  if (oldest != XP_ENGINE_NONE &&
      (mode == XP_SAME_NOTE_SINGLE || matches >= 2))
    recycleNote(engine, oldest);
}

/* The position a part-pan of zero asks for: seven bits with zero rejected,
   which is what the firmware takes from the XP readback. The chip's own
   distribution and seeding are not in the CPU path, so the sequence here is
   a stand-in; only the range and the per-voice freshness are recovered
   (`09_mixer/mixer_output.md`). */
uint8_t panDraw(struct xp_engine *engine)
{
  for (unsigned tries = 0; tries < 8u; ++tries) {
    uint16_t s = engine->pan_seed;
    s ^= (uint16_t)(s << 7);
    s ^= (uint16_t)(s >> 9);
    s ^= (uint16_t)(s << 8);
    engine->pan_seed = s;
    if ((uint8_t)(s & 0x7fu))
      return (uint8_t)(s & 0x7fu);
  }
  return 64u;
}

/* Where the next note on this part glides FROM, as a 16.16 key.
 *
 * On the device the source is not looked up at all: `0x5fb0` takes it from
 * `0x245a + slot`, the voice slot's own current key, and it is the previous
 * note's because the mono/portamento allocation path hands the new note that
 * part's sounding voice. That path (`0x1c09..0x1e80`) is listed as
 * unrecovered in scdb `06_voice_engine/note_lifecycle.md` - "the exact
 * portamento glide inputs through those paths remain to be recovered" - and
 * it is not reproduced here. What is reproduced is its outcome for the case
 * portamento exists for: the newest note still allocated on the part, read
 * at its live glide position so that a note arriving mid-glide continues
 * from where the glide stands rather than from where it was aimed.
 *
 * For overlapping notes on one part the device's answer depends on which
 * slot the allocator hands over and ours does not, so ours is A source and
 * not necessarily THE source. It is stated here rather than in a comment
 * somewhere downstream because it is the one part of this that is not read
 * off the ROM.
 *
 * CC84 overrides it and is consumed here, whatever the switch says, because
 * `0x2c76` clears `d8a0 + part` on every melodic Note On. */
bool glideSource(struct xp_engine *engine, uint8_t part, uint32_t *from)
{
  uint8_t control = engine->parts[part].portamento_control;
  engine->parts[part].portamento_control = 0xffu;
  if (!engine->parts[part].portamento || !engine->parts[part].portamento_time)
    return false;
  if (control <= 127) {
    *from = (uint32_t)control << 16;
    return true;
  }
  uint8_t newest = XP_ENGINE_NONE;
  uint64_t serial = 0;
  for (unsigned i = 0; i < XP_ENGINE_NOTE_COUNT; ++i) {
    const struct xp_engine_note *note = engine->notes + i;
    if (!note->allocated || note->part != part || note->slot_count == 0)
      continue;
    if (newest == XP_ENGINE_NONE || note->serial > serial) {
      newest = (uint8_t)i;
      serial = note->serial;
    }
  }
  if (newest == XP_ENGINE_NONE)
    return false;
  *from = engine->slots[engine->notes[newest].slots[0]].component
            .portamento.current;
  return true;
}

/* The shared-oscillator table. A voice whose share byte is nonzero does not
   own its oscillator: it joins the one already running for its tone, and
   the first voice of a tone creates it. The entry outlives any single
   voice, which is what the firmware achieves by copying the oscillator's
   words into a replacement owner when the original is released. */
int sharedLfoSlot(struct xp_engine *engine, uint32_t tone, uint32_t comp,
                   uint8_t which)
{
  for (unsigned i = 0; i < XP_ENGINE_SHARED_LFO_COUNT; ++i) {
    const struct xp_engine_shared_lfo *e = engine->shared_lfo + i;
    if (e->active && e->which == which && e->tone_offset == tone &&
        e->component_offset == comp)
      return (int)i;
  }
  return -1;
}

void sharedLfoJoin(struct xp_engine *engine, uint32_t tone, uint32_t comp,
                    uint8_t which, struct xp_lfo *lfo)
{
  int at = sharedLfoSlot(engine, tone, comp, which);
  if (at >= 0) {
    /* Adopt the running oscillator's phase and output. The ramp stays the
       voice's own: it is the note's delay and fade, not the waveform. */
    struct xp_lfo *shared = &engine->shared_lfo[at].lfo;
    lfo->phase = shared->phase;
    lfo->output = shared->output;
    lfo->random_target = shared->random_target;
    return;
  }
  for (unsigned i = 0; i < XP_ENGINE_SHARED_LFO_COUNT; ++i) {
    struct xp_engine_shared_lfo *e = engine->shared_lfo + i;
    if (e->active)
      continue;
    e->active = true;
    e->used = true;
    e->which = which;
    e->tone_offset = tone;
    e->component_offset = comp;
    e->lfo = *lfo;
    return;
  }
  /* Table full: the voice keeps its own oscillator, as it would if the
     firmware's comparison had failed. */
}

void runScheduler(struct xp_engine *engine)
{
  engine->scheduler_clocks += kXpControlTimerHz / engine->renderer->output_rate;
  unsigned elapsed =
    (unsigned)(engine->scheduler_clocks / kXpControlPeriodClocks);
  if (!elapsed)
    return;
  engine->scheduler_clocks -= elapsed * kXpControlPeriodClocks;

  /* One oscillator per sharing tone, advanced once for the whole period
     before any voice reads it. Entries nothing used last period are
     retired, which is how a shared oscillator outlives its first owner
     and stops when the last voice of its tone does. */
  for (unsigned k = 0; k < XP_ENGINE_SHARED_LFO_COUNT; ++k) {
    struct xp_engine_shared_lfo *e = engine->shared_lfo + k;
    if (!e->active)
      continue;
    if (!e->used) {
      e->active = false;
      continue;
    }
    e->used = false;
    (void)lfo_advance(&engine->renderer->rom, &e->lfo, 0,
                       (uint8_t)(elapsed - 1u), &engine->lfo_seed);
  }
  /* Slots at or past max_voices are never allocated (engine_set_max_voices
     never adds them to the free list), so bounding this - the per-voice
     per-control-tick work - to max_voices instead of the full
     XP_ENGINE_SLOT_COUNT is exact, not an approximation. */
  for (unsigned i = 0; i < engine->max_voices; ++i) {
    struct xp_engine_slot *slot = engine->slots + i;
    if (!slot->allocated)
      continue;
    /* The period that has just ended is the one the chip's amplitude
       register spent approaching the target composed for it, so the
       register now stands where `0x2a7` left it - two parts in a hundred
       thousand short of the target, not on it. */
    if (slot->component.release_zeroed) {
      freeSlot(engine, (uint8_t)i, false);
      continue;
    }
    slot->component.static_gain_current_q17 =
      xp_render_static_gain_q17(&slot->component, 1.0);
    struct xp_engine_note *note = engine->notes + slot->note;
    if (slot->component.pan_position < slot->component.pan_target_position)
      ++slot->component.pan_position;
    else if (slot->component.pan_position >
             slot->component.pan_target_position)
      --slot->component.pan_position;
    (void)pan_pair_q15(&engine->renderer->rom, slot->component.pan_position,
                       &slot->component.left_gain_q15,
                       &slot->component.right_gain_q15);
    /* A shared oscillator was advanced once for the whole tone above; this
       voice reads it rather than running its own, which is what keeps an
       ensemble's vibrato coherent. The ramp is always the voice's own: it
       is the note's delay and fade, not the waveform. It runs on its own
       clock and is not gated by a stalled oscillator. */
    {
      uint32_t tone = engine->notes[slot->note].tone_offset;
      int at = slot->component.lfo1.share_request
        ? sharedLfoSlot(engine, tone, 0, 1) : -1;
      if (at >= 0) {
        engine->shared_lfo[at].used = true;
        slot->component.lfo1.phase = engine->shared_lfo[at].lfo.phase;
        slot->component.lfo1.output = engine->shared_lfo[at].lfo.output;
        slot->component.lfo1.random_target =
          engine->shared_lfo[at].lfo.random_target;
      } else {
        (void)lfo_advance(&engine->renderer->rom, &slot->component.lfo1, 0,
                           (uint8_t)(elapsed - 1u), &engine->lfo_seed);
      }
      at = slot->component.lfo2.share_request
        ? sharedLfoSlot(engine, tone, slot->component.rom_component_offset, 2)
        : -1;
      if (at >= 0) {
        engine->shared_lfo[at].used = true;
        slot->component.lfo2.phase = engine->shared_lfo[at].lfo.phase;
        slot->component.lfo2.output = engine->shared_lfo[at].lfo.output;
        slot->component.lfo2.random_target =
          engine->shared_lfo[at].lfo.random_target;
      } else {
        (void)lfo_advance(&engine->renderer->rom, &slot->component.lfo2, 0,
                           (uint8_t)(elapsed - 1u), &engine->lfo_seed);
      }
    }
    (void)lfo_ramp_advance(&slot->component.lfo1.ramp, (uint8_t)(elapsed - 1u));
    (void)lfo_ramp_advance(&slot->component.lfo2.ramp, (uint8_t)(elapsed - 1u));
    slot->lfo_amplitude_gain = lfoAmplitude(slot);
    if (slot->component.envelope.active)
      (void)tva_envelope_advance(&engine->renderer->rom,
                                  &slot->component.envelope, elapsed);
    if (slot->component.pitch_envelope.active)
      (void)pitch_envelope_advance(&slot->component.pitch_envelope, elapsed);
    if (slot->component.pitch_release.active)
      (void)pitch_release_advance(&slot->component.pitch_release, elapsed);
    portamento_advance(&slot->component.portamento, elapsed);
    static const bool traceEnabled = std::getenv("XP_TRACE_PITCH") != nullptr;
    if (traceEnabled) {
      static unsigned n;
      if (n < 10)
        std::fprintf(stderr, "period %2u  pitch env current %6d stage %u "
                "phase %5u active %d\n", n++,
                slot->component.pitch_envelope.current,
                slot->component.pitch_envelope.stage,
                slot->component.pitch_envelope.phase,
                (int)slot->component.pitch_envelope.active);
      if (n == 1) {
        std::fprintf(stderr, "          tva stages:");
        for (unsigned q = 0; q < 4; ++q)
          std::fprintf(stderr, " [%u] target %5u inc %5u",
                  q, slot->component.envelope.target_attenuations[q],
                  slot->component.envelope.increments[q]);
        std::fprintf(stderr, "\n");
      }
      if (n <= 2)
        std::fprintf(stderr, "          tva stage %u atten start %5u target %5u "
                "inc %5u phase %5u gain_q17 %8u\n",
                slot->component.envelope.stage,
                slot->component.envelope.start_attenuation,
                slot->component.envelope.target_attenuations[
                  slot->component.envelope.stage],
                slot->component.envelope.increments[
                  slot->component.envelope.stage],
                slot->component.envelope.phase,
                slot->component.envelope.current_q17);
    }
    updateSlotPitch(engine, slot);
    /* The approach of TVF-F and TVF-Q toward their targets is the chip's
       own interpolation and runs every period; it does not wait for the
       CPU's envelope. A tone with envelope depth 0 has no active envelope
       or release at all, and gating the approach on them left such a
       tone's resonance at the quarter-target the note opens with.

       It runs FIRST, and once, because it closes out the period that has
       just ended: the register spent that period approaching the target
       standing during it, and that target is still the one in the struct.
       Everything below composes the target for the period about to start,
       which is what the audio path then reads through `period_fraction`.
       Composing first and approaching afterwards makes the register cover
       its whole gap to a target the period has not begun with. */
    tvf_advance_registers(&slot->component.tvf, elapsed);
    /* `0x6a27`, the base recompose. The firmware runs it every serviced
       period (`0x695b` `1e 00 c9`, unconditional); we run it when a part
       controller has moved or the filter LFO term has, which is the same
       thing here because its other inputs - the component's own bytes,
       the note's key modulation and the part's cutoff and resonance
       controls - do not change otherwise.

       It clears the register struct, so the two values the CPU does not
       rewrite mid-note are carried across it by hand. The firmware writes
       the TVF-F current value only at note on (`0x68d0`/`0x68d3`); every
       serviced period after that, `0x69a1`..`0x69af` writes the target and
       the 0x4100 interpolation word beside it and nothing else. So a
       retarget leaves the chip's register exactly where it stood and the
       approach carries its own progress. */
    {
      int16_t lfoFilterValue = lfoFilter(slot);
      if (engine->parts[note->part].tvf_dirty ||
          lfoFilterValue != slot->tvf_lfo_term) {
        struct xp_component component;
        uint32_t previousCurrent = slot->component.tvf.frequency_current;
        uint32_t previousResonance = slot->component.tvf.resonance_current;
        component.bytes = engine->renderer->rom.bytes +
          slot->component.rom_component_offset;
        component.offset = slot->component.rom_component_offset;
        component.directory_offset = 0;
        if (tvf_prepare_registers(
              &engine->renderer->rom, &component,
              (int16_t)((uint16_t)slot->component.tvf_key_modulation +
                        (uint16_t)lfoFilterValue),
              &engine->parts[note->part].tvf_controls,
              &slot->component.tvf)) {
          slot->component.tvf.frequency_current = previousCurrent;
          slot->component.tvf.resonance_current = previousResonance;
        }
        slot->tvf_lfo_term = lfoFilterValue;
      }
    }
    /* `0x6e6b`, called from `0x6964` when the envelope depth at 0x32da is
       nonzero. It follows the base recompose and precedes the
       composition, so the word the chip is given carries this period's
       envelope, not last period's. */
    if (slot->component.tvf_envelope.active)
      (void)tvf_envelope_advance(&slot->component.tvf_envelope, elapsed);
    if (slot->component.tvf_release.active)
      (void)tvf_release_advance(&slot->component.tvf_release, elapsed);
    /* `0x6967`..`0x69af`: one composition per serviced period, carrying
       the envelope and the release, written as the chip's target with
       0x4100 beside it. The filter LFO is not here - it reached the
       accumulator above, before the shift right one. */
    (void)tvf_update_frequency(
      &engine->renderer->rom,
      (int16_t)((uint16_t)slot->component.tvf_envelope.current +
                (uint16_t)slot->component.tvf_release.current),
      &slot->component.tvf);
    if (!slot->component.release.active)
      continue;
    /* `7228..7232`: when the release counter underflows, the firmware
       clears it, drops the release flag and composes amplitude 0, which
       `71a9` writes as the chip's TARGET with the interpolation word
       beside it like any other. The chip glides down to it. Freeing the
       slot in the period the zero is composed cuts the voice where it
       stands instead, and what it stands at is 4/131072 - the floor the
       gain table puts under every decay (`72bf`, and `coarse[0]*fine[n]`
       quantising to 1 for every remaining headroom below about 1024).
       A steady level ending in a step is a click, and this one is on the
       end of every note. */
    if (!tva_release_advance(&slot->component.release, elapsed) ||
        !tva_gain_from_headroom_q17(
          &engine->renderer->rom, slot->component.release.current,
          &note->levels, slot->component.drum_level,
          slot->component.static_attenuation,
          &slot->component.static_gain_q17) ||
        slot->component.static_gain_q17 == 0) {
      slot->component.static_gain_q17 = 0;
      slot->component.release_zeroed = true;
    }
  }
  for (unsigned i = 0; i < XP_ENGINE_PART_COUNT; ++i)
    engine->parts[i].tvf_dirty = false;
  if (engine->control_service)
    engine->control_service(engine->control_user, elapsed);
}

}  // namespace

bool engine_init(struct xp_engine *engine,
                  const struct xp_renderer *renderer)
{
  if (!engine || !renderer || renderer->output_rate <= 0.0)
    return false;
  std::memset(engine, 0, sizeof *engine);
  engine->renderer = renderer;
  engine->free_note_head = 0;
  engine->free_note_tail = XP_ENGINE_NOTE_COUNT - 1;
  engine->free_slot_head = 0;
  engine->free_slot_tail = XP_ENGINE_SLOT_COUNT - 1;
  engine->free_slot_count = XP_ENGINE_SLOT_COUNT;
  engine->profile = xp_profile(&renderer->rom);
  engine->max_voices = engine->profile->defaultMaxVoices;
  engine->lfo_seed = 0x1234u;
  engine->next_serial = 1;
  for (unsigned i = 0; i < XP_ENGINE_NOTE_COUNT; ++i)
    engine->note_next_free[i] = i + 1 < XP_ENGINE_NOTE_COUNT
      ? (uint8_t)(i + 1) : XP_ENGINE_NONE;
  for (unsigned i = 0; i < XP_ENGINE_SLOT_COUNT; ++i) {
    engine->slots[i].note = XP_ENGINE_NONE;
    engine->slots[i].next_free = i + 1 < XP_ENGINE_SLOT_COUNT
      ? (uint8_t)(i + 1) : XP_ENGINE_NONE;
  }
  engine->pan_seed = 0x4d55u;
  for (unsigned i = 0; i < XP_ENGINE_PART_COUNT; ++i) {
    engine->parts[i].levels.master = 127;
    engine->parts[i].levels.secondary = 127;
    engine->parts[i].levels.part = 127;
    engine->parts[i].levels.expression = 127;
    engine->parts[i].pan.master = 64;
    engine->parts[i].pan.part = 64;
    engine->parts[i].pan.random_position = 64;
    engine->parts[i].delay_send = 0;
    engine->parts[i].tone_map = XP_TONE_MAP_SC88;
    engine->parts[i].lfo1_pitch_depth = 0;
    engine->parts[i].lfo_controls.rate = 64;
    engine->parts[i].lfo_controls.delay = 64;
    engine->parts[i].lfo_controls.depth = 64;
    engine->parts[i].tva_controls.part_attack = 64;
    engine->parts[i].tva_controls.secondary_attack = 64;
    engine->parts[i].tva_controls.part_decay = 64;
    engine->parts[i].tva_controls.secondary_decay = 64;
    engine->parts[i].tvf_controls.part_cutoff = 64;
    engine->parts[i].tvf_controls.secondary_cutoff = 64;
    engine->parts[i].tvf_controls.part_resonance = 64;
    engine->parts[i].tvf_controls.secondary_resonance = 64;
    engine->parts[i].portamento_control = 0xffu;
  }
  return true;
}

bool engine_set_max_voices(struct xp_engine *engine, unsigned max_voices)
{
  if (!engine || engine->free_slot_count != XP_ENGINE_SLOT_COUNT)
    return false;
  if (max_voices < 1)
    max_voices = 1;
  else if (max_voices > XP_ENGINE_SLOT_COUNT)
    max_voices = XP_ENGINE_SLOT_COUNT;
  engine->max_voices = max_voices;
  engine->free_slot_head = 0;
  engine->free_slot_tail = (uint8_t)(max_voices - 1);
  engine->free_slot_count = max_voices;
  for (unsigned i = 0; i < max_voices; ++i)
    engine->slots[i].next_free = i + 1 < max_voices
      ? (uint8_t)(i + 1) : XP_ENGINE_NONE;
  return true;
}

void engine_set_part_pan(struct xp_engine *engine, uint8_t part,
                          const struct xp_pan_controls *pan)
{
  if (!engine || !pan || part >= XP_ENGINE_PART_COUNT ||
      pan->master < 1 || pan->master > 127 || pan->part > 127)
    return;
  engine->parts[part].pan = *pan;
  if (pan->part == 0)
    return;
  for (unsigned i = 0; i < XP_ENGINE_SLOT_COUNT; ++i) {
    struct xp_engine_slot *slot = engine->slots + i;
    if (!slot->allocated || slot->note >= XP_ENGINE_NOTE_COUNT ||
        engine->notes[slot->note].part != part)
      continue;
    int target = pan->part + ((int)pan->master - 64) +
      slot->component.pan_component_offset;
    if (target < 1)
      target = 1;
    else if (target > 127)
      target = 127;
    slot->component.pan_target_position = (uint8_t)target;
  }
}

void engine_set_part_rhythm(struct xp_engine *engine, uint8_t part,
                             uint8_t setup)
{
  if (!engine || part >= XP_ENGINE_PART_COUNT || setup > 2)
    return;
  engine->parts[part].rhythm_setup = setup;
}

void engine_set_part_tone_map(struct xp_engine *engine, uint8_t part,
                               uint8_t map)
{
  if (!engine || part >= XP_ENGINE_PART_COUNT ||
      map < XP_TONE_MAP_SC55 || map > XP_TONE_MAP_SC88)
    return;
  engine->parts[part].tone_map = map;
}

void engine_set_part_lfo1_pitch_depth(struct xp_engine *engine,
                                       uint8_t part, uint16_t depth)
{
  if (!engine || part >= XP_ENGINE_PART_COUNT)
    return;
  engine->parts[part].lfo1_pitch_depth = depth;
}

void engine_set_part_reverb_send(struct xp_engine *engine, uint8_t part,
                                  uint8_t send)
{
  if (!engine || part >= XP_ENGINE_PART_COUNT || send > 127)
    return;
  engine->parts[part].reverb_send = send;
}

void engine_set_part_portamento(struct xp_engine *engine, uint8_t part,
                                 bool enabled)
{
  if (!engine || part >= XP_ENGINE_PART_COUNT)
    return;
  engine->parts[part].portamento = enabled;
}

void engine_set_part_portamento_time(struct xp_engine *engine, uint8_t part,
                                      uint8_t time)
{
  if (!engine || part >= XP_ENGINE_PART_COUNT || time > 127)
    return;
  engine->parts[part].portamento_time = time;
}

void engine_set_part_portamento_control(struct xp_engine *engine,
                                         uint8_t part, uint8_t key)
{
  if (!engine || part >= XP_ENGINE_PART_COUNT)
    return;
  engine->parts[part].portamento_control = key;
}

bool engine_set_drum_parameter(struct xp_engine *engine, uint8_t setup,
                                uint8_t field, uint8_t note, uint8_t value)
{
  if (!engine || setup < 1 || setup > 2 || field < 1 ||
      field > XP_DRUM_FIELDS || note > 127)
    return false;
  engine->drum_overlay.value[setup - 1u][field - 1u][note] = value;
  engine->drum_overlay.present[setup - 1u][field - 1u][note] = 1u;
  return true;
}

void engine_clear_drum_overlay(struct xp_engine *engine, uint8_t setup)
{
  if (!engine || setup < 1 || setup > 2)
    return;
  std::memset(engine->drum_overlay.present[setup - 1u], 0,
              sizeof engine->drum_overlay.present[setup - 1u]);
}

void engine_set_part_delay_send(struct xp_engine *engine, uint8_t part,
                                 uint8_t send)
{
  if (!engine || part >= XP_ENGINE_PART_COUNT || send > 127)
    return;
  engine->parts[part].delay_send = send;
}

void engine_set_part_chorus_send(struct xp_engine *engine, uint8_t part,
                                  uint8_t send)
{
  if (!engine || part >= XP_ENGINE_PART_COUNT || send > 127)
    return;
  engine->parts[part].chorus_send = send;
}

void engine_set_part_pitch_offset(struct xp_engine *engine, uint8_t part,
                                   int32_t pitchOffset)
{
  if (!engine || !engine->renderer || part >= XP_ENGINE_PART_COUNT)
    return;
  engine->parts[part].pitch_offset = pitchOffset;
  for (unsigned i = 0; i < XP_ENGINE_SLOT_COUNT; ++i) {
    struct xp_engine_slot *slot = engine->slots + i;
    if (slot->allocated && slot->note < XP_ENGINE_NOTE_COUNT &&
        engine->notes[slot->note].part == part) {
      updateSlotPitch(engine, slot);
    }
  }
}

void engine_set_part_lfo_controls(struct xp_engine *engine, uint8_t part,
                                   const struct xp_lfo_controls *controls)
{
  if (!engine || part >= XP_ENGINE_PART_COUNT || !controls)
    return;
  engine->parts[part].lfo_controls = *controls;
}

void engine_set_part_tva_controls(struct xp_engine *engine, uint8_t part,
                                   const struct xp_tva_controls *controls)
{
  if (!engine || part >= XP_ENGINE_PART_COUNT || !controls)
    return;
  engine->parts[part].tva_controls = *controls;
}

void engine_set_part_tvf_controls(struct xp_engine *engine, uint8_t part,
                                   const struct xp_tvf_controls *controls)
{
  if (!engine || !controls || part >= XP_ENGINE_PART_COUNT ||
      controls->part_cutoff > 127 || controls->secondary_cutoff > 127 ||
      controls->part_resonance > 127 ||
      controls->secondary_resonance > 127)
    return;
  engine->parts[part].tvf_controls = *controls;
  engine->parts[part].tvf_dirty = true;
}

void engine_destroy(struct xp_engine *engine)
{
  if (!engine)
    return;
  for (unsigned i = 0; i < XP_ENGINE_SLOT_COUNT; ++i)
    if (engine->slots[i].allocated)
      renderer_component_release(engine->renderer,
                                 &engine->slots[i].component);
  for (unsigned i = 0; i < XP_ENGINE_STOPPING_COUNT; ++i)
    if (engine->stopping[i].active)
      renderer_component_release(engine->renderer,
                                 &engine->stopping[i].component);
  delete engine->segment;
  std::memset(engine, 0, sizeof *engine);
}

void engine_set_control_service(struct xp_engine *engine,
                                 xp_control_service_fn service, void *user)
{
  if (!engine)
    return;
  engine->control_service = service;
  engine->control_user = user;
}

void engine_set_part_levels(struct xp_engine *engine, uint8_t part,
                             const struct xp_tva_levels *levels)
{
  if (!engine || !levels || part >= XP_ENGINE_PART_COUNT ||
      levels->master > 127 || levels->secondary > 127 ||
      levels->part > 127 || levels->expression > 127)
    return;
  engine->parts[part].levels = *levels;
  for (unsigned i = 0; i < XP_ENGINE_NOTE_COUNT; ++i) {
    struct xp_engine_note *note = engine->notes + i;
    if (!note->allocated || note->part != part)
      continue;
    note->levels = *levels;
    for (unsigned componentIndex = 0;
         componentIndex < XP_MAX_TONE_COMPONENTS; ++componentIndex) {
      uint8_t slotIndex = note->slots[componentIndex];
      if (slotIndex == XP_ENGINE_NONE)
        continue;
      struct xp_render_component *component =
        &engine->slots[slotIndex].component;
      (void)tva_gain_from_headroom_q17(
        &engine->renderer->rom, component->release.current, levels,
        component->drum_level, component->static_attenuation,
        &component->static_gain_q17);
    }
  }
}

bool engine_note_on(struct xp_engine *engine, uint8_t part,
                     uint8_t variation, uint8_t program, uint8_t key,
                     uint8_t velocity, uint8_t context,
                     enum xp_same_note_mode mode, float provisionalGain)
{
  struct xp_render_voice voice = {0};
  uint32_t glideFrom = 0;
  bool glide = false;

  if (engine && part < XP_ENGINE_PART_COUNT &&
      engine->parts[part].pan.part == 0)
    engine->parts[part].pan.random_position = panDraw(engine);
  /* Read before anything is allocated: the note the glide starts from can be
     the one a same-note recycle or a slot reclaim is about to take. */
  if (engine && part < XP_ENGINE_PART_COUNT && key <= 127 &&
      !engine->parts[part].rhythm_setup)
    glide = glideSource(engine, part, &glideFrom);
  if (!engine || !engine->renderer || part >= XP_ENGINE_PART_COUNT ||
      mode > XP_SAME_NOTE_FULL_MULTI || velocity == 0 ||
      !(engine->parts[part].rhythm_setup
          ? renderer_note_on_drum(
              engine->renderer, &voice, engine->parts[part].tone_map,
              program, key, velocity,
              &engine->parts[part].levels, &engine->parts[part].pan,
              &engine->parts[part].tvf_controls,
              &engine->parts[part].tva_controls,
              &engine->parts[part].lfo_controls,
              &engine->drum_overlay, engine->parts[part].rhythm_setup,
              nullptr)
          : renderer_note_on_with_glide(
              engine->renderer, &voice, engine->parts[part].tone_map,
              variation, program, key,
              /* `0x6052`: the zone is the higher of the glide's two ends. */
              (glide && (glideFrom >> 16) > key)
                ? (uint8_t)(glideFrom >> 16) : key,
              velocity,
              &engine->parts[part].levels,
              &engine->parts[part].pan,
              &engine->parts[part].tvf_controls,
              &engine->parts[part].tva_controls,
              &engine->parts[part].lfo_controls)))
    return false;
  applySameNoteMode(engine, &voice, part, key, context, mode);
  if (engine->free_slot_count < voice.component_count)
    reclaimSlots(engine, voice.component_count - engine->free_slot_count);
  uint8_t noteIndex = popNote(engine);
  if (noteIndex == XP_ENGINE_NONE ||
      engine->free_slot_count < voice.component_count) {
    renderer_voice_destroy(engine->renderer, &voice);
    return false;
  }
  struct xp_engine_note *note = engine->notes + noteIndex;
  std::memset(note, 0, sizeof *note);
  note->slots[0] = XP_ENGINE_NONE;
  note->slots[1] = XP_ENGINE_NONE;
  note->allocated = true;
  note->key_down = true;
  note->part = part;
  note->key = key;
  note->velocity = velocity;
  note->context = context;
  note->tone_offset = voice.tone_offset;
  note->serial = engine->next_serial++;
  /* The engine's own output trim, and the only place it lives: the renderer
     takes no gain and the voice carries no gain field, so there is no second
     copy that could silently disagree with this one. */
  note->provisional_gain = provisionalGain;
  note->ignore_note_off = voice.ignore_note_off;
  note->levels = engine->parts[part].levels;
  for (unsigned i = 0; i < voice.component_count; ++i) {
    uint8_t slotIndex = popSlot(engine);
    struct xp_engine_slot *slot = engine->slots + slotIndex;
    slot->allocated = true;
    {
      unsigned pos = engine->active_slot_count;
      while (pos > 0 && engine->active_slots[pos - 1] > slotIndex) {
        engine->active_slots[pos] = engine->active_slots[pos - 1];
        --pos;
      }
      engine->active_slots[pos] = slotIndex;
      ++engine->active_slot_count;
    }
    slot->note = noteIndex;
    slot->serial = engine->next_serial++;
    slot->tvf_lfo_term = 0;
    slot->component = voice.components[i];
    /* A nonzero share byte means this voice joins its tone's oscillator
       rather than starting one of its own, so overlapping notes of the
       same tone modulate in phase (`07_synthesis/lfo.md`). */
    if (slot->component.lfo1.share_request)
      sharedLfoJoin(engine, voice.tone_offset, 0, 1, &slot->component.lfo1);
    if (slot->component.lfo2.share_request)
      sharedLfoJoin(engine, voice.tone_offset,
                    slot->component.rom_component_offset, 2,
                    &slot->component.lfo2);
    slot->lfo_amplitude_gain = lfoAmplitude(slot);
    /* `0x5f89`: the target is stored whether or not a glide starts, which
       is what makes the slot's own key the source for the next note. The
       glide runs only when the source differs from it (`0x5f8f`), the
       switch is on and the time is not zero (`0x5fa2`); the direction is
       fixed here (`0x5fbe`) and never revisited. */
    slot->component.portamento.target = (uint32_t)key << 16;
    slot->component.portamento.current = slot->component.portamento.target;
    if (glide && glideFrom != slot->component.portamento.target &&
        slot->component.portamento.key_table != 0) {
      slot->component.portamento.current = glideFrom;
      slot->component.portamento.ascending =
        glideFrom < slot->component.portamento.target;
      slot->component.portamento.rate = portamento_rate(
        &engine->renderer->rom, engine->parts[part].portamento_time);
      slot->component.portamento.active = true;
    }
    updateSlotPitch(engine, slot);
    voice.components[i].pcm24 = nullptr;
    voice.components[i].active = false;
    note->slots[i] = slotIndex;
    ++note->slot_count;
  }
  return true;
}

bool engine_note_off(struct xp_engine *engine, uint8_t part, uint8_t key)
{
  uint8_t candidate = XP_ENGINE_NONE;
  uint64_t serial = UINT64_MAX;
  if (!engine || part >= XP_ENGINE_PART_COUNT || key > 127)
    return false;
  for (unsigned i = 0; i < XP_ENGINE_NOTE_COUNT; ++i) {
    const struct xp_engine_note *note = engine->notes + i;
    if (note->allocated && note->key_down && note->part == part &&
        note->key == key && note->serial < serial) {
      candidate = (uint8_t)i;
      serial = note->serial;
    }
  }
  /* A note the kit exempts is left sounding: the key is no longer down but
     nothing is released, so the sample and its envelope run to their end. */
  if (candidate != XP_ENGINE_NONE &&
      engine->notes[candidate].ignore_note_off) {
    engine->notes[candidate].key_down = false;
    return true;
  }
  if (candidate == XP_ENGINE_NONE)
    return false;
  engine->notes[candidate].key_down = false;
  engine->notes[candidate].hold_retained = engine->parts[part].hold;
  engine->notes[candidate].sostenuto_retained =
    engine->parts[part].sostenuto &&
    (engine->parts[part].sostenuto_keys[key >> 3] &
     (uint8_t)(1u << (key & 7))) != 0;
  startRelease(engine, candidate);
  return true;
}

void engine_hold(struct xp_engine *engine, uint8_t part, bool enabled)
{
  engine_hold_value(engine, part, enabled ? 127 : 0);
}

void engine_hold_value(struct xp_engine *engine, uint8_t part,
                        uint8_t value)
{
  if (!engine || part >= XP_ENGINE_PART_COUNT || value > 127)
    return;
  bool enabled = value >= 64;
  engine->parts[part].hold = enabled;
  engine->parts[part].hold_value = value;
  if (enabled)
    return;
  for (unsigned i = 0; i < XP_ENGINE_NOTE_COUNT; ++i) {
    if (engine->notes[i].allocated && engine->notes[i].part == part) {
      engine->notes[i].hold_retained = false;
      startRelease(engine, (uint8_t)i);
    }
  }
}

void engine_sostenuto(struct xp_engine *engine, uint8_t part, bool enabled)
{
  if (!engine || part >= XP_ENGINE_PART_COUNT)
    return;
  engine->parts[part].sostenuto = enabled;
  std::memset(engine->parts[part].sostenuto_keys, 0,
              sizeof engine->parts[part].sostenuto_keys);
  if (enabled) {
    for (unsigned i = 0; i < XP_ENGINE_NOTE_COUNT; ++i) {
      const struct xp_engine_note *note = engine->notes + i;
      if (note->allocated && note->key_down && note->part == part)
        engine->parts[part].sostenuto_keys[note->key >> 3] |=
          (uint8_t)(1u << (note->key & 7));
    }
  } else {
    for (unsigned i = 0; i < XP_ENGINE_NOTE_COUNT; ++i) {
      if (engine->notes[i].allocated && engine->notes[i].part == part) {
        engine->notes[i].sostenuto_retained = false;
        startRelease(engine, (uint8_t)i);
      }
    }
  }
}

unsigned engine_active_slots(const struct xp_engine *engine)
{
  return engine ? engine->max_voices - engine->free_slot_count : 0;
}

unsigned engine_released_slots(const struct xp_engine *engine)
{
  unsigned count = 0;
  if (!engine)
    return 0;
  for (unsigned i = 0; i < XP_ENGINE_SLOT_COUNT; ++i)
    if (engine->slots[i].allocated &&
        !engine->notes[engine->slots[i].note].key_down)
      ++count;
  return count;
}

void engine_set_stage_taps(struct xp_engine *engine,
                            const struct xp_engine_stage_taps *taps)
{
  if (!engine)
    return;
  if (taps)
    engine->stage_taps = *taps;
  else
    std::memset(&engine->stage_taps, 0, sizeof engine->stage_taps);
}

namespace {

/* The four amplitude-stage taps of one frame, for the stage-tap probe. */
struct FrameTaps {
  float oscillator, tvf, staticGain, tva, lfo;
};

/* The buses one frame of voices is summed into. */
struct FrameMix {
  float left, right, reverb, chorus, delay;
};

/* What a voice's gained sample is multiplied by on its way into each bus.
   Every factor is fixed between two serviced control periods - the pan
   moves only in runScheduler, the sends only with MIDI between blocks -
   which is what lets one voice's frames be rendered ahead of the others'
   and summed afterwards. */
struct VoiceBuses {
  float left, right;
  float reverbGain, chorusGain, delayGain;
  bool reverb, chorus, delay;
};

inline void slotBuses(const struct xp_engine *engine,
                      const struct xp_engine_slot *slot, bool send,
                      bool chorusSend, bool delaySend,
                      struct VoiceBuses *buses)
{
  buses->left = slot->component.left_gain_q15 / 32768.0f;
  buses->right = slot->component.right_gain_q15 / 32768.0f;
  buses->reverb = buses->chorus = buses->delay = false;
  if (slot->note >= XP_ENGINE_NOTE_COUNT)
    return;
  const struct xp_engine_part *part =
    engine->parts + engine->notes[slot->note].part;
  /* A rhythm note's send is its part's control combined with its own from
     its kit record, through the firmware's rounded product and the ROM's
     send curve. A melodic component carries 127 and so keeps its part's
     control unchanged. */
  if (send)
    buses->reverb = xp_renderer_send_gain(
      engine->renderer,
      send_combine(part->reverb_send, slot->component.reverb_send),
      &buses->reverbGain);
  /* The delay bus takes the part's send alone: a rhythm note's own delay
     send lives in RAM at `+0x50c`, not in the kit record. */
  if (delaySend)
    buses->delay = xp_renderer_send_gain(engine->renderer, part->delay_send,
                                         &buses->delayGain);
  /* The chorus bus is formed the same way, from part byte `+0e` and the
     kit's `+0x400` (`08_effects/routing.md`). */
  if (chorusSend)
    buses->chorus = xp_renderer_send_gain(
      engine->renderer,
      send_combine(part->chorus_send, slot->component.chorus_send),
      &buses->chorusGain);
}

inline void stopBuses(const struct xp_engine *engine,
                      const struct xp_engine_stopping *stop, bool send,
                      bool chorusSend, bool delaySend,
                      struct VoiceBuses *buses)
{
  const struct xp_engine_part *part = engine->parts + stop->part;
  buses->left = stop->component.left_gain_q15 / 32768.0f;
  buses->right = stop->component.right_gain_q15 / 32768.0f;
  buses->reverb = send && xp_renderer_send_gain(
    engine->renderer,
    send_combine(part->reverb_send, stop->component.reverb_send),
    &buses->reverbGain);
  buses->delay = delaySend &&
    xp_renderer_send_gain(engine->renderer, part->delay_send,
                          &buses->delayGain);
  buses->chorus = chorusSend && xp_renderer_send_gain(
    engine->renderer,
    send_combine(part->chorus_send, stop->component.chorus_send),
    &buses->chorusGain);
}

inline void mixVoice(struct FrameMix *mix, const struct VoiceBuses *buses,
                     float gained)
{
  mix->left += gained * buses->left;
  mix->right += gained * buses->right;
  if (buses->reverb)
    mix->reverb += gained * buses->reverbGain;
  if (buses->delay)
    mix->delay += gained * buses->delayGain;
  if (buses->chorus)
    mix->chorus += gained * buses->chorusGain;
}

/* One frame of an allocated slot's voice, as the gain every bus takes it
   at. False, producing nothing, once the voice has run out: the caller
   frees the slot at that frame. */
inline bool slotFrame(const struct xp_engine *engine,
                      struct xp_engine_slot *slot, double periodFraction,
                      double fractionStep, double staticGainProgress,
                      struct FrameTaps *taps, float *gainedOut)
{
  float sample;
  if (!slot->component.active ||
      !oscillator_next(&slot->component.oscillator, &sample))
    return false;
  taps->oscillator += sample;
  if (engine->renderer->tvf_audio_transfer)
    sample = engine->renderer->tvf_audio_transfer(
      engine->renderer->tvf_audio_user, &slot->component.tvf_audio,
      &slot->component.tvf, periodFraction, sample);
  taps->tvf += sample;
  uint32_t envelopeGain = tva_envelope_render_q17(
    &engine->renderer->rom, &slot->component.envelope, periodFraction,
    fractionStep);
  /* The chip's amplitude register, not the CPU's composed target: the
     target is a step once per control period and the register is the ramp
     between two of them. */
  float staticGain = xp_render_static_gain_q17_from_progress(
    &slot->component, staticGainProgress) / 131072.0f;
  taps->staticGain += sample * staticGain;
  taps->tva += sample * staticGain * (envelopeGain / 131072.0f);
  float gained = sample * staticGain *
    (envelopeGain / 131072.0f) *
    slot->lfo_amplitude_gain *
    engine->notes[slot->note].provisional_gain;
  taps->lfo += gained;
  if (slot->component.oscillator.ended)
    slot->component.active = false;
  *gainedOut = gained;
  return true;
}

/* One frame of a voice the CPU has already handed its slot back. Nothing
   composes for it any more: the amplitude register runs down to the zero
   the stop wrote, on its own clock, and the rest of the voice - oscillator,
   filter, pan - carries on from exactly where it stood. False, producing
   nothing, once the register has arrived at zero or the sample has run out
   first: the caller releases the voice at that frame. */
inline bool stopFrame(const struct xp_engine *engine,
                      struct xp_engine_stopping *stop, double periodFraction,
                      struct FrameTaps *taps, float *gainedOut)
{
  uint32_t registerQ17 =
    xp_render_static_gain_q17(&stop->component, stop->periods);
  float sample;
  if (!registerQ17 || !stop->component.active ||
      !oscillator_next(&stop->component.oscillator, &sample))
    return false;
  taps->oscillator += sample;
  if (engine->renderer->tvf_audio_transfer)
    sample = engine->renderer->tvf_audio_transfer(
      engine->renderer->tvf_audio_user, &stop->component.tvf_audio,
      &stop->component.tvf, periodFraction, sample);
  taps->tvf += sample;
  /* One frozen product for the three amplitude taps: the envelope, the
     oscillators and the note gain no longer move apart. */
  float gained = sample * (registerQ17 / 131072.0f) * stop->gain;
  taps->staticGain += gained;
  taps->tva += gained;
  taps->lfo += gained;
  if (stop->component.oscillator.ended)
    stop->component.active = false;
  stop->periods += kXpControlTimerHz /
    (engine->renderer->output_rate * kXpControlPeriodClocks);
  *gainedOut = gained;
  return true;
}

void releaseStop(struct xp_engine *engine, struct xp_engine_stopping *stop)
{
  renderer_component_release(engine->renderer, &stop->component);
  std::memset(stop, 0, sizeof *stop);
}

void storeFrame(const struct FrameMix *mix, float *stereo, float *send,
                float *chorusSend, float *delaySend, size_t frame)
{
  stereo[frame * 2] = mix->left;
  stereo[frame * 2 + 1] = mix->right;
  if (send)
    send[frame] = mix->reverb;
  if (chorusSend)
    chorusSend[frame] = mix->chorus;
  if (delaySend)
    delaySend[frame] = mix->delay;
}

constexpr unsigned kSegmentVoices =
  XP_ENGINE_SLOT_COUNT + XP_ENGINE_STOPPING_COUNT;

}  // namespace

/* One segment of the multi-threaded render: the frames from one scheduler
   call that may service a period up to the next, or to the end of the
   block. */
struct EngineSegment {
  struct xp_engine *engine;
  size_t frames;
  double fractionStep;
  double periodFraction[XP_ENGINE_SEGMENT_FRAMES];
  double staticGainProgress[XP_ENGINE_SEGMENT_FRAMES];
  /* The voices in the order the single-threaded loop visits them: the
     allocated slots ascending, then the stopping voices ascending. */
  unsigned slotCount;
  uint8_t index[kSegmentVoices];
  struct VoiceBuses buses[kSegmentVoices];
  /* The frame each voice ran out at, or `frames` if it did not. */
  size_t end[kSegmentVoices];
  float gained[kSegmentVoices][XP_ENGINE_SEGMENT_FRAMES];
};

namespace {

void renderSegmentVoice(void *context, unsigned v)
{
  EngineSegment *segment = (EngineSegment *)context;
  const struct xp_engine *engine = segment->engine;
  float *gained = segment->gained[v];
  /* Nothing reads a threaded voice's taps: a tapped engine is never
     threaded. */
  struct FrameTaps taps = {};
  size_t f = 0;
  if (v < segment->slotCount) {
    struct xp_engine_slot *slot = segment->engine->slots + segment->index[v];
    for (; f < segment->frames; ++f)
      if (!slotFrame(engine, slot, segment->periodFraction[f],
                     segment->fractionStep, segment->staticGainProgress[f],
                     &taps, gained + f))
        break;
  } else {
    struct xp_engine_stopping *stop =
      segment->engine->stopping + segment->index[v];
    for (; f < segment->frames; ++f)
      if (!stopFrame(engine, stop, segment->periodFraction[f], &taps,
                     gained + f))
        break;
  }
  segment->end[v] = f;
}

void renderThreaded(struct xp_engine *engine, float *stereo, float *send,
                    float *chorusSend, float *delaySend, size_t frames,
                    double fractionStep)
{
  EngineSegment *segment = engine->segment;
  segment->engine = engine;
  segment->fractionStep = fractionStep;
  const double clockStep = kXpControlTimerHz / engine->renderer->output_rate;
  size_t done = 0;
  while (done < frames) {
    /* The segment opens on a scheduler call that may service a period and
       runs on through every frame whose call would only move the clock -
       which is all runScheduler does when no period has elapsed, so that
       step is taken here instead, with the same arithmetic. */
    runScheduler(engine);
    size_t n = 0;
    for (;;) {
      segment->periodFraction[n] =
        engine->scheduler_clocks / kXpControlPeriodClocks;
      segment->staticGainProgress[n] =
        xp_static_gain_progress(segment->periodFraction[n]);
      ++n;
      if (done + n >= frames || n >= XP_ENGINE_SEGMENT_FRAMES)
        break;
      double clocks = engine->scheduler_clocks + clockStep;
      if ((unsigned)(clocks / kXpControlPeriodClocks))
        break;
      engine->scheduler_clocks = clocks;
    }
    segment->frames = n;

    unsigned count = 0;
    for (unsigned k = 0; k < engine->active_slot_count; ++k, ++count) {
      segment->index[count] = engine->active_slots[k];
      slotBuses(engine, engine->slots + engine->active_slots[k],
                send != nullptr, chorusSend != nullptr,
                delaySend != nullptr, segment->buses + count);
    }
    segment->slotCount = count;
    for (unsigned i = 0; i < XP_ENGINE_STOPPING_COUNT; ++i) {
      if (!engine->stopping[i].active)
        continue;
      segment->index[count] = (uint8_t)i;
      stopBuses(engine, engine->stopping + i, send != nullptr,
                chorusSend != nullptr, delaySend != nullptr,
                segment->buses + count);
      ++count;
    }

    engine->pool->run(count, renderSegmentVoice, segment);

    for (size_t f = 0; f < n; ++f) {
      struct FrameMix mix = {};
      for (unsigned v = 0; v < count; ++v)
        if (f < segment->end[v])
          mixVoice(&mix, segment->buses + v, segment->gained[v][f]);
      storeFrame(&mix, stereo, send, chorusSend, delaySend, done + f);
    }
    /* The voices that ran out, handed back in the order the
       single-threaded loop hands them back: by frame, and within a frame
       the slots before the stopping voices, each ascending. */
    for (size_t f = 0; f < n; ++f)
      for (unsigned v = 0; v < count; ++v) {
        if (segment->end[v] != f)
          continue;
        if (v < segment->slotCount)
          freeSlot(engine, segment->index[v], false);
        else
          releaseStop(engine, engine->stopping + segment->index[v]);
      }
    done += n;
  }
}

}  // namespace

bool engine_set_worker_pool(struct xp_engine *engine,
                             EmuSC::Xp::WorkerPool *pool)
{
  if (!engine)
    return false;
  engine->pool = nullptr;
  if (!pool || pool->threads() <= 1)
    return true;
  if (!engine->segment) {
    engine->segment = new (std::nothrow) EngineSegment;
    if (!engine->segment)
      return false;
  }
  engine->pool = pool;
  return true;
}

void engine_render_with_send(struct xp_engine *engine, float *stereo,
                              float *send, float *chorusSend,
                              float *delaySend, size_t frames)
{
  if (!engine || !engine->renderer || !stereo)
    return;
  /* How far one output sample moves the control period's fraction: the
     clock step runScheduler adds, in periods. The TVA envelope's
     per-sample recurrence is built for exactly this step. */
  const double fractionStep =
    (kXpControlTimerHz / engine->renderer->output_rate) /
    kXpControlPeriodClocks;
  const bool tapped = engine->stage_taps.oscillator ||
    engine->stage_taps.after_tvf || engine->stage_taps.after_static ||
    engine->stage_taps.after_tva || engine->stage_taps.after_lfo;
  if (engine->pool && !tapped) {
    renderThreaded(engine, stereo, send, chorusSend, delaySend, frames,
                   fractionStep);
    return;
  }
  for (size_t frame = 0; frame < frames; ++frame) {
    struct FrameMix mix = {};
    struct FrameTaps taps = {};
    runScheduler(engine);
    /* Loop-invariant: scheduler_clocks only moves inside runScheduler,
       above, so this is the same value for every voice this frame - and
       so is the static-gain glide's exp()-based progress derived from
       it, which every voice below would otherwise recompute for itself. */
    const double periodFraction =
      engine->scheduler_clocks / kXpControlPeriodClocks;
    const double staticGainProgress =
      xp_static_gain_progress(periodFraction);
    /* Walks only the currently allocated slots (engine->active_slots,
       kept in ascending order - see its declaration), instead of every
       slot in [0, max_voices) as this - the per-voice per-sample mix,
       the most expensive loop in the engine - once did: an idle slot no
       longer costs a cache-line touch just to read its allocated flag,
       regardless of how many of the max_voices ceiling are unused. A
       freed slot's removal shifts the array down under activeIndex, so
       the index is only advanced when this iteration's slot survives -
       otherwise the slot that shift brought into this position is
       examined next, exactly the one ascending-order scan would have
       reached anyway. */
    unsigned activeIndex = 0;
    while (activeIndex < engine->active_slot_count) {
      uint8_t slotIndex = engine->active_slots[activeIndex];
      struct xp_engine_slot *slot = engine->slots + slotIndex;
      float gained;
      if (!slotFrame(engine, slot, periodFraction, fractionStep,
                     staticGainProgress, &taps, &gained)) {
        freeSlot(engine, slotIndex, false);
        continue;
      }
      struct VoiceBuses buses;
      slotBuses(engine, slot, send != nullptr, chorusSend != nullptr,
                delaySend != nullptr, &buses);
      mixVoice(&mix, &buses, gained);
      ++activeIndex;
    }
    for (unsigned i = 0; i < XP_ENGINE_STOPPING_COUNT; ++i) {
      struct xp_engine_stopping *stop = engine->stopping + i;
      if (!stop->active)
        continue;
      float gained;
      if (!stopFrame(engine, stop, periodFraction, &taps, &gained)) {
        releaseStop(engine, stop);
        continue;
      }
      struct VoiceBuses buses;
      stopBuses(engine, stop, send != nullptr, chorusSend != nullptr,
                delaySend != nullptr, &buses);
      mixVoice(&mix, &buses, gained);
    }
    storeFrame(&mix, stereo, send, chorusSend, delaySend, frame);
    if (engine->stage_taps.oscillator)
      engine->stage_taps.oscillator[frame] = taps.oscillator;
    if (engine->stage_taps.after_tvf)
      engine->stage_taps.after_tvf[frame] = taps.tvf;
    if (engine->stage_taps.after_static)
      engine->stage_taps.after_static[frame] = taps.staticGain;
    if (engine->stage_taps.after_tva)
      engine->stage_taps.after_tva[frame] = taps.tva;
    if (engine->stage_taps.after_lfo)
      engine->stage_taps.after_lfo[frame] = taps.lfo;
  }
}

void engine_render(struct xp_engine *engine, float *stereo, size_t frames)
{
  engine_render_with_send(engine, stereo, nullptr, nullptr, nullptr, frames);
}

}}  // namespace EmuSC::Xp

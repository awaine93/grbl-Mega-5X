# Tool Change (M6) — Thread Pitch Matching Notes

**Status: deferred.** The `M6` load cycle runs today at placeholder values (`TOOL_LOAD_FEED 100`,
`TOOL_LOAD_RPM_LOW 200`). Those values are *not* pitch-matched and will bind once the collet nut
threads engage. This document records the physics, the measurement procedure, and where the fix goes
so the tuning can be done on the machine in one pass.

---

## The rule

The ER11 collet nut thread is **M13 × 1**: nominal diameter 13 mm, pitch **P = 1.0 mm** per
revolution.

For the nut to turn freely on the spindle thread, the axial feed must advance exactly one pitch per
revolution:

```
F (mm/min) = RPM × P
           = RPM × 1.0
```

| True RPM | Required feed F (mm/min) |
|---------:|-------------------------:|
|       50 |                       50 |
|      100 |                      100 |
|      200 |                      200 |
|      300 |                      300 |
|      500 |                      500 |
|     1000 |                     1000 |

Any mismatch is a **ratio error** in mm/rev:

```
mm/rev = F / RPM          (target: 1.0)
```

Faster than 1 mm/rev → the spindle thread drives the nut forward faster than it turns → jams.
Slower → it back-drives or binds on the last turns.

Only the final ~5–8 mm of the plunge engages the thread (`TOOL_LOAD_Z -53` down to about `-45`), so
the error has a short distance to manifest — which is why a bad ratio can feel like "it almost works"
before it seizes.

---

## `S` is not RPM

Grbl's `S` word is mapped through `$30` / `$31` to a **PWM duty cycle**, not a shaft speed. With the
stock settings `$30=1000` and `$31=0`, `spindle_compute_pwm_value()` interpolates linearly:

```
duty = (S - $31) / ($30 - $31)
S200 → 20 % duty      S1000 → 100 % duty
```

The real RPM at a given duty depends entirely on the motor, driver, supply and load. **There is no
formula that turns `S` into RPM on this machine.**

The piecewise-linear RPM model in `config.h` (`RPM_MIN`/`RPM_MAX`) is not available here: it is
`#error`-disabled when `SPINDLE_BTS7960_ON_D44_D45` is selected. So there is no compile-time table
to consult either.

Consequence: `TOOL_LOAD_RPM_LOW 200` is an **S value (duty), not a speed**. Any comment or constant
naming that says "RPM" for it should be read as "S".

---

## Measurement procedure

Do this with the spindle free-running — no tool in the rack, no load on the spindle.

1. Command `M3 S200` (or whatever `S` you want to characterise).
2. Time **10 full revolutions** of the spindle with a stopwatch or phone timer. Get `t` in seconds.
3. Compute:

```
RPM = 600 / t
```

4. Compute the matching feed:

```
F = RPM × 1.0
```

5. Set `TOOL_LOAD_FEED` to that `F`.

Two equivalent ways to converge on a pair:

- **Choose F, match RPM to it** — pick the feed you want (limited by `$132` Z travel and how fast
  you're willing to cross the engagement zone), measure RPM at a few `S` values, pick the `S` whose
  measured RPM gives `F = RPM × 1.0`.
- **Choose S, match F to it** — fix `S`, measure RPM, set `F = RPM × 1.0`.

Either way you end up with **one** knob (measure → set `TOOL_LOAD_FEED`), and you should re-measure
if the spindle's supply voltage or load changes meaningfully.

### Spin-up

`spindle_sync()` waits for the motion buffer to drain, then sets the spindle state. It does **not**
wait for the spindle to physically reach speed. The rapid to `TOOL_LOAD_Z_APPROACH` (roughly 40 mm at
rapid rate) is the only spin-up window before the thread engages.

Once `F` is pitch-matched, the ramp matters: if engagement starts while the spindle is still
accelerating, the instantaneous mm/rev is wrong even though the commanded pair is correct. Consider a
`mc_dwell()` between `M3 S<low>` and the approach when implementing.

---

## Where it goes

Constants in `grbl/tool_change.h`:

```c
#define TOOL_LOAD_THREAD_PITCH 1.0f   // mm/rev, ER11 nut M13x1
#define TOOL_LOAD_THREAD_RPM   ???     // measured, from the procedure above
#define TOOL_LOAD_FEED (TOOL_LOAD_THREAD_RPM * TOOL_LOAD_THREAD_PITCH)
```

Insertion point: `tool_load()` in `grbl/tool_change.c`, in the block between `spindle_sync(CW,
TOOL_LOAD_RPM_LOW)` and the plunge to `TOOL_LOAD_Z`. The command sequence is:

```
M5                      stop spindle
G53 G0 Z0               clear the rack
G0 X<slot> Y<slot>      position over the slot
M3 S<low>               start slow rotation
G0 Z-40                 rapid to approach (in air)
G1 Z-53 F<matched>      feed through the engagement zone   <-- here
G4 P1                   seat
M3 S<full>              tighten
G4 P1
M5
G53 G0 Z0               retract
```

The values themselves are tuned on the machine; nothing in this file is a substitute for measuring
the actual RPM at the `S` you settle on.

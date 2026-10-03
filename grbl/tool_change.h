/*
  tool_change.h - tool change cycle (M6) configuration and interface
  Part of Grbl

  Copyright (c) 2017-2026 Gauthier Briere

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef tool_change_h
#define tool_change_h

// Physical tool slot count (TOOL_CHANGE_SLOTS) and rack positions (tool_slot_xy) are
// defined in config.h. Valid tool numbers are T0-T5: T1-T5 select a slot, T0 means
// "no tool" (skip load after unload).
#define TOOL_CHANGE_EMPTY 0 // Tool number representing an empty spindle.

// ---------------------------------------------------------------------------------
// Tool change cycle parameters, ported from the FreeCAD grbl post processor.
// Applied to all slots. Units: mm, mm/min, seconds.
// NOTE: every S value here is an S word, i.e. a PWM duty under $30/$31, NOT a shaft
// speed, and none of the S/F pairs are pitch matched yet (the ER11 nut is M13x1, so
// F must equal true RPM x 1.0). See doc/markdown/tool_change.md for the measurement
// procedure and where to apply the tuned values.
// ---------------------------------------------------------------------------------

// Retract / safe height. Work coordinate 0. Rack moves are work-coordinate relative,
// so this is wherever the operator zeroed Z. Assumption: the operator resets zero
// after homing, which leaves work 0 at the post-homing machine Z position.
#define TOOL_SAFE_Z 0.0f

// --- Load (rack -> spindle): two-stage forward, slow engage then fast seat --------
#define TOOL_LOAD_Z (-53.0f) // Engagement depth in work coordinates.
#define TOOL_LOAD_Z_APPROACH (-40.0f) // Rapid stop height above TOOL_LOAD_Z.
#define TOOL_LOAD_ENGAGE_SPEED 100.0f // S while engaging (M3 CW).
#define TOOL_LOAD_ENGAGE_FEED 200.0f // Z feed through the engagement zone.
#define TOOL_LOAD_ENGAGE_DWELL 0.5f // Dwell after slow engage / around the M5.
#define TOOL_LOAD_SEAT_SPEED 750.0f // S to tighten the holder (M3 CW).
#define TOOL_LOAD_SEAT_DWELL 0.5f // Dwell at seat speed before stopping.

// --- Unload (spindle -> rack): engage then reverse unscrew ------------------------
#define TOOL_UNLOAD_ENGAGE_SPEED 50.0f // S while plunging (M4 CCW).
#define TOOL_UNLOAD_ENGAGE_FEED 100.0f // Z feed during the engage plunge.
#define TOOL_UNLOAD_ENGAGE_DWELL 1.0f // Dwell after slow engage.
#define TOOL_UNLOAD_FINE_Z (-40.0f) // Fine Z below TOOL_LOAD_Z, work coordinates.
#define TOOL_UNLOAD_FINE_FEED 50.0f // Feed for the fine Z adjustment.
#define TOOL_UNLOAD_START_SPEED 500.0f // S for the initial reverse (M4 CCW).
#define TOOL_UNLOAD_START_DWELL 0.5f // Dwell at initial reverse speed.
#define TOOL_UNLOAD_MAIN_SPEED 750.0f // S for the main unscrew (M4 CCW).
#define TOOL_UNLOAD_RETRACT_Z (-50.0f) // Z to retract to while unscrewing, work coords.
#define TOOL_UNLOAD_RETRACT_FEED 200.0f // Feed for the unscrew retract.

// --- Probe check (establish the new Z work offset) --------------------------------
// TOOL_PROBE_X / TOOL_PROBE_Y are offsets from WORK X0 Y0, converted to machine
// coordinates through wco[] like every other move in the cycle (same convention as
// tool_slot_xy in config.h). 0.0f puts the plate at the work origin.
#define TOOL_PROBE_X 0.0f
#define TOOL_PROBE_Y 0.0f
#define TOOL_PROBE_PLATE_THICKNESS 0.0f // 0.0 = direct copper contact (PCB milling).
#define TOOL_PROBE_G92_OFFSET (TOOL_PROBE_PLATE_THICKNESS - 0.3f) // G92 Z value.
#define TOOL_PROBE_MAX_DEPTH (-55.0f) // Fast probe travel, relative mm.
#define TOOL_PROBE_FAST_FEED 100.0f
#define TOOL_PROBE_SLOW_FEED 10.0f
#define TOOL_PROBE_RETRACT 2.0f // Lift between the fast and slow probe, relative mm.
#define TOOL_PROBE_LIFT_Z 5.0f // Work Z lifted to right after G92 (off the surface).
#define TOOL_PROBE_RETURN_Z 10.0f // Work Z parked at before the job resumes.

// Runs the complete M6 tool change cycle: unload current tool, load requested tool,
// probe check. Motion is planned in work coordinates converted to machine coordinates
// through wco[] (G5x + G92 only; the active tool length offset is deliberately
// excluded so it cannot shift the rack). wcs[] is the active work coordinate system
// origin (block_coord_system) and is needed to reproduce G92's offset math.
// Returns STATUS_OK or an error status (reported as error:N to the sender).
uint8_t tool_change_cycle(uint8_t tool, float *wco, float *wcs);

// Returns the tool currently held in the spindle (TOOL_CHANGE_EMPTY if none).
// Read-only: does not change any state. Used for the |TL: field in the '?' status
// report, so the sender can show what is physically in the spindle.
uint8_t tool_change_current_tool(void);

#endif

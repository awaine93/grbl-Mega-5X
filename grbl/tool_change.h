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

// Tool load cycle parameters. Applied to all slots. Units: mm, mm/min, seconds.
// NOTE: TOOL_LOAD_FEED and TOOL_LOAD_RPM_* are NOT pitch matched yet (the ER11 nut is
// M13x1, so F must equal RPM x 1.0). TOOL_LOAD_RPM_* is an S value, i.e. a PWM duty
// under $30/$31, not a shaft speed. See doc/markdown/tool_change.md for the measurement
// procedure and where to apply the tuned values.
#define TOOL_LOAD_Z (-53.0f) // Final tool depth in work coordinates.
#define TOOL_LOAD_Z_APPROACH (-40.0f) // Rapid stop height above TOOL_LOAD_Z.
#define TOOL_LOAD_FEED 100.0f // DEFERRED: engagement feed rate, not pitch matched.
#define TOOL_LOAD_RPM_LOW 100.0f // DEFERRED: slow rotation S value while seating.
#define TOOL_LOAD_RPM_HIGH 1000.0f // Full speed S value to tighten the holder.
#define TOOL_LOAD_DWELL 1.0f // Dwell after each speed change.

// Runs the complete M6 tool change cycle: unload current tool, load requested tool,
// probe check. wco[] is the work coordinate offset (G5x + G92 only; the active tool
// length offset is deliberately excluded so it cannot shift the rack) used to convert
// work coordinates to absolute machine coordinates.
// Returns STATUS_OK or an error status (reported as error:N to the sender).
uint8_t tool_change_cycle(uint8_t tool, float *wco);

// Returns the tool currently held in the spindle (TOOL_CHANGE_EMPTY if none).
// Used by the parser to reject M6 during error checking, before any block state is
// applied. Read-only: does not change any state.
uint8_t tool_change_current_tool(void);

#endif

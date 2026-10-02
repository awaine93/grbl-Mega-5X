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

// Tool load cycle parameters. Applied to all slots. Units: mm, mm/min, seconds, rpm.
#define TOOL_LOAD_Z (-48.0f) // Plunge depth in work coordinates.
#define TOOL_LOAD_FEED 100.0f // Plunge feed rate.
#define TOOL_LOAD_RPM_LOW 200.0f // Slow rotation while seating the tool.
#define TOOL_LOAD_RPM_HIGH 1000.0f // Spin-up to tighten the holder.
#define TOOL_LOAD_DWELL 1.0f // Dwell after each speed change.

// Runs the complete M6 tool change cycle: unload current tool, load requested tool,
// probe check. wco[] is the work coordinate offset (WCS + G92 + tool length offset)
// used to convert work coordinates to absolute machine coordinates.
// Returns STATUS_OK or an error status (reported as error:N to the sender).
uint8_t tool_change_cycle(uint8_t tool, float *wco);

#endif

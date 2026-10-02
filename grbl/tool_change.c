/*
  tool_change.c - tool change cycle (M6) for automatic/manual tool racks
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

#include "grbl.h"


// Axis indices resolved from the configured axis names (compile-time constants).
static const uint8_t axis_x = (AXIS_1_NAME == 'X') ? AXIS_1 : ((AXIS_2_NAME == 'X') ? AXIS_2 : AXIS_3);
static const uint8_t axis_y = (AXIS_1_NAME == 'Y') ? AXIS_1 : ((AXIS_2_NAME == 'Y') ? AXIS_2 : AXIS_3);
static const uint8_t axis_z = (AXIS_1_NAME == 'Z') ? AXIS_1 : ((AXIS_2_NAME == 'Z') ? AXIS_2 : AXIS_3);

// Tool currently held in the spindle. Not persisted: after reset the spindle is
// assumed empty (the tool rack state must be re-established by the operator).
static uint8_t current_tool = TOOL_CHANGE_EMPTY;

static uint8_t tool_load(uint8_t tool, float *wco);
static uint8_t tool_unload(uint8_t tool, float *wco);
static uint8_t tool_probe_check(uint8_t tool, float *wco);


// Runs the M6 cycle: unload (if a tool is loaded), load the requested tool, probe check.
uint8_t tool_change_cycle(uint8_t tool, float *wco)
{
  uint8_t status;

  if (current_tool != TOOL_CHANGE_EMPTY) {
    if ((status = tool_unload(current_tool, wco)) != STATUS_OK) { return(status); }
  }
  if (tool != TOOL_CHANGE_EMPTY) {
    if ((status = tool_load(tool, wco)) != STATUS_OK) { return(status); }
    if ((status = tool_probe_check(tool, wco)) != STATUS_OK) { return(status); }
  }
  current_tool = tool;
  return(STATUS_OK);
}


// Loads tool (1..TOOL_CHANGE_SLOTS) from its rack slot into the spindle.
// Sequence: stop spindle, retract to machine Z0, rapid to slot XY, slow rotation,
// plunge to seat, dwell, spin-up, dwell, stop spindle, retract to machine Z0.
static uint8_t tool_load(uint8_t tool, float *wco)
{
  const float *slot = tool_slot_xy[tool-1];
  float target[N_AXIS];
  plan_line_data_t pl;

  // M5: stop the spindle before moving over the tool rack.
  spindle_sync(SPINDLE_DISABLE, 0.0);

  // G53 G0 Z0: rapid to machine Z zero at the current XY.
  memset(&pl, 0, sizeof(plan_line_data_t));
  pl.condition = PL_COND_FLAG_RAPID_MOTION;
  memcpy(target, gc_state.position, sizeof(target));
  target[axis_z] = 0.0;
  mc_line(target, &pl);

  // G0 X.. Y..: rapid to the tool slot (work coordinates -> machine coordinates).
  memset(&pl, 0, sizeof(plan_line_data_t));
  pl.condition = PL_COND_FLAG_RAPID_MOTION;
  target[axis_x] = slot[0] + wco[axis_x];
  target[axis_y] = slot[1] + wco[axis_y];
  mc_line(target, &pl);

  // M3 S200: slow rotation while engaging the tool holder.
  spindle_sync(SPINDLE_ENABLE_CW, TOOL_LOAD_RPM_LOW);

  // G1 Z.. F100: plunge to seat the tool (work coordinate depth).
  // NOTE: The stepper re-applies spindle PWM from every block's condition/spindle_speed
  // (stepper.c), so blocks planned while the spindle must keep running must declare
  // PL_COND_FLAG_SPINDLE_* + spindle_speed, or the spindle is forced off mid-cycle.
  memset(&pl, 0, sizeof(plan_line_data_t));
  pl.feed_rate = TOOL_LOAD_FEED;
  pl.condition = PL_COND_FLAG_SPINDLE_CW;
  pl.spindle_speed = TOOL_LOAD_RPM_LOW;
  target[axis_z] = TOOL_LOAD_Z + wco[axis_z];
  mc_line(target, &pl);

  // G4 P1: allow the tool to seat.
  mc_dwell(TOOL_LOAD_DWELL);

  // M3 S1000: spin up to tighten the holder.
  spindle_sync(SPINDLE_ENABLE_CW, TOOL_LOAD_RPM_HIGH);

  // G4 P1
  mc_dwell(TOOL_LOAD_DWELL);

  // M5: stop the spindle.
  spindle_sync(SPINDLE_DISABLE, 0.0);

  // G53 G0 Z0: retract to machine Z zero.
  memset(&pl, 0, sizeof(plan_line_data_t));
  pl.condition = PL_COND_FLAG_RAPID_MOTION;
  target[axis_z] = 0.0;
  mc_line(target, &pl);

  // Keep the parser position consistent for any following cycle step.
  memcpy(gc_state.position, target, sizeof(target));

  return(STATUS_OK);
}


// Returns the tool currently in the spindle to its rack slot.
// TODO: implement once the unload sequence is defined. Refuses to run so the machine
// never attempts to pick up a tool while another is still loaded.
static uint8_t tool_unload(uint8_t tool, float *wco)
{
  (void)tool; (void)wco;
  return(STATUS_TOOL_CHANGE_NOT_READY);
}


// Probes the tool at the XY home position to establish the new Z zero.
// TODO: implement (probe check runs after every load).
static uint8_t tool_probe_check(uint8_t tool, float *wco)
{
  (void)tool; (void)wco;
  return(STATUS_OK);
}

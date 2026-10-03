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

// Spindle state the cycle last commanded. Motion blocks re-declare it, because the
// stepper re-applies PWM from every block's condition/spindle_speed (stepper.c) and a
// block with no spindle flag forces PWM off part-way through the sequence.
static uint8_t cycle_spindle_state = SPINDLE_DISABLE;
static float cycle_spindle_speed = 0.0;

static void tool_change_pause(void);
static void tool_spindle(uint8_t state, float speed);
static void tool_rapid(float *target);
static void tool_feed(float *target, float feed_rate);
static void tool_retract(float *wco);
static uint8_t tool_home_z(void);
static uint8_t tool_unload(uint8_t tool, float *wco);
static uint8_t tool_load(uint8_t tool, float *wco);
static uint8_t tool_probe_check(float *wco, float *wcs);


// Blocks until the operator presses cycle start, standing in for the post processor's
// M0 operator checkpoints. Returns early in check mode, on alarm, or on reset.
static void tool_change_pause(void)
{
  if (sys.state & (STATE_CHECK_MODE | STATE_ALARM)) { return; }

  // Drain the planner first: a feed hold requested from IDLE is marked complete
  // straight away (protocol.c), so the wait below exits its first loop immediately.
  //protocol_buffer_synchronize();
  //if (sys.abort) { return; }
 
  // Enter the hold and wait for it to finish decelerating.
  system_set_exec_state_flag(EXEC_FEED_HOLD);
  do {
    protocol_execute_realtime();
    if (sys.abort || (sys.state & STATE_ALARM)) { return; }
  } while (!((sys.state & STATE_HOLD) && (sys.suspend & SUSPEND_HOLD_COMPLETE)));

  // Wait for cycle start to resume. With an empty planner this clears the suspend
  // and drops back to STATE_IDLE (protocol.c).
  do {
    protocol_execute_realtime();
    if (sys.abort || (sys.state & STATE_ALARM)) { return; }
  } while (sys.state & STATE_HOLD);
}


// Change spindle state and remember it for the motion helpers below.
static void tool_spindle(uint8_t state, float speed)
{
  spindle_sync(state, speed);
  cycle_spindle_state = state;
  cycle_spindle_speed = speed;
}


// Queue a rapid to target and advance the parser position.
static void tool_rapid(float *target)
{
  plan_line_data_t pl;
  memset(&pl, 0, sizeof(plan_line_data_t));
  pl.condition = PL_COND_FLAG_RAPID_MOTION | cycle_spindle_state;
  pl.spindle_speed = cycle_spindle_speed;
  mc_line(target, &pl);
  memcpy(gc_state.position, target, sizeof(gc_state.position));
}


// Queue a feed move to target and advance the parser position.
static void tool_feed(float *target, float feed_rate)
{
  plan_line_data_t pl;
  memset(&pl, 0, sizeof(plan_line_data_t));
  pl.feed_rate = feed_rate;
  pl.condition = cycle_spindle_state;
  pl.spindle_speed = cycle_spindle_speed;
  mc_line(target, &pl);
  memcpy(gc_state.position, target, sizeof(gc_state.position));
}


// Rapid to the safe height (work coordinate 0) at the current XY.
static void tool_retract(float *wco)
{
  float target[N_AXIS];
  memcpy(target, gc_state.position, sizeof(target));
  target[axis_z] = TOOL_SAFE_Z + wco[axis_z];
  tool_rapid(target);
}


// Returns the tool currently in the spindle (TOOL_CHANGE_EMPTY if none).
uint8_t tool_change_current_tool(void)
{
  return(current_tool);
}


// Runs the M6 cycle: unload (if a tool is loaded), load the requested tool, probe check.
// Motion is work-coordinate relative, converted to machine coordinates through wco[].
// wcs[] is the active work coordinate system origin, needed for the probe's G92.
uint8_t tool_change_cycle(uint8_t tool, float *wco, float *wcs)
{
  uint8_t status;

  printPgmString(PSTR("tool change cycle"));

  // Stop whatever the job left running before the machine moves over the rack.
  tool_spindle(SPINDLE_DISABLE, 0.0);

  if (current_tool != TOOL_CHANGE_EMPTY) {
    if ((status = tool_unload(current_tool, wco)) != STATUS_OK) { return(status); }
  }
  if (tool != TOOL_CHANGE_EMPTY) {
    if ((status = tool_load(tool, wco)) != STATUS_OK) { return(status); }
    if ((status = tool_probe_check(wco, wcs)) != STATUS_OK) { return(status); }
  }

  // Check mode ($C) validates motion without moving, so don't record a load that
  // never happened — otherwise a later real M6 would be wrongly driven to unload.
  if (sys.state != STATE_CHECK_MODE) { current_tool = tool; }
  return(STATUS_OK);
}


// Returns the loaded tool from its rack slot back to the spindle's rack position,
// i.e. unscrews it. Post processor sequence plus a safety retract before the XY move.
static uint8_t tool_unload(uint8_t tool, float *wco)
{
  const float *slot = tool_slot_xy[tool-1];
  float target[N_AXIS];

  // Safety addition over the post processor: clear the work area before traversing,
  // so an unload never moves in XY at whatever Z the job stopped at.
  //tool_retract(wco);
  tool_home_z();
  // M0: operator checkpoint before the rack move.
 // tool_change_pause();

  // M5
  tool_spindle(SPINDLE_DISABLE, 0.0);

  // G0 X.. Y..: rapid to the tool slot (work coordinates -> machine coordinates).
  memcpy(target, gc_state.position, sizeof(target));
  target[axis_x] = slot[0] + wco[axis_x];
  target[axis_y] = slot[1] + wco[axis_y];
  tool_rapid(target);

  // M3 S150: slow rotation while plunging to the engagement depth.
  tool_spindle(SPINDLE_ENABLE_CCW, TOOL_UNLOAD_ENGAGE_SPEED);

  // G1 Z.. F150
  //target[axis_z] = TOOL_LOAD_Z + wco[axis_z];
  target[axis_z] = TOOL_LOAD_Z;
  tool_feed(target, TOOL_UNLOAD_ENGAGE_FEED);

  // G4 P1: let the spindle settle at engage depth.
  mc_dwell(TOOL_UNLOAD_ENGAGE_DWELL);

  // M0: operator confirms the nut is seated on the rack.
 // tool_change_pause();

  // G1 Z.. F50: fine adjustment below the load depth.
  target[axis_z] = TOOL_UNLOAD_FINE_Z;
  tool_feed(target, TOOL_UNLOAD_FINE_FEED);

  // M4 S500: begin reversing.
  tool_spindle(SPINDLE_ENABLE_CCW, TOOL_UNLOAD_START_SPEED);

  // G4 P0.5
  mc_dwell(TOOL_UNLOAD_START_DWELL);

  // M4 S200: main unscrew speed.
  tool_spindle(SPINDLE_ENABLE_CCW, TOOL_UNLOAD_MAIN_SPEED);

  // G1 Z.. F200: back off while reversing to unwind the thread.
  target[axis_z] = TOOL_UNLOAD_RETRACT_Z;
  tool_feed(target, TOOL_UNLOAD_RETRACT_FEED);

  // M0: operator confirms the tool has released.
 // tool_change_pause();

  // G0 Z safe, then stop the spindle for the load phase.
  tool_home_z();
  tool_spindle(SPINDLE_DISABLE, 0.0);

  return(STATUS_OK);
}


// Loads tool (1..TOOL_CHANGE_SLOTS) from its rack slot into the spindle.
// Two-stage forward: slow engage, then fast seat.
static uint8_t tool_load(uint8_t tool, float *wco)
{
  //printPgmString(PSTR("tool change load"));


  const float *slot = tool_slot_xy[tool-1];
  float target[N_AXIS];

  // G0 Z safe
  tool_home_z();

  // G0 X.. Y..: rapid to the tool slot.
  memcpy(target, gc_state.position, sizeof(target));
  target[axis_x] = slot[0] + wco[axis_x];
  target[axis_y] = slot[1] + wco[axis_y];
  tool_rapid(target);

  // M3 S150: slow rotation, started before the approach so the spindle has the
  // approach travel to reach speed before the thread engages.
  tool_spindle(SPINDLE_ENABLE_CW, TOOL_LOAD_ENGAGE_SPEED);

  // G0 Z..: rapid to the approach height, still in air above the engagement zone.
  // The block must carry the spindle state or the stepper forces PWM off here.
  target[axis_z] = TOOL_LOAD_Z_APPROACH + wco[axis_z];
  tool_rapid(target);

  // G1 Z.. F200: feed through the engagement zone to the final tool depth.
  target[axis_z] = TOOL_LOAD_Z + wco[axis_z];
  tool_feed(target, TOOL_LOAD_ENGAGE_FEED);

  // G4 P0.5: allow the tool to seat.
  mc_dwell(TOOL_LOAD_ENGAGE_DWELL);

  // M5
  tool_spindle(SPINDLE_DISABLE, 0.0);

  // G4 P0.5
  mc_dwell(TOOL_LOAD_ENGAGE_DWELL);

  // M3 S750: spin up to tighten the holder.
  tool_spindle(SPINDLE_ENABLE_CW, TOOL_LOAD_SEAT_SPEED);

  // G4 P0.5
  mc_dwell(TOOL_LOAD_SEAT_DWELL);

  // M5: stop the spindle.
  tool_spindle(SPINDLE_DISABLE, 0.0);

  // G4 P0.5
  mc_dwell(TOOL_LOAD_ENGAGE_DWELL);

  // M3 S750: spin up to tighten the holder.
  tool_spindle(SPINDLE_ENABLE_CW, TOOL_LOAD_SEAT_SPEED);

  // G4 P0.5
  mc_dwell(TOOL_LOAD_SEAT_DWELL);

  // M5: stop the spindle.
  tool_spindle(SPINDLE_DISABLE, 0.0);

  return(STATUS_OK);
}


// Probes the work surface and establishes the new Z work offset (G92), then parks at
// the job's work origin so the next block resumes where the post processor would have
// left it. Probe XY is work coordinate: the plate sits at work X0 Y0 plus the
// TOOL_PROBE_X / TOOL_PROBE_Y offset.
static uint8_t tool_probe_check(float *wco, float *wcs)
{

  //printPgmString(PSTR("tool change probe 1"));

  float target[N_AXIS];
  float wco_now[N_AXIS];
  plan_line_data_t pl;
  uint8_t idx;

  // G0 Z safe, then G0 X0 Y0 (work): rapid over the plate.
  tool_home_z();
  memcpy(target, gc_state.position, sizeof(target));
  target[axis_x] = TOOL_PROBE_X + wco[axis_x];
  target[axis_y] = TOOL_PROBE_Y + wco[axis_y];
  tool_rapid(target);

    //printPgmString(PSTR("tool change probe 2"));

  // M0: operator positions the probe.
  //tool_change_pause();

    //printPgmString(PSTR("tool change probe 3"));

  // Check mode never contacts, so there is no probe position to work from. The moves
  // above were still soft-limit checked by mc_line; stop before anything dependent
  // on the contact point, and leave the G92 offset alone.
  if (sys.state == STATE_CHECK_MODE) { return(STATUS_OK); }

  //printPgmString(PSTR("tool change probe 4"));

  // G91 G38.2 Z-60 F100: fast probe, relative to the current position.
  memcpy(target, gc_state.position, sizeof(target));
  target[axis_z] += TOOL_PROBE_MAX_DEPTH;
  memset(&pl, 0, sizeof(plan_line_data_t));
  pl.feed_rate = TOOL_PROBE_FAST_FEED;
  #ifndef ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES
    pl.condition |= PL_COND_FLAG_NO_FEED_OVERRIDE;
  #endif
  if (mc_probe_cycle(target, &pl, 0) != GC_PROBE_FOUND) { return(STATUS_TOOL_PROBE_FAILED); }
  gc_sync_position(); // Parser now sits at the contact point.

  //printPgmString(PSTR("tool change probe 5"));

  // G0 Z2: lift relative to the contact point.
  memcpy(target, gc_state.position, sizeof(target));
  target[axis_z] += TOOL_PROBE_RETRACT;
  tool_rapid(target);

  //printPgmString(PSTR("tool change probe 6"));

  // G38.2 Z-4 F10: slow precision probe. The post processor steps -4 relative from
  // the lifted position, i.e. -(RETRACT + 2) from the first contact point.
  memcpy(target, gc_state.position, sizeof(target));
  target[axis_z] -= (TOOL_PROBE_RETRACT + 2.0f);
  memset(&pl, 0, sizeof(plan_line_data_t));
  pl.feed_rate = TOOL_PROBE_SLOW_FEED;
  #ifndef ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES
    pl.condition |= PL_COND_FLAG_NO_FEED_OVERRIDE;
  #endif
  if (mc_probe_cycle(target, &pl, 0) != GC_PROBE_FOUND) { return(STATUS_TOOL_PROBE_FAILED); }
  gc_sync_position();

  //printPgmString(PSTR("tool change probe 7"));

  // G92 Z..: offset current system so the machine reads the given work value.
  // Same math as the parser's G92 (gcode.c): G92 = MPos - WCS - WPos (- TLO on the
  // tool length axis). Only the Z axis is named, so the other offsets are untouched.
  gc_state.coord_offset[axis_z] = gc_state.position[axis_z] - wcs[axis_z]
                                - TOOL_PROBE_G92_OFFSET - gc_state.tool_length_offset;
  system_flag_wco_change();


  //printPgmString(PSTR("tool change probe 8"));

  // The work coordinate offset has changed under us; rebuild it for the park moves.
  for (idx = 0; idx < N_AXIS; idx++) { wco_now[idx] = wcs[idx] + gc_state.coord_offset[idx]; }

  // G0 Z5 (work): lift clear of the surface under the new offset.
  memcpy(target, gc_state.position, sizeof(target));
  target[axis_z] = TOOL_PROBE_LIFT_Z + wco_now[axis_z];
  tool_rapid(target);

  //printPgmString(PSTR("tool change probe 9"));

  // G0 X0 Y0 (work): return to the job origin.
  memcpy(target, gc_state.position, sizeof(target));
  target[axis_x] = wco_now[axis_x];
  target[axis_y] = wco_now[axis_y];
  tool_rapid(target);

    //printPgmString(PSTR("tool change probe 10"));

  // G0 Z20 (work): park above the work.
  target[axis_z] = TOOL_PROBE_RETURN_Z + wco_now[axis_z];
  tool_rapid(target);

  //printPgmString(PSTR("tool change probe 11"));

  // M0 x2: operator removes the probe.
  //tool_change_pause();
  //tool_change_pause();

  //printPgmString(PSTR("tool change probe 12"));
  
  tool_home_z();
  
  return(STATUS_OK);
}

// Homes only the Z axis. Call with the planner drained and the spindle stopped.
static uint8_t tool_home_z(void)
{
  // Homing must be enabled in settings ($22=1) and hardware.
  if (bit_isfalse(settings.flags, BITFLAG_HOMING_ENABLE)) { return(STATUS_SETTING_DISABLED); }

  // Check mode never moves, so skip.
  if (sys.state == STATE_CHECK_MODE) { return(STATUS_OK); }

  // Homing can't run with queued motion. Drain the planner first.
  protocol_buffer_synchronize();
  if (sys.abort) { return(STATUS_OK); }

  uint8_t prev_state = sys.state;
  sys.state = STATE_HOMING;

  mc_homing_cycle(bit(axis_z));   // or HOMING_CYCLE_Z if it maps to your axis

  if (sys.abort) { return(STATUS_OK); }   // reset/alarm during homing

  // Restore state and resync everything to the new homed position.
  sys.state = prev_state;                  // normally STATE_CYCLE/IDLE
  st_go_idle();
  plan_sync_position();
  gc_sync_position();                      // gc_state.position now = homed machine position

  return(STATUS_OK);
}

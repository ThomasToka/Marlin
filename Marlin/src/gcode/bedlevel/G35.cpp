/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(ASSISTED_TRAMMING)

#include "../gcode.h"
#include "../../module/planner.h"
#include "../../module/probe.h"
#include "../../feature/bedlevel/bedlevel.h"

#if ENABLED(BLTOUCH)
  #include "../../feature/bltouch.h"
#endif

#if ENABLED(E3S1PRO_RTS)
  #include "../../lcd/rts/e3s1pro/lcd_rts.h"  
#endif

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../core/debug_out.h"

//
// Define tramming point names.
//

#include "../../feature/tramming.h"

xy_pos_t tramming_points[4]; // Global array for tramming points

// Implementation of updateTrammingPoints
void updateTrammingPoints() {
    tramming_points[0].x = lcd_rts_settings.probe_margin_x;
    tramming_points[0].y = lcd_rts_settings.probe_margin_y_front;

    tramming_points[1].x = X_BED_SIZE - lcd_rts_settings.probe_margin_x;
    tramming_points[1].y = lcd_rts_settings.probe_margin_y_front;

    tramming_points[2].x = lcd_rts_settings.probe_margin_x;
    tramming_points[2].y = Y_BED_SIZE - lcd_rts_settings.probe_margin_y_back;

    tramming_points[3].x = X_BED_SIZE - lcd_rts_settings.probe_margin_x;
    tramming_points[3].y = Y_BED_SIZE - lcd_rts_settings.probe_margin_y_back;
    // Add more points if needed
}

/**
 * G35: Read bed corners to help adjust bed screws
 *
 *   S<screw_thread>
 *
 * Screw thread: 30 - Clockwise M3
 *               31 - Counter-Clockwise M3
 *               40 - Clockwise M4
 *               41 - Counter-Clockwise M4
 *               50 - Clockwise M5
 *               51 - Counter-Clockwise M5
 **/
void GcodeSuite::G35() {
  updateTrammingPoints();
  DEBUG_SECTION(log_G35, "G35", DEBUGGING(LEVELING));

  if (DEBUGGING(LEVELING)) log_machine_info();

  float z_measured[G35_PROBE_COUNT] = { 0 };

  const uint8_t screw_thread = parser.byteval('S', TRAMMING_SCREW_THREAD);
  if (!WITHIN(screw_thread, 30, 51) || screw_thread % 10 > 1) {
    SERIAL_ECHOLNPGM(GCODE_ERR_MSG("(S)crew thread must be 30, 31, 40, 41, 50, or 51."));
    return;
  }

  // Wait for planner moves to finish!
  planner.synchronize();

  // Disable the leveling matrix before auto-aligning
  #if HAS_LEVELING
    #if ENABLED(RESTORE_LEVELING_AFTER_G35)
      const bool leveling_was_active = planner.leveling_active;
    #endif
    set_bed_leveling_enabled(false);
  #endif

  TERN_(CNC_WORKSPACE_PLANES, workspace_plane = PLANE_XY);

  probe.use_probing_tool();

  // Disable duplication mode on homing
  TERN_(HAS_DUPLICATION_MODE, set_duplication_enabled(false));

  // Home only Z axis when X and Y is trusted, otherwise all axes, if needed before this procedure
  if (!all_axes_trusted()) process_subcommands_now(F("G28Z"));

  bool err_break = false;

  bool probed_points[G35_PROBE_COUNT] = {false};

  // Probe all positions
  for (uint8_t i = 0; i < G35_PROBE_COUNT; ++i) {
    const float z_probed_height = probe.probe_at_point(tramming_points[i], PROBE_PT_RAISE);
    if (isnan(z_probed_height)) {
      SERIAL_ECHOLN(
        F("G35 failed at point "), i + 1, 
        F(" ("), FPSTR(pgm_read_ptr(&tramming_point_name[i])), C(')'),
        FPSTR(SP_X_STR), tramming_points[i].x, 
        FPSTR(SP_Y_STR), tramming_points[i].y
      );
      err_break = true;
      break;
    }

    if (DEBUGGING(LEVELING)) {
      DEBUG_ECHOLN(
        F("Probing point "), i + 1, F(" ("), FPSTR(pgm_read_ptr(&tramming_point_name[i])), C(')'),
        FPSTR(SP_X_STR), tramming_points[i].x, FPSTR(SP_Y_STR), tramming_points[i].y,
        FPSTR(SP_Z_STR), z_probed_height
      );
    }
    #if ENABLED(E3S1PRO_RTS)    
      rtscheck.RTS_SndData(z_probed_height * 1000, ASSISTED_TRAMMING_POINT_1_VP + i);
      #if ENABLED(LCD_RTS_DEBUG_LEVELING)
        SERIAL_ECHO_MSG("tramming_point_name[i] ", tramming_point_name[i]);
        SERIAL_ECHO_MSG("z_probed_height ", z_probed_height);
      #endif
    #endif
    z_measured[i] = z_probed_height;
    probed_points[i] = true; // Mark this point as probed
  }

  if (!err_break) {
    const float threads_factor[] = { 0.5, 0.7, 0.8 };

    // Calculate adjusts
    for (uint8_t i = 0; i < G35_PROBE_COUNT; ++i) {
      if (!probed_points[i]) continue; // Skip points that were not probed

      const float diff = z_measured[0] - z_measured[i],
                  adjust = ABS(diff) < 0.001f ? 0 : diff / threads_factor[(screw_thread - 30) / 10];

      const int full_turns = trunc(adjust);
      const float decimal_part = adjust - float(full_turns);
      const int minutes = trunc(decimal_part * 60.0f);

      char turns[4];
      char mins[4];
      itoa(ABS(full_turns), turns, 10);
      itoa(ABS(minutes), mins, 10);

      char str[26];
      strcpy(str, (screw_thread & 1) == (adjust > 0) ? "DOWN " : "UP ");
      strcat(str, turns);
      strcat(str, " turns & ");
      strcat(str, mins);
      strcat(str, " mins");
      #if ENABLED(LCD_RTS_DEBUG_LEVELING)
        SERIAL_ECHOPGM("Turn ");
        SERIAL_ECHOPGM_P((char *)pgm_read_ptr(&tramming_point_name[i]));
        SERIAL_ECHOPGM(" ", (screw_thread & 1) == (adjust > 0) ? "DOWN" : "UP", " by ", ABS(full_turns), " turns");
        if (minutes) SERIAL_ECHOPGM(" and ", ABS(minutes), " minutes");
        if (ENABLED(REPORT_TRAMMING_MM)) SERIAL_ECHOPGM(" (", -diff, "mm)");
        SERIAL_EOL();
      #endif
      
      #if ENABLED(E3S1PRO_RTS)
        unsigned long addr = ASSISTED_TRAMMING_POINT_TEXT_VP + i * 26;
        for(int j = 0; j < 26; j++) {
          RTS_ResetSingleVP(addr + j);
        }
        rtscheck.RTS_SndData(str, addr);
      #endif
    }
    #if ENABLED(E3S1PRO_RTS)    
      leveling_running = 0;
      RTS_ShowPage(98);
    #endif
  
  } else {
  
    SERIAL_ECHOLNPGM("G35 aborted.");
  
  }

  // Restore the active tool after homing
  probe.use_probing_tool(false);

  #if ALL(HAS_LEVELING, RESTORE_LEVELING_AFTER_G35)
    set_bed_leveling_enabled(leveling_was_active);
  #endif

  // Stow the probe, as the last call to probe.probe_at_point(...) left
  // the probe deployed if it was successful.
  probe.stow();

  move_to_tramming_wait_pos();

  // After this operation the Z position needs correction
  set_axis_never_homed(Z_AXIS);
}

#endif // ASSISTED_TRAMMING

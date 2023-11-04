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

#if HAS_BED_PROBE

#include "../gcode.h"
#include "../../module/motion.h"
#include "../../module/probe.h"
#include "../../feature/bedlevel/bedlevel.h"
#include "../../lcd/marlinui.h"

#if ENABLED(E3S1PRO_RTS)
  #include "../../lcd/rts/e3s1pro/lcd_rts.h"  
#endif

#if HAS_PTC
  #include "../../feature/probe_temp_comp.h"
#endif

#if HAS_MULTI_HOTEND
  #include "../../module/tool_change.h"
#endif

/**
 * G30: Do a single Z probe at the given XY (default: current)
 *
 * Parameters:
 *
 *   X   Probe X position (default current X)
 *   Y   Probe Y position (default current Y)
 *   E   Engage the probe for each probe (default 1)
 *   C   Enable probe temperature compensation (0 or 1, default 1)
 */
void GcodeSuite::G30() {

  xy_pos_t old_pos = current_position,
           probepos = current_position;

  const bool seenX = parser.seenval('X');
  if (seenX) probepos.x = RAW_X_POSITION(parser.value_linear_units());
  const bool seenY = parser.seenval('Y');
  if (seenY) probepos.y = RAW_Y_POSITION(parser.value_linear_units());

  probe.use_probing_tool();

  if (probe.can_reach(probepos)) {

    if (seenX) old_pos.x = probepos.x;
    if (seenY) old_pos.y = probepos.y;

    // Disable leveling so the planner won't mess with us
    TERN_(HAS_LEVELING, set_bed_leveling_enabled(false));

    remember_feedrate_scaling_off();

    #if ANY(DWIN_LCD_PROUI, DWIN_CREALITY_LCD_JYERSUI)
      process_subcommands_now(F("G28O"));
    #endif

    const ProbePtRaise raise_after = parser.boolval('E', true) ? PROBE_PT_STOW : PROBE_PT_NONE;

    TERN_(HAS_PTC, ptc.set_enabled(parser.boolval('C', true)));
    const float measured_z = probe.probe_at_point(probepos, raise_after);
    TERN_(HAS_PTC, ptc.set_enabled(true));
    if (!isnan(measured_z)) {
      const xy_pos_t lpos = probepos.asLogical();
      SString<30> msg(
        F("Bed X:"), p_float_t(lpos.x, 2),
        F(  " Y:"), p_float_t(lpos.y, 2),
        F(  " Z:"), p_float_t(measured_z, 3)
      );
      msg.echoln();
      #if ANY(DWIN_LCD_PROUI, DWIN_CREALITY_LCD_JYERSUI)
        ui.set_status(msg);
      #endif

      #if ENABLED(E3S1PRO_RTS)
        struct TrammingPoint {
          float x;
          float y;
          int vp;
        };
        // Create an array of TrammingPoint objects for all ten points
        TrammingPoint trammingPoints[10] = {
          // Center definition
          {117.50, 117.50, CRTOUCH_TRAMMING_POINT_1_VP},
          {155.00, 157.50, CRTOUCH_TRAMMING_POINT_1_VP},
        };
        uint16_t min_margin_x;
        uint16_t min_margin_y_front;
        uint16_t min_margin_y_back;          
        float probe_offset_x_temp;
        float probe_offset_y_temp;  

        if (probe.offset_xy.x < 0) {
          probe_offset_x_temp = fabs(probe.offset_xy.x);
        }else{
          probe_offset_x_temp = -fabs(probe.offset_xy.x);
        }

        if (probe.offset_xy.y < 0) {
          probe_offset_y_temp = fabs(probe.offset_xy.y);
        }else{
          probe_offset_y_temp = -fabs(probe.offset_xy.y);
        }

        uint16_t max_reachable_pos_x = X_MAX_POS - custom_ceil(probe_offset_x_temp);
        uint16_t max_reachable_pos_y = Y_MAX_POS - custom_ceil(probe_offset_y_temp);       
        uint16_t min_calc_margin_x = X_BED_SIZE - max_reachable_pos_x;
        uint16_t min_calc_margin_y = Y_BED_SIZE - max_reachable_pos_y;      

        if(min_calc_margin_x >= lcd_rts_settings.probe_margin_x){
          min_margin_x = min_calc_margin_x;
        }else{
          min_margin_x = lcd_rts_settings.probe_margin_x;
        }

        if (min_calc_margin_y <= lcd_rts_settings.probe_margin_y){
          if (min_calc_margin_x <= lcd_rts_settings.probe_margin_y){
            if (lcd_rts_settings.probe_margin_y <= lcd_rts_settings.probe_margin_x){
              min_margin_y_front = lcd_rts_settings.probe_margin_y;
            }else{
              min_margin_y_front = lcd_rts_settings.probe_margin_x;
            }              
            }else{
              min_margin_y_front = lcd_rts_settings.probe_margin_y;
            }
        }else{
          min_margin_y_front = min_calc_margin_y;
        }
        if (min_margin_y_front <= 10){
          min_margin_y_front = 10;
        }
        if(min_calc_margin_y >= lcd_rts_settings.probe_margin_y){
          min_margin_y_back = static_cast<int>(std::ceil(min_calc_margin_y));
        }else{
          min_margin_y_back = static_cast<int>(std::ceil(lcd_rts_settings.probe_margin_y));
        }
        if (min_margin_y_back <= 10){
          min_margin_y_back = 10;
        }
        trammingPoints[2] = {static_cast<float>(min_margin_x), static_cast<float>(min_margin_y_front), CRTOUCH_TRAMMING_POINT_6_VP};
        trammingPoints[3] = {(X_BED_SIZE - static_cast<float>(min_margin_x)), static_cast<float>(min_margin_y_front), CRTOUCH_TRAMMING_POINT_7_VP};
        trammingPoints[4] = {static_cast<float>(min_margin_x), (Y_BED_SIZE - static_cast<float>(min_margin_y_back)), CRTOUCH_TRAMMING_POINT_8_VP};
        trammingPoints[5] = {(X_BED_SIZE - static_cast<float>(min_margin_x)), (Y_BED_SIZE - static_cast<float>(min_margin_y_back)), CRTOUCH_TRAMMING_POINT_9_VP};
        int POINTS[10] = {0};
        for (int i = 0; i < 10; i++) {
          const auto& point = trammingPoints[i];
          if (probepos.x == point.x && probepos.y == point.y) {
            rtscheck.RTS_SndData(measured_z * 1000, point.vp);
            POINTS[i] = 1;
          }
        }
        if (POINTS[0] == 1 || POINTS[1] == 1) {
          leveling_running = 0;
        }
      #endif

    }

    restore_feedrate_and_scaling();

    do_blocking_move_to(old_pos);

    if (raise_after == PROBE_PT_STOW)
      probe.move_z_after_probing();

    report_current_position();
  }
  else {
    SERIAL_ECHOLN(GET_EN_TEXT_F(MSG_ZPROBE_OUT));
    LCD_MESSAGE(MSG_ZPROBE_OUT);
  }

  probe.use_probing_tool(false);
}

#endif // HAS_BED_PROBE

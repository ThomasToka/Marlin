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

#if HAS_STATUS_MESSAGE

#include "../gcode.h"
#include "../../lcd/marlinui.h"

#if ENABLED(E3S1PRO_RTS)
  #include "../../lcd/rts/e3s1pro/lcd_rts.h"
#endif

/**
 * M117: Set LCD Status Message
 */
void GcodeSuite::M117() {

  #if ENABLED(E3S1PRO_RTS)
    if (parser.string_arg[0] == 'L' && (isdigit(parser.string_arg[1]) || (parser.string_arg[1] == '-' && isdigit(parser.string_arg[2])))) {
      bool hasL = false, hasG = false, hasM = false;
      int16_t m117_layer = 0;
      uint16_t picFilament_g_todo = 0, picFilament_m_todo = 0;
      // Check for the presence of 'L', 'G', and 'M' commands
      for (int i = 0; parser.string_arg[i] != '\0'; ++i) {
          if ((parser.string_arg[i] == 'L' || parser.string_arg[i] == 'G' || parser.string_arg[i] == 'M' || parser.string_arg[i] == 'Q' || parser.string_arg[i] == 'Z') &&
              (isdigit(parser.string_arg[i + 1]) || (parser.string_arg[i + 1] == '-' && isdigit(parser.string_arg[i + 2])))) {

              int start = i + 1;
              int end = start;
              bool isFloat = false;
              while (isdigit(parser.string_arg[end]) || parser.string_arg[end] == '.' || (end == start && parser.string_arg[end] == '-')) {
                  if (parser.string_arg[end] == '.') {
                      isFloat = true;
                  }
                  ++end;
              }

              char number_str[20];  // Adjust buffer size as needed
              strncpy(number_str, parser.string_arg + start, end - start);
              number_str[end - start] = '\0';

              if (isFloat) {
                  float number = strtof(number_str, nullptr);  // Parse as float
                  if (parser.string_arg[i] == 'Z') {
                    picLayerHeight = number;  // Float for Z variable
                    #if ENABLED(LCD_RTS_DEBUG_LCD)
                      SERIAL_ECHO_MSG("picLayerHeight: ", picLayerHeight);
                    #endif
                  }
              } else {
                  int number = strtol(number_str, nullptr, 10);  // Parse as int
                  if (parser.string_arg[i] == 'L') {
                      hasL = true;
                      m117_layer = number;  // Integer for L variable
                  } else if (parser.string_arg[i] == 'G') {
                      hasG = true;
                      picFilament_g_todo = number;  // Integer for G variable
                  } else if (parser.string_arg[i] == 'M') {
                      hasM = true;
                      picFilament_m_todo = number;  // Integer for M variable
                  } else if (parser.string_arg[i] == 'Q') {
                      picLayers = number;  // Integer for Q variable
                      #if ENABLED(LCD_RTS_DEBUG_LCD)
                        SERIAL_ECHO_MSG("picLayers: ", picLayers);
                      #endif
                  }
              }
          }
      }

      if (hasL && hasG && hasM) {
        rtscheck.RTS_SndData(m117_layer, PRINT_LAYERS_DONE_VP);
        float current_z_pos = current_position.z;
          rtscheck.RTS_SndData(current_z_pos * 100, PRINT_CURRENT_Z_VP);
          rtscheck.RTS_SndData(picFilament_g_todo, PRINT_FILAMENT_G_TODO_VP);
          rtscheck.RTS_SndData(picFilament_m_todo, PRINT_FILAMENT_M_TODO_VP);
        if(m117_layer == 1){
          rtscheck.RTS_SndData(picFilament_g_todo, PRINT_FILAMENT_G_VP);
          rtscheck.RTS_SndData(picFilament_m_todo, PRINT_FILAMENT_M_VP);
          rtscheck.RTS_SndData(picLayers, PRINT_LAYERS_VP);
          rtscheck.RTS_SndData(picLayerHeight * 100, PRINT_LAYER_HEIGHT_VP);
          RTS_SendM600Icon(true);
          RTS_ShowPage(10);
        }
        #if ENABLED(LCD_RTS_DEBUG_LCD)
          SERIAL_ECHO_MSG("Current Position Z: ", current_z_pos);
          SERIAL_ECHO_MSG("L-command: ", m117_layer);
          SERIAL_ECHO_MSG("G-command: ", picFilament_g_todo);
          SERIAL_ECHO_MSG("M-command: ", picFilament_m_todo);
          SERIAL_ECHO_MSG("picLayers: ", picLayers);
          SERIAL_ECHO_MSG("picLayerHeight: ", picLayerHeight);                    
        #endif
      }
    } else {
      RTS_CleanPrintAndSelectFile();      
      char msg[55];
      if (strlen(parser.string_arg) >= TEXTBYTELEN) {
        strncpy(msg, parser.string_arg, TEXTBYTELEN - 1);
        msg[TEXTBYTELEN - 1] = '\0';
      } else {
        strcpy(msg, parser.string_arg);
      }
      #if ENABLED(LCD_RTS_DEBUG_LCD)      
        SERIAL_ECHO_MSG("msg M117: ", msg);
      #endif
      if (strlen(msg) > 25) {
        RTS_ResetSingleVP(PRINT_FILE_TEXT_VP);
        rtscheck.RTS_SndData(msg, SELECT_FILE_TEXT_VP);
      }else{
        RTS_ResetSingleVP(SELECT_FILE_TEXT_VP);        
        rtscheck.RTS_SndData(msg, PRINT_FILE_TEXT_VP);
      }
    }
  #else
    if (parser.string_arg && parser.string_arg[0])
      ui.set_status_no_expire(parser.string_arg);
    else
      ui.reset_status();
  #endif
  
}

#endif // HAS_STATUS_MESSAGE

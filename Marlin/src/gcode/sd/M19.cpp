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

#if HAS_MEDIA && ENABLED(E3S1PRO_RTS)

#include "../gcode.h"
#include "../../sd/cardreader.h"
#include "../../lcd/marlinui.h"
#include "../../lcd/rts/e3s1pro/lcd_rts.h"
#include "../../lcd/rts/e3s1pro/preview.h"
uint8_t action;
uint8_t m19_x;
uint8_t m19_y;
uint8_t m19_f;
uint8_t m19_p;

/**
 * M19: Open a file readonly without starting a print
 *
 * The path is relative to the root directory
 */
void GcodeSuite::M19() {
if (parser.seenval('S')) action = parser.value_int();
const char* filename_input = "OCTODGUS.GCO";
const char* filename_output = "TEMP.GCO";

// Declare outputCopy here
char outputCopy[64];
strcpy(outputCopy, filename_output);

switch(action){
    case 1: {
      if (!card.fileExists(filename_input)) return;
      card.openFileReadonly(filename_input);
      if (!card.isFileOpen()) return;
      size_t fileLength = card.getFileSize();
      if (fileLength > 24500) { card.closefile(); return; }
      char fileContents[fileLength + 1];
      const size_t bytesRead = card.read((uint8_t*)fileContents, fileLength);
      fileContents[bytesRead] = '\0';
      card.closefile();
      card.openFileWrite(outputCopy);
      if (!card.isFileOpen()) return;
      auto starts_with = [](const char* s, const char* e, const char* prefix) -> bool {
        const size_t n = strlen(prefix);
        return (size_t)(e - s) >= n && strncmp(s, prefix, n) == 0;
      };
      auto skip_ws = [](char* p, char* e) -> char* {
        while (p < e && (*p == ' ' || *p == '\t')) ++p;
        return p;
      };
      auto is_comment_begin = [&](char* s, char* e) -> bool {
        return starts_with(s, e, "; thumbnail begin")
            || starts_with(s, e, "; thumbnail_JPG begin")
            || starts_with(s, e, "; jpg begin");
      };
      auto is_comment_end = [&](char* s, char* e) -> bool {
        return starts_with(s, e, "; thumbnail end")
            || starts_with(s, e, "; thumbnail_JPG end")
            || starts_with(s, e, "; jpg end");
      };
      auto is_m4010_begin = [&](char* s, char* e) -> bool {
        if (!starts_with(s, e, "M4010")) return false;
        char* a = skip_ws(s + 5, e);
        const char* p1 = "thumbnail ";
        const char* p2 = "thumbnail_JPG ";
        const char* p3 = "jpg ";
        if (starts_with(a, e, p1))      return starts_with(a + strlen(p1), e, "begin");
        else if (starts_with(a, e, p2)) return starts_with(a + strlen(p2), e, "begin");
        else if (starts_with(a, e, p3)) return starts_with(a + strlen(p3), e, "begin");
        return false;
      };
      auto is_m4010_end = [&](char* s, char* e) -> bool {
        if (!starts_with(s, e, "M4010")) return false;
        char* a = skip_ws(s + 5, e);
        const char* p1 = "thumbnail ";
        const char* p2 = "thumbnail_JPG ";
        const char* p3 = "jpg ";
        if (starts_with(a, e, p1))      return starts_with(a + strlen(p1), e, "end");
        else if (starts_with(a, e, p2)) return starts_with(a + strlen(p2), e, "end");
        else if (starts_with(a, e, p3)) return starts_with(a + strlen(p3), e, "end");
        return false;
      };
      char* p = fileContents;
      bool inBlock = false;
      while (*p) {
        char* line_start = p;
        char* e = p;
        while (*e && *e != '\n' && *e != '\r') ++e;
        const size_t line_len = (size_t)(e - line_start);

        if (line_len) {
          if (line_start[0] == ';') {
            // comment markers: write them too
            if (is_comment_begin(line_start, e)) {
              card.write((void*)line_start, (uint16_t)line_len);
              const char nl = '\n'; card.write((void*)&nl, 1);
              inBlock = true;
            }
            else if (is_comment_end(line_start, e)) {
              card.write((void*)line_start, (uint16_t)line_len);
              const char nl = '\n'; card.write((void*)&nl, 1);
              inBlock = false;
            }
            // else: ignore other comment lines
          }
          else if (line_len >= 5 && strncmp(line_start, "M4010", 5) == 0) {
            if (is_m4010_begin(line_start, e)) {
              // write marker as "; ..." then enter block
              const char sc = ';';
              card.write((void*)&sc, 1);
              if (line_len > 5) card.write((void*)(line_start + 5), (uint16_t)(line_len - 5));
              const char nl = '\n'; card.write((void*)&nl, 1);
              inBlock = true;
            }
            else if (is_m4010_end(line_start, e)) {
              // write marker as "; ..." then leave block
              const char sc = ';';
              card.write((void*)&sc, 1);
              if (line_len > 5) card.write((void*)(line_start + 5), (uint16_t)(line_len - 5));
              const char nl = '\n'; card.write((void*)&nl, 1);
              inBlock = false;
            }
            else if (inBlock) {
              // payload inside block: "M4010 ..." -> "; ..."
              const char sc = ';';
              card.write((void*)&sc, 1);
              if (line_len > 5) card.write((void*)(line_start + 5), (uint16_t)(line_len - 5));
              const char nl = '\n'; card.write((void*)&nl, 1);
            }
            // else: outside block, ignore
          }
          // else: other non-marker, non-M4010 lines → ignore
        }

        // next line (CRLF / LFCR / CR / LF)
        if (*e == '\r' && e[1] == '\n') p = e + 2;
        else if (*e == '\n' && e[1] == '\r') p = e + 2;
        else if (*e == '\r' || *e == '\n')   p = e + 1;
        else                                 p = e;
      }
      card.closefile();
      int32_t ret = gcodePicDataOctoPrintSendToDwin(outputCopy, VP_OVERLAY_PIC_PTINT, PIC_FORMAT_JPG, PIC_RESOLUTION_250_250);
      if (ret == PIC_OK) {
        RTS_ResetPrintData(false);
        RTS_SendPrintData();
        RTS_ResetProgress();
        RTS_ShowPage(1);
      } else {
        RTS_ResetPrintData(true);
      }
      card.removeFile(filename_input);
      card.removeFile(outputCopy);
      break;
    }

    case 2:
        RTS_CleanPrintAndSelectFile();
        RTS_ResetPrintData(true);
        RTS_ResetProgress();
        RTS_SendM600Icon(false);
        RTS_SendM73Icon(false);
        lcd_rts_settings.external_m73 = false;
        RTS_ShowPage(1);
        return;                
    break;
    case 3:
        RTS_SendPrintData();
        RTS_LoadMeshPointOffsets();
        RTS_LoadMainsiteIcons();
        RTS_SendLevelingSiteData(0);
        RTS_ShowPage(10);
        return;
    break;
    case 4:
        RTS_ShowPage(12);
        return;
    break;
    case 5:
        RTS_ShowPage(10);
        return;
    break;
    case 6:
        RTS_SendM600Icon(false);                
        RTS_SendM73Icon(false);
        lcd_rts_settings.external_m73 = false;        
        RTS_ShowPage(9);
        return;
    break;
    case 7:
        if(leveling_running == 0)
        {    
            if (parser.seenval('P')){
                m19_p = parser.value_int();
                RTS_SetProbeCount(m19_p, 1);
            } 
            if (parser.seenval('F')){
                m19_f = parser.value_int();
                RTS_SetGridMaxPoints(m19_f, 1);
            }
            if (parser.seenval('X')){
                m19_x = parser.value_int();
                RTS_SetProbeMarginX(m19_x, 1);
            }
            if (parser.seenval('Y')){
                m19_y = parser.value_int();
                RTS_SetProbeMarginY(m19_y, 1);
            }
            settings.save();
        }
    break;
    case 8:
        if(leveling_running == 0)
        {    
          // removed in v035
        }
    break;
    case 9:
        if(leveling_running == 0)
        {    
            if (parser.seenval('F')){
                m19_f = parser.value_int();
                if (m19_f == 0){
                    lcd_rts_settings.boot_zraise = false;
                }else if (m19_f == 1){
                    lcd_rts_settings.boot_zraise = true;
                }
            }
            settings.save();
        }
    break;    
    default:
        return;
    break;
}

}

#endif // HAS_MEDIA && ENABLED(E3S1PRO_RTS)
/**
 * The idea for this class was taken from "Professional Firmware by Author: Miguel A. Risco-Castillo (MRISCOC)"
 * This class was modified by Thomas Toka for MARLIN-E3S1PRO-FORK-BYTT
 */

/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#pragma once

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  constexpr int8_t DEF_GRID_MAX_POINTS = GRID_MAX_POINTS_X;
#endif
#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  #ifndef PROBING_MARGIN
    #define PROBING_MARGIN 45
  #endif
  #ifndef PROBING_MARGIN_BACK
    #define PROBING_MARGIN_BACK 45
  #endif  
constexpr int16_t DEF_PROBING_MARGIN = PROBING_MARGIN;
#define MIN_PROBE_MARGIN 5
#define MAX_PROBE_MARGIN 60
#endif


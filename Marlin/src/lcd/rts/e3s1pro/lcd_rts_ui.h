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

#define X_BED_MIN 150
#define Y_BED_MIN 150
constexpr int16_t DEF_X_BED_SIZE = X_BED_SIZE;
constexpr int16_t DEF_Y_BED_SIZE = Y_BED_SIZE;
constexpr int16_t DEF_X_MIN_POS = X_MIN_POS;
constexpr int16_t DEF_Y_MIN_POS = Y_MIN_POS;
constexpr int16_t DEF_X_MAX_POS = X_MAX_POS;
constexpr int16_t DEF_Y_MAX_POS = Y_MAX_POS;
constexpr int16_t DEF_Z_MAX_POS = Z_MAX_POS;
//constexpr int8_t DEF_GRID_MAX_POINTS = TERN(HAS_MESH, GRID_MAX_POINTS_X, 3);
constexpr int8_t DEF_GRID_MAX_POINTS = 5;
#define GRID_MIN 3
#define GRID_LIMIT 9
#ifndef MESH_INSET
  #define MESH_INSET 25
#endif
#ifndef MESH_MIN_X
  #define MESH_MIN_X MESH_INSET
#endif
#ifndef MESH_MIN_Y
  #define MESH_MIN_Y MESH_INSET
#endif
#ifndef MESH_MAX_X
  #define MESH_MAX_X  X_BED_SIZE - (MESH_INSET)
#endif
#ifndef MESH_MAX_Y
  #define MESH_MAX_Y  Y_BED_SIZE - (MESH_INSET)
#endif
constexpr int16_t DEF_MESH_MIN_X = MESH_MIN_X;
constexpr int16_t DEF_MESH_MAX_X = MESH_MAX_X;
constexpr int16_t DEF_MESH_MIN_Y = MESH_MIN_Y;
constexpr int16_t DEF_MESH_MAX_Y = MESH_MAX_Y;
#define MIN_MESH_INSET TERN(HAS_BED_PROBE, PROBING_MARGIN, 5)
#define MAX_MESH_INSET X_BED_SIZE
constexpr int16_t DEF_PROBING_MARGIN = PROBING_MARGIN;
#define MIN_PROBE_MARGIN 5
#define MAX_PROBE_MARGIN 60
#ifndef MULTIPLE_PROBING
  #define MULTIPLE_PROBING 0
#endif

typedef struct {
  int16_t bed_size_x = DEF_X_BED_SIZE;
  int16_t bed_size_y = DEF_Y_BED_SIZE;
  int16_t x_min_pos  = DEF_X_MIN_POS;
  int16_t y_min_pos  = DEF_Y_MIN_POS;
  int16_t x_max_pos  = DEF_X_MAX_POS;
  int16_t y_max_pos  = DEF_Y_MAX_POS;
  int16_t z_max_pos  = DEF_Z_MAX_POS;
  uint8_t grid_max_points = DEF_GRID_MAX_POINTS;
  float mesh_min_x = DEF_MESH_MIN_X;
  float mesh_max_x = DEF_MESH_MAX_X;
  float mesh_min_y = DEF_MESH_MIN_Y;
  float mesh_max_y = DEF_MESH_MAX_Y;
  uint8_t multiple_probing = MULTIPLE_PROBING;
} lcd_rts_data_t;
extern lcd_rts_data_t lcd_rts_data;
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

#if HAS_SOUND

#include "../gcode.h"

#include "../../lcd/marlinui.h" // i2c-based BUZZ
#include "../../libs/buzzer.h"  // Buzzer, if possible
#if ENABLED(E3S1PRO_RTS)
  #include "../../lcd/rts/e3s1pro/lcd_rts.h"
#endif

/**
 * M300: Play a Tone / Add a tone to the queue
 *
 *  S<frequency> - (Hz) The frequency of the tone. 0 for silence.
 *  P<duration>  - (ms) The duration of the tone.
 *
 * With SOUND_MENU_ITEM:
 *  E<0|1>       - Mute or enable sound
 */
void GcodeSuite::M300() {

  #if ENABLED(SOUND_MENU_ITEM)
    if (parser.seen('E')) {
      ui.sound_on = parser.value_bool();
      return;
    }
  #endif

  #if ENABLED(E3S1PRO_RTS)
    if (rts_is_dacai) {
        // Dacai: try command-register style first (0xA0..0xA3), then keep
        // existing variable-write style fallbacks for broader compatibility.
        const uint8_t vol = (uint8_t)lcd_rts_settings.display_volume;

        // Register-byte writes: [sound_id, section_id, volume, play_mode].
        rtscheck.RTS_SndData(0x06, 0x00A0, RegAddr_W);
        rtscheck.RTS_SndData(0x01, 0x00A1, RegAddr_W);
        rtscheck.RTS_SndData(vol, 0x00A2, RegAddr_W);
        rtscheck.RTS_SndData(0x02, 0x00A3, RegAddr_W);

        // Variable write of the same 4-byte payload at 0x00A0.
        const uint8_t dacai_music_play[] = { 0x06, 0x01, vol, 0x02 };
        rtscheck.writeVariable(SoundAddr, dacai_music_play, sizeof(dacai_music_play));

        // Sovol RTS uses this 4-byte beep command family on the same SoundAddr.
        rtscheck.RTS_SndData(0x02AF0100UL, SoundAddr);
        rtscheck.RTS_SndData(0xFFFF0101UL, SoundAddr);
    }
    else {
      // DWIN T5L: original StartSoundSet command works correctly.
      rtscheck.RTS_SndData(StartSoundSet, SoundAddr);
    }
  #else
    const uint16_t frequency = parser.ushortval('S', 260);
    uint16_t duration = parser.ushortval('P', 1000);
    // Limits the tone duration to 0-5 seconds.
    NOMORE(duration, 5000U);
    BUZZ(duration, frequency);
  #endif

}

#endif // HAS_SOUND

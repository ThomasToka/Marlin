/**
 * This class was initially released by Creality for the Ender 3 S1 Pro
 * This class was modified by Thomas Toka for MARLIN-E3S1PRO-FORK-BYTT
 * This class is backward compatible to the stock screen firmware with its stock features.
 */

//#define GCODE_PREVIEW_ENABLED

//#define LCD_RTS_DEBUG_LCD
//#define LCD_RTS_DEBUG_EEPROM_SETTINGS
//#define LCD_RTS_DEBUG_SETTINGS
//#define LCD_RTS_DEBUG_MARGIN_X
//#define LCD_RTS_DEBUG_MARGIN_Y
//#define LCD_RTS_DEBUG_SDCARD
//#define LCD_RTS_DEBUG_PRINTTIME
//#define LCD_RTS_DEBUG_LEVELING

#include <Wstring.h>
#include <stdio.h>
#include <string.h>
//#include <Arduino.h>

#include "../../../inc/MarlinConfig.h"

#include "lcd_rts.h"
#include "../../marlinui.h"
#include "../../../MarlinCore.h"
#include "../../../module/settings.h"
#include "../../../core/serial.h"
#include "../../../core/macros.h"
#include "../../utf8.h"
#include "../../../sd/cardreader.h"
#include "../../../feature/babystep.h"
#include "../../../module/temperature.h"
#include "../../../module/printcounter.h"
#include "../../../module/motion.h"
#include "../../../module/planner.h"
#include "../../../gcode/queue.h"
#include "../../../gcode/gcode.h"
#include "../../../module/probe.h"

#if ENABLED(GCODE_PREVIEW_ENABLED)
  #include "preview.h"
#endif

#if ENABLED(AUTO_BED_LEVELING_BILINEAR)
  #include "../../../feature/bedlevel/abl/bbl.h"
#endif
#include "../../../feature/bedlevel/bedlevel.h"

#if ENABLED(EEPROM_SETTINGS)
  #include "../../../HAL/shared/eeprom_api.h"
  #include "../../../module/settings.h"
#endif

#if ENABLED(HOST_ACTION_COMMANDS)
  #include "../../../feature/host_actions.h"
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  #include "../../../feature/pause.h"
  #include "../../../gcode/queue.h"
#endif

#include "../../../libs/duration_t.h"

#if ENABLED(BLTOUCH)
  #include "../../../module/endstops.h"
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #include "../../../feature/runout.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../../feature/powerloss.h"
#endif

#if HAS_CUTTER
#include "../../../feature/spindle_laser.h"
#endif

#ifdef LCD_SERIAL_PORT
  #define LCDSERIAL LCD_SERIAL
#elif SERIAL_PORT_2
  #define LCDSERIAL MYSERIAL2
#endif

#if ENABLED(E3S1PRO_RTS)
RTSSHOW rtscheck;
bool hasSelected = false;
short previousSelectionIndex;
extern CardReader card;
char errorway = 0;
char errornum = 0;
char home_errornum  = 0; 

#if ENABLED(BABYSTEPPING)
  float zprobe_zoffset;
  float xprobe_xoffset;
  float yprobe_yoffset;
  float last_zoffset = 0.0;
  float last_xoffset = 0.0;
  float last_yoffset = 0.0;    
  float rec_zoffset;
#endif

uint8_t min_margin_y_back;
uint8_t min_margin_x;

bool power_off_type_yes = false;
uint8_t bltouch_tramming = 0;
uint8_t leveling_running = 0;
uint8_t color_sp_offset = 0;
uint8_t current_point = 255;
int touchscreen_requested_mesh = 0;

const float manual_feedrate_mm_m[] = {50 * 60, 50 * 60, 4 * 60, 180};
constexpr float default_max_feedrate[]        = DEFAULT_MAX_FEEDRATE;
constexpr float default_max_acceleration[]    = DEFAULT_MAX_ACCELERATION;
constexpr float default_max_jerk[]            = { DEFAULT_XJERK, DEFAULT_YJERK, DEFAULT_ZJERK, DEFAULT_EJERK };
constexpr float default_axis_steps_per_unit[] = DEFAULT_AXIS_STEPS_PER_UNIT;

float default_nozzle_ptemp = DEFAULT_Kp;
float default_nozzle_itemp = DEFAULT_Ki;
float default_nozzle_dtemp = DEFAULT_Kd;

float default_hotbed_ptemp = DEFAULT_bedKp;
float default_hotbed_itemp = DEFAULT_bedKi;
float default_hotbed_dtemp = DEFAULT_bedKd;

uint8_t startprogress = 0;

CRec CardRecbuf; 
int16_t temphot = 0;
int8_t tempbed = 0;
float temp_bed_display = 0;
uint8_t afterprobe_fan0_speed = 0;

bool sdcard_pause_check = true;
bool pause_action_flag = false;
bool print_preheat_check = false;
bool probe_offset_flag = false;
float probe_offset_x_temp;
float probe_offset_y_temp;
uint16_t max_reachable_pos_y;
uint16_t min_calc_margin_y_bedlevel;
uint16_t max_reachable_pos_x;
uint16_t min_calc_margin_x_bedlevel;

unsigned int picLayers = 0;   // picture end line
unsigned int picFilament_m = 0;
unsigned int picFilament_g = 0;
float picLayerHeight = 0.0f;

millis_t next_rts_update_ms      = 0;
int PrintFlag = 0;

float ChangeFilamentTemp = 200; 
int heatway = 0;

int last_target_temperature[4] = {0};
int last_target_temperature_bed;

char waitway = 0;

int change_page_font = 1;
unsigned char Percentrecord = 0;
bool CardUpdate = false;  

int16_t fileCnt = 0;
uint8_t file_current_page = 1;
uint8_t file_total_page = 1;
uint8_t page_total_file = 0;

DB RTSSHOW::recdat;
DB RTSSHOW::snddat;

uint8_t lang = 2; 
bool lcd_sd_status;

float rec_dat_temp_last_x = 0.0;
float rec_dat_temp_last_y = 0.0;
float rec_dat_temp_real_x = 0.0;
float rec_dat_temp_real_y = 0.0;

uint16_t rectWidth = 0;
uint16_t rectHeight = 0;
uint16_t rect_0_y_top = 0;
uint16_t rect_1_x_top_odd = 0;
uint16_t rect_0_x_top_even = 0;
uint16_t rect_x_offset = 0;
uint16_t rect_y_offset = 0;

const float THRESHOLD_VALUE_X = 101.0;
const size_t FIRST_ELEMENT_INDEX_X = 0;

const float THRESHOLD_VALUE_Y = 101.0;
const size_t FIRST_ELEMENT_INDEX_Y = 0;

char cmdbuf[20] = {0};

float FilamentLOAD = 10;

float FilamentUnLOAD = 10;

unsigned char AxisUnitMode;
unsigned char AutoHomeIconNum;
float axis_unit = 10.0;
int Update_Time_Value = 0;
bool PoweroffContinue = false;
char commandbuf[30];
static bool last_card_insert_st;
bool card_insert_st;
bool sd_printing;

bool home_flag = false;
bool rts_start_print = false;  

const int manual_level_5position[9][2] = MANUALL_BED_LEVEING_5POSITION;
const int manual_crtouch_5position[9][2] = MANUALL_BED_CRTOUCH_5POSITION;

uint8_t settingsload = 0;
const char* settings_filename2 = "SETTINGS.GCO";

int custom_ceil(float x) {
    float decimal_part_x = x - static_cast<int>(x);
    
    if (decimal_part_x > 0) {
        return static_cast<int>(x) + 1;
    } else {
        return static_cast<int>(x);
    }
}

enum{
  PREHEAT_PLA = 0,
  PREHEAT_ABS = 1,
  PREHEAT_PETG = 2,
  PREHEAT_CUST = 3,
};

int temp_preheat_nozzle = 0, temp_preheat_bed = 0, temp_probe_margin_x = 0, temp_probe_margin_y = 0;
uint8_t temp_grid_max_points = 0;
uint8_t temp_grid_probe_count = 0;
uint8_t preheat_flag = PREHEAT_PLA; // 0=PLA，1=ABS, 2=PETG, 3=CUST

uint8_t  last_progress_percent = 0;
uint32_t last_start_time       = 0;
uint32_t last_remaining_time   = 0;

bool g_heaterLoadTempAdd = false;
bool g_uiXYAxisEnable = false;
bool g_uiZAxisEnable = false;
bool g_uiZOffsetHomeOkFlag = false;
bool g_uiAutoPIDFlag =false;
int16_t g_autoPIDHeaterTempTarget = 300;

#if ENABLED(ENDER_3S1_PRO)
int16_t g_autoPIDHotBedTempTarget = 110;
#elif ENABLED(ENDER_3S1_PLUS)
int16_t g_autoPIDHotBedTempTarget = 100;
#endif

int8_t g_autoPIDHeaterCycles = 8;
int8_t g_autoPIDHotBedCycles = 8;

int16_t g_autoPIDHeaterTempTargetset = 0;
int16_t g_autoPIDHotBedTempTargetset = 0;
int8_t g_autoPIDHeaterCyclesTargetset = 0;
int8_t g_autoPIDHotBedCyclesTargetset = 0;

bool g_uiAutoPIDHotbedRunningFlag = false;
bool g_uiAutoPIDNozzleRunningFlag = false;
int8_t g_uiAutoPIDRunningDiff = 0;
int16_t g_uiCurveDataCnt = 0;

int16_t advance_k_set = 0;
uint8_t lcd_rts_settings_version = 1;
lcd_rts_settings_t lcd_rts_settings;

/*************************************END***************************************/

inline void RTS_line_to_current(AxisEnum axis)
{
  if (!planner.is_full())
  {
    planner.buffer_line(current_position, MMM_TO_MMS(manual_feedrate_mm_m[(int8_t)axis]), active_extruder);
  }
}

void resetSettings() {
  lcd_rts_settings.settings_size         = sizeof(lcd_rts_settings_t);
  lcd_rts_settings.settings_version      = lcd_rts_settings_version;
  lcd_rts_settings.display_sound         = true;
  lcd_rts_settings.display_volume        = 256;
  lcd_rts_settings.display_standby       = true;
  lcd_rts_settings.standby_brightness    = 20;
  lcd_rts_settings.screen_brightness     = 100;
  lcd_rts_settings.standby_time_seconds  = 60;
  lcd_rts_settings.max_points = 5;
  lcd_rts_settings.probe_margin_x = 45;
  lcd_rts_settings.probe_margin_y_front = 45;
  lcd_rts_settings.probe_margin_y_back = 45;
  lcd_rts_settings.external_m73 = false;
  lcd_rts_settings.total_probing = 3;
  lcd_rts_settings.plr_zraise = 5;
  lcd_rts_settings.boot_zraise = true;
  //lcd_rts_settings.hotend_fan = 255;  
  #if ENABLED(LCD_RTS_DEBUG_EEPROM_SETTINGS)
    SERIAL_ECHOLNPGM("------Reset lcd_rts_settings from lcd_rts.cpp!-------");  
  #endif
}

void loadSettings(const char * const buff) {
  memcpy(&lcd_rts_settings, buff, _MIN(sizeof(lcd_rts_settings), eeprom_data_size));
  #if ENABLED(LCD_RTS_DEBUG_EEPROM_SETTINGS)  
    SERIAL_ECHOLNPGM("Saved settings: ");
    SERIAL_ECHOLNPGM("settings_size: ", lcd_rts_settings.settings_size);
    SERIAL_ECHOLNPGM("settings_version: ", lcd_rts_settings.settings_version);
    SERIAL_ECHOLNPGM("display_sound: ", lcd_rts_settings.display_sound);
    SERIAL_ECHOLNPGM("display_volume: ", lcd_rts_settings.display_volume);
    SERIAL_ECHOLNPGM("screen_brightness: ", lcd_rts_settings.screen_brightness);
    SERIAL_ECHOLNPGM("display_standby: ", lcd_rts_settings.display_standby);
    SERIAL_ECHOLNPGM("standby_brightness: ", lcd_rts_settings.standby_brightness);
    SERIAL_ECHOLNPGM("standby_time_seconds: ", lcd_rts_settings.standby_time_seconds); 
    SERIAL_ECHOLNPGM("------------------");
    SERIAL_ECHOLNPGM("max_points: ", lcd_rts_settings.max_points);
    SERIAL_ECHOLNPGM("probe_margin x: ", lcd_rts_settings.probe_margin_x);
    SERIAL_ECHOLNPGM("probe_margin y: ", lcd_rts_settings.probe_margin_y_front);
    SERIAL_ECHOLNPGM("probe_min_margin y: ", lcd_rts_settings.probe_margin_y_back);
    SERIAL_ECHOLNPGM("external m73: ", lcd_rts_settings.external_m73);
    SERIAL_ECHOLNPGM("total_probing: ", lcd_rts_settings.total_probing);
    SERIAL_ECHOLNPGM("plr_zraise: ", lcd_rts_settings.plr_zraise);
    SERIAL_ECHOLNPGM("boot_zraise: ", lcd_rts_settings.boot_zraise);
    //SERIAL_ECHOLNPGM("hotend_fan: ", lcd_rts_settings.hotend_fan);    
    SERIAL_ECHOLNPGM("------Load lcd_rts_settings from lcd_rts.cpp!-------");    
  #endif
}

void saveSettings(char * const buff) {
  memcpy(buff, &lcd_rts_settings, _MIN(sizeof(lcd_rts_settings), eeprom_data_size));
  #if ENABLED(LCD_RTS_DEBUG_EEPROM_SETTINGS)  
    SERIAL_ECHOLNPGM("Saved settings: ");
    SERIAL_ECHOLNPGM("settings_size: ", lcd_rts_settings.settings_size);
    SERIAL_ECHOLNPGM("settings_version: ", lcd_rts_settings.settings_version);
    SERIAL_ECHOLNPGM("display_sound: ", lcd_rts_settings.display_sound);
    SERIAL_ECHOLNPGM("display_volume: ", lcd_rts_settings.display_volume);
    SERIAL_ECHOLNPGM("screen_brightness: ", lcd_rts_settings.screen_brightness);
    SERIAL_ECHOLNPGM("display_standby: ", lcd_rts_settings.display_standby);
    SERIAL_ECHOLNPGM("standby_brightness: ", lcd_rts_settings.standby_brightness);
    SERIAL_ECHOLNPGM("standby_time_seconds: ", lcd_rts_settings.standby_time_seconds); 
    SERIAL_ECHOLNPGM("------------------");
    SERIAL_ECHOLNPGM("max_points: ", lcd_rts_settings.max_points);    
    SERIAL_ECHOLNPGM("probe_margin x: ", lcd_rts_settings.probe_margin_x);
    SERIAL_ECHOLNPGM("probe_margin y: ", lcd_rts_settings.probe_margin_y_front);
    SERIAL_ECHOLNPGM("probe_min_margin y: ", lcd_rts_settings.probe_margin_y_back);
    SERIAL_ECHOLNPGM("external m73: ", lcd_rts_settings.external_m73);
    SERIAL_ECHOLNPGM("total_probing: ", lcd_rts_settings.total_probing);
    SERIAL_ECHOLNPGM("plr_zraise: ", lcd_rts_settings.plr_zraise);
    SERIAL_ECHOLNPGM("boot_zraise: ", lcd_rts_settings.boot_zraise);
    //SERIAL_ECHOLNPGM("hotend_fan: ", lcd_rts_settings.hotend_fan);      
    SERIAL_ECHOLNPGM("------Save lcd_rts_settings from lcd_rts.cpp!-------");
  #endif
}

RTSSHOW::RTSSHOW(void)
{
  recdat.head[0] = snddat.head[0] = FHONE;
  recdat.head[1] = snddat.head[1] = FHTWO;
  memset(databuf, 0, sizeof(databuf));
}

static void RTS_line_to_filelist() {

  char statStr1[4];
  snprintf(statStr1, sizeof(statStr1), "%d", file_current_page);
  char statStr2[4];
  snprintf(statStr2, sizeof(statStr2), "%d", file_total_page);
  for (int h = 0; h < 2; h++) {
  RTS_ResetSingleVP(PAGE_STATUS_TEXT_CURRENT_VP);
  RTS_ResetSingleVP(PAGE_STATUS_TEXT_TOTAL_VP);    
  }

  rtscheck.RTS_SndData(statStr1, PAGE_STATUS_TEXT_CURRENT_VP);
  rtscheck.RTS_SndData(statStr2, PAGE_STATUS_TEXT_TOTAL_VP);

  for (int i = 0; i < 5; i += 5) {
    RTS_ResetSingleVP(FILE1_TEXT_VP + i * 60);
  }

  // clean filename Icon
  for (int j = 0; j < 5; j++)
    for (int i = 0; i < TEXTBYTELEN; i++)
      RTS_ResetSingleVP(CardRecbuf.addr[j] + i);

  memset(&CardRecbuf, 0, sizeof(CardRecbuf));

  int num = 0;
  for (int16_t i = (file_current_page - 1) * 5; i < (file_current_page * 5); i++) {  
      card.selectFileByIndexSorted(i);
      #if ENABLED(LCD_RTS_DEBUG_SDCARD)    
          SERIAL_ECHO_MSG("card.longFilename ", card.longFilename);
      #endif
      char *pointFilename = card.longFilename;
      int filenamelen = strlen(card.longFilename);
      //CardRecbuf.filenamelen[num] = strlen(card.longFilename);      

      if (!CardRecbuf.Cardshowlongfilename[num]) {
          CardRecbuf.Cardshowlongfilename[num] = new char[filenamelen + 1];
          strcpy(CardRecbuf.Cardshowlongfilename[num], card.longFilename);
      }

      int j = 1;
      while ((strncmp(&pointFilename[j], ".gcode", 6) != 0 && strncmp(&pointFilename[j], ".GCODE", 6) != 0 && strncmp(&pointFilename[j], ".GCO", 4) != 0 && strncmp(&pointFilename[j], ".gco", 4) != 0) && (j++ < filenamelen));
      int filenameLenWithoutExt = j;
      CardRecbuf.filenamelen[num] = filenameLenWithoutExt;
      // Check if the file extension is corrupted
      const char* expectedExtensions[] = {".gcode", ".GCODE", ".gco", ".GCO"};
      bool extensionCorrupted = true;

      for (size_t k = 0; k < sizeof(expectedExtensions) / sizeof(expectedExtensions[0]); ++k) {
          if (EndsWith(card.longFilename, expectedExtensions[k])) {
              extensionCorrupted = false;
              break;
          }
      }

      if (j >= TEXTBYTELEN) {
        strncpy(&card.longFilename[TEXTBYTELEN - 2], "..", 2); // Reserve 2 characters for ".."
        card.longFilename[TEXTBYTELEN] = '\0';
        j = TEXTBYTELEN;
      } else {
        j = min(j, TEXTBYTELEN); // Use the smaller of j and TEXTBYTELEN
      }
      if (extensionCorrupted) {
        rtscheck.RTS_SndData((unsigned long)0xFFFF, FilenameNature + (num + 1) * 16);
        rtscheck.RTS_SndData(204, FILE6_SELECT_ICON_VP + num);
      }
      // Debugging
      #if ENABLED(LCD_RTS_DEBUG_SDCARD)
        SERIAL_ECHO_MSG("Filename after truncation: ", card.longFilename);
        SERIAL_ECHO_MSG("j value: ", j);
      #endif

      strncpy(CardRecbuf.Cardshowfilename[num], card.longFilename, min(j, TEXTBYTELEN));
      CardRecbuf.Cardshowfilename[num][TEXTBYTELEN - 1] = '\0';

      #if ENABLED(LCD_RTS_DEBUG_SDCARD)
          SERIAL_ECHO("inside rts_line_to_filelist");
          SERIAL_ECHOLN("");
      #endif

      strcpy(CardRecbuf.Cardfilename[num], card.filename);
      CardRecbuf.addr[num] = FILE1_TEXT_VP + (num * 60);
      rtscheck.RTS_SndData(CardRecbuf.Cardshowfilename[num], CardRecbuf.addr[num]);

      if (!EndsWith(CardRecbuf.Cardshowlongfilename[num], "gcode") && !EndsWith(CardRecbuf.Cardshowlongfilename[num], "GCO") 
        && !EndsWith(CardRecbuf.Cardshowlongfilename[num], "GCODE") && !EndsWith(CardRecbuf.Cardshowlongfilename[num], "gco")) 
      {
          rtscheck.RTS_SndData((unsigned long)0x073F, FilenameNature + (num + 1) * 16);
          rtscheck.RTS_SndData(203, FILE6_SELECT_ICON_VP + num);
      }
      if (EndsWith(CardRecbuf.Cardshowlongfilename[num], "gcode") || EndsWith(CardRecbuf.Cardshowlongfilename[num], "GCO") 
        || EndsWith(CardRecbuf.Cardshowlongfilename[num], "GCODE") || EndsWith(CardRecbuf.Cardshowlongfilename[num], "gco")) 
      {
          rtscheck.RTS_SndData((unsigned long)0xFFFF, FilenameNature + (num + 1) * 16);
          rtscheck.RTS_SndData(204, FILE6_SELECT_ICON_VP + num);
      }

      if (filenamelen == 0) 
      {
        RTS_ResetSingleVP(FILE6_SELECT_ICON_VP + num);
      }
      CardRecbuf.Filesum = (++num);
  }
  page_total_file = CardRecbuf.Filesum;
  CardRecbuf.Filesum = ((file_total_page - 1) * 5) + page_total_file;
}

void RTSSHOW::RTS_SDCardInit(void) {
  if (RTS_SD_Detected())
    card.mount();

  //DEBUG_ECHOLNPGM(" card.flag.mounted=: ", card.flag.mounted);

  if (card.flag.mounted) {
    int16_t fileCnt = card.get_num_items();
    card.getWorkDirName();
    if (card.filename[0] != '/') card.cdup();

    if (fileCnt > 0) {
      file_total_page = fileCnt / 5;
      if (fileCnt % 5 > 0) { // Add an extra page only if there are leftover files
        file_total_page++;
      }
      if (file_total_page > 8) file_total_page = 8; // Limit the maximum number of pages
    }
    else {
      file_total_page = 1;
    }

    RTS_SndData(file_total_page, PAGE_STATUS_TEXT_TOTAL_VP);
    file_current_page = 1;
    RTS_SndData(file_current_page, PAGE_STATUS_TEXT_CURRENT_VP);
    RTS_line_to_filelist();
    CardRecbuf.selectFlag = false;

    if (PoweroffContinue /*|| print_job_timer.isRunning()*/) return;

    // clean print file
    RTS_CleanPrintAndSelectFile();
    lcd_sd_status = IS_SD_INSERTED();
  }
  else {
    if (PoweroffContinue) return;

    // Clean filename Icon
    for (int j = 0; j < MaxFileNumber; j++)
      for (int i = 0; i < TEXTBYTELEN; i++)
        RTS_ResetSingleVP(CardRecbuf.addr[j] + i);

    memset(&CardRecbuf, 0, sizeof(CardRecbuf));
  }
}

bool RTSSHOW::RTS_SD_Detected() {
  static bool last, state;
  static bool flag_stable;
  static uint32_t stable_point_time;

  bool tmp = IS_SD_INSERTED();

  if (tmp != last)
    flag_stable = false;
  else if (!flag_stable) {
    flag_stable = true;
    stable_point_time = millis() + 30;
  }

  if (flag_stable && ELAPSED(millis(), stable_point_time))
    state = tmp;

  last = tmp;

  return state;
}

void RTSSHOW::RTS_SDCardUpdate() {
  const bool sd_status = RTS_SD_Detected();

  if (sd_status != lcd_sd_status) {
    if (sd_status) {
      // SD card power on
      RTS_SDCardInit();
    }
    else {
      if (PoweroffContinue /*|| print_job_timer.isRunning()*/) return;

      card.release();
      for (int i = 0; i < CardRecbuf.Filesum; i++) {
        for (int j = 0; j < 20; j++) RTS_ResetSingleVP(CardRecbuf.addr[i] + j);
      //  RTS_SndData((unsigned long)0xFFFF, FilenameNature + (i + 1) * 16);
      }
      RTS_CleanPrintAndSelectFile();
      for (int j = 0; j < 5; j++) {
        // clean screen.
        RTS_ResetSingleVP(FILE6_SELECT_ICON_VP + j);
      }      
      memset(&CardRecbuf, 0, sizeof(CardRecbuf));
      RTS_ShowPreviewImage(true);
      RTS_SetOneToVP(PAGE_STATUS_TEXT_TOTAL_VP);
      file_total_page = 1;
      RTS_SetOneToVP(PAGE_STATUS_TEXT_CURRENT_VP);
      file_current_page = 1;
    }
    lcd_sd_status = sd_status;
  }

  // represents to update file list
  if (CardUpdate && lcd_sd_status && RTS_SD_Detected()) {
    RTS_line_to_filelist();
    for (uint16_t i = 0; i < 5; i++) {
      delay(1);
      RTS_SndData((unsigned long)0xFFFF, FilenameNature + (i + 1) * 16);
    }
    //hal.watchdog_refresh();
    CardUpdate = false;
  }
}

void RTSSHOW::RTS_SDcard_Stop(void)
{
  planner.synchronize();
  card.flag.abort_sd_printing = true;
  queue.clear();
  quickstop_stepper();
  print_job_timer.stop();
  IF_DISABLED(SD_ABORT_NO_COOLDOWN, thermalManager.disable_all_heaters());
  print_job_timer.reset();
  TERN_(HOST_PAUSE_M76, hostui.cancel());
  RTS_ResetHotendBed();
  RTS_ResetHeadAndBedSetTemp();
  temphot = 0;
  thermalManager.zero_fan_speeds();
  wait_for_heatup = wait_for_user = false;
  PoweroffContinue = false;
  TERN_(POWER_LOSS_RECOVERY, if (card.flag.mounted) card.removeJobRecoveryFile());
  delay(2);
  RTS_CleanPrintAndSelectFile();
  RTS_ResetPrintData(true);
  delay(2);
  RTS_SendM600Icon(false);
  CardRecbuf.recordcount = -1;
}

void RTSSHOW::writeVariable(const uint16_t adr, const void * const values, uint8_t valueslen, const bool isstr/*=false*/, const char fillChar/*=' '*/) {
  const char* myvalues = static_cast<const char*>(values);
  bool strend = !myvalues;
  LCDSERIAL.write(FHONE);
  LCDSERIAL.write(FHTWO);
  LCDSERIAL.write(valueslen + 3);
  LCDSERIAL.write(0x82);
  LCDSERIAL.write(adr >> 8);
  LCDSERIAL.write(adr & 0xFF);
  while (valueslen--) {
    char x;
    if (!strend) x = *myvalues++;
    if ((isstr && !x) || strend) {
      strend = true;
      x = fillChar;
    }
    LCDSERIAL.write(x);
  }
}

void RTSSHOW::setTouchScreenConfiguration() {
  // Main configuration (System_Config)
  LIMIT(lcd_rts_settings.screen_brightness, 20, 100); // Prevent a possible all-dark screen
  LIMIT(lcd_rts_settings.standby_time_seconds, 10, 655); // Prevent a possible all-dark screen for standby, yet also don't go higher than the DWIN limitation

  uint8_t cfg_bits = (0x0
    |                                   _BV(7)       // 7: Enable Control ... TERN0(DWINOS_4, _BV(7))
    |                                   _BV(5)       // 5: load 22 touch file
    |                                   _BV(4)       // 4: auto-upload should always be enabled
    | (lcd_rts_settings.display_sound         ? _BV(3) : 0)  // 3: audio
    | (lcd_rts_settings.display_standby       ? _BV(2) : 0)  // 2: backlight on standby
    | _BV(1)       // Always set as if screen_rotation is 62
    #if LCD_SCREEN_ROTATE == 90
    | _BV(0)       // Portrait Mode or 800x480 display has 0 point rotated 90deg from 480x272 display
    #elif LCD_SCREEN_ROTATE
    #error "Only 90° rotation is supported for the selected LCD."
    #endif    
  );

  const uint8_t config_set[] = { 0x5A, 0x00, TERN(DWINOS_4, 0x00, 0xFF), cfg_bits };
  writeVariable(0x80 /*System_Config*/, config_set, sizeof(config_set));

  // Standby brightness (LED_Config)
  uint16_t dwinStandbyTimeSeconds = 100 * lcd_rts_settings.standby_time_seconds; /* milliseconds, but divided by 10 (not 5 like the docs say) */
  const uint8_t brightness_set[] = {
    lcd_rts_settings.screen_brightness /*% active*/,
    lcd_rts_settings.standby_brightness /*% standby*/,
    static_cast<uint8_t>(dwinStandbyTimeSeconds >> 8),
    static_cast<uint8_t>(dwinStandbyTimeSeconds)
  };
  writeVariable(0x82 /*LED_Config*/, brightness_set, sizeof(brightness_set));

  if (!lcd_rts_settings.display_sound) {
    RTS_SndData(193, VolumeIcon);
    RTS_SndData(102, SoundIcon);
    RTS_SndData(191, VOLUME_DISPLAY);     
  }
  else {
    RTS_SndData((lcd_rts_settings.display_volume + 1) / 32 - 1 + 193, VolumeIcon);
    RTS_SndData(101, SoundIcon);
    RTS_SndData(192, VOLUME_DISPLAY); 
  }

  if (lcd_rts_settings.screen_brightness <= 20) {
    RTS_SndData(193, BrightnessIcon);  
  }
  else {
    RTS_SndData((lcd_rts_settings.screen_brightness + 1) / 12 - 1 + 193, BrightnessIcon);     
  }

  RTS_SndData(lcd_rts_settings.display_volume << 8, SoundAddr + 1);
  RTS_SndData(lcd_rts_settings.screen_brightness, DISPLAY_BRIGHTNESS);
  RTS_SndData(lcd_rts_settings.standby_brightness, DISPLAYSTANDBY_BRIGHTNESS);
  RTS_SndData(lcd_rts_settings.standby_time_seconds, DISPLAYSTANDBY_SECONDS);
  RTS_SndData(lcd_rts_settings.display_standby ? 3 : 2, DISPLAYSTANDBY_ENABLEINDICATOR);
}

void RTSSHOW::RTS_Init(void)
{
  delay(200);
  AxisUnitMode = 1;
  lang = language_change_font;
  delay(50);
  last_zoffset = zprobe_zoffset = probe.offset.z;
  touchscreen_requested_mesh = 0;
  feedrate_percentage = 100;
  RTS_SendZoffsetFeedratePercentage(true);
  for(int i = 0;i < 9;i ++)
  {
    RTS_ResetSingleVP(LANGUAGE_CHINESE_TITLE_VP + i);
  }
  RTS_SetOneToVP(LANGUAGE_CHINESE_TITLE_VP + (language_change_font - 1));
  languagedisplayUpdate();
  delay(500);
  last_target_temperature[0] = thermalManager.temp_hotend[0].target;
  last_target_temperature_bed = thermalManager.temp_bed.target;
  RTS_ShowMotorFreeIcon(false);
  RTS_ResetHeadAndBedSetTemp();
  rtscheck.RTS_SendLoadedData(255);
  #if ENABLED(GCODE_PREVIEW_ENABLED)
    RTS_ResetSingleVP(DEFAULT_PRINT_MODEL_VP);
    RTS_ResetSingleVP(DOWNLOAD_PREVIEW_VP);
  #endif
  RTS_SetBltouchHSMode();
  RTS_LoadMesh();
  thermalManager.set_fan_speed(0, 0);
  delay(5);
  RTS_SDCardInit();
  RTS_ShowPreviewImage(true);
  delay(5);

  RTS_LoadMainsiteIcons();
  RTS_SendM600Icon(false);
  setTouchScreenConfiguration();
  RTS_ResetPrintData(true);
  RTS_SetOneToVP(PREHAEAT_NOZZLE_ICON_VP);
  RTS_SetOneToVP(PREHAEAT_HOTBED_ICON_VP);
  RTS_CleanPrintAndSelectFile();
  RTS_ShowPage(0);
  hal.watchdog_refresh();
  for(startprogress = 0; startprogress <= 100; startprogress++)
  {
    rtscheck.RTS_SndData(startprogress, START_PROCESS_ICON_VP);
    hal.watchdog_refresh();
    delay(50);
  }
  hal.watchdog_refresh();
  delay(50);
  Update_Time_Value = RTS_UPDATE_VALUE;
}

int RTSSHOW::RTS_RecData(void)
{
  static int recnum = 0;

  while((LCDSERIAL.available() > 0) && (recnum < SizeofDatabuf))
  {
    delay(1);
    databuf[recnum] = LCDSERIAL.read();
    if(databuf[0] == FHONE)
    {
      recnum++;
    }
    else if(databuf[0] == FHTWO)
    {
      databuf[0] = FHONE;
      databuf[1] = FHTWO;
      recnum += 2;
    }
    else if(databuf[0] == FHLENG)
    {
      databuf[0] = FHONE;
      databuf[1] = FHTWO;
      databuf[2] = FHLENG;
      recnum += 3;
    }
    else if(databuf[0] == VarAddr_R)
    {
      databuf[0] = FHONE;
      databuf[1] = FHTWO;
      databuf[2] = FHLENG;
      databuf[3] = VarAddr_R;
      recnum += 4;
    }
    else
    {
      recnum = 0;
    }
  }
  
  // receive nothing
  if(recnum < 1)
  {
    return -1;
  }
  else if((recdat.head[0] == databuf[0]) && (recdat.head[1] == databuf[1]) && (recnum > 2))
  {
    recdat.len = databuf[2];
    recdat.command = databuf[3];
    // response for writing byte
    if((recdat.len == 0x03) && ((recdat.command == 0x82) || (recdat.command == 0x80)) && (databuf[4] == 0x4F) && (databuf[5] == 0x4B))
    {
      memset(databuf, 0, sizeof(databuf));
      recnum = 0;
      return -1;
    }
    else if(recdat.command == 0x83)
    {
      // response for reading the data from the variate
      recdat.addr = databuf[4];
      recdat.addr = (recdat.addr << 8) | databuf[5];
      recdat.bytelen = databuf[6];
      for(unsigned int i = 0; i < recdat.bytelen; i += 2)
      {
        recdat.data[i / 2] = databuf[7 + i];
        recdat.data[i / 2] = (recdat.data[i / 2] << 8) | databuf[8 + i];
      }
    }
    else if(recdat.command == 0x81)
    {
      // response for reading the page from the register
      recdat.addr = databuf[4];
      recdat.bytelen = databuf[5];
      for(unsigned int i = 0; i < recdat.bytelen; i ++)
      {
        recdat.data[i] = databuf[6 + i];
        // recdat.data[i] = (recdat.data[i] << 8 )| databuf[7 + i];
      }
    }
  }
  else
  {
    memset(databuf, 0, sizeof(databuf));
    recnum = 0;
    // receive the wrong data
    return -1;
  }
  memset(databuf, 0, sizeof(databuf));
  recnum = 0;
  return 2;
}

void RTSSHOW::RTS_SndData(void)
{
  if((snddat.head[0] == FHONE) && (snddat.head[1] == FHTWO) && (snddat.len >= 3))
  {
    databuf[0] = snddat.head[0];
    databuf[1] = snddat.head[1];
    databuf[2] = snddat.len;
    databuf[3] = snddat.command;

    // to write data to the register
    if(snddat.command == 0x80)
    {
      databuf[4] = snddat.addr;
      for(int i = 0;i <(snddat.len - 2);i ++)
      {
        databuf[5 + i] = snddat.data[i];
      }
    }
    else if((snddat.len == 3) && (snddat.command == 0x81))
    {
      // to read data from the register
      databuf[4] = snddat.addr;
      databuf[5] = snddat.bytelen;
    }
    else if(snddat.command == 0x82)
    {
      // to write data to the variate
      databuf[4] = snddat.addr >> 8;
      databuf[5] = snddat.addr & 0xFF;
      for(int i =0;i <(snddat.len - 3);i += 2)
      {
        databuf[6 + i] = snddat.data[i/2] >> 8;
        databuf[7 + i] = snddat.data[i/2] & 0xFF;
      }
    }
    else if((snddat.len == 4) && (snddat.command == 0x83))
    {
      // to read data from the variate
      databuf[4] = snddat.addr >> 8;
      databuf[5] = snddat.addr & 0xFF;
      databuf[6] = snddat.bytelen;
    }
    // usart_tx(LCDSERIAL.c_dev(), databuf, snddat.len + 3);
    // LCDSERIAL.flush();
    for(int i = 0;i < (snddat.len + 3); i ++)
    {
      LCDSERIAL.write(databuf[i]);
      delayMicroseconds(1);
    }

    memset(&snddat, 0, sizeof(snddat));
    memset(databuf, 0, sizeof(databuf));
    snddat.head[0] = FHONE;
    snddat.head[1] = FHTWO;
  }
}

void RTSSHOW::RTS_SndData(const String &s, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  if(s.length() < 1)
  {
    return;
  }
  RTS_SndData(s.c_str(), addr, cmd);
}

void RTSSHOW::RTS_SndData(const char *str, unsigned long addr, unsigned char cmd/*= VarAddr_W*/)
{
  int len = strlen(str);
  if(len > 0)
  {
    databuf[0] = FHONE;
    databuf[1] = FHTWO;
    databuf[2] = 3 + len;
    databuf[3] = cmd;
    databuf[4] = addr >> 8;
    databuf[5] = addr & 0x00FF;
    for(int i = 0;i < len;i ++)
    {
      databuf[6 + i] = str[i];
    }
    for(int i = 0;i < (len + 6);i ++)
    {
      //SERIAL_ECHOPGM("databuff3++: ");
      //SERIAL_ECHO(databuf[i]);      
      LCDSERIAL.write(databuf[i]);
      delayMicroseconds(1);
    }
    memset(databuf, 0, sizeof(databuf));
  }
}

void RTSSHOW::RTS_SendCurveData(uint8_t channel, uint16_t *vaule, uint8_t size)
{
    databuf[0] = FHONE;
    databuf[1] = FHTWO;
    databuf[2] = 9 + size * 2;
    databuf[3] = VarAddr_W;
    databuf[4] = 0x03;
    databuf[5] = 0x10;
    databuf[6] = 0x5A;
    databuf[7] = 0xA5;
    databuf[8] = 0x01;
    databuf[9] = 0x00;
    databuf[10] = channel;
    databuf[11] = size;

    for (int i = 0,j = 0; j < size; j++) {
      databuf[i + 12] = vaule[j] >> 8;
      i++;
      databuf[i +12] = vaule[j] & 0x00FF;
      i++;
      if (i >= SizeofDatabuf)break;
    }
    for(int i = 0;i < (size * 2 + 12); i++)
    {

      //SERIAL_ECHOPGM("databuff2++: ");
      //SERIAL_ECHO(databuf[i]);
      LCDSERIAL.write(databuf[i]);
      delayMicroseconds(1);
    }
    memset(databuf, 0, sizeof(databuf));
}

void RTSSHOW::RTS_SndData(char c, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  snddat.command = cmd;
  snddat.addr = addr;
  snddat.data[0] = (unsigned long)c;
  snddat.data[0] = snddat.data[0] << 8;
  snddat.len = 5;
  RTS_SndData();
}

void RTSSHOW::RTS_SndData(unsigned char *str, unsigned long addr, unsigned char cmd) { RTS_SndData((char *)str, addr, cmd); }

void RTSSHOW::RTS_SndData(int n, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  if (cmd == VarAddr_W)
  {
    if (n > 0xFFFF)
    {
      snddat.data[0] = n >> 16;
      snddat.data[1] = n & 0xFFFF;
      snddat.len = 7;
    }
    else
    {
      snddat.data[0] = n;
      snddat.len = 5;
    }
  }
  else if (cmd == RegAddr_W)
  {
    snddat.data[0] = n;
    snddat.len = 3;
  }
  else if (cmd == VarAddr_R)
  {
    snddat.bytelen = n;
    snddat.len = 4;
  }
  snddat.command = cmd;
  snddat.addr = addr;
  RTS_SndData();
}

void RTSSHOW::RTS_SndData(unsigned int n, unsigned long addr, unsigned char cmd) { RTS_SndData((int)n, addr, cmd); }

void RTSSHOW::RTS_SndData(float n, unsigned long addr, unsigned char cmd) { RTS_SndData((int)n, addr, cmd); }

void RTSSHOW::RTS_SndData(long n, unsigned long addr, unsigned char cmd) { RTS_SndData((unsigned long)n, addr, cmd); }

void RTSSHOW::RTS_SndData(unsigned long n, unsigned long addr, unsigned char cmd /*= VarAddr_W*/)
{
  if (cmd == VarAddr_W)
  {
    if (n > 0xFFFF)
    {
      snddat.data[0] = n >> 16;
      snddat.data[1] = n & 0xFFFF;
      snddat.len = 7;
    }
    else
    {
      snddat.data[0] = n;
      snddat.len = 5;
    }
  }
  else if (cmd == VarAddr_R)
  {
    snddat.bytelen = n;
    snddat.len = 4;
  }
  snddat.command = cmd;
  snddat.addr = addr;
  RTS_SndData();
}

void RTSSHOW::RTS_SndText(const char string[], unsigned long addr, uint8_t textSize) {
  for (int8_t i = 0; i < textSize; i++) RTS_ResetSingleVP(addr + i);
  rtscheck.RTS_SndData(string, addr);
}

void RTSSHOW::calculateProbePoints(uint8_t current_point, uint8_t& x_probe_point, uint8_t& y_probe_point){
    y_probe_point = current_point / lcd_rts_settings.max_points;
    bool isEvenMesh = (lcd_rts_settings.max_points % 2 == 0);

    if (isEvenMesh) {
        x_probe_point = (y_probe_point % 2 == 0)
                        ? (lcd_rts_settings.max_points - 1) - (current_point % lcd_rts_settings.max_points)
                        : current_point % lcd_rts_settings.max_points;
    } else {
        x_probe_point = (y_probe_point % 2 == 0)
                        ? current_point % lcd_rts_settings.max_points
                        : lcd_rts_settings.max_points - 1 - (current_point % lcd_rts_settings.max_points);
    }
}

void RTSSHOW::sendRectangleCommand(uint16_t vpAddress, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color) {
    uint8_t commandBuffer[] = {
        0x5A, 0xA5, // Frame header
        0x13,       // Data length
        0x82,       // Write instruction
        static_cast<uint8_t>((vpAddress >> 8) & 0xFF), // High byte of VP address
        static_cast<uint8_t>(vpAddress & 0xFF),       // Low byte of VP address
        0x00, 0x03, // Draw rectangle
        0x00, 0x01, // Draw one rectangle
        static_cast<uint8_t>((x >> 8) & 0xFF),        // Upper left coordinate x (high byte)
        static_cast<uint8_t>(x & 0xFF),               // Upper left coordinate x (low byte)
        static_cast<uint8_t>((y >> 8) & 0xFF),        // Upper left coordinate y (high byte)
        static_cast<uint8_t>(y & 0xFF),               // Upper left coordinate y (low byte)
        static_cast<uint8_t>(((x + width) >> 8) & 0xFF), // Lower right coordinate x (high byte)
        static_cast<uint8_t>((x + width) & 0xFF),        // Lower right coordinate x (low byte)
        static_cast<uint8_t>(((y + height) >> 8) & 0xFF), // Lower right coordinate y (high byte)
        static_cast<uint8_t>((y + height) & 0xFF),        // Lower right coordinate y (low byte)
        static_cast<uint8_t>((color >> 8) & 0xFF),    // Color (high byte)
        static_cast<uint8_t>(color & 0xFF),           // Color (low byte)
        0xFF, 0x00  // The drawing operation has ended
    };
    // Send the buffer
    for (size_t i = 0; i < sizeof(commandBuffer); ++i) {
        LCDSERIAL.write(commandBuffer[i]);
        delayMicroseconds(1);
    }
}

void RTSSHOW::sendOneFilledRectangle(uint16_t baseAddress, uint16_t showcount, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color) {
    // Calculate the address based on the base address and showcount
    uint16_t vpAddress = baseAddress + (showcount * 16);
    // Calculate lower right coordinates
    uint16_t x_lr = x + width - 1;
    uint16_t y_lr = y + height - 1;
    uint8_t commandBuffer[] = {
        0x5A, 0xA5, // Frame header
        0x15,       // Data length (21 bytes for one rectangle)
        0x82,       // Write instruction
        static_cast<uint8_t>((vpAddress >> 8) & 0xFF), // High byte of VP address
        static_cast<uint8_t>(vpAddress & 0xFF),       // Low byte of VP address
        0x00, 0x04, // Fill rectangle command
        0x00, 0x01, // Number of rectangles (one)
        // Rectangle coordinates and color
        static_cast<uint8_t>((x >> 8) & 0xFF),
        static_cast<uint8_t>(x & 0xFF),
        static_cast<uint8_t>((y >> 8) & 0xFF),
        static_cast<uint8_t>(y & 0xFF),
        static_cast<uint8_t>((x_lr >> 8) & 0xFF),
        static_cast<uint8_t>(x_lr & 0xFF),
        static_cast<uint8_t>((y_lr >> 8) & 0xFF),
        static_cast<uint8_t>(y_lr & 0xFF),
        static_cast<uint8_t>((color >> 8) & 0xFF), // Color (high byte)
        static_cast<uint8_t>(color & 0xFF),       // Color (low byte)
        0xFF, 0x00  // The drawing operation ends
    };
    // Send the buffer
    for (size_t i = 0; i < sizeof(commandBuffer); ++i) {
        LCDSERIAL.write(commandBuffer[i]);
        delayMicroseconds(5);
    }
}

void RTSSHOW::RTS_HandleData(void)
{
  int Checkkey = -1;
  // for waiting
  if(waitway > 0)
  {
    memset(&recdat, 0, sizeof(recdat));
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
    return;
  }
  for(int i = 0;Addrbuf[i] != 0;i++)
  {
    if(recdat.addr == Addrbuf[i])
    {
      if(Addrbuf[i] >= ChangePageKey)
      {
        Checkkey = i;
      }
      break;
    }
  }

  if(Checkkey < 0)
  {
    memset(&recdat, 0, sizeof(recdat));
    recdat.head[0] = FHONE;
    recdat.head[1] = FHTWO;
    return;
  }
  #if ENABLED(LCD_RTS_DEBUG_LCD)
    SERIAL_ECHO_MSG("\nCheckkey=", Checkkey, "recdat.data[0]=", recdat.data[0]);
  #endif
  switch(Checkkey)
  {
    //SERIAL_ECHO_MSG("Recorded value Catchall\n", Checkkey);            
    case MainEnterKey:
      RTS_LoadMainsiteIcons();
      if (recdat.data[0] == 1) {
        CardUpdate = true;
        CardRecbuf.recordcount = -1;
        std::string currentdir;
        currentdir = card.getWorkDirName();
        if (card.getWorkDirName() != std::string("/")) {
        card.cdup();
        }

        if (card.flag.mounted)
        {
        int16_t fileCnt = card.get_num_items();

        if (fileCnt > 0) {
          file_total_page = fileCnt / 5;
          if (fileCnt % 5 > 0) { // Add an extra page only if there are leftover files
            file_total_page++;
          }
          if (file_total_page > 8) file_total_page = 8; // Limit the maximum number of pages
        }
        else {
          file_total_page = 1;
        }

        RTS_SndData(file_total_page, PAGE_STATUS_TEXT_TOTAL_VP);
        file_current_page = 1;
        RTS_SndData(file_current_page, PAGE_STATUS_TEXT_CURRENT_VP);

        if (IS_SD_INSERTED()) RTS_line_to_filelist();
        RTS_ShowPage(2);
        }
        CardUpdate = false;
      }
      else if (recdat.data[0] == 2) {
        AxisUnitMode = 1;
        axis_unit = 10.0;

        if(axes_should_home()) {
          waitway = 4;
          RTS_G28MoveOne();
        }else{
          RTS_ShowPage(16);
          RTS_SendCurrentPosition(4);
        }
      }
      else if (recdat.data[0] == 3) {
        RTS_ShowPage(21);
      }
      else if (recdat.data[0] == 4) {     
        RTS_ShowPage(25);
        planner.synchronize();
        queue.enqueue_now_P(PSTR("G28\nG1 F200 Z0.0"));
        RTS_ShowMotorFreeIcon(false);
      }
      else if (recdat.data[0] == 5) {  
        queue.clear();
        quickstop_stepper();
        print_job_timer.stop();
        RTS_ShowMotorFreeIcon(true);
        delay(2);
        RTS_ResetPrintData(true);

        print_job_timer.reset();
        RTS_CleanPrintAndSelectFile();
        CardRecbuf.recordcount = -1;
        RTS_ShowPage(1);
        RTS_ShowPreviewImage(true);
      }
      else if (recdat.data[0] == 6) { // Start bedleveling
        waitway = 3;
        RTS_SetOneToVP(AUTO_BED_LEVEL_TITLE_VP);
        RTS_SndData(AUTO_BED_LEVEL_PREHEAT, AUTO_BED_PREHEAT_HEAD_DATA_VP);
        RTS_ResetSingleVP(AUTO_LEVELING_PERCENT_DATA_VP);
        if(thermalManager.temp_hotend[0].celsius < (AUTO_BED_LEVEL_PREHEAT - 5))
        {
          queue.enqueue_now_P(PSTR("G4 S40"));
        }

        if(axes_should_home())  queue.enqueue_one_P(PSTR("G28"));
        RTS_ChangeLevelingPage();
        rtscheck.RTS_SndData(lang + 10, AUTO_LEVELING_START_TITLE_VP);        
        #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
          queue.enqueue_one_P(PSTR("G29"));
        #else
          touchscreen_requested_mesh = 1;
          queue.enqueue_one_P(PSTR("G29 P1 T"));
          // queue.enqueue_one_P(PSTR("G29 P3"));
          queue.enqueue_one_P(PSTR("G29 S0"));
          queue.enqueue_one_P(PSTR("M500"));          
        #endif
        RTS_ShowMotorFreeIcon(false);        
      }
      else if (recdat.data[0] == 7) {
        if (errorway == 1) {
        }
        else if (errorway == 2) {
          // auto home fail
        }
        else if (errorway == 3) {
          // bed leveling fail
        }
        else if (errorway == 4) {
        }
      }
      else if (recdat.data[0] == 8) {
        RTS_ShowPage(1);
        #if ENABLED(GCODE_PREVIEW_ENABLED)
          if (false == CardRecbuf.selectFlag) {
            RTS_ShowPreviewImage(true);
          }        
        #endif      
      }
      else if(recdat.data[0] == 9)
      {
        RTS_ShowPage(11);
      }else if(recdat.data[0] == 0x0A)
      {
        RTS_ShowPage(13);
      }
      else if(recdat.data[0] == 161)
      {

        g_autoPIDHeaterTempTargetset = g_autoPIDHeaterTempTargetset;     
        if (g_autoPIDHeaterTempTargetset != 0) { 
         g_autoPIDHeaterTempTarget =  g_autoPIDHeaterTempTargetset;  
        }
        g_autoPIDHeaterCyclesTargetset = g_autoPIDHeaterCyclesTargetset;
        if (g_autoPIDHeaterCyclesTargetset != 0) {        
         g_autoPIDHeaterCycles =  g_autoPIDHeaterCyclesTargetset;  
        }    
        RTS_SndData(g_autoPIDHeaterTempTarget, AUTO_PID_SET_NOZZLE_TEMP);
        RTS_SndData(g_autoPIDHeaterCycles, AUTO_PID_SET_NOZZLE_CYCLES);        
        RTS_ResetSingleVP(PID_TEXT_OUT_CUR_CYCLE_HOTBED_VP);        
        RTS_ResetSingleVP(AUTO_PID_NOZZLE_CYCLES);        
        RTS_ResetSingleVP(AUTO_PID_HOTBED_CYCLES);                
        RTS_ResetSingleVP(WRITE_CURVE_DDR_CMD);
        RTS_SndData("                           ", PID_TEXT_OUT_VP);        
        RTS_ShowPage(83);
        if(g_uiAutoPIDNozzleRunningFlag == true){
        }else{          
        RTS_ResetSingleVP(AUTO_PID_RUN_NOZZLE_TIS_VP);
        RTS_SendLang(AUTO_PID_NOZZLE_TIS_VP);        
        }
      }
      else if(recdat.data[0] == 162)
      {
        g_autoPIDHotBedTempTargetset = g_autoPIDHotBedTempTargetset;
        if (g_autoPIDHotBedTempTargetset != 0) { 
         g_autoPIDHotBedTempTarget =  g_autoPIDHotBedTempTargetset;  
        }
        g_autoPIDHotBedCyclesTargetset = g_autoPIDHotBedCyclesTargetset;
        if (g_autoPIDHotBedCyclesTargetset != 0) { 
         g_autoPIDHotBedCycles =  g_autoPIDHotBedCyclesTargetset;  
        }
        RTS_SndData(g_autoPIDHotBedTempTarget, AUTO_PID_SET_HOTBED_TEMP);
        RTS_SndData(g_autoPIDHotBedCycles, AUTO_PID_SET_HOTBED_CYCLES);        
        RTS_ResetSingleVP(PID_TEXT_OUT_CUR_CYCLE_HOTBED_VP);
        RTS_ResetSingleVP(AUTO_PID_HOTBED_CYCLES);
        RTS_ResetSingleVP(WRITE_CURVE_DDR_CMD);    
        RTS_SndData("                           ", PID_TEXT_OUT_VP);             
        RTS_ShowPage(84);
        if(g_uiAutoPIDHotbedRunningFlag == true){        
        }else{
        RTS_ResetSingleVP(AUTO_PID_RUN_HOTBED_TIS_VP);
        RTS_SendLang(AUTO_PID_HOTBED_TIS_VP);
        }
      }
      else if (recdat.data[0] == 163) {
        AxisUnitMode = 1;
        axis_unit = 10.0;

        if(axes_should_home()) {
          waitway = 4;
          queue.enqueue_one_P(PSTR("G28"));
          RTS_ChangeLevelingPage();  
        }else{
          RTS_ShowPage(16);
          RTS_SendCurrentPosition(4);
        }
      }                                      
      break;

    case AdjustEnterKey:
      rtscheck.RTS_SendLoadedData(6);     
      if(recdat.data[0] == 1)
      {
        rtscheck.RTS_SendLoadedData(5);              
        RTS_ShowPage(14);
      }
      else if(recdat.data[0] == 2)
      {
        if(printingIsActive())
        {
          RTS_ShowPage(10);
        }
        else
        {
          RTS_ShowPage(12);
        }
      }
      else if(recdat.data[0] == 5)
      {
        RTS_ShowPage(15);
      }
      else if(recdat.data[0] == 7)
      {
        RTS_ShowPage(14);
        settings.save();
      }
      else if(recdat.data[0] == 8)
      {
        if(runout.enabled)
        {
          RTS_SndData(102, FILAMENT_CONTROL_ICON_VP);
          runout.enabled = false;
        }
        else
        {
          RTS_SndData(101, FILAMENT_CONTROL_ICON_VP);
          runout.enabled = true;
        }
        settings.save();
      }      
      else if(recdat.data[0] == 9)
      {
        if (recovery.enabled) {
          RTS_SndData(102, POWERCONTINUE_CONTROL_ICON_VP);
          recovery.enabled = false;
          if (card.flag.mounted) { // rock_20220701 Fix the bug that the switch is always on when the power is off
            #if ENABLED(POWER_LOSS_RECOVERY)
              if (card.jobRecoverFileExists()){
                card.removeFile(recovery.filename);
              }
            #endif
          }
        }
        else {
          RTS_SndData(101, POWERCONTINUE_CONTROL_ICON_VP);
          recovery.enabled = true;
          recovery.save(true);
        }
        settings.save();
      }
      break;

    case PrintSpeedEnterKey:
      feedrate_percentage = recdat.data[0];
      break;

    case StopPrintKey:
      RTS_LoadMainsiteIcons();
      if(recdat.data[0] == 1)
      {
        RTS_ShowPage(13);
      }
      else if(recdat.data[0] == 2)
      {
        Update_Time_Value = 0;
        temphot = 0;
        runout.reset();
        wait_for_user = false;
        RTS_ShowPreviewImage(true);
        RTS_SDcard_Stop();
        queue.clear();
        RTS_ShowPage(1);
      }
      else if(recdat.data[0] == 4)
      {
        if(!planner.has_blocks_queued()) //rock_20220401
        {
          if(PoweroffContinue == true)
          {
            runout.filament_ran_out = false;
            waitway = 7;
            RTS_ShowPage(40);
            #if ENABLED(FILAMENT_RUNOUT_SENSOR)
              if(runout.enabled == true)
              {
                pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
                ui.pause_show_message(PAUSE_MESSAGE_RESUME);
                queue.inject_P(PSTR("M108"));
              }
            #endif
            RTS_ResetProgress();
            Update_Time_Value = 0;
            temphot = 0;
            card.flag.abort_sd_printing = true;
            queue.clear();
            quickstop_stepper();
            print_job_timer.abort();
            // delay(10);
            while(planner.has_blocks_queued())
            {
              idle();
            }
            RTS_ResetHotendBed();
            thermalManager.zero_fan_speeds();
            while(thermalManager.temp_hotend[0].target > 0)
            {
              thermalManager.setTargetHotend(0, 0);
              idle();
            }
            RTS_SDcard_Stop();
            Update_Time_Value = 0;
            RTS_ShowPage(1);
            RefreshBrightnessAtPrint(0);
            print_job_timer.stop();
          }
          else if(PoweroffContinue == false)
          {
            PoweroffContinue = true;
            runout.filament_ran_out = false;
            RTS_ShowPage(40);
            waitway = 7;
            #if ENABLED(FILAMENT_RUNOUT_SENSOR)
              if(runout.enabled == true)
              {
                pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
                ui.pause_show_message(PAUSE_MESSAGE_RESUME);
                queue.inject_P(PSTR("M108"));
              }
            #endif
            RTS_ResetPrintData(true);
            Update_Time_Value = 0;
            temphot = 0;
            card.flag.abort_sd_printing = true;
            queue.clear();
            quickstop_stepper();
            print_job_timer.abort();
            // delay(10);
            while(planner.has_blocks_queued())
            {
              idle();
            }
            RTS_ResetHotendBed();
            thermalManager.zero_fan_speeds();
            while(thermalManager.temp_hotend[0].target > 0)
            {
              thermalManager.setTargetHotend(0, 0);
              idle();
            }
            RTS_SDcard_Stop();
            PoweroffContinue = false;
            RTS_ShowPage(1);
            print_job_timer.stop();            
          }
        }
      }      
      break;

    case PausePrintKey:
      if(recdat.data[0] == 1)
      {
        if(printingIsActive() && (thermalManager.temp_hotend[0].celsius > (thermalManager.temp_hotend[0].target - 5)) && (thermalManager.temp_bed.celsius > (thermalManager.temp_bed.target - 3)))
        {
          RTS_SndData(runout.enabled ? 101 : 102, FILAMENT_CONTROL_ICON_VP);
          RTS_ShowPage(11);
        }
        else 
        {
          RTS_LoadMainsiteIcons();
          RTS_ShowPage(10);
        }
      }
      else if(recdat.data[0] == 2)
      {
        if(printingIsActive() && (thermalManager.temp_hotend[0].celsius > (thermalManager.temp_hotend[0].target - 5)) && (thermalManager.temp_bed.celsius > (thermalManager.temp_bed.target - 3)))
        {
        }
        else 
        {
          RTS_LoadMainsiteIcons();
          RTS_ShowPage(10);
          break;
        }

        if(!temphot)
        {
          temphot = thermalManager.temp_hotend[0].target;
        }

        queue.inject_P(PSTR("M25"));

      }
      else if(recdat.data[0] == 3)
      {
        if(printingIsActive())
        {
          RTS_LoadMainsiteIcons();
          RTS_ShowPage(10);
        }
        else
        {
          RTS_ShowPage(12);
        }
      }
      break;

    case ResumePrintKey:
      RTS_LoadMainsiteIcons();
      if(recdat.data[0] == 1)
      {
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if ((1 == READ(FIL_RUNOUT_PIN)) && (runout.enabled == true))
          {
            RTS_ShowPage(7);
            break;
          }
        #endif

        RTS_ShowPage(10);

        #if ENABLED(HAS_RESUME_CONTINUE)
          if(wait_for_user)
          {
            wait_for_user = false;
          }
          else
        #endif
          {
            planner.synchronize();
            memset(commandbuf, 0, sizeof(commandbuf));
            sprintf_P(commandbuf, PSTR("M109 S%i"), temphot);
            queue.enqueue_one_now(commandbuf);
            queue.inject_P(PSTR("M24"));
            Update_Time_Value = 0;
            sdcard_pause_check = true;
          }
      }
      else if(recdat.data[0] == 2)
      {
        if(thermalManager.temp_hotend[0].target >= EXTRUDE_MINTEMP)
        {
          thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
        }
        else
        {
          thermalManager.setTargetHotend(200, 0);
        }
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if ((1 == READ(FIL_RUNOUT_PIN)) && (runout.enabled == true))
          {
            RTS_ShowPage(7);
          }
          else
          {
            RTS_ShowPage(8);
          }
        #endif
      }
      else if(recdat.data[0] == 3)
      {
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if ((1 == READ(FIL_RUNOUT_PIN)) && (runout.enabled == true))
          {
            RTS_ShowPage(7);
            break;
          }
        #endif    
        runout.filament_ran_out = false; 
        pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;
        ui.pause_show_message(PAUSE_MESSAGE_RESUME);
        queue.inject_P(PSTR("M108"));
        runout.reset();
        RTS_ShowPage(10);
        card.startOrResumeFilePrinting();
        print_job_timer.start();
        Update_Time_Value = 0;
        sdcard_pause_check = true;
        RTS_SendM600Icon(true);
      }
      else if (recdat.data[0] == 4) {
        if (IS_SD_INSERTED()) { //有卡
          lcd_sd_status = true;
          card.startOrResumeFilePrinting();
          print_job_timer.start();
          Update_Time_Value = 0;
          sdcard_pause_check = true;
          RTS_ShowPage(10);
          gcode.process_subcommands_now(F("M24"));
        }
        else {
          CardUpdate = true;
          rtscheck.RTS_SDCardUpdate();
          RTS_ShowPage(47);
        }
      }
      break;

    case ZoffsetEnterKey:
      last_zoffset = zprobe_zoffset;
      if(recdat.data[0] >= 32768)
      {
        zprobe_zoffset = ((float)recdat.data[0] - 65536) / 100;
        zprobe_zoffset -= 0.001;
      }
      else
      {
        zprobe_zoffset = ((float)recdat.data[0]) / 100;
        zprobe_zoffset += 0.001;
      }
      if(WITHIN((zprobe_zoffset), PROBE_OFFSET_ZMIN, PROBE_OFFSET_XMAX))
      {
        babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
      }
      probe.offset.z = zprobe_zoffset;
      RTS_SendZoffsetFeedratePercentage(true);
      hal.watchdog_refresh();
      break;

    case Zoffset005EnterKey:
      last_zoffset = zprobe_zoffset;
      if(recdat.data[0] >= 32768)
      {
        zprobe_zoffset = ((float)recdat.data[0] - 65536) / 100;
        zprobe_zoffset -= 0.001;
      }
      else
      {
        zprobe_zoffset = ((float)recdat.data[0]) / 100;
        zprobe_zoffset += 0.001;
      }
      if(WITHIN((zprobe_zoffset), PROBE_OFFSET_ZMIN, PROBE_OFFSET_XMAX))
      {
        babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
      }
      probe.offset.z = zprobe_zoffset;
      RTS_SendZoffsetFeedratePercentage(true);
      hal.watchdog_refresh();
      break;  

    case XoffsetEnterKey: {
      last_xoffset = xprobe_xoffset;
      if (recdat.data[0] >= 32768) {
        xprobe_xoffset = ((float)recdat.data[0] - 65536) / 100;
        xprobe_xoffset -= 0.001;
      }
      else
      {
        xprobe_xoffset = ((float)recdat.data[0]) / 100;
        xprobe_xoffset += 0.001;
      }      
      float x_offset_change = xprobe_xoffset - last_xoffset;
      if (WITHIN((xprobe_xoffset), -100, 100)) {
        float babystep_x_offset = -x_offset_change;
        babystep.add_mm(X_AXIS, babystep_x_offset);
      }     
      probe.offset.x = xprobe_xoffset;

      if (probe.offset.x < 0) {
        probe_offset_x_temp = fabs(probe.offset.x);
      }else{
        probe_offset_x_temp = -fabs(probe.offset.x);
      }
      int max_reachable_pos_x = X_MAX_POS - custom_ceil(probe_offset_x_temp);
      int min_calc_margin_x = X_BED_SIZE - max_reachable_pos_x;
      min_calc_margin_x = fabs(min_calc_margin_x); // Ensure it's positive      
      #if ENABLED(LCD_RTS_DEBUG_MARGIN_X)
        SERIAL_ECHO_MSG("probe_margin_x old: ", lcd_rts_settings.probe_margin_x);
      #endif
      if(min_calc_margin_x >= lcd_rts_settings.probe_margin_x){
      lcd_rts_settings.probe_margin_x= min_calc_margin_x;
      }
      #if ENABLED(ENDER_3S1_PLUS)
        if(lcd_rts_settings.probe_margin_x <= 27){
          lcd_rts_settings.probe_margin_x = 27;
        }
      #endif
      #if ENABLED(LCD_RTS_DEBUG_MARGIN_X)
        SERIAL_ECHO_MSG("min_calc_margin_x: ", min_calc_margin_x);
        SERIAL_ECHO_MSG("probe_margin_x new: ", lcd_rts_settings.probe_margin_x);
      #endif
      RTS_SendLevelingSiteData(1);
      RTS_SndData(xprobe_xoffset * 100, HOTEND_X_ZOFFSET_VP);
      hal.watchdog_refresh();
      break;
    }

    case YoffsetEnterKey: {
      last_yoffset = yprobe_yoffset;
      if (recdat.data[0] >= 32768) {
        yprobe_yoffset = ((float)recdat.data[0] - 65536) / 100;
        yprobe_yoffset -= 0.001;
      }
      else
      {
        yprobe_yoffset = ((float)recdat.data[0]) / 100;
        yprobe_yoffset += 0.001;
      }    
      float y_offset_change = yprobe_yoffset - last_yoffset;
      if (WITHIN((yprobe_yoffset), -100, 100)) {
        float babystep_y_offset = -y_offset_change;
        babystep.add_mm(Y_AXIS, babystep_y_offset);
      }
      probe.offset.y = yprobe_yoffset;
      probe_offset_y_temp = fabs(probe.offset.y);

      int max_reachable_pos_y = Y_MAX_POS - custom_ceil(probe_offset_y_temp);
      int min_calc_margin_y = Y_BED_SIZE - max_reachable_pos_y;
      min_calc_margin_y = fabs(min_calc_margin_y); // Ensure it's positive
      #if ENABLED(LCD_RTS_DEBUG_MARGIN_Y)
        SERIAL_ECHO_MSG("Y probe_margin_y old: ", lcd_rts_settings.probe_margin_y_front);
        SERIAL_ECHO_MSG("Y probe_min_margin_y old: ", lcd_rts_settings.probe_margin_y_back);
      #endif
      if(min_calc_margin_y <= 10){
        min_calc_margin_y = 10;
      }

    if(min_calc_margin_y > lcd_rts_settings.probe_margin_y_back){
        if(lcd_rts_settings.probe_margin_x <= min_calc_margin_y){      
          lcd_rts_settings.probe_margin_y_front = lcd_rts_settings.probe_margin_x;
          lcd_rts_settings.probe_margin_y_back = min_calc_margin_y;
        }else{
          lcd_rts_settings.probe_margin_y_front = min_calc_margin_y;
          lcd_rts_settings.probe_margin_y_back = min_calc_margin_y;
        }
        RTS_SendLevelingSiteData(2);
      }

      #if ENABLED(LCD_RTS_DEBUG_MARGIN_Y)
          SERIAL_ECHO_MSG("Y probe_min_margin_y new: ", lcd_rts_settings.probe_margin_y_front);
          SERIAL_ECHO_MSG("Y probe_min_margin_y new: ", lcd_rts_settings.probe_margin_y_back);
      #endif
           
      RTS_SndData(yprobe_yoffset * 100, HOTEND_Y_ZOFFSET_VP);
      hal.watchdog_refresh();
      break;
    }

    case TempControlKey:
      if (!printingIsActive() && !planner.has_blocks_queued()) { 
        if(recdat.data[0] == 2)
        {
          RTS_ShowPage(20);
        }
        else if(recdat.data[0] == 3)
        {
          preheat_flag = PREHEAT_PLA;
          temp_preheat_nozzle = ui.material_preset[0].hotend_temp;
          temp_preheat_bed = ui.material_preset[0].bed_temp;
          RTS_SndData(ui.material_preset[0].hotend_temp, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
          RTS_SndData(ui.material_preset[0].bed_temp, PREHEAT_PLA_SET_BED_TEMP_VP);
          delay(2);
          RTS_ShowPage(22);
        }
        else if(recdat.data[0] == 4)
        {
          preheat_flag = PREHEAT_ABS;
          temp_preheat_nozzle = ui.material_preset[1].hotend_temp;
          temp_preheat_bed = ui.material_preset[1].bed_temp;
          RTS_SndData(ui.material_preset[1].hotend_temp, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
          RTS_SndData(ui.material_preset[1].bed_temp, PREHEAT_ABS_SET_BED_TEMP_VP);
          delay(2);
          RTS_ShowPage(23);
        }
        else if(recdat.data[0] == 5)
        {  
          thermalManager.temp_hotend[0].target = ui.material_preset[0].hotend_temp;
          RTS_SendHeadTemp();
          thermalManager.temp_bed.target = ui.material_preset[0].bed_temp;
          RTS_SendBedTemp();
        }
        else if(recdat.data[0] == 6)
        {
          thermalManager.temp_hotend[0].target = ui.material_preset[1].hotend_temp;
          RTS_SendHeadTemp();
          thermalManager.temp_bed.target = ui.material_preset[1].bed_temp;
          RTS_SendBedTemp();
        }
        else if(recdat.data[0] == 7)
        {
          RTS_ShowPage(21);
        }
        else if(recdat.data[0] == 8)
        {
          RTS_ShowPage(20);
        }
        else if(recdat.data[0] == 161)
        {
          preheat_flag = PREHEAT_PETG;
          temp_preheat_nozzle = ui.material_preset[2].hotend_temp;
          temp_preheat_bed = ui.material_preset[2].bed_temp;
          RTS_SndData(ui.material_preset[2].hotend_temp, PREHEAT_PETG_SET_NOZZLE_TEMP_VP);
          RTS_SndData(ui.material_preset[2].bed_temp, PREHEAT_PETG_SET_BED_TEMP_VP);
          delay(2);
          RTS_ShowPage(90);
        }
        else if(recdat.data[0] == 162)
        {
          preheat_flag = PREHEAT_CUST;
          temp_preheat_nozzle = ui.material_preset[3].hotend_temp;
          temp_preheat_bed = ui.material_preset[3].bed_temp;
          RTS_SndData(ui.material_preset[3].hotend_temp, PREHEAT_CUST_SET_NOZZLE_TEMP_VP);
          RTS_SndData(ui.material_preset[3].bed_temp, PREHEAT_CUST_SET_BED_TEMP_VP);
          delay(2);
          RTS_ShowPage(91);
        }
        else if(recdat.data[0] == 163)
        {  
          thermalManager.temp_hotend[0].target = ui.material_preset[2].hotend_temp;
          RTS_SendHeadTemp();
          thermalManager.temp_bed.target = ui.material_preset[2].bed_temp;
          RTS_SendBedTemp();
        }
        else if(recdat.data[0] == 164)
        {
          thermalManager.temp_hotend[0].target = ui.material_preset[3].hotend_temp;
          RTS_SendHeadTemp();
          thermalManager.temp_bed.target = ui.material_preset[3].bed_temp;
          RTS_SendBedTemp();
        }    
      }        
      break;

    case CoolDownKey:
      if (!printingIsActive() && !planner.has_blocks_queued()) { 
        if(recdat.data[0] == 1)
        {
          RTS_ResetHotendBed();
          RTS_ResetHeadAndBedSetTemp();
          thermalManager.fan_speed[0] = 255;
        }
        else if(recdat.data[0] == 2)
        {
          RTS_ShowPage(21);
        } else if (recdat.data[0] == 3) {
          settings.save();
        }else if (recdat.data[0] == 4) {
          RTS_ShowPage(48);
          g_uiAutoPIDFlag = true;
          RTS_ResetHotendBed();
          last_target_temperature[0] = thermalManager.temp_hotend[0].target;
          last_target_temperature_bed = thermalManager.temp_bed.target;
          RTS_SndData(g_autoPIDHeaterTempTarget, HEAD_SET_TEMP_VP);
          RTS_SndData(g_autoPIDHotBedTempTarget, BED_SET_TEMP_VP);
        }else if (recdat.data[0] == 5) {
          RTS_ResetHotendBed();
          RTS_ResetHeadAndBedSetTemp();
          thermalManager.fan_speed[0] = 0;
        }
      }
      break;

    case HeaterTempEnterKey:
      if (false == g_uiAutoPIDFlag) {
        temphot = recdat.data[0];
        thermalManager.setTargetHotend(temphot, 0);
        RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
      } else { // è‡ªåŠ¨PID
          if ((g_uiAutoPIDNozzleRunningFlag == true) || (recdat.data[0] < 200)) {
              RTS_SndData(g_autoPIDHeaterTempTargetset, HEAD_SET_TEMP_VP);
              break;
          }
          g_autoPIDHeaterTempTargetset = recdat.data[0];
          RTS_SndData(g_autoPIDHeaterTempTargetset, HEAD_SET_TEMP_VP);
      }      
      break;

    case HotBedTempEnterKey:
      if (false == g_uiAutoPIDFlag) {
        tempbed = recdat.data[0];
        temp_bed_display=recdat.data[0];
        #if ENABLED(BED_TEMP_COMP)
            if (tempbed > 60 && tempbed <= 80)
                tempbed += 5;
            else if (tempbed > 80 && tempbed <= 120)
                tempbed += 7;
        #endif
        thermalManager.setTargetBed(tempbed);
         RTS_SndData(temp_bed_display, BED_SET_TEMP_VP);
      } else { // è‡ªåŠ¨PID
        if ((g_uiAutoPIDHotbedRunningFlag == true) || (recdat.data[0] < 60)) {
            RTS_SndData(g_autoPIDHotBedTempTargetset, BED_SET_TEMP_VP);
            break;
        }
        tempbed = 0;
        g_autoPIDHotBedTempTargetset = recdat.data[0];
        RTS_SndData(g_autoPIDHotBedTempTargetset, BED_SET_TEMP_VP);
      }
      break;
    case PrepareEnterKey:
      if(recdat.data[0] == 1)
      {
        RTS_ShowPage(28);
      }
      else if(recdat.data[0] == 2)
      {
        // Adv.set       
        if(g_uiAutoPIDNozzleRunningFlag == true) break;          
        if(g_uiAutoPIDHotbedRunningFlag == true) break;        
        RTS_ShowPage(33);
      }
      else if(recdat.data[0] == 3)
      {
        RTS_SendCurrentPosition(4);
        delay(2);
        RTS_ShowPage(16);
      }
      else if(recdat.data[0] == 5)
      {  
        RTS_SendMachineData();
        delay(5);
        RTS_ShowPage(24);
        delay(1000);
      }
      else if(recdat.data[0] == 6)
      {
        if (leveling_running == 0 && !planner.has_blocks_queued() && !printingIsActive()) {        
          queue.enqueue_now_P(PSTR("M84"));
          queue.enqueue_now_P(PSTR("G92.9Z0"));
          RTS_ShowMotorFreeIcon(true);
        }
      }
      else if(recdat.data[0] == 7)
      {
        RTS_ShowPage(43);
      }
      else if(recdat.data[0] == 8)
      {
        ui.material_preset[preheat_flag].hotend_temp = temp_preheat_nozzle;
        ui.material_preset[preheat_flag].bed_temp = temp_preheat_bed;

        settings.save();
        RTS_ShowPage(21);
      }
      else if(recdat.data[0] == 9)
      {
        RTS_ShowPage(1);
        RTS_ShowPreviewImage(true);
      }
      else if(recdat.data[0] == 0xA)
      {
        RTS_ShowPage(42);
      }
      else if(recdat.data[0] == 0xB)
      {
        RTS_ResetMesh();
        RTS_ResetSingleVP(MESH_POINT_MIN);
        RTS_ResetSingleVP(MESH_POINT_MAX);
        RTS_ResetSingleVP(MESH_POINT_DEVIATION);
        (void)settings.reset();
        (void)settings.save();
        RTS_Init(); 
        RTS_ShowPage(1);
      }
      else if(recdat.data[0] == 0xC)
      {
        RTS_ShowPage(44);
      }
      else if(recdat.data[0] == 0xD)
      {
        settings.reset();
        settings.save();
        RTS_ShowPage(33);
      }
      else if(recdat.data[0] == 0xE)
      {
        if(!planner.has_blocks_queued())
        {
	        RTS_ShowPage(33);
	      }
      }
      else if(recdat.data[0] == 0xF)
      {
        if(!printingIsActive() && leveling_running == 0){
          RTS_ShowPage(21);
          //settings.save();
          delay(100);
        }
      }
      else if(recdat.data[0] == 0x10)
      {          
        RTS_ShowPage(25);
      }
      else if(recdat.data[0] == 0x11)
      {
        RTS_ShowPage(21);
      }
      break;

    case BedLevelKey:
      if(recdat.data[0] == 1)
      {
        planner.synchronize();
        waitway = 6;
        RTS_ShowPage(26);
        queue.enqueue_now_P(PSTR("G28"));
        RTS_ResetSingleVP(AUTO_LEVELING_PERCENT_DATA_VP);  
        Update_Time_Value = 0;
      }
      else if(recdat.data[0] == 2)
      {      
        last_zoffset = zprobe_zoffset;
        float rec_zoffset = 0;
        if(recdat.data[0] >= 32768) {
          rec_zoffset = ((float)recdat.data[0] - 65536) / 100;
        } else {
          rec_zoffset = ((float)recdat.data[0]) / 100;
        }
        if(WITHIN((zprobe_zoffset + 0.01), PROBE_OFFSET_ZMIN, PROBE_OFFSET_XMAX)) {
          #if ENABLED(HAS_LEVELING)
          if (rec_zoffset > last_zoffset) {
            zprobe_zoffset = last_zoffset;
            zprobe_zoffset += 0.01;
          }
          #endif
          babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          probe.offset.z = zprobe_zoffset;
        }
        RTS_SendZoffsetFeedratePercentage(true);
      }
      else if(recdat.data[0] == 3)
      {
        last_zoffset = zprobe_zoffset;
        float rec_zoffset = 0;
        if(recdat.data[0] >= 32768) {
          rec_zoffset = ((float)recdat.data[0] - 65536) / 100;
        } else {
          rec_zoffset = ((float)recdat.data[0]) / 100;
        }
        if(WITHIN((zprobe_zoffset - 0.01), PROBE_OFFSET_ZMIN, PROBE_OFFSET_XMAX)) {
          #if ENABLED(HAS_LEVELING)
          if (rec_zoffset > last_zoffset) {
            zprobe_zoffset = last_zoffset;
            zprobe_zoffset -= 0.01;
          }
          #endif
          babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          probe.offset.z = zprobe_zoffset;
        }
        RTS_SendZoffsetFeedratePercentage(true);
      }
      else if(recdat.data[0] == 4)
      {
        if(!planner.has_blocks_queued() && !printingIsActive())
        {
          bltouch_tramming = 0;            
	        RTS_ShowPage(25);
        }
      }
      else if(recdat.data[0] == 5)
      {
        char cmd[23];
        // Assitant Level , Center 1
        if(axes_should_home()) {
          if (bltouch_tramming == 0){
          waitway = 16;
          }
          if (bltouch_tramming == 1){
          waitway = 17;
          }          
          RTS_G28MoveOne();
        }else{
          if(!planner.has_blocks_queued())
          {
            waitway = 4;
            if (bltouch_tramming == 0){
            queue.enqueue_now_P(PSTR("G1 F600 Z3"));
            #if ENABLED(ENDER_3S1_PRO) || ENABLED(ENDER_3S1)
              sprintf_P(cmd, "G1 X117.5 Y117.5 F2000");
            #elif ENABLED(ENDER_3S1_PLUS)
              sprintf_P(cmd, "G1 X155 Y155 F2000");
            #else
              sprintf_P(cmd, "G1 X%d Y%d F2000", manual_level_5position[0][0],manual_level_5position[0][1]);
            #endif          
            queue.enqueue_now_P(cmd);               
            #if ENABLED(ENDER_3S1_PRO) || ENABLED(ENDER_3S1)
              rtscheck.RTS_SndData((unsigned char)10 * (unsigned char)117.5, AXIS_X_COORD_VP);
              rtscheck.RTS_SndData((unsigned char)10 * (unsigned char)117.5, AXIS_Y_COORD_VP);
            #elif ENABLED(ENDER_3S1_PLUS)
              rtscheck.RTS_SndData((unsigned char)10 * (unsigned char)155, AXIS_X_COORD_VP);
              rtscheck.RTS_SndData((unsigned char)10 * (unsigned char)155, AXIS_Y_COORD_VP);
            #else
              RTS_TrammingPosition(0, 0, 0, 1);
            #endif
            queue.enqueue_now_P(PSTR("G1 F600 Z0"));
            RTS_ResetSingleVP(AXIS_Z_COORD_VP);
            }
            if (bltouch_tramming == 1){
              #if ENABLED(ENDER_3S1_PRO) || ENABLED(ENDER_3S1)
                queue.enqueue_now_P(PSTR("G30 X117.5 Y117.5")); 
              #elif ENABLED(ENDER_3S1_PLUS)
                queue.enqueue_now_P(PSTR("G30 X155 Y155")); 
              #else
                sprintf_P(cmd, "G30 X%d Y%d", manual_level_5position[0][0],manual_level_5position[0][1]);
                queue.enqueue_now_P(cmd);              
              #endif             
              RTS_ShowPage(89);
            }
            waitway = 0;
          }
        }        
      }
      else if (recdat.data[0] == 6)
      {
        char cmd[20];
        // Assitant Level , Front Left 2
        if(axes_should_home()) {
          if (bltouch_tramming == 0){
          waitway = 16;
          }
          if (bltouch_tramming == 1){
          waitway = 17;
          }
          RTS_G28MoveOne();
        }else{     
          if(!planner.has_blocks_queued())
          {
            waitway = 4;
            if (bltouch_tramming == 0){            
              RTS_TrammingPosition(1, 0, 1, 1);
            }
            if (bltouch_tramming == 1){
            // Cr-Touch measuring point 6
            sprintf_P(cmd, "G30 X%d Y%d", lcd_rts_settings.probe_margin_x,lcd_rts_settings.probe_margin_y_front);
            queue.enqueue_now_P(cmd);
            RTS_ShowPage(89);                  
            }
            waitway = 0;
          }
        }                
      }
      else if (recdat.data[0] == 7)
      {
        char cmd[20];
        // Assitant Level , Front Right 3
        if(axes_should_home()) {
          if (bltouch_tramming == 0){
          waitway = 16;
          }
          if (bltouch_tramming == 1){
          waitway = 17;
          }
          RTS_G28MoveOne();
        }else{     
          if(!planner.has_blocks_queued())
          {
            waitway = 4;
            if (bltouch_tramming == 0){          
              RTS_TrammingPosition(2, 0, 2, 1);
            }
            if (bltouch_tramming == 1){
            // Cr-Touch measuring point 7
            sprintf_P(cmd, "G30 X%d Y%d", (X_BED_SIZE - lcd_rts_settings.probe_margin_x),lcd_rts_settings.probe_margin_y_front);
            queue.enqueue_now_P(cmd);
            RTS_ShowPage(89);
            }
            waitway = 0;
          }
        }                
      }
      else if (recdat.data[0] == 8)
      {
        char cmd[20];
        // Assitant Level , Back Right 4
        if(axes_should_home()) {
          if (bltouch_tramming == 0){
          waitway = 16;
          }
          if (bltouch_tramming == 1){
          waitway = 17;
          }
          RTS_G28MoveOne();
        }else{       
          if(!planner.has_blocks_queued())
          {
            waitway = 4;
            if (bltouch_tramming == 0){          
            RTS_TrammingPosition(3, 0, 3, 1);
            }
            if (bltouch_tramming == 1){
            // Cr-Touch measuring point 8
            sprintf_P(cmd, "G30 X%d Y%d", lcd_rts_settings.probe_margin_x,(Y_BED_SIZE - lcd_rts_settings.probe_margin_y_back));
            queue.enqueue_now_P(cmd);
            RTS_ShowPage(89);                      
            }
            waitway = 0;
          }
        }                
      }
      else if (recdat.data[0] == 9)
      {
        char cmd[20];
        // Assitant Level , Back Left 5
        if(axes_should_home()) {
          if (bltouch_tramming == 0){
          waitway = 16;
          }
          if (bltouch_tramming == 1){
          waitway = 17;
          }
          RTS_G28MoveOne();
        }else{
          if(!planner.has_blocks_queued())
          {
            waitway = 4;
            if (bltouch_tramming == 0){         
              RTS_TrammingPosition(4, 0, 4, 1);
            }
            if (bltouch_tramming == 1){
            // Cr-Touch measuring point 9
            sprintf_P(cmd, "G30 X%d Y%d", (X_BED_SIZE - lcd_rts_settings.probe_margin_x),(Y_BED_SIZE - lcd_rts_settings.probe_margin_y_back));
            queue.enqueue_now_P(cmd);
            RTS_ShowPage(89);                    
            }  
            waitway = 0;
          }
        }                
      }
      else if (recdat.data[0] == 0x0B)
      {
        // Assitant Level , Back Left 6
        if(axes_should_home()) {
          if (bltouch_tramming == 0){
          waitway = 16;
          }
          if (bltouch_tramming == 1){
          waitway = 17;
          }
          RTS_G28MoveOne();
        }else{     
          if(!planner.has_blocks_queued())
          {
            if (bltouch_tramming == 0){
            waitway = 4;
              RTS_TrammingPosition(5, 0, 5, 1);
            }
            waitway = 0;
          }
        }                
      }
      else if (recdat.data[0] == 0x0C)
      {
        // Assitant Level , Back Left 7
        if(axes_should_home()) {
          if (bltouch_tramming == 0){
          waitway = 16;
          }
          if (bltouch_tramming == 1){
          waitway = 17;
          }
          RTS_G28MoveOne();
        }else{      
          if(!planner.has_blocks_queued())
          {
            if (bltouch_tramming == 0){
            waitway = 4;
              RTS_TrammingPosition(6, 0, 6, 1);
            }
            waitway = 0;
          }
        }                
      }
      else if (recdat.data[0] == 0x0D)
      {
        // Assitant Level , Back Left 8
        if(axes_should_home()) {
          if (bltouch_tramming == 0){
          waitway = 16;
          }
          if (bltouch_tramming == 1){
          waitway = 17;
          }
          RTS_G28MoveOne();
        }else{     
          if(!planner.has_blocks_queued())
          {
            if (bltouch_tramming == 0){
            waitway = 4;
            RTS_TrammingPosition(7, 0, 7, 1);
            }    
            waitway = 0;          
          }
        }                
      }
      else if (recdat.data[0] == 0x0E)
      {
        //char cmd[20];
        // Assitant Level , Back Left 9
        if(axes_should_home()) {
          if (bltouch_tramming == 0){
          waitway = 16;
          }
          if (bltouch_tramming == 1){
          waitway = 17;
          }
          RTS_G28MoveOne();
        }else{    
          if(!planner.has_blocks_queued())
          {
            if (bltouch_tramming == 0){
            waitway = 4;
              RTS_TrammingPosition(8, 0, 8, 1);
            }      
            waitway = 0;
          }
        }                
      }
      else if(recdat.data[0] == 0x0A)
      {
        if(!planner.has_blocks_queued())
        {
	        RTS_ShowPage(26);
		    }
      }
      else if(recdat.data[0] == 161)
      { // 00A1  
        last_zoffset = zprobe_zoffset;
        float rec_zoffset = 0;
        if(recdat.data[0] >= 32768) {
          rec_zoffset = ((float)recdat.data[0] - 65536) / 100;
        } else {
          rec_zoffset = ((float)recdat.data[0]) / 100;
        }
        if(WITHIN((zprobe_zoffset + 0.05), PROBE_OFFSET_ZMIN, PROBE_OFFSET_XMAX)) {
          #if ENABLED(HAS_LEVELING)
          if (rec_zoffset > last_zoffset) {
            zprobe_zoffset = last_zoffset;
            zprobe_zoffset += 0.05;
          }
          #endif
          babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          probe.offset.z = zprobe_zoffset;
        }
        RTS_SendZoffsetFeedratePercentage(true);
      }
      else if(recdat.data[0] == 162)
      { // 00A2
        last_zoffset = zprobe_zoffset;
        float rec_zoffset = 0;
        if(recdat.data[0] >= 32768) {
          rec_zoffset = ((float)recdat.data[0] - 65536) / 100;
        } else {
          rec_zoffset = ((float)recdat.data[0]) / 100;
        }
        if(WITHIN((zprobe_zoffset - 0.05), PROBE_OFFSET_ZMIN, PROBE_OFFSET_XMAX)) {
          #if ENABLED(HAS_LEVELING)
          if (rec_zoffset > last_zoffset) {
            zprobe_zoffset = last_zoffset;
            zprobe_zoffset -= 0.05;
          }
          #endif
          babystep.add_mm(Z_AXIS, zprobe_zoffset - last_zoffset);
          probe.offset.z = zprobe_zoffset;
        }
        RTS_SendZoffsetFeedratePercentage(true);
      }    
      else if (recdat.data[0] == 163)
      { // 00A3
        if(!printingIsActive() && leveling_running == 0){
          #if ENABLED(BLTOUCH)
            RTS_SndData(lang + 10, AUTO_LEVELING_START_TITLE_VP);
            if(axes_should_home()){
              waitway = 15;
              RTS_G28MoveOne();
            }else{
              RTS_ChangeLevelingPage();
            }
            leveling_running = 1;
            RTS_ResetMesh();
            #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
              queue.enqueue_one_P(PSTR("G29"));
            #else
              touchscreen_requested_mesh = 1;
              queue.enqueue_one_P(PSTR("G29 P1 T"));
              queue.enqueue_one_P(PSTR("G29 S0"));
              queue.enqueue_one_P(PSTR("M420 S1"));
              queue.enqueue_one_P(PSTR("M500"));
            #endif
          #endif
        }
      }  
      else if(recdat.data[0] == 164)
      { // 00A4
        RTS_SendLang(AUTO_LEVELING_START_TITLE_VP);
        if(axes_should_home()) {
          waitway = 15;
          RTS_G28MoveOne();
        }
        RTS_SendLevelingSiteData(0);
        RTS_ResetSingleVP(AUTO_BED_LEVEL_CUR_POINT_VP);
        RTS_ResetSingleVP(AUTO_LEVELING_PERCENT_DATA_VP);
        RTS_AutoBedLevelPage();
        Update_Time_Value = 0;
      }
      else if (recdat.data[0] == 165) 
      { // 00A5 
        bltouch_tramming = 1;
        RTS_SendLevelingSiteData(1);        
        RTS_SendLevelingSiteData(2);        
        RTS_SndData(lcd_rts_settings.total_probing, PROBE_COUNT_VP);         
        RTS_ShowPage(89);
      }
      else if (recdat.data[0] == 166)
      { // 00A6 
        if (leveling_running == 0){
          if (bltouch_tramming == 1){
            leveling_running = 1;
            // Point 6
            char cmd6[20];
            sprintf_P(cmd6, "G30 X%d Y%d", lcd_rts_settings.probe_margin_x,lcd_rts_settings.probe_margin_y_front);
            queue.enqueue_now_P(cmd6);
            // Point 7
            char cmd7[20];
            sprintf_P(cmd7, "G30 X%d Y%d", (X_BED_SIZE - lcd_rts_settings.probe_margin_x),lcd_rts_settings.probe_margin_y_front);
            queue.enqueue_now_P(cmd7);
            // Point 8
            char cmd8[20];
            sprintf_P(cmd8, "G30 X%d Y%d", lcd_rts_settings.probe_margin_x,(Y_BED_SIZE - lcd_rts_settings.probe_margin_y_back));
            queue.enqueue_now_P(cmd8);
            // Point 9
            char cmd9[20];
            sprintf_P(cmd9, "G30 X%d Y%d", (X_BED_SIZE - lcd_rts_settings.probe_margin_x),(Y_BED_SIZE - lcd_rts_settings.probe_margin_y_back));
            queue.enqueue_now_P(cmd9);
            // Finally center
            #if ENABLED(ENDER_3S1_PRO) || ENABLED(ENDER_3S1)
              queue.enqueue_now_P(PSTR("G30 X117.5 Y117.5")); 
            #elif ENABLED(ENDER_3S1_PLUS)
              queue.enqueue_now_P(PSTR("G30 X155 Y155")); 
            #else     
              char cmd1[20];       
              sprintf_P(cmd1, "G30 X%d Y%d", manual_crtouch_5position[0][0],manual_crtouch_5position[0][1]);
              queue.enqueue_now_P(cmd1);
            #endif  
            RTS_ShowPage(89);                   
          }
        }
      }
      else if (recdat.data[0] == 167) 
      {
        lcd_rts_settings.display_volume = 0;
        lcd_rts_settings.display_sound = false;
        setTouchScreenConfiguration();
      }
      else if (recdat.data[0] == 168) 
      {
        lcd_rts_settings.display_volume = 255;
        lcd_rts_settings.display_sound = true;
        setTouchScreenConfiguration();
      }
      else if (recdat.data[0] == 169) 
      {
        lcd_rts_settings.screen_brightness = 10;
        setTouchScreenConfiguration();
      }
      else if (recdat.data[0] == 170) 
      {
        lcd_rts_settings.screen_brightness = 100;
        setTouchScreenConfiguration();
      }
      else if (recdat.data[0] == 171) 
      { // 0x00AB
        lcd_rts_settings.display_standby ^= true;
        setTouchScreenConfiguration();
      }
      else if (recdat.data[0] == 172) 
      { // 0x00AC
        if(!planner.has_blocks_queued())
        {
          bltouch_tramming = 1;
          RTS_SndData(190, BEDLEVELING_WAIT_TITLE_VP);                 
	        RTS_ShowPage(98);
        }
      }
      else if (recdat.data[0] == 173) 
      { // 0x00AD
        if (leveling_running == 0){
          if (bltouch_tramming == 1){
            leveling_running = 1;
            for(int i = 0; i < 4; i++) {
              unsigned long addr = ASSISTED_TRAMMING_POINT_TEXT_VP + i * 24;
              for(int j = 0; j < 24; j++) {
                RTS_ResetSingleVP(addr + j);
              }
              RTS_ResetSingleVP(ASSISTED_TRAMMING_POINT_1_VP + i);
            }
            queue.enqueue_now_P(PSTR("G35"));
            RTS_ShowPage(98);
          }
        }
      }
      break;

    case AutoHomeKey:
      last_xoffset = xprobe_xoffset = probe.offset_xy.x;
      RTS_SndData(xprobe_xoffset * 100, HOTEND_X_ZOFFSET_VP);  
      last_yoffset = yprobe_yoffset  = probe.offset_xy.y;
      RTS_SndData(yprobe_yoffset * 100, HOTEND_Y_ZOFFSET_VP);    
      if(recdat.data[0] == 1)
      {
        AxisUnitMode = 1;
        axis_unit = 10.0;
        RTS_ShowPage(16);
        RTS_SendMoveaxisUnitIcon(3);
      }
      else if(recdat.data[0] == 2)
      {
        AxisUnitMode = 2;
        axis_unit = 1.0;
        RTS_ShowPage(17);
        RTS_SendMoveaxisUnitIcon(2);
      }
      else if(recdat.data[0] == 3)
      {
        AxisUnitMode = 3;
        axis_unit = 0.1;
        RTS_ShowPage(18);
        RTS_SendMoveaxisUnitIcon(1);
      }
      else if(recdat.data[0] == 4)
      {
        waitway = 4;
        RTS_ShowPage(40);
        queue.enqueue_now_P(PSTR("G28 X Y"));
        Update_Time_Value = 0;
        RTS_ShowMotorFreeIcon(false);
      }
      else if(recdat.data[0] == 5)
      {
        waitway = 4;
        RTS_G28MoveNow();
      }
      else if(recdat.data[0] == 6)
      {
        if (leveling_running == 0 && !planner.has_blocks_queued() && !printingIsActive()) {
          waitway = 2;
          RTS_G28MoveNow();
        }
      }
      else if(recdat.data[0] == 161)
      { // 00A1
        AxisUnitMode = 1;
        axis_unit = 10.0;
        RTS_ShowPage(86);
        RTS_SendMoveaxisUnitIcon(3);
      }
      else if(recdat.data[0] == 162)
      { // 00A2
        AxisUnitMode = 2;
        axis_unit = 1.0;
        RTS_ShowPage(87);
        RTS_SendMoveaxisUnitIcon(2);
      }
      else if(recdat.data[0] == 163)
      { // 00A3
        AxisUnitMode = 3;
        axis_unit = 0.1;
        RTS_ShowPage(88);
        RTS_SendMoveaxisUnitIcon(1);
      }
      else if(recdat.data[0] == 164)
      { // 00A4
        waitway = 14;
        RTS_ShowPage(40);
        queue.enqueue_now_P(PSTR("G28 X Y"));
        Update_Time_Value = 0;
        RTS_ShowMotorFreeIcon(false);
      }
      else if(recdat.data[0] == 165)
      { // 00A5
        waitway = 14;
        RTS_G28MoveNow();
      }
      else if(recdat.data[0] == 166)
      { // 00A6
        if(leveling_running == 0 && !planner.has_blocks_queued()) {
        waitway = 16;
        RTS_G28MoveNow();
        }else{
        RTS_ShowPage(25);         
        }
        Update_Time_Value = 0;
      }
      else if(recdat.data[0] == 167)
      { // 00A7
        if (leveling_running == 0 && !planner.has_blocks_queued()) {
        waitway = 17;
        RTS_G28MoveNow();
        }else{
        RTS_ShowPage(89);
        }
        Update_Time_Value = 0;
      }
      else if(recdat.data[0] == 168)
      { // 00A8 // 00A1 before Offsetrouting
        RTS_ShowPage(92);
      }
      else if(recdat.data[0] == 169)
      { // 00A9 // Home Offsets
        RTS_SndData(home_offset.x * 10, HOME_X_OFFSET_VP);
        RTS_SndData(home_offset.y * 10, HOME_Y_OFFSET_VP);        
        RTS_ShowPage(93);
      } 
      else if(recdat.data[0] == 177)
      { // 00B1 Home X
        home_offset.x = 0;
        queue.enqueue_now_P(PSTR("G28"));
        queue.enqueue_now_P(PSTR("G28 X"));
        queue.enqueue_now_P(PSTR("G1 Z5 F1000"));        
        RTS_ResetSingleVP(HOME_X_OFFSET_VP);
        RTS_ResetSingleVP(HOME_X_OFFSET_SET_VP);      
        RTS_ShowPage(93); 
      }  
      else if(recdat.data[0] == 178)
      { // 00B2 Home y
        home_offset.y = 0;
        queue.enqueue_now_P(PSTR("G28"));      
        queue.enqueue_now_P(PSTR("G28 Y"));
        queue.enqueue_now_P(PSTR("G1 Z5 F1000"));
        RTS_ResetSingleVP(HOME_Y_OFFSET_VP);
        RTS_ResetSingleVP(HOME_Y_OFFSET_SET_VP);            
        RTS_ShowPage(93); 
      }
      else if(recdat.data[0] == 179)
      { // 00B3 // 00A1 before Offsetrouting
        RTS_ShowPage(97);
      }
      else if(recdat.data[0] == 180)
      { // 00A7
        if (leveling_running == 0 && !planner.has_blocks_queued()) {
        waitway = 18;
        RTS_G28MoveNow();
        }else{
        RTS_ShowPage(98);
        }
        Update_Time_Value = 0;
      }          
      break;

    case XaxismoveKey:
      float x_min, x_max;
      waitway = 4;
      x_min = 0;
      x_max = X_MAX_POS;
      current_position[X_AXIS] = ((float)recdat.data[0]) / 10;
      if(current_position[X_AXIS] < x_min)
      {
        current_position[X_AXIS] = x_min;
      }
      else if(current_position[X_AXIS] > x_max)
      {
        current_position[X_AXIS] = x_max;
      }
      RTS_line_to_current(X_AXIS);
      RTS_SendCurrentPosition(1);
      delay(1);
      RTS_ShowMotorFreeIcon(false);
      waitway = 0;
      break;

    case YaxismoveKey:
      float y_min, y_max;
      waitway = 4;
      y_min = 0;
      y_max = Y_MAX_POS;
      current_position[Y_AXIS] = ((float)recdat.data[0]) / 10;
      if(current_position[Y_AXIS] < y_min)
      {
        current_position[Y_AXIS] = y_min;
      }
      else if(current_position[Y_AXIS] > y_max)
      {
        current_position[Y_AXIS] = y_max;
      }
      RTS_line_to_current(Y_AXIS);
      RTS_SendCurrentPosition(2);
      delay(1);
      RTS_ShowMotorFreeIcon(false);
      waitway = 0;
      break;

    case ZaxismoveKey:
      float z_min, z_max;
      waitway = 4;
      z_min = Z_MIN_POS;
      z_max = Z_MAX_POS;
      current_position[Z_AXIS] = ((float)recdat.data[0])/10;
      if (current_position[Z_AXIS] < z_min)
      {
        current_position[Z_AXIS] = z_min;
      }
      else if (current_position[Z_AXIS] > z_max)
      {
        current_position[Z_AXIS] = z_max;
      }
      RTS_line_to_current(Z_AXIS);
      RTS_SendCurrentPosition(3);
      delay(1);
      RTS_ShowMotorFreeIcon(false);
      waitway = 0;
      break;

    case XaxismoveKeyHomeOffset:
      waitway = 4;
      if (recdat.data[FIRST_ELEMENT_INDEX_X] >= THRESHOLD_VALUE_X) {
        recdat.data[FIRST_ELEMENT_INDEX_X] = rec_dat_temp_last_x;
      }      
      current_position[X_AXIS] = ((float)recdat.data[0]) / 10;
      rec_dat_temp_real_x = ((float)recdat.data[0]) / 10;
      rec_dat_temp_last_x = recdat.data[0];                              
      RTS_line_to_current(X_AXIS);

      RTS_SndData(10 * current_position[X_AXIS], HOME_X_OFFSET_SET_VP);
      RTS_SendCurrentPosition(1);
      delay(1);
      RTS_ShowMotorFreeIcon(false);
      waitway = 0;
      break;

    case YaxismoveKeyHomeOffset:
      waitway = 4;
      if (recdat.data[FIRST_ELEMENT_INDEX_Y] >= THRESHOLD_VALUE_Y) {
        recdat.data[FIRST_ELEMENT_INDEX_Y] = rec_dat_temp_last_y;
      }
      current_position[Y_AXIS] = ((float)recdat.data[0]) / 10;      
      rec_dat_temp_real_y = ((float)recdat.data[0]) / 10;
      rec_dat_temp_last_y = recdat.data[0];                              
      RTS_line_to_current(Y_AXIS);

      RTS_SndData(10 * current_position[Y_AXIS], HOME_Y_OFFSET_SET_VP);
      RTS_SendCurrentPosition(2);     
      delay(1);
      RTS_ShowMotorFreeIcon(false);
      waitway = 0;
      break;

    case E0FlowKey:
      planner.flow_percentage[0] = recdat.data[0];
      RTS_SndData(recdat.data[0], E0_SET_FLOW_VP);
      if(!printingIsActive()){
        settings.save();
      }
      break;

    case HeaterLoadEnterKey:
      FilamentLOAD = ((float)recdat.data[0]) / 10;
      RTS_SndData(10 * FilamentLOAD, HEAD_FILAMENT_LOAD_DATA_VP);
      if(!planner.has_blocks_queued())
      {
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if ((1 == READ(FIL_RUNOUT_PIN)) && (runout.enabled == true))
          {
            RTS_ShowPage(46);
            break;
          }
        #endif
        current_position[E_AXIS] += FilamentLOAD;

        if((thermalManager.temp_hotend[0].target > EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (thermalManager.temp_hotend[0].celsius - 5)))
        {
          RTS_SendHeadTemp();
        }
        else if((thermalManager.temp_hotend[0].target < EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (ChangeFilamentTemp - 5)))
        {
          thermalManager.setTargetHotend(ChangeFilamentTemp, 0);
          RTS_SndData(ChangeFilamentTemp, HEAD_SET_TEMP_VP);
        }
        
        while(ABS(thermalManager.degHotend(0) - thermalManager.degTargetHotend(0)) > TEMP_WINDOW)
        {
          idle();
        }

        {
          RTS_line_to_current(E_AXIS);
          RTS_SndData(10 * FilamentLOAD, HEAD_FILAMENT_LOAD_DATA_VP);
          planner.synchronize();
        }
      }
      break;

    case HeaterUnLoadEnterKey:
      FilamentUnLOAD = ((float)recdat.data[0]) / 10;
      RTS_SndData(10 * FilamentUnLOAD, HEAD_FILAMENT_UNLOAD_DATA_VP);
      if(!planner.has_blocks_queued())
      {
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if ((1 == READ(FIL_RUNOUT_PIN)) && (runout.enabled == true))
          {
            RTS_ShowPage(46);
            break;
          }
        #endif

        current_position[E_AXIS] -= FilamentUnLOAD;

        if((thermalManager.temp_hotend[0].target > EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (thermalManager.temp_hotend[0].celsius - 5)))
        {
          RTS_SendHeadTemp();
        }
        else if((thermalManager.temp_hotend[0].target < EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (ChangeFilamentTemp - 5)))
        {
          thermalManager.setTargetHotend(ChangeFilamentTemp, 0);
          RTS_SndData(ChangeFilamentTemp, HEAD_SET_TEMP_VP);
        }
        while(ABS(thermalManager.degHotend(0) - thermalManager.degTargetHotend(0)) > TEMP_WINDOW)
        {
          idle();
        }

        {
          RTS_line_to_current(E_AXIS);
          RTS_SndData(10 * FilamentUnLOAD, HEAD_FILAMENT_UNLOAD_DATA_VP);
          planner.synchronize();
        }
      }
      break;

    case HeaterLoadStartKey:
      rtscheck.RTS_SendLoadedData(6);    
      if(recdat.data[0] == 1)
      {
        if(!planner.has_blocks_queued())
        {
          #if ENABLED(FILAMENT_RUNOUT_SENSOR)
            if ((1 == READ(FIL_RUNOUT_PIN)) && (runout.enabled == true))
            {
              RTS_ShowPage(46);
              break;
            }
            else if(rts_start_print)
            {
              RTS_ShowPage(1);
              break;
            }
          #endif

          if((thermalManager.temp_hotend[0].target > EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (thermalManager.temp_hotend[0].celsius - 5)))
          {
            RTS_SendHeadTemp();
            break;
          }
          else if((thermalManager.temp_hotend[0].target < EXTRUDE_MINTEMP) && (thermalManager.temp_hotend[0].celsius < (ChangeFilamentTemp - 5)))
          {
            thermalManager.setTargetHotend(ChangeFilamentTemp, 0);
            RTS_SndData(ChangeFilamentTemp, HEAD_SET_TEMP_VP);
            break;
          }
          else
          {
            RTS_line_to_current(E_AXIS);
            planner.synchronize();
          }
          RTS_ShowPage(19);
        }
      }
      else if(recdat.data[0] == 2)
      {
        if(!planner.has_blocks_queued())
        {
          RTS_ShowPage(1);
        }
      }
      else if(recdat.data[0] == 3)
      {
        RTS_ShowPage(19);
      }
      break;

    case SelectLanguageKey:
      if(recdat.data[0] != 0)
      {
        lang = recdat.data[0];
      }
      language_change_font = lang;
      for(int i = 0;i < 9;i ++)
      {
        RTS_ResetSingleVP(LANGUAGE_CHINESE_TITLE_VP + i);
      }
      RTS_SetOneToVP(LANGUAGE_CHINESE_TITLE_VP + (language_change_font - 1));
      languagedisplayUpdate();
      // settings.save();
      break;

    case PowerContinuePrintKey:
      if(recdat.data[0] == 1)
      {

      #if ENABLED(POWER_LOSS_RECOVERY)
        //if(recovery.recovery_flag && PoweroffContinue)
        //{
          PoweroffContinue = true;
          power_off_type_yes = true;
          Update_Time_Value = 0;
          RTS_LoadMainsiteIcons();          
          RTS_ShowPage(10);
          #if ENABLED(GCODE_PREVIEW_ENABLED)
            RTS_ShowPreviewImage(false);
            int32_t ret = gcodePicDataSendToDwin(recovery.info.sd_filename,VP_OVERLAY_PIC_PTINT,PIC_FORMAT_JPG, PIC_RESOLUTION_250_250);
            if (ret == PIC_OK) {
              RTS_ShowPreviewImage(false);
            } else {              
              RTS_ShowPreviewImage(true);
            }
            rtscheck.RTS_SndData(picLayers, PRINT_LAYERS_VP);
            rtscheck.RTS_SndData(picFilament_m, PRINT_FILAMENT_M_VP);
            rtscheck.RTS_SndData(picFilament_g, PRINT_FILAMENT_G_VP);
            rtscheck.RTS_SndData(picLayerHeight * 100, PRINT_LAYER_HEIGHT_VP);            
          #endif
          for(uint16_t i = 0;i < CardRecbuf.Filesum;i ++) 
          {
            if(!strcmp(CardRecbuf.Cardfilename[i], &recovery.info.sd_filename[1]))
            {
              if (CardRecbuf.filenamelen[i] > 25){
                rtscheck.RTS_SndData(CardRecbuf.Cardshowfilename[i], SELECT_FILE_TEXT_VP);
              }else{
                rtscheck.RTS_SndData(CardRecbuf.Cardshowfilename[i], PRINT_FILE_TEXT_VP);
              }
              #if ENABLED(LCD_RTS_DEBUG_SDCARD)
                SERIAL_ECHO_MSG("CardRecbuf.Cardshowfilename[i]", CardRecbuf.Cardshowfilename[i]);
              #endif
            }
          }
          queue.enqueue_now_P(PSTR("M1000"));
          sdcard_pause_check = true;
          zprobe_zoffset = probe.offset.z;
          RTS_SendZoffsetFeedratePercentage(true);
        //}
      #endif
      }
      else if(recdat.data[0] == 2)
      {
        Update_Time_Value = RTS_UPDATE_VALUE;
        RTS_SDcard_Stop();
        RTS_ShowPage(1);        
        Update_Time_Value = 0;
      }
      break;

    case PLAHeadSetEnterKey:
      temp_preheat_nozzle = recdat.data[0];
      RTS_SndData(temp_preheat_nozzle, PREHEAT_PLA_SET_NOZZLE_TEMP_VP);
      break;

    case PLABedSetEnterKey:
      temp_preheat_bed = recdat.data[0];
      RTS_SndData(temp_preheat_bed, PREHEAT_PLA_SET_BED_TEMP_VP);     
      break;

    case ABSHeadSetEnterKey:
      temp_preheat_nozzle = recdat.data[0];
      RTS_SndData(temp_preheat_nozzle, PREHEAT_ABS_SET_NOZZLE_TEMP_VP);
      break;

    case ABSBedSetEnterKey:
      temp_preheat_bed = recdat.data[0];
      RTS_SndData(temp_preheat_bed, PREHEAT_ABS_SET_BED_TEMP_VP);
      break;

    case PETGHeadSetEnterKey:
      temp_preheat_nozzle = recdat.data[0];
      RTS_SndData(temp_preheat_nozzle, PREHEAT_PETG_SET_NOZZLE_TEMP_VP);
      break;

    case PETGBedSetEnterKey:
      temp_preheat_bed = recdat.data[0];
      RTS_SndData(temp_preheat_bed, PREHEAT_PETG_SET_BED_TEMP_VP);     
      break;

    case CUSTHeadSetEnterKey:
      temp_preheat_nozzle = recdat.data[0];
      RTS_SndData(temp_preheat_nozzle, PREHEAT_CUST_SET_NOZZLE_TEMP_VP);
      break;

    case CUSTBedSetEnterKey:
      temp_preheat_bed = recdat.data[0];
      RTS_SndData(temp_preheat_bed, PREHEAT_CUST_SET_BED_TEMP_VP);
      break;

    if (leveling_running == 0 && !printingIsActive()){   
      case SetGridMaxPoints: 
        {
          temp_grid_max_points = recdat.data[0];
          RTS_SetGridMaxPoints(temp_grid_max_points, 0);
        }    
        break;

        case SetProbeMarginX: 
        {
          temp_probe_margin_x = recdat.data[0];
          RTS_SetProbeMarginX(temp_probe_margin_x, 0);
        }    
        break;      

        case SetProbeMarginY: 
        {
          temp_probe_margin_y = recdat.data[0];
          RTS_SetProbeMarginY(temp_probe_margin_y, 0);
        }    
        break;      

      case SetProbeCount: 
        {
          temp_grid_probe_count = recdat.data[0];
          RTS_SetProbeCount(temp_grid_probe_count, 0);
        }    
        break;
    }
    case StoreMemoryKey:
      // Motion
      if(recdat.data[0] == 1)
      {
        RTS_SndData(planner.extruder_advance_K[0] * 1000, ADVANCE_K_SET);
        RTS_ShowPage(34); 
      } 
      // RX
      else if(recdat.data[0] == 2)
      {
        RTS_ShowPage(38);
      }
      // Acceleration
      else if(recdat.data[0] == 3)
      {
        rtscheck.RTS_SendLoadedData(5);        
        RTS_ShowPage(36);
      }
      // Jerk
      else if(recdat.data[0] == 4)
      {    
        RTS_ShowPage(37);  
      }
      // Feedrate
      else if(recdat.data[0] == 5)
      {
        RTS_ShowPage(35);
      }      
      // Manual PID
      else if(recdat.data[0] == 6)
      {  
        RTS_ShowPage(39);
      }
      // Device and save
      else if(recdat.data[0] == 7)
      {
        RTS_ShowPage(33);
        settings.save();
        delay(100);
      } // Leave to Motion and save
      else if(recdat.data[0] == 8)
      {
        RTS_ShowPage(34);
        settings.save();
        delay(100);
      }
      else if(recdat.data[0] == 9)
      { // Leave to device and save
        if(!printingIsActive() && leveling_running == 0){
          RTS_ShowPage(21);
          settings.save();
          delay(100);
        }
      }        
      else if (recdat.data[0] == 161)
      { // 00A1
         g_autoPIDHeaterTempTargetset = g_autoPIDHeaterTempTargetset;     
        if (g_autoPIDHeaterTempTargetset != 0) { 
         g_autoPIDHeaterTempTarget =  g_autoPIDHeaterTempTargetset;  
        }
        g_autoPIDHeaterCyclesTargetset = g_autoPIDHeaterCyclesTargetset;
        if (g_autoPIDHeaterCyclesTargetset != 0) {        
         g_autoPIDHeaterCycles =  g_autoPIDHeaterCyclesTargetset;  
        }                                     
        char cmd[30];
        g_uiAutoPIDNozzleRunningFlag = true;
        g_uiAutoPIDRunningDiff = 1;
        g_uiCurveDataCnt = 0;
        RTS_SndData(lang + 10, AUTO_PID_START_NOZZLE_VP);
        RTS_ResetSingleVP(AUTO_PID_NOZZLE_TIS_VP);
        RTS_SendLang(AUTO_PID_RUN_NOZZLE_TIS_VP);
        memset(cmd, 0, sizeof(cmd));
        sprintf_P(cmd, PSTR("M303 E0 C%d S%d"), g_autoPIDHeaterCycles, g_autoPIDHeaterTempTarget);
        gcode.process_subcommands_now(cmd);
        PID_PARAM(Kp, 0) = g_autoPID.p;
        PID_PARAM(Ki, 0) = scalePID_i(g_autoPID.i);
        PID_PARAM(Kd, 0) = scalePID_d(g_autoPID.d);     
        RTS_SndData(lang + 10, AUTO_PID_RUN_NOZZLE_TIS_VP);
        RTS_SndData(PID_PARAM(Kp, 0) * 100, NOZZLE_TEMP_P_DATA_VP);
        RTS_SndData(unscalePID_i(PID_PARAM(Ki, 0)) * 100, NOZZLE_TEMP_I_DATA_VP);
        RTS_SndData(unscalePID_d(PID_PARAM(Kd, 0)) * 100, NOZZLE_TEMP_D_DATA_VP);
        hal.watchdog_refresh();
        settings.save();
        delay(1000);
        g_uiAutoPIDRunningDiff = 0;
        RTS_SendLang(AUTO_PID_START_NOZZLE_VP);
        g_uiAutoPIDNozzleRunningFlag = false;
      } 
      else if (recdat.data[0] == 162)
      {    
        g_autoPIDHotBedTempTargetset = g_autoPIDHotBedTempTargetset;
        if (g_autoPIDHotBedTempTargetset != 0) { 
         g_autoPIDHotBedTempTarget =  g_autoPIDHotBedTempTargetset;  
        }
        g_autoPIDHotBedCyclesTargetset = g_autoPIDHotBedCyclesTargetset;
        if (g_autoPIDHotBedCyclesTargetset != 0) { 
         g_autoPIDHotBedCycles =  g_autoPIDHotBedCyclesTargetset;  
        }
        g_uiAutoPIDHotbedRunningFlag = true;
        g_uiAutoPIDRunningDiff = 2;
        g_uiCurveDataCnt = 0;
        char cmd[30];
        RTS_SndData(lang + 10, AUTO_PID_START_HOTBED_VP);
        RTS_ResetSingleVP(AUTO_PID_HOTBED_TIS_VP);
        RTS_SendLang(AUTO_PID_RUN_HOTBED_TIS_VP);
        memset(cmd, 0, sizeof(cmd));
        sprintf_P(cmd, PSTR("M303 E-1 C%d S%d"), g_autoPIDHotBedCycles, g_autoPIDHotBedTempTarget);
        gcode.process_subcommands_now(cmd);
        thermalManager.temp_bed.pid.Kp = g_autoPID.p;
        thermalManager.temp_bed.pid.Ki = scalePID_i(g_autoPID.i);
        thermalManager.temp_bed.pid.Kd = scalePID_d(g_autoPID.d);
        RTS_SndData(lang + 10, AUTO_PID_RUN_HOTBED_TIS_VP);
        RTS_SndData(thermalManager.temp_bed.pid.Kp * 100, HOTBED_TEMP_P_DATA_VP);
        RTS_SndData(unscalePID_i(thermalManager.temp_bed.pid.Ki) * 100, HOTBED_TEMP_I_DATA_VP);
        RTS_SndData(unscalePID_d(thermalManager.temp_bed.pid.Kd) * 10, HOTBED_TEMP_D_DATA_VP);
        hal.watchdog_refresh();
        settings.save();
        delay(1000);
        g_uiAutoPIDRunningDiff = 0;
        RTS_SendLang(AUTO_PID_START_HOTBED_VP);
        g_uiAutoPIDHotbedRunningFlag = false;
      }
      else if(recdat.data[0] == 163)
      { // 00A3
        // Jail
        if(g_uiAutoPIDNozzleRunningFlag == true) break;          
        if(g_uiAutoPIDHotbedRunningFlag == true) break;   
        RTS_ShowPage(85);
      }
      break;

    case FanSpeedEnterKey:
      thermalManager.fan_speed[0] = recdat.data[0];
      RTS_SndData(thermalManager.fan_speed[0], FAN_SPEED_CONTROL_DATA_VP);
      break;

    case VelocityXaxisEnterKey:
      float velocity_xaxis;
      velocity_xaxis = planner.settings.max_feedrate_mm_s[0];
      velocity_xaxis = recdat.data[0];
      RTS_SndData(velocity_xaxis, MAX_VELOCITY_XAXIS_DATA_VP);
      planner.set_max_feedrate(X_AXIS, velocity_xaxis);
      break;

    case VelocityYaxisEnterKey:
      float velocity_yaxis;
      velocity_yaxis = planner.settings.max_feedrate_mm_s[1];
      velocity_yaxis = recdat.data[0];
      RTS_SndData(velocity_yaxis, MAX_VELOCITY_YAXIS_DATA_VP);
      planner.set_max_feedrate(Y_AXIS, velocity_yaxis);
      break;

    case VelocityZaxisEnterKey:
      float velocity_zaxis;
      velocity_zaxis = planner.settings.max_feedrate_mm_s[2];
      velocity_zaxis = recdat.data[0];
      RTS_SndData(velocity_zaxis, MAX_VELOCITY_ZAXIS_DATA_VP);
      planner.set_max_feedrate(Z_AXIS, velocity_zaxis);
      break;

    case VelocityEaxisEnterKey:
      float velocity_eaxis;
      velocity_eaxis = planner.settings.max_feedrate_mm_s[3];
      velocity_eaxis = recdat.data[0];
      RTS_SndData(velocity_eaxis, MAX_VELOCITY_EAXIS_DATA_VP);
      planner.set_max_feedrate(E_AXIS, velocity_eaxis);
      break;

    case AccelXaxisEnterKey:
      float accel_xaxis;
      accel_xaxis = planner.settings.max_acceleration_mm_per_s2[0];
      accel_xaxis = recdat.data[0];
      RTS_SndData(accel_xaxis, MAX_ACCEL_XAXIS_DATA_VP);
      planner.set_max_acceleration(X_AXIS, accel_xaxis);
      break;

    case AccelYaxisEnterKey:
      float accel_yaxis;
      accel_yaxis = planner.settings.max_acceleration_mm_per_s2[1];
      accel_yaxis = recdat.data[0];
      RTS_SndData(accel_yaxis, MAX_ACCEL_YAXIS_DATA_VP);
      planner.set_max_acceleration(Y_AXIS, accel_yaxis);
      break;

    case AccelZaxisEnterKey:
      float accel_zaxis;
      accel_zaxis = planner.settings.max_acceleration_mm_per_s2[2];
      accel_zaxis = recdat.data[0];
      RTS_SndData(accel_zaxis, MAX_ACCEL_ZAXIS_DATA_VP);
      planner.set_max_acceleration(Z_AXIS, accel_zaxis);
      break;

    case AccelEaxisEnterKey:
      float accel_eaxis;
      accel_eaxis = planner.settings.max_acceleration_mm_per_s2[3];
      accel_eaxis = recdat.data[0];
      RTS_SndData(accel_eaxis, MAX_ACCEL_EAXIS_DATA_VP);
      planner.set_max_acceleration(E_AXIS, accel_eaxis);
      break;

    case JerkXaxisEnterKey:
      float jerk_xaxis;
      jerk_xaxis = planner.max_jerk.x;
      jerk_xaxis = (float)recdat.data[0] / 100;
      RTS_SndData(jerk_xaxis * 100, MAX_JERK_XAXIS_DATA_VP);
      planner.set_max_jerk(X_AXIS, jerk_xaxis);
      break;

    case JerkYaxisEnterKey:
      float jerk_yaxis;
      jerk_yaxis = planner.max_jerk.y;
      jerk_yaxis = (float)recdat.data[0] / 100;
      RTS_SndData(jerk_yaxis * 100, MAX_JERK_YAXIS_DATA_VP);
      planner.set_max_jerk(Y_AXIS, jerk_yaxis);
      break;

    case JerkZaxisEnterKey:
      float jerk_zaxis;
      jerk_zaxis = planner.max_jerk.z;
      jerk_zaxis = (float)recdat.data[0] / 100;
      RTS_SndData(jerk_zaxis * 100, MAX_JERK_ZAXIS_DATA_VP);
      planner.set_max_jerk(Z_AXIS, jerk_zaxis);
      break;

    case JerkEaxisEnterKey:
      float jerk_eaxis;
      jerk_eaxis = planner.max_jerk.e;
      jerk_eaxis = (float)recdat.data[0] / 100;
      RTS_SndData(jerk_eaxis * 100, MAX_JERK_EAXIS_DATA_VP);
      planner.set_max_jerk(E_AXIS, jerk_eaxis);
      break;

    case StepsmmXaxisEnterKey:
      float stepsmm_xaxis;
      stepsmm_xaxis = planner.settings.axis_steps_per_mm[0];
      stepsmm_xaxis = (float)recdat.data[0] / 10;
      RTS_SndData(stepsmm_xaxis * 10, MAX_STEPSMM_XAXIS_DATA_VP);
      planner.settings.axis_steps_per_mm[X_AXIS] = stepsmm_xaxis;
      break;

    case StepsmmYaxisEnterKey:
      float stepsmm_yaxis;
      stepsmm_yaxis = planner.settings.axis_steps_per_mm[1];
      stepsmm_yaxis = (float)recdat.data[0] / 10;
      RTS_SndData(stepsmm_yaxis * 10, MAX_STEPSMM_YAXIS_DATA_VP);
      planner.settings.axis_steps_per_mm[Y_AXIS] = stepsmm_yaxis;
      break;

    case StepsmmZaxisEnterKey:
      float stepsmm_zaxis;
      stepsmm_zaxis = planner.settings.axis_steps_per_mm[2];
      stepsmm_zaxis = (float)recdat.data[0] / 10;
      RTS_SndData(stepsmm_zaxis * 10, MAX_STEPSMM_ZAXIS_DATA_VP);
      planner.settings.axis_steps_per_mm[Z_AXIS] = stepsmm_zaxis;
      break;

    case StepsmmEaxisEnterKey:
      float stepsmm_eaxis;
      stepsmm_eaxis = planner.settings.axis_steps_per_mm[3];
      stepsmm_eaxis = (float)recdat.data[0] / 10;
      RTS_SndData(stepsmm_eaxis * 10, MAX_STEPSMM_EAXIS_DATA_VP);
      planner.settings.axis_steps_per_mm[E_AXIS] = stepsmm_eaxis;
      break;

    case NozzlePTempEnterKey:
      float nozzle_ptemp;
      nozzle_ptemp = (float)recdat.data[0] / 100;
      RTS_SndData(nozzle_ptemp * 100, NOZZLE_TEMP_P_DATA_VP);
      thermalManager.temp_hotend[0].pid.set_Kp(nozzle_ptemp);
      break;

    case NozzleITempEnterKey:
      float nozzle_itemp;
      nozzle_itemp = (float)recdat.data[0] / 100;
      RTS_SndData(nozzle_itemp * 100, NOZZLE_TEMP_I_DATA_VP);
      thermalManager.temp_hotend[0].pid.set_Ki(nozzle_itemp);
      break;

    case NozzleDTempEnterKey:
      float nozzle_dtemp;
      nozzle_dtemp = (float)recdat.data[0] / 100;
      RTS_SndData(nozzle_dtemp * 100, NOZZLE_TEMP_D_DATA_VP);
      thermalManager.temp_hotend[0].pid.set_Kd(nozzle_dtemp);
      break;

    case HotbedPTempEnterKey:
      float hotbed_ptemp;
      hotbed_ptemp = (float)recdat.data[0] / 100;
      RTS_SndData(hotbed_ptemp * 100, HOTBED_TEMP_P_DATA_VP);
      thermalManager.temp_bed.pid.set_Kp(hotbed_ptemp);
      break;

    case HotbedITempEnterKey:
      float hotbed_itemp;
      hotbed_itemp = (float)recdat.data[0] / 100;
      RTS_SndData(hotbed_itemp * 100, HOTBED_TEMP_I_DATA_VP);
      thermalManager.temp_bed.pid.set_Ki(hotbed_itemp);      
      break;

    case HotbedDTempEnterKey:
      float hotbed_dtemp;
      hotbed_dtemp = (float)recdat.data[0] / 10;
      RTS_SndData(hotbed_dtemp * 10, HOTBED_TEMP_D_DATA_VP);
      thermalManager.temp_bed.pid.set_Kd(hotbed_dtemp);      
      break;

    case PrintFanSpeedkey:
      uint8_t fan_speed;
      fan_speed = (uint8_t)recdat.data[0];
      RTS_SndData(fan_speed , PRINTER_FAN_SPEED_DATA_VP);
      thermalManager.set_fan_speed(0, fan_speed);
      break;

    //case HotendFanSpeedkey:
    //  //uint8_t material_fan_speed;
    //  lcd_rts_settings.hotend_fan = (uint8_t)recdat.data[0];
    //  RTS_SndData(lcd_rts_settings.hotend_fan , HOTEND_FAN_SPEED_DATA_VP);
    //  //thermalManager.set_fan_speed(0, hotend_fan_speed);
    //  break;

    case AutopidSetNozzleTemp:       
      g_autoPIDHeaterTempTargetset = recdat.data[0];
      RTS_SndData(g_autoPIDHeaterTempTargetset, AUTO_PID_SET_NOZZLE_TEMP);
      break;

    case AutopidSetNozzleCycles:          
      g_autoPIDHeaterCyclesTargetset = recdat.data[0];
      RTS_SndData(g_autoPIDHeaterCyclesTargetset, AUTO_PID_SET_NOZZLE_CYCLES);
      break;    

    case AutopidSetHotbedTemp:       
      g_autoPIDHotBedTempTargetset = recdat.data[0];
      RTS_SndData(g_autoPIDHotBedTempTargetset, AUTO_PID_SET_HOTBED_TEMP);   
      break;

    case AutopidSetHotbedCycles:      
      g_autoPIDHotBedCyclesTargetset = recdat.data[0];
      RTS_SndData(g_autoPIDHotBedCyclesTargetset, AUTO_PID_SET_HOTBED_CYCLES);           
      break;        
    
    case Advance_K_Key:  
      planner.extruder_advance_K[0] = ((float)recdat.data[0])/1000;
      RTS_SndData(planner.extruder_advance_K[0] * 1000, ADVANCE_K_SET);
      if(!printingIsActive()){
        settings.save();
      }      
      break;
    case XShapingFreqsetEnterKey:
      stepper.set_shaping_frequency(X_AXIS, (float)recdat.data[0]/100);      
      RTS_SndData(stepper.get_shaping_frequency(X_AXIS) * 100, SHAPING_X_FREQUENCY_VP);
      if(!printingIsActive()){
        settings.save();
      }
      break;

    case YShapingFreqsetEnterKey:
      stepper.set_shaping_frequency(Y_AXIS, (float)recdat.data[0]/100);      
      RTS_SndData(stepper.get_shaping_frequency(Y_AXIS) * 100, SHAPING_Y_FREQUENCY_VP);
      if(!printingIsActive()){
        settings.save();
      }     
      break;

    case XShapingZetasetEnterKey:  
      stepper.set_shaping_damping_ratio(X_AXIS, (float)recdat.data[0]/100);      
      RTS_SndData(stepper.get_shaping_damping_ratio(X_AXIS) * 100, SHAPING_X_ZETA_VP);
      if(!printingIsActive()){
        settings.save();
      }
      break;

    case YShapingZetasetEnterKey:  
      stepper.set_shaping_damping_ratio(Y_AXIS, (float)recdat.data[0]/100);      
      RTS_SndData(stepper.get_shaping_damping_ratio(Y_AXIS) * 100, SHAPING_Y_ZETA_VP);
      if(!printingIsActive()){
        settings.save();
      }
      break;   

    case XMinPosEepromEnterKey:
      float home_offset_x_temp;
      if(recdat.data[0] >= 32768)
      {
        home_offset_x_temp = ((float)recdat.data[0] - 65536) / 10;
        home_offset_x_temp -= 0.001;
      }
      else
      {
        home_offset_x_temp = ((float)recdat.data[0])/10;;
        home_offset_x_temp += 0.001;
      }
      home_offset.x = home_offset_x_temp;
      RTS_SndData(home_offset.x * 10, HOME_X_OFFSET_VP);
      settings.save();
      break;
      
    case YMinPosEepromEnterKey:
      float home_offset_y_temp;
      if(recdat.data[0] >= 32768)
      {
        home_offset_y_temp = ((float)recdat.data[0] - 65536) / 10;
        home_offset_y_temp -= 0.001;
      }
      else
      {
        home_offset_y_temp = ((float)recdat.data[0])/10;;
        home_offset_y_temp += 0.001;
      }
      home_offset.y = home_offset_y_temp;     
      RTS_SndData(home_offset.y * 10, HOME_Y_OFFSET_VP);      
      settings.save();      
      break;                     

    case Volume:
      if (recdat.data[0] <= 32){
        lcd_rts_settings.display_volume = 32;
        lcd_rts_settings.display_sound = false;        
      }else if (recdat.data[0] > 255){
        lcd_rts_settings.display_volume = 0xFF;
        lcd_rts_settings.display_sound = true;              
      }else{
        lcd_rts_settings.display_volume = recdat.data[0];
        lcd_rts_settings.display_sound = true;        
      }
      setTouchScreenConfiguration();      
      break;

    case VolumeDisplay:
    if (lcd_rts_settings.display_volume == 255){
      if (recdat.data[0] == 0) {
        lcd_rts_settings.display_volume = 0;
        lcd_rts_settings.display_sound = false;
        RTS_SndData(191, VOLUME_DISPLAY);        
      }
    }else{
        lcd_rts_settings.display_volume = 255;
        lcd_rts_settings.display_sound = true;
        RTS_SndData(192, VOLUME_DISPLAY);         
    }
    setTouchScreenConfiguration();
    break;

    case DisplayBrightness:
      if (recdat.data[0] <= 20){
        lcd_rts_settings.screen_brightness = 20;
      }else if (recdat.data[0] > 100){
        lcd_rts_settings.screen_brightness = 100;
      }else{
        lcd_rts_settings.screen_brightness = (uint8_t)recdat.data[0];
      }
      setTouchScreenConfiguration();
      break;

    case DisplayStandbyBrightness:
      if (recdat.data[0] < 20)
        lcd_rts_settings.standby_brightness = 20;
      else if (recdat.data[0] > 100)
        lcd_rts_settings.standby_brightness = 100;
      else
        lcd_rts_settings.standby_brightness = (uint8_t)recdat.data[0];
      setTouchScreenConfiguration();
      break;

    case DisplayStandbySeconds:
      if (recdat.data[0] < 5)
        lcd_rts_settings.standby_time_seconds = 5;
      else if (recdat.data[0] > 600)
        lcd_rts_settings.standby_time_seconds = 600;
      else
        lcd_rts_settings.standby_time_seconds = (uint8_t)recdat.data[0];
      setTouchScreenConfiguration();
      break;

    case ExternalMToggle:
      if (recdat.data[0] == 1){
        if (!lcd_rts_settings.external_m73){
            lcd_rts_settings.external_m73 = true;
            RTS_SendM73Icon(true);
        }
        else{
          lcd_rts_settings.external_m73 = false;
          RTS_SendM73Icon(false);
        }
      }else if (recdat.data[0] == 2){
        if (printingIsActive() && planner.has_blocks_queued()) {        
          queue.inject(F(FILAMENT_RUNOUT_SCRIPT));          
          delay(2);
        }
      }
      break;

    case EditMeshpoint:
    {
      if (leveling_running == 0 && bedlevel.mesh_is_valid() && !printingIsActive()){      
        current_point = recdat.data[0] - 1;
        uint8_t x_probe_point, y_probe_point;
        calculateProbePoints(current_point, x_probe_point, y_probe_point);
        RTS_LoadMeshPointOffsets();
        if (current_point >= 0 && current_point <= 100) {
          uint16_t upperLeftX = (y_probe_point % 2 == 0) 
                              ? rect_0_x_top_even + (x_probe_point * rect_x_offset)
                              : rect_1_x_top_odd - ((lcd_rts_settings.max_points - 1 - x_probe_point) * rect_x_offset);
          uint16_t upperLeftY = rect_0_y_top - (y_probe_point * rect_y_offset);
          sendRectangleCommand(0x2490, upperLeftX, upperLeftY, rectWidth, rectHeight, 0x75DE);
          RTS_SndData(bedlevel.z_values[x_probe_point][y_probe_point] * 1000, CURRENT_MESH_POINT);
        }
      }
    }
    break;

    case CurrentMeshpoint: 
    {
      if (leveling_running == 0 && bedlevel.mesh_is_valid() && !printingIsActive()){
        if (current_point >= 0 && current_point <= 100) {
            float new_point_height = (recdat.data[0] >= 32768)
                                    ? ((float)recdat.data[0] - 65536) / 1000
                                    : ((float)recdat.data[0]) / 1000;
            RTS_SndData(new_point_height * 1000, current_point == 0 
                                                ? AUTO_BED_LEVEL_1POINT_NEW_VP 
                                                : AUTO_BED_LEVEL_1POINT_NEW_VP + current_point * 2);
            uint8_t x_probe_point, y_probe_point;
            calculateProbePoints(current_point, x_probe_point, y_probe_point);
            char cmd_point[39];
            const char* sign = (new_point_height < 0) ? "-" : "";
            int intPart = abs(static_cast<int>(new_point_height));
            int fracPart = abs(static_cast<int>((new_point_height - static_cast<int>(new_point_height)) * 1000));
            snprintf(cmd_point, sizeof(cmd_point), "M421 I%d J%d Z%s%d.%03d", static_cast<int>(x_probe_point), static_cast<int>(y_probe_point), sign, intPart, fracPart);
            queue.enqueue_now_P(cmd_point);
        }
      }
    }
    break;

    case SelectFileKey:
      if (RTS_SD_Detected()) {
        if (recdat.data[0] > CardRecbuf.Filesum) break;
        CardRecbuf.recordcount = recdat.data[0] - 1;
        std::string filename = CardRecbuf.Cardfilename[CardRecbuf.recordcount];
        // Find the last occurrence of the '.' character in the filename
        std::size_t dot_pos = filename.find_last_of('.');

        if (dot_pos == std::string::npos) {
          card.cd(CardRecbuf.Cardfilename[CardRecbuf.recordcount]);
          int16_t fileCnt = card.get_num_items();
          card.getWorkDirName();
          if (fileCnt > 0) {
            file_total_page = fileCnt / 5;
            if (fileCnt % 5 > 0) { // Add an extra page only if there are leftover files
              file_total_page++;
            }
            if (file_total_page > 8) file_total_page = 8; // Limit the maximum number of pages
          }
          else {
            file_total_page = 1;
          }
          RTS_SndData(file_total_page, PAGE_STATUS_TEXT_TOTAL_VP);
          file_current_page = 1;
          RTS_SndData(file_current_page, PAGE_STATUS_TEXT_CURRENT_VP);
          RTS_line_to_filelist();
          CardRecbuf.selectFlag = false;
          if (PoweroffContinue /*|| print_job_timer.isRunning()*/) return;
          // clean print file
          RTS_CleanPrintAndSelectFile();
          lcd_sd_status = IS_SD_INSERTED();
        }
        else {
          CardRecbuf.selectFlag = true;
          CardRecbuf.recordcount = recdat.data[0] - 1;
          RTS_CleanPrintAndSelectFile();          
          delay(2);
          RTS_SndData((unsigned long)0xFFFF, FilenameNature + recdat.data[0] * 16);      
          RTS_ShowPage(1);
          #if ENABLED(GCODE_PREVIEW_ENABLED)
            char ret;
            RTS_ShowPreviewImage(false);
            ret = gcodePicDataSendToDwin(CardRecbuf.Cardfilename[CardRecbuf.recordcount],VP_OVERLAY_PIC_PTINT,PIC_FORMAT_JPG, PIC_RESOLUTION_250_250);
            #if ENABLED(LCD_RTS_DEBUG_SDCARD)
              SERIAL_ECHO_MSG("Loaded filename = ", CardRecbuf.Cardfilename[CardRecbuf.recordcount]);
            #endif
            if (ret == PIC_OK) {
              #if ENABLED(LCD_RTS_DEBUG_SDCARD)
                SERIAL_ECHO_MSG("picLen lcd_rts = ", picLen);
                SERIAL_ECHO_MSG("picStartLine lcd_rts = ", picStartLine);
                SERIAL_ECHO_MSG("picEndLine lcd_rts = ", picEndLine);
                SERIAL_ECHO_MSG("picFilament_m lcd_rts = ", picFilament_m);
                SERIAL_ECHO_MSG("picFilament_g lcd_rts = ", picFilament_g);
                SERIAL_ECHO_MSG("picLayerHeight lcd_rts = ", picLayerHeight);
                SERIAL_ECHO_MSG("picLayers lcd_rts = ", picLayers);
              #endif
              RTS_ResetPrintData(false);
              RTS_SendPrintData();
            } else {
              picLen = 0;
              RTS_ResetPrintData(true);
              RTS_ResetSingleVP(VP_OVERLAY_PIC_PTINT); 
            }
          #endif

          rts_start_print = true;
          delay(5);
          if (CardRecbuf.filenamelen[CardRecbuf.recordcount] > 25){
            #if ENABLED(LCD_RTS_DEBUG_SDCARD)
              SERIAL_ECHO_MSG("CardRecbuf.filenamelen[CardRecbuf.recordcount] select", CardRecbuf.filenamelen[CardRecbuf.recordcount]);
            #endif
            RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], SELECT_FILE_TEXT_VP);
          }else{
            #if ENABLED(LCD_RTS_DEBUG_SDCARD) 
              SERIAL_ECHO_MSG("CardRecbuf.filenamelen[CardRecbuf.recordcount] print", CardRecbuf.filenamelen[CardRecbuf.recordcount]);
            #endif
            RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);            
          }
          #if ENABLED(GCODE_PREVIEW_ENABLED)          
            RefreshBrightnessAtPrint(0);
          #endif
        }
      }
      break;

    case SaveM503Settings:
      if (recdat.data[0] == 1 && RTS_SD_Detected()) {
        card.removeFile(settings_filename2);
        card.openFileWrite(settings_filename2);
        if (!card.isFileOpen()) {
          #if ENABLED(LCD_RTS_DEBUG_SDCARD)
            SERIAL_ECHO_MSG(F("Failed to open output file"));
          #endif
          return;
        }

        const char* commands[] = {
            "M92", "M201", "M203", "M204", "M205", "M206", "M301", "M304", "M593 X", "M593 Y", "M851", "M900", "M19", "M19"
        };
        // Define the buffer to hold the command string
        char buffer[100];
        // Loop through the array and generate/write the command strings
        for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); i++) {
            //SERIAL_ECHO_MSG("commands[i]: ", commands[i]);
            // Use snprintf to format the command string with variables
            snprintf(buffer, sizeof(buffer), "%s", commands[i]);
            card.write(buffer, strlen(buffer));
            if (i == 0) {
                //M92
                char valueStr1[10];
                dtostrf(planner.settings.axis_steps_per_mm[0], 4, 1, valueStr1);
                char valueStr2[10];
                dtostrf(planner.settings.axis_steps_per_mm[1], 4, 1, valueStr2);
                char valueStr3[10];
                dtostrf(planner.settings.axis_steps_per_mm[2], 4, 1, valueStr3);
                char valueStr4[10];
                dtostrf(planner.settings.axis_steps_per_mm[3], 4, 1, valueStr4);        
                snprintf(buffer, sizeof(buffer), " X%s Y%s Z%s E%s", valueStr1, valueStr2, valueStr3, valueStr4);
                card.write(buffer, strlen(buffer));
            } else if (i == 1) {
                // M201
                char valueStr10[11]; // 11 characters to accommodate a 10-digit uint32_t and null terminator
                snprintf(valueStr10, sizeof(valueStr10), "%lu", planner.settings.max_acceleration_mm_per_s2[0]);
                char valueStr11[11]; // 11 characters to accommodate a 10-digit uint32_t and null terminator
                snprintf(valueStr11, sizeof(valueStr11), "%lu", planner.settings.max_acceleration_mm_per_s2[1]);
                char valueStr12[11]; // 11 characters to accommodate a 10-digit uint32_t and null terminator
                snprintf(valueStr12, sizeof(valueStr12), "%lu", planner.settings.max_acceleration_mm_per_s2[2]);
                char valueStr13[11]; // 11 characters to accommodate a 10-digit uint32_t and null terminator
                snprintf(valueStr13, sizeof(valueStr13), "%lu", planner.settings.max_acceleration_mm_per_s2[3]);
                snprintf(buffer, sizeof(buffer), " X%s Y%s Z%s E%s", valueStr10, valueStr11, valueStr12, valueStr13);
                card.write(buffer, strlen(buffer));

            } else if (i == 2) {
                // M203
                char valueStr8[10];
                dtostrf(planner.settings.max_feedrate_mm_s[0], 4, 2, valueStr8);
                char valueStr9[10];
                dtostrf(planner.settings.max_feedrate_mm_s[1], 4, 2, valueStr9);
                char valueStr10[10];
                dtostrf(planner.settings.max_feedrate_mm_s[2], 4, 2, valueStr10);
                char valueStr11[10];
                dtostrf(planner.settings.max_feedrate_mm_s[3], 4, 2, valueStr11); 
                snprintf(buffer, sizeof(buffer), " X%s Y%s Z%s E%s", valueStr8, valueStr9, valueStr10, valueStr11);
                card.write(buffer, strlen(buffer));
            } else if (i == 3) {
                char valueStr12[10];
                dtostrf(planner.settings.acceleration, 4, 2, valueStr12);
                char valueStr13[10];
                dtostrf(planner.settings.retract_acceleration, 4, 2, valueStr13);
                char valueStr14[10];
                dtostrf(planner.settings.travel_acceleration, 4, 2, valueStr14);        
                snprintf(buffer, sizeof(buffer), " P%s R%s T%s", valueStr12, valueStr13, valueStr14);
                card.write(buffer, strlen(buffer));
            } else if (i == 4) {
                char valueStr1[10];
                dtostrf(planner.max_jerk.x, 2, 2, valueStr1);
                char valueStr2[10];
                dtostrf(planner.max_jerk.y, 2, 2, valueStr2);
                char valueStr3[10];
                dtostrf(planner.max_jerk.z, 1, 2, valueStr3);
                char valueStr4[10];
                dtostrf(planner.max_jerk.e, 2, 2, valueStr4);        
                snprintf(buffer, sizeof(buffer), " X%s Y%s Z%s E%s", valueStr1, valueStr2, valueStr3, valueStr4);
                card.write(buffer, strlen(buffer));
            } else if (i == 5) {
                char valueStr1[10];
                dtostrf(home_offset.x, 3, 2, valueStr1);
                char valueStr2[10];
                dtostrf(home_offset.y, 3, 2, valueStr2);
                snprintf(buffer, sizeof(buffer), " X%s Y%s", valueStr1, valueStr2);
                card.write(buffer, strlen(buffer));
            } else if (i == 6) {
                char valueStr1[10];
                dtostrf(thermalManager.temp_hotend[0].pid.p(), 4, 2, valueStr1);
                char valueStr2[10];
                dtostrf(thermalManager.temp_hotend[0].pid.i(), 4, 2, valueStr2);
                char valueStr3[10];
                dtostrf(thermalManager.temp_hotend[0].pid.d(), 4, 2, valueStr3);        
                snprintf(buffer, sizeof(buffer), " P%s I%s D%s", valueStr1, valueStr2, valueStr3);
                card.write(buffer, strlen(buffer));
            } else if (i == 7) {
                char valueStr1[10];
                dtostrf(thermalManager.temp_bed.pid.p(), 4, 2, valueStr1);
                char valueStr2[10];
                dtostrf(thermalManager.temp_bed.pid.i(), 4, 2, valueStr2);
                char valueStr3[10];
                dtostrf(thermalManager.temp_bed.pid.d(), 4, 1, valueStr3);        
                snprintf(buffer, sizeof(buffer), " P%s I%s D%s", valueStr1, valueStr2, valueStr3);
                card.write(buffer, strlen(buffer));
            } else if (i == 8 || i == 9) {
                char valueStr1[10];
                dtostrf(stepper.get_shaping_frequency(i == 8 ? X_AXIS : Y_AXIS), 4, 2, valueStr1);
                char valueStr2[10];
                dtostrf(stepper.get_shaping_damping_ratio(i == 8 ? X_AXIS : Y_AXIS), 4, 2, valueStr2);
                snprintf(buffer, sizeof(buffer), " F%s D%s", valueStr1, valueStr2);
                card.write(buffer, strlen(buffer));
            } else if (i == 10) {
                char valueStr1[10];
                dtostrf(probe.offset.x, 3, 2, valueStr1);
                char valueStr2[10];
                dtostrf(probe.offset.y, 3, 2, valueStr2);
                char valueStr3[10];
                dtostrf(probe.offset.z, 3, 2, valueStr3);                 
                snprintf(buffer, sizeof(buffer), " X%s Y%s Z%s", valueStr1, valueStr2, valueStr3);
                card.write(buffer, strlen(buffer));
            } else if (i == 11) {
                char valueStr[10];
                dtostrf(planner.extruder_advance_K[0], 1, 3, valueStr);
                snprintf(buffer, sizeof(buffer), " K%s", valueStr);
                card.write(buffer, strlen(buffer));
            } else if (i == 12) {
                char valueStr1[4]; // 4 characters to accommodate a 3-digit uint8_t and null terminator
                snprintf(valueStr1, sizeof(valueStr1), "%u", lcd_rts_settings.probe_margin_x);
                char valueStr2[4]; // 4 characters to accommodate a 3-digit uint8_t and null terminator
                snprintf(valueStr2, sizeof(valueStr2), "%u", lcd_rts_settings.probe_margin_y_back);
                char valueStr3[4]; // 4 characters to accommodate a 3-digit uint8_t and null terminator
                snprintf(valueStr3, sizeof(valueStr3), "%u", lcd_rts_settings.max_points);
                char valueStr4[4]; // 4 characters to accommodate a 3-digit uint8_t and null terminator
                snprintf(valueStr4, sizeof(valueStr4), "%u", lcd_rts_settings.total_probing);
                snprintf(buffer, sizeof(buffer), " S7 X%s Y%s F%s P%s", valueStr1, valueStr2, valueStr3, valueStr4);
                card.write(buffer, strlen(buffer));
            } else if (i == 13) {
                char valueStr1[2]; // 2 characters: 1 for '0' or '1' and 1 for null terminator
                snprintf(valueStr1, sizeof(valueStr1), "%u", lcd_rts_settings.boot_zraise ? 1 : 0); // Convert bool to 0 or 1
                snprintf(buffer, sizeof(buffer), " S9 F%s", valueStr1);
                card.write(buffer, strlen(buffer));
            }
            card.write((char*)"\n", 1);
        }
        card.closefile(settings_filename2);
      }
      else if(recdat.data[0] == 2 && RTS_SD_Detected())
      {
        if (recdat.data[0] == 2 && card.fileExists(settings_filename2)) {
          settingsload = 1;
          queue.enqueue_one_P(PSTR("M23 SETTINGS.GCO"));
          queue.enqueue_now_P(PSTR("M24"));
          card.abortFilePrintNow();
          card.closefile(settings_filename2);
          settings.save();
          planner.synchronize();
          planner.refresh_acceleration_rates();
          planner.refresh_positioning();
          delay(500);            
          rtscheck.RTS_SendLoadedData(255);
          queue.enqueue_now_P(PSTR("M27 S0"));
          RTS_ShowPage(43);
        }
      }
      break;

     case StartFileKey:
      if((recdat.data[0] == 1) && RTS_SD_Detected())
      {
        if(CardRecbuf.recordcount < 0)
        {
          break;
        }
        if(!rts_start_print)
        {
          break;
        }

        char cmd[20];
        char *c;
        sprintf_P(cmd, PSTR("M23 %s"), CardRecbuf.Cardfilename[CardRecbuf.recordcount]);
        for (c = &cmd[4]; *c; c++)
        {
          *c = tolower(*c);
        }

        memset(cmdbuf, 0, sizeof(cmdbuf));
        strcpy(cmdbuf, cmd);
        #if ENABLED(FILAMENT_RUNOUT_SENSOR)
          if ((1 == READ(FIL_RUNOUT_PIN)) && (runout.enabled == true))
          {
            RTS_LoadMainsiteIcons();            
            RTS_ShowPage(46);
            sdcard_pause_check = false;
            break;
          }
        #endif

        rts_start_print = false;

        queue.enqueue_one_now(cmd);
        delay(20);
        queue.enqueue_now_P(PSTR("M24"));
        RTS_CleanPrintAndSelectFile();
        if (CardRecbuf.filenamelen[CardRecbuf.recordcount] > 25){
          RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], SELECT_FILE_TEXT_VP);
        }else{
          RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);
        }
        delay(2);
        #if ENABLED(BABYSTEPPING)
          RTS_ResetSingleVP(AUTO_BED_LEVEL_ZOFFSET_VP);
        #endif
        feedrate_percentage = 100;
        zprobe_zoffset = probe.offset.z;
        RTS_SendZoffsetFeedratePercentage(true);
        PoweroffContinue = true;
        RTS_ShowPage(10);
        RTS_SendM600Icon(true);
        Update_Time_Value = 0;
      }
      else if(recdat.data[0] == 2)
      {
        if (!planner.has_blocks_queued()) {
          if ((file_total_page > file_current_page) && (file_current_page < (MaxFileNumber / 5))){
            file_current_page++;
          }else{
            break;
          }
          if (card.flag.mounted){
            RTS_line_to_filelist();              
          }       
          RTS_ShowPage(2);
        }
      }
      else if(recdat.data[0] == 3)
      {
        if (!planner.has_blocks_queued()) {
          if (file_current_page > 1){
            file_current_page--;
          }else{
            break;
          }
          if (card.flag.mounted){
            RTS_line_to_filelist();
          }
          RTS_ShowPage(2);
        }
      }
      else if(recdat.data[0] == 4)
      {
        if (!planner.has_blocks_queued()) {
          file_current_page = 1;
          RTS_ShowPage(2);
          RTS_line_to_filelist();
        }

      }
      else if(recdat.data[0] == 5)
      {
        if (!planner.has_blocks_queued()) {
          file_current_page = file_total_page;
          RTS_ShowPage(2);
          RTS_line_to_filelist();
        }
      }
      else if(recdat.data[0] == 6)
      {
        RTS_ShowPage(5);
      }
      else if(recdat.data[0] == 7)
      {
        RTS_ShowPage(4); 
      }
      else if(recdat.data[0] == 8)
      {
        RTS_ShowPage(5);  
      }
      else if(recdat.data[0] == 9)
      {
        if (!planner.has_blocks_queued()) {
          file_current_page = 1;
          RTS_ShowPage(2);
          if (card.flag.mounted){
            RTS_line_to_filelist();
          }
        }
      }
      else if(recdat.data[0] == 0x0A)
      {
        if (!planner.has_blocks_queued()) {
          file_current_page = file_total_page;
          RTS_ShowPage(2);
          if (card.flag.mounted){
            RTS_line_to_filelist();              
          }
        }           
      }
      else if(recdat.data[0] == 0x0C)
      {
        RTS_ShowPage(24);
      }        
      break;

    case ChangePageKey:
      // represents to update file list
      if (CardUpdate && lcd_sd_status && IS_SD_INSERTED()) {
        RTS_line_to_filelist();
        for (uint16_t i = 0; i < 5; i++) {
          delay(1);
          RTS_SndData((unsigned long)0xFFFF, FilenameNature + (i + 1) * 16);
          RTS_ResetSingleVP(FILE1_SELECT_ICON_VP + i);
        }
      }

      RTS_SendMachineData();
      RTS_SendProgress(card.percentDone());
      RTS_SendZoffsetFeedratePercentage(true);
      RTS_SndData(thermalManager.degTargetHotend(0), HEAD_SET_TEMP_VP);
      #if ENABLED(BED_TEMP_COMP)
        if (WITHIN(thermalManager.degTargetBed(), 66, 85))
          RTS_SndData(thermalManager.degTargetBed() - 5, BED_SET_TEMP_VP);
        else if (WITHIN(thermalManager.degTargetBed(), 86, 127))
          RTS_SndData(thermalManager.degTargetBed() - 7, BED_SET_TEMP_VP);
      #else
        RTS_SndData(thermalManager.degTargetBed(), BED_SET_TEMP_VP);
      #endif
      languagedisplayUpdate();
      RTS_SndData(change_page_font + ExchangePageBase, ExchangepageAddr);
      break;   

    #if HAS_CUTTER
      case SwitchDeviceKey:
        if(recdat.data[0] == 1)
        {
          RTS_ShowPage(57);
        }else if(recdat.data[0] == 2)
        {
          RTS_ShowPage(56);
        }else if(recdat.data[0] == 0x03)
        {
          if(change_page_font == 64)
          {
            RTS_ShowPage(33);
          }else{
            RTS_ShowPage(1);
          }
          laser_device.set_current_device(DEVICE_FDM);
        }else if(recdat.data[0] == 0x04)
        {
          if(change_page_font == 64)
          {
            RTS_ShowPage(64);
          }else{
            RTS_ShowPage(50);
          }
        }else if(recdat.data[0] == 0x05)
        {
          uint8_t language;
          RTS_ShowPage(77);
          laser_device.set_current_device(DEVICE_LASER);
          language = language_change_font; 
          settings.reset();
          language_change_font = language;
          settings.save();
          probe.offset.z = zprobe_zoffset = 0;
          RTS_SendZoffsetFeedratePercentage(true);
          queue.enqueue_now_P(PSTR("M999\nG92.9 Z0"));
          planner.synchronize();
          RTS_ResetSingleVP(SW_FOCUS_Z_VP);
          laser_device.laser_power_open();
        }else if(recdat.data[0] == 0x06)
        {
          if(change_page_font == 33){
            RTS_ShowPage(33);
          }else{
            RTS_ShowPage(50);
          }
        }
        else if(recdat.data[0] == 0x0B)
        {
          RTS_ShowPage(56);
        }
      break;
    #endif
    case ErrorKey:
      {
        if(recdat.data[0] == 1)
        {
          if(printingIsActive())
          {
            RTS_ShowPage(10);
          }
          else if(printingIsPaused())
          {
            RTS_ShowPage(12);
          }
          else
          {
            RTS_ShowPage(1);
            RTS_ShowPreviewImage(true);
          }

          if(errorway == 4)
          {
            hal.reboot();
          }
        }
      }
      break;

    default:
      break;
  }
  memset(&recdat, 0, sizeof(recdat));
  recdat.head[0] = FHONE;
  recdat.head[1] = FHTWO;
}

int EndsWith(const char *str, const char *suffix)
{
    if (!str || !suffix)
        return 0;
    size_t lenstr = strlen(str);
    size_t lensuffix = strlen(suffix);
    if (lensuffix >  lenstr)
        return 0;
    return strncmp(str + lenstr - lensuffix, suffix, lensuffix) == 0;
}

void EachMomentUpdate(void)
{
  millis_t ms = millis();
  if(ms > next_rts_update_ms)

  {
  #if ENABLED(POWER_LOSS_RECOVERY)
    // in case of power continue this is the startup
    if(!power_off_type_yes && lcd_sd_status && recovery.recovery_flag)
    {
      rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
      if(startprogress < 100)
      {
        rtscheck.RTS_SndData(startprogress, START_PROCESS_ICON_VP);
      }
      if((startprogress += 1) > 100)
      {
        power_off_type_yes = true;
        for(uint16_t i = 0;i < CardRecbuf.Filesum;i ++) 
        {
          if(!strcmp(CardRecbuf.Cardfilename[i], &recovery.info.sd_filename[1]))
          {
            if (CardRecbuf.filenamelen[i] > 25){
              rtscheck.RTS_SndData(CardRecbuf.Cardshowfilename[i], SELECT_FILE_TEXT_VP);
            }else{
              rtscheck.RTS_SndData(CardRecbuf.Cardshowfilename[i], PRINT_FILE_TEXT_VP);
            }
            RTS_ShowPage(27);
            break;
          }
        }
      }
      return;
    }
    // simple power up without power loss recovery run
    else if(!power_off_type_yes && !recovery.recovery_flag)
    {
      rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
      if(startprogress < 100)
      {
        rtscheck.RTS_SndData(startprogress, START_PROCESS_ICON_VP);
      }
      if((startprogress += 1) > 100)
      {
        power_off_type_yes = true;
        Update_Time_Value = RTS_UPDATE_VALUE; 
        change_page_font = 1;
        int16_t fileCnt = card.get_num_items();
        card.getWorkDirName();
        if (card.filename[0] != '/') card.cdup();
        for (int16_t i = 0; (i < fileCnt) && (i < MaxFileNumber); i++) {
          card.selectFileByIndexSorted(i);
          char *pointFilename = card.longFilename;
          int filenamelen = strlen(card.longFilename);
          int j = 1; 
          while ((strncmp(&pointFilename[j], ".gcode", 6) != 0 && strncmp(&pointFilename[j], ".GCODE", 6) != 0 && strncmp(&pointFilename[j], ".GCO", 4) != 0 && strncmp(&pointFilename[j], ".gco", 4) != 0) && (j++ < filenamelen));
          RTS_CleanPrintAndSelectFile();
          if (j >= TEXTBYTELEN) {
            strncpy(&card.longFilename[TEXTBYTELEN - 3], "..", 2);
            card.longFilename[TEXTBYTELEN - 1] = '\0';
            j = TEXTBYTELEN - 1;
          }
          strncpy(CardRecbuf.Cardshowfilename[i], card.longFilename, j);
          strcpy(CardRecbuf.Cardfilename[i], card.filename);
          rtscheck.RTS_SndData(CardRecbuf.Cardshowfilename[i], CardRecbuf.addr[i]);
          if (!strcmp(CardRecbuf.Cardfilename[i], &recovery.info.sd_filename[1])) {
            rtscheck.RTS_SndData(CardRecbuf.Cardshowfilename[i], PRINT_FILE_TEXT_VP);
            break;
          }
        }

        #if HAS_CUTTER
          if(laser_device.is_laser_device()){
            RTS_ShowPage(51);
          }else if(laser_device.get_current_device()== DEVICE_UNKNOWN){
            RTS_ShowPage(50);
          }else
        #endif
        {
          RTS_ShowPage(1);
          RTS_ShowPreviewImage(true);
        }
      }    
      return;
    }
    else
    {
        static unsigned char last_cardpercentValue = 100;
        // Host Printing without M73
        if(!lcd_rts_settings.external_m73 && printingIsActive() && !card.isPrinting()){
          duration_t elapsed = print_job_timer.duration();
          rtscheck.RTS_SndData(elapsed.value / 3600, PRINT_TIME_HOUR_VP);
          rtscheck.RTS_SndData((elapsed.value % 3600) / 60, PRINT_TIME_MIN_VP);
          rtscheck.RTS_SndData(ui.get_remaining_time() / 3600, PRINT_REMAIN_TIME_HOUR_VP);
          rtscheck.RTS_SndData((ui.get_remaining_time() % 3600) / 60, PRINT_REMAIN_TIME_MIN_VP);
          rtscheck.RTS_SndData((unsigned char) ui.get_progress_percent(), PRINT_PROCESS_ICON_VP);
          rtscheck.RTS_SndData((unsigned char) ui.get_progress_percent(), PRINT_PROCESS_VP);          
          #if ENABLED(LCD_RTS_DEBUG_PRINTTIME)
            SERIAL_ECHO_MSG("lcd_rts 1 PRINT_TIME_HOUR_VP: ", elapsed.value / 3600);
            SERIAL_ECHO_MSG("lcd_rts 1 PRINT_TIME_MIN_VP: ", (elapsed.value % 3600));
            SERIAL_ECHO_MSG("lcd_rts 1 PRINT_REMAIN_TIME_HOUR_VP: ", ui.get_remaining_time() / 3600);
            SERIAL_ECHO_MSG("lcd_rts 1 PRINT_REMAIN_TIME_MIN_VP: ", (ui.get_remaining_time() % 3600));
            SERIAL_ECHO_MSG("lcd_rts 1 ui.get_progress_percent(): ", ui.get_progress_percent());
          #endif
        }
        // if printing and card.precentDone < 100
        // basically always while SD printing
        if(card.isPrinting() && (last_cardpercentValue != card.percentDone()) && !lcd_rts_settings.external_m73)
        {
          if((unsigned char) card.percentDone() > 0)
          {
            Percentrecord = card.percentDone();
            if(Percentrecord <= 100)
            {
              #if ENABLED(LCD_RTS_DEBUG_PRINTTIME)
                SERIAL_ECHO_MSG("(unsigned char)Percentrecord(): ", (unsigned char)Percentrecord);
              #endif
              RTS_SendProgress((unsigned char)Percentrecord);
            }
          }
          else
          {
            RTS_ResetSingleVP(PRINT_PROCESS_ICON_VP);
          }
          #if ENABLED(LCD_RTS_DEBUG_PRINTTIME)
            SERIAL_ECHO_MSG("(unsigned char)card.percentDone(): ", (unsigned char)card.percentDone());
          #endif
          RTS_SendProgress((unsigned char)card.percentDone());
          last_cardpercentValue = card.percentDone();
          RTS_SendCurrentPosition(3);
        } // here then also 
        else if(card.isPrinting() && !lcd_rts_settings.external_m73)
        {
          // sends time while printing as of 2 percent remain time also
          duration_t elapsed = print_job_timer.duration();
          Percentrecord = card.percentDone();
          rtscheck.RTS_SndData(elapsed.value / 3600, PRINT_TIME_HOUR_VP);
          rtscheck.RTS_SndData((elapsed.value % 3600) / 60, PRINT_TIME_MIN_VP);
          #if ENABLED(LCD_RTS_DEBUG_PRINTTIME)
            SERIAL_ECHO_MSG("lcd_rts 2 PRINT_TIME_HOUR_VP: ", elapsed.value / 3600);
            SERIAL_ECHO_MSG("lcd_rts 2 PRINT_TIME_MIN_VP: ", (elapsed.value % 3600));
          #endif          
          if(Percentrecord<2)
          {
            RTS_ResetSingleVP(PRINT_REMAIN_TIME_HOUR_VP);
            RTS_ResetSingleVP(PRINT_REMAIN_TIME_MIN_VP);
          }else{
              int _remain_time = 0;
              _remain_time = ((elapsed.value) * ((float)card.getFileSize() / (float)card.getIndex())) - (elapsed.value);

              if(_remain_time < 0) _remain_time = 0;
              rtscheck.RTS_SndData(_remain_time / 3600, PRINT_REMAIN_TIME_HOUR_VP);
              rtscheck.RTS_SndData((_remain_time % 3600) / 60, PRINT_REMAIN_TIME_MIN_VP);

              #if ENABLED(LCD_RTS_DEBUG_PRINTTIME)
                SERIAL_ECHO_MSG("lcd_rts 3 PRINT_REMAIN_TIME_HOUR_VP: ", _remain_time / 3600);
                SERIAL_ECHO_MSG("lcd_rts 3 PRINT_REMAIN_TIME_MIN_VP: ", (_remain_time % 3600));
              #endif
          }
        } else if ((ui.get_progress_percent() != last_progress_percent || ui.get_remaining_time() != last_remaining_time) && card.isPrinting() && !lcd_rts_settings.external_m73) {
          rtscheck.RTS_SndData(ui.get_remaining_time() / 3600, PRINT_REMAIN_TIME_HOUR_VP);
          rtscheck.RTS_SndData((ui.get_remaining_time() % 3600) / 60, PRINT_REMAIN_TIME_MIN_VP);
          #if ENABLED(LCD_RTS_DEBUG_PRINTTIME)
            SERIAL_ECHO_MSG("lcd_rts 4 PRINT_REMAIN_TIME_HOUR_VP: ", ui.get_remaining_time() / 3600);
            SERIAL_ECHO_MSG("lcd_rts 4 PRINT_REMAIN_TIME_MIN_VP: ", (ui.get_remaining_time() % 3600));
          #endif          
          rtscheck.RTS_SndData((unsigned char) ui.get_progress_percent(), PRINT_PROCESS_ICON_VP);
          rtscheck.RTS_SndData((unsigned char) ui.get_progress_percent(), PRINT_PROCESS_VP);
          if ((ui.get_remaining_time() > 0 && last_start_time == 0) || last_progress_percent > ui.get_progress_percent()) {
            last_start_time = HAL_GetTick();
          }
          if (last_start_time > 0 && ui.get_progress_percent() < 100) {
            uint32_t elapsed_seconds = (HAL_GetTick() - last_start_time) / 1000;
            rtscheck.RTS_SndData(elapsed_seconds / 3600, PRINT_TIME_HOUR_VP);
            rtscheck.RTS_SndData((elapsed_seconds % 3600) / 60, PRINT_TIME_MIN_VP);
            #if ENABLED(LCD_RTS_DEBUG_PRINTTIME)
              SERIAL_ECHO_MSG("lcd_rts 5 PRINT_TIME_HOUR_VP: ", elapsed_seconds / 3600);
              SERIAL_ECHO_MSG("lcd_rts 5 PRINT_TIME_MIN_VP: ", (elapsed_seconds % 3600));
            #endif            
          }
          last_progress_percent = ui.get_progress_percent();
          last_remaining_time = ui.get_remaining_time();
        }

      if(pause_action_flag && !sdcard_pause_check && printingIsPaused() && !planner.has_blocks_queued())
      {
        pause_action_flag = false;
        queue.enqueue_now_P(PSTR("G0 F3000 X0 Y0"));
        thermalManager.setTargetHotend(0, 0);
        RTS_ResetSingleVP(HEAD_SET_TEMP_VP);
      }

      rtscheck.RTS_SndData(lcd_rts_settings.screen_brightness, DISPLAY_BRIGHTNESS);
      rtscheck.RTS_SndData(lcd_rts_settings.standby_brightness, DISPLAYSTANDBY_BRIGHTNESS);
      rtscheck.RTS_SndData(lcd_rts_settings.standby_time_seconds, DISPLAYSTANDBY_SECONDS);
      if (lcd_rts_settings.display_standby)
        rtscheck.RTS_SndData(3, DISPLAYSTANDBY_ENABLEINDICATOR);
      else
        rtscheck.RTS_SndData(2, DISPLAYSTANDBY_ENABLEINDICATOR);

      rtscheck.RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD_CURRENT_TEMP_VP);
      rtscheck.RTS_SndData(thermalManager.temp_bed.celsius, BED_CURRENT_TEMP_VP);

      #if ENABLED(SDSUPPORT)
        if(!sdcard_pause_check && (false == card.isPrinting()) && !planner.has_blocks_queued())
        {
          if (card.flag.mounted)
          {
            RTS_SetOneToVP(CHANGE_SDCARD_ICON_VP);
          }
          else
          {
            RTS_ResetSingleVP(CHANGE_SDCARD_ICON_VP);
          }
        }
      #endif

      if((last_target_temperature[0] != thermalManager.temp_hotend[0].target) || (last_target_temperature_bed != thermalManager.temp_bed.target))
      {
        RTS_SendHeadTemp();
        RTS_SendBedTemp();
        last_target_temperature[0] = thermalManager.temp_hotend[0].target;
        last_target_temperature_bed = thermalManager.temp_bed.target;
      }

      if((thermalManager.temp_hotend[0].celsius >= thermalManager.temp_hotend[0].target) && (heatway == 1))
      {
        RTS_ShowPage(19);
        heatway = 0;
        rtscheck.RTS_SndData(10 * FilamentLOAD, HEAD_FILAMENT_LOAD_DATA_VP);
        rtscheck.RTS_SndData(10 * FilamentUnLOAD, HEAD_FILAMENT_UNLOAD_DATA_VP);
      }
      #if ENABLED(FILAMENT_RUNOUT_SENSOR)
        if(1 == READ(FIL_RUNOUT_PIN))
        {
          RTS_ResetSingleVP(FILAMENT_LOAD_ICON_VP);
        }
        else
        {
          RTS_SetOneToVP(FILAMENT_LOAD_ICON_VP);
        }
      #endif
      rtscheck.RTS_SndData(thermalManager.fan_speed[0] , PRINTER_FAN_SPEED_DATA_VP);
    }
  #endif

    next_rts_update_ms = ms + RTS_UPDATE_INTERVAL + Update_Time_Value;
  }
}

void RTSSHOW::languagedisplayUpdate(void)
{
  RTS_SendLang(MAIN_PAGE_BLUE_TITLE_VP);
  RTS_SendLang(SELECT_FILE_BLUE_TITLE_VP);
  RTS_SendLang(PREPARE_PAGE_BLUE_TITLE_VP);
  RTS_SendLang(SETTING_PAGE_BLUE_TITLE_VP);
  RTS_SendLang(MAIN_PAGE_BLACK_TITLE_VP);
  RTS_SendLang(SELECT_FILE_BLACK_TITLE_VP);
  RTS_SendLang(PREPARE_PAGE_BLACK_TITLE_VP);
  RTS_SendLang(SETTING_PAGE_BLACK_TITLE_VP);
  RTS_SendLang(PRINT_ADJUST_MENT_TITLE_VP);
  RTS_SendLang(PRINT_SPEED_TITLE_VP);
  RTS_SendLang(HEAD_SET_TITLE_VP);
  RTS_SendLang(BED_SET_TITLE_VP);
  RTS_SendLang(LEVEL_ZOFFSET_TITLE_VP);
  RTS_SendLang(FAN_CONTROL_TITLE_VP);
  RTS_SendLang(LED_CONTROL_TITLE_VP);
  RTS_SendLang(MOVE_AXIS_ENTER_GREY_TITLE_VP);
  RTS_SendLang(CHANGE_FILAMENT_GREY_TITLE_VP);
  RTS_SendLang(PREHAET_PAGE_GREY_TITLE_VP);
  RTS_SendLang(MOVE_AXIS_ENTER_BLACK_TITLE_VP);
  RTS_SendLang(CHANGE_FILAMENT_BLACK_TITLE_VP);
  RTS_SendLang(PREHAET_PAGE_BLACK_TITLE_VP);
  RTS_SendLang(PREHEAT_PLA_BUTTON_TITLE_VP);
  RTS_SendLang(PREHEAT_ABS_BUTTON_TITLE_VP);
  RTS_SendLang(PREHEAT_PETG_BUTTON_TITLE_VP);
  RTS_SendLang(PREHEAT_CUST_BUTTON_TITLE_VP);  
  RTS_SendLang(COOL_DOWN_BUTTON_TITLE_VP);
  RTS_SendLang(FILAMENT_LOAD_BUTTON_TITLE_VP);
  RTS_SendLang(FILAMENT_UNLOAD_BUTTON_TITLE_VP);
  RTS_SendLang(LANGUAGE_SELECT_ENTER_VP);
  RTS_SendLang(FACTORY_DEFAULT_ENTER_TITLE_VP);
  RTS_SendLang(LEVELING_PAGE_TITLE_VP);
  RTS_SendLang(PRINTER_DEVICE_GREY_TITLE_VP);
  RTS_SendLang(PRINTER_ADVINFO_GREY_TITLE_VP);
  RTS_SendLang(PRINTER_INFO_ENTER_GREY_TITLE_VP);
  RTS_SendLang(PRINTER_DEVICE_BLACK_TITLE_VP);
  RTS_SendLang(PRINTER_ADVINFO_BLACK_TITLE_VP);
  RTS_SendLang(PRINTER_INFO_ENTER_BLACK_TITLE_VP);
  RTS_SendLang(PREHEAT_PLA_SET_TITLE_VP);
  RTS_SendLang(PREHEAT_ABS_SET_TITLE_VP);
  RTS_SendLang(PREHEAT_PETG_SET_TITLE_VP);
  RTS_SendLang(PREHEAT_CUST_SET_TITLE_VP);
  RTS_SendLang(STORE_MEMORY_CONFIRM_TITLE_VP);
  RTS_SendLang(STORE_MEMORY_CANCEL_TITLE_VP);
  RTS_SendLang(FILAMENT_UNLOAD_IGNORE_TITLE_VP);
  RTS_SendLang(FILAMENT_USEUP_TITLE_VP);
  RTS_SendLang(BUTTON_CHECK_CONFIRM_TITLE_VP);
  RTS_SendLang(BUTTON_CHECK_CANCEL_TITLE_VP);
  RTS_SendLang(FILAMENT_LOAD_TITLE_VP);
  RTS_SendLang(FILAMENT_LOAD_RESUME_TITLE_VP);
  RTS_SendLang(PAUSE_PRINT_POP_TITLE_VP);
  RTS_SendLang(STOP_PRINT_POP_TITLE_VP);
  RTS_SendLang(POWERCONTINUE_POP_TITLE_VP);
  RTS_SendLang(AUTO_HOME_WAITING_POP_TITLE_VP);
  RTS_SendLang(BEDLEVELING_WAIT_TITLE_VP);
  RTS_SendLang(RESTORE_FACTORY_TITLE_VP);
  RTS_SendLang(RESET_WIFI_SETTING_TITLE_VP);
  RTS_SendLang(KILL_THERMAL_RUNAWAY_TITLE_VP);
  RTS_SendLang(KILL_HEATING_FAIL_TITLE_VP);
  RTS_SendLang(KILL_THERMISTOR_ERROR_TITLE_VP);
  RTS_SendLang(WIND_AUTO_SHUTDOWN_TITLE_VP);
  RTS_SendLang(RESET_WIFI_SETTING_BUTTON_VP);
  RTS_SendLang(PRINTER_AUTO_SHUTDOWN_TITLE_VP);
  RTS_SendLang(WIND_AUTO_SHUTDOWN_PAGE_VP);
  RTS_SendLang(AUTO_LEVELING_START_TITLE_VP);
  RTS_SendLang(AUX_LEVELING_GREY_TITLE_VP);
  RTS_SendLang(AUTO_LEVELING_GREY_TITLE_VP);
  RTS_SendLang(AUX_LEVELING_BLACK_TITLE_VP);
  RTS_SendLang(AUTO_LEVELING_BLACK_TITLE_VP);
  RTS_SendLang(LANGUAGE_SELECT_PAGE_TITLE_VP);
  RTS_SendLang(ADV_SETTING_MOTION_TITLE_VP);
  RTS_SendLang(ADV_SETTING_PID_TITLE_VP);
  RTS_SendLang(ADV_SETTING_WIFI_TITLE_VP);
  RTS_SendLang(MOTION_SETTING_TITLE_VP);
  RTS_SendLang(MOTION_SETTING_STEPSMM_TITLE_VP);
  RTS_SendLang(MOTION_SETTING_ACCEL_TITLE_VP);
  RTS_SendLang(MOTION_SETTING_JERK_TITLE_VP);
  RTS_SendLang(MOTION_SETTING_VELOCITY_TITLE_VP);
  RTS_SendLang(MAX_VELOCITY_SETTING_TITLE_VP);
  RTS_SendLang(MAX_VELOCITY_XAXIS_TITLE_VP);
  RTS_SendLang(MAX_VELOCITY_YAXIS_TITLE_VP);
  RTS_SendLang(MAX_VELOCITY_ZAXIS_TITLE_VP);
  RTS_SendLang(MAX_VELOCITY_EAXIS_TITLE_VP);
  RTS_SendLang(MAX_ACCEL_SETTING_TITLE_VP);
  RTS_SendLang(MAX_ACCEL_XAXIS_TITLE_VP);
  RTS_SendLang(MAX_ACCEL_YAXIS_TITLE_VP);
  RTS_SendLang(MAX_ACCEL_ZAXIS_TITLE_VP);
  RTS_SendLang(MAX_ACCEL_EAXIS_TITLE_VP);
  RTS_SendLang(MAX_JERK_SETTING_TITLE_VP);
  RTS_SendLang(MAX_JERK_XAXIS_TITLE_VP);
  RTS_SendLang(MAX_JERK_YAXIS_TITLE_VP);
  RTS_SendLang(MAX_JERK_ZAXIS_TITLE_VP);
  RTS_SendLang(MAX_JERK_EAXIS_TITLE_VP);
  RTS_SendLang(MAX_STEPSMM_SETTING_TITLE_VP);
  RTS_SendLang(MAX_STEPSMM_XAXIS_TITLE_VP);
  RTS_SendLang(MAX_STEPSMM_YAXIS_TITLE_VP);
  RTS_SendLang(MAX_STEPSMM_ZAXIS_TITLE_VP);
  RTS_SendLang(MAX_STEPSMM_EAXIS_TITLE_VP);
  RTS_SendLang(TEMP_PID_SETTING_TITLE_VP);
  RTS_SendLang(NOZZLE_TEMP_P_TITLE_VP);
  RTS_SendLang(NOZZLE_TEMP_I_TITLE_VP);
  RTS_SendLang(NOZZLE_TEMP_D_TITLE_VP);
  RTS_SendLang(HOTBED_TEMP_P_TITLE_VP);
  RTS_SendLang(HOTBED_TEMP_I_TITLE_VP);
  RTS_SendLang(HOTBED_TEMP_D_TITLE_VP);
  RTS_SendLang(FILAMENT_CONTROL_TITLE_VP);
  RTS_SendLang(POWERCONTINUE_CONTROL_TITLE_VP);
  RTS_SendLang(MACHINE_TYPE_ABOUT_CHAR_VP);
  RTS_SendLang(FIRMWARE_VERSION_ABOUT_CHAR_VP);
  RTS_SendLang(PRINTER_DISPLAY_VERSION_TITLE_VP);
  RTS_SendLang(HARDWARE_VERSION_ABOUT_TITLE_VP);
  RTS_SendLang(WEBSITE_ABOUT_CHAR_VP);
  RTS_SendLang(PRINTER_PRINTSIZE_TITLE_VP);
  RTS_SendLang(PLA_SETTINGS_TITLE_VP);
  RTS_SendLang(ABS_SETTINGS_TITLE_VP);
  RTS_SendLang(PETG_SETTINGS_TITLE_VP);
  RTS_SendLang(CUST_SETTINGS_TITLE_VP);  
  RTS_SendLang(LEVELING_WAY_TITLE_VP);
  RTS_SendLang(SOUND_SETTING_VP);
  RTS_SendLang(PRINT_FINISH_ICON_VP);
  #if HAS_CUTTER
    RTS_SendLang(SELECT_LASER_WARNING_TIPS_VP);
    RTS_SendLang(SELECT_FDM_WARNING_TIPS_VP);
    RTS_SendLang(PRINT_MOVE_AXIS_VP);
    RTS_SendLang(PRINT_DIRECT_ENGRAV_VP);
    RTS_SendLang(PRINT_RUN_RANGE_VP);
    RTS_SendLang(PRINT_RETURN_VP);
    RTS_SendLang(PRINT_WARNING_TIPS_VP);
    RTS_SendLang(DEVICE_SWITCH_LASER_VP);
    RTS_SendLang(FIRST_SELECT_DEVICE_TYPE);
    RTS_SendLang(HOME_LASER_ENGRAVE_VP);
    RTS_SendLang(PREPARE_ADJUST_FOCUS_VP);
    RTS_SendLang(PREPARE_SWITCH_FDM_VP);
    RTS_SendLang(FIRST_DEVICE_FDM);
    RTS_SendLang(FIRST_DEVICE_LASER);
    RTS_SendLang(FOCUS_SET_FOCUS_TIPS);   
  #endif
  RTS_SendLang(AUTO_PID_INLET_VP);
  RTS_SendLang(AUTO_PID_HOTBED_INLET_VP);
  RTS_SendLang(AUTO_PID_HOTBED_TIS_VP);
  RTS_SendLang(AUTO_PID_NOZZLE_INLET_VP);
  RTS_SendLang(AUTO_PID_NOZZLE_TIS_VP);  
  RTS_SendLang(AUTO_PID_START_NOZZLE_VP);
  RTS_SendLang(AUTO_PID_START_HOTBED_VP);
  RTS_SendLang(MESH_LEVELING_BLACK_TITLE_VP);
  RTS_SendLang(SHAPING_X_TITLE_VP);
  RTS_SendLang(SHAPING_Y_TITLE_VP);
}

// looping at the loop function
void RTS_Update(void)
{
  // Check the status of card
  rtscheck.RTS_SDCardUpdate();

	sd_printing = IS_SD_PRINTING();
	card_insert_st = IS_SD_INSERTED() ;
	if(!card_insert_st && sd_printing){
		RTS_ShowPage(47);   
		RTS_ResetSingleVP(CHANGE_SDCARD_ICON_VP);
		card.pauseSDPrint();
		print_job_timer.pause();
    pause_action_flag = true;
    sdcard_pause_check = false;
	}

	if(last_card_insert_st != card_insert_st){
		rtscheck.RTS_SndData(card_insert_st ? 1 : 0, CHANGE_SDCARD_ICON_VP);
		last_card_insert_st = card_insert_st;
	}

  EachMomentUpdate();
  // wait to receive massage and response
  if (rtscheck.RTS_RecData() > 0)
  {
    rtscheck.RTS_HandleData();
  }
  hal.watchdog_refresh();
}

void RTS_PauseMoveAxisPage(void)
{
  #if HAS_CUTTER
    if(laser_device.is_laser_device()) return;
  #endif

  if(waitway == 1)
  {
    RTS_ShowPage(12);
    waitway = 0;
  }
  else if(waitway == 5)
  {
    RTS_ShowPage(7);
    waitway = 0;
  }
  else if (waitway == 7) {
    // Click stop print
    RTS_ShowPage(1);
    waitway = 0;
  }  
}

void RTS_AutoBedLevelPage() {
    if (waitway != 15) {
            bool zig = false;
            int8_t inStart, inStop, inInc, showcount;
            showcount = 0;
            float min_value = bedlevel.z_values[0][0];
            float max_value = bedlevel.z_values[0][0];
            // Determine the min and max values from the mesh data
            for (int y = 0; y < lcd_rts_settings.max_points; y++) {
                for (int x = 0; x < lcd_rts_settings.max_points; x++) {
                    float current_z_value = bedlevel.z_values[x][y];
                    if (current_z_value < min_value) min_value = current_z_value;
                    if (current_z_value > max_value) max_value = current_z_value;
                }
            }
            float deviation = max_value - min_value;
            float median = (min_value + max_value) / 2.0f;
            ColorRange color_ranges[7];
            int num_ranges = 0;
            // Center green in the range
            color_ranges[num_ranges++] = {median - 0.04f / 2, median + 0.04f / 2, 0x07E0};
            // Calculate and add additional colors if the range exceeds GREEN_RANGE = 0.04f
            float additional_range = (deviation - 0.04f) / 2;
            float lower_bound = median - 0.04f / 2;
            float upper_bound = median + 0.04f / 2;
            // Add colors below green
            while (lower_bound > min_value) {
                float range_end = std::max(min_value, lower_bound - additional_range);
                color_ranges[num_ranges++] = {range_end, lower_bound, 0x07E0}; // Light Green
                lower_bound = range_end;
            }
            // Add colors above green
            while (upper_bound < max_value) {
                float range_start = std::min(max_value, upper_bound + additional_range);
                color_ranges[num_ranges++] = {upper_bound, range_start, 0xFFE0}; // Yellow
                upper_bound = range_start;
            }
            RTS_LoadMeshPointOffsets();
            for (int y = 0; y < lcd_rts_settings.max_points; y++) {
              if (zig) {
                  inStart = lcd_rts_settings.max_points - 1;
                  inStop = -1;
                  inInc = -1;
              } else {
                  inStart = 0;
                  inStop = lcd_rts_settings.max_points;
                  inInc = 1;
              }
              zig ^= true;
              bool isEvenMesh = (lcd_rts_settings.max_points % 2 == 0);

              for (int x = inStart; x != inStop; x += inInc) {
                  int display_x = isEvenMesh ? (lcd_rts_settings.max_points - 1 - x) : x;
                  float current_z_value = bedlevel.z_values[display_x][y];
                  
                  // Calculate upperLeftX and upperLeftY
                  uint16_t upperLeftX = rect_0_x_top_even + (display_x * rect_x_offset);
                  uint16_t upperLeftY = rect_0_y_top - (y * rect_y_offset);
                  
                  rtscheck.RTS_SndData(current_z_value * 1000, AUTO_BED_LEVEL_1POINT_NEW_VP + showcount * 2);
                  unsigned long color;
                  color = getColor(current_z_value, min_value, max_value, median);
                  rtscheck.sendOneFilledRectangle(MESH_RECTANGLE_BASE_VP + (color_sp_offset * 16), showcount, upperLeftX, upperLeftY, rectWidth, rectHeight, static_cast<uint16_t>(color));
                  //rtscheck.RTS_SndData(color, TrammingpointNature + (color_sp_offset + showcount + 1) * 16);
                  showcount++;
              }
            }
            //delay(1000);
            rtscheck.RTS_SndData(min_value * 1000, MESH_POINT_MIN);
            rtscheck.RTS_SndData(min_value * 1000, MESH_POINT_MIN); // this is intended to send this variable twice. it simply does not work sending once..
            rtscheck.RTS_SndData(max_value * 1000, MESH_POINT_MAX);
            rtscheck.RTS_SndData(deviation * 1000, MESH_POINT_DEVIATION);
            RTS_SendLang(AUTO_LEVELING_START_TITLE_VP);
            if (bedlevel.mesh_is_valid()) {
              int counter1 = 0;
              for (int blupp = 0; blupp < (lcd_rts_settings.max_points * lcd_rts_settings.max_points); blupp++) {
                      rtscheck.RTS_SndData((unsigned long)0x0000, TrammingpointNature + (color_sp_offset + counter1 + 1) * 16);
                      counter1++;
              }
            }
            rtscheck.RTS_ChangeLevelingPage();
            waitway = 0;
        RTS_ResetSingleVP(AXIS_Z_COORD_VP);
    }
}

void RTSSHOW::RTS_ChangeLevelingPage(void)
{
  if (lcd_rts_settings.max_points == 5){
    RTS_ShowPage(81);
  }
  if (lcd_rts_settings.max_points == 7){
    RTS_ShowPage(94);
  }
  if (lcd_rts_settings.max_points == 10){
    RTS_ShowPage(95);
  }
}

void RTS_SetBltouchHSMode(void)
{
  if (lcd_rts_settings.max_points == 5){
    queue.enqueue_now_P(PSTR("M401 S1"));
  }
  if (lcd_rts_settings.max_points == 7){
    if (lcd_rts_settings.total_probing == 1){
      queue.enqueue_now_P(PSTR("M401 S1"));
    }else{
      queue.enqueue_now_P(PSTR("M401 S0"));
    }
  }
  if (lcd_rts_settings.max_points == 10){
      queue.enqueue_now_P(PSTR("M401 S0"));
  }
}

void RTS_LoadMeshPointOffsets(void)
{
  if (lcd_rts_settings.max_points == 5){
    color_sp_offset = 0;
    rectWidth = 83; rectHeight = 54;
    rect_0_y_top = 451; rect_1_x_top_odd = 364; rect_0_x_top_even = 32;
    rect_x_offset = 83; rect_y_offset = 54;
    rtscheck.RTS_SndData(209, MESH_SIZE_ICON_VP);
  }
  if (lcd_rts_settings.max_points == 7){
    color_sp_offset = 25;
    rectWidth = 61; rectHeight = 36;
    rect_0_y_top = 463; rect_1_x_top_odd = 392; rect_0_x_top_even = 26;
    rect_x_offset = 61; rect_y_offset = 36;
    rtscheck.RTS_SndData(210, MESH_SIZE_ICON_VP);
  }
  if (lcd_rts_settings.max_points == 10){
    color_sp_offset = 74;
    rectWidth = 43; rectHeight = 28;
    rect_0_y_top = 481; rect_1_x_top_odd = 413; rect_0_x_top_even = 26;    
    rect_x_offset = 43; rect_y_offset = 28;
    rtscheck.RTS_SndData(211, MESH_SIZE_ICON_VP);
  }
}

void RTS_ResetMesh(void)
{
  RTS_LoadMeshPointOffsets();
  rtscheck.sendRectangleCommand(0x2490, 0, 0, 1, 1, 0x1000);
  if (!printingIsActive()){
    bedlevel.reset();
  }
  bool zig = false;
  int8_t inStart, inStop, inInc, showcount;
  showcount = 0;
  for (int y = 0; y < lcd_rts_settings.max_points; y++)
  {
    // away from origin
    if (zig)
    {
      inStart = lcd_rts_settings.max_points - 1;
      inStop = -1;
      inInc = -1;
    }
    else
    {
      // towards origin
      inStart = 0;
      inStop = lcd_rts_settings.max_points;
      inInc = 1;
    }
    zig ^= true;
    for (int x = inStart; x != inStop; x += inInc)
    {
      // Set the value to 0 directly
      RTS_ResetSingleVP(AUTO_BED_LEVEL_1POINT_NEW_VP + showcount * 2);
      rtscheck.sendOneFilledRectangle(MESH_RECTANGLE_BASE_VP + (color_sp_offset * 16), showcount, 0, 0, 1, 1, static_cast<uint16_t>(0xFFFF));
      rtscheck.RTS_SndData((unsigned long)0xFFFF, TrammingpointNature + (color_sp_offset + showcount + 1) * 16);
      showcount++;
    }
  } 
  int counter1 = 0;
  for (int blupp = 0; blupp < (lcd_rts_settings.max_points * lcd_rts_settings.max_points); blupp++) {
    rtscheck.RTS_SndData((unsigned long)0xFFFF, TrammingpointNature + (color_sp_offset + counter1 + 1) * 16);
    counter1++;
  }
  if (!printingIsActive()){
    settings.save();
  }
  RTS_ResetSingleVP(AUTO_BED_LEVEL_CUR_POINT_VP);
  if (printingIsActive()){
    RTS_SendLevelingSiteData(0);
    rtscheck.RTS_SndData(lang + 10, AUTO_LEVELING_START_TITLE_VP);    
  }
}

void RTS_LoadMesh(void)
{
    RTS_LoadMeshPointOffsets();
    if (bedlevel.mesh_is_valid()){
      int8_t inStart, inStop, inInc, showcount;
      bool zig = false;              
      showcount = 0;
      float min_value = bedlevel.z_values[0][0];
      float max_value = bedlevel.z_values[0][0];
      // Determine the min and max values from the mesh data
      for (int y = 0; y < lcd_rts_settings.max_points; y++) {
        for (int x = 0; x < lcd_rts_settings.max_points; x++) {
            float current_z_value = bedlevel.z_values[x][y];
            if (current_z_value < min_value) min_value = current_z_value;
            if (current_z_value > max_value) max_value = current_z_value;
        }
      }
      float deviation = max_value - min_value;
      float median = (min_value + max_value) / 2.0f;
      ColorRange color_ranges[7];
      int num_ranges = 0;
      // Center green in the range
      color_ranges[num_ranges++] = {median - 0.04f / 2, median + 0.04f / 2, 0x07E0};
      // Calculate and add additional colors if the range exceeds GREEN_RANGE = 0.04f
      float additional_range = (deviation - 0.04f) / 2;
      float lower_bound = median - 0.04f / 2;
      float upper_bound = median + 0.04f / 2;
      // Add colors below green
      while (lower_bound > min_value) {
        float range_end = std::max(min_value, lower_bound - additional_range);
        color_ranges[num_ranges++] = {range_end, lower_bound, 0x07E0}; // Light Green
        lower_bound = range_end;
      }
      // Add colors above green
      while (upper_bound < max_value) {
        float range_start = std::min(max_value, upper_bound + additional_range);
        color_ranges[num_ranges++] = {upper_bound, range_start, 0xFFE0}; // Yellow
        upper_bound = range_start;
      }
      for (int y = 0; y < lcd_rts_settings.max_points; y++) {
        if (zig) {
          inStart = lcd_rts_settings.max_points - 1;
          inStop = -1;
          inInc = -1;
        } else {
          inStart = 0;
          inStop = lcd_rts_settings.max_points;
          inInc = 1;
        }
        zig ^= true;
        bool isEvenMesh = (lcd_rts_settings.max_points % 2 == 0);
        for (int x = inStart; x != inStop; x += inInc) {
          int display_x = isEvenMesh ? (lcd_rts_settings.max_points - 1 - x) : x; // Flip x-coordinate for even-sized mesh
          float current_z_value = bedlevel.z_values[display_x][y];
          rtscheck.RTS_SndData(current_z_value * 1000, AUTO_BED_LEVEL_1POINT_NEW_VP + showcount * 2);
          rtscheck.RTS_SndData((unsigned long)0x0000, TrammingpointNature + (color_sp_offset + showcount + 1) * 16);
          showcount++;
        }
      }
      rtscheck.RTS_SndData(min_value * 1000, MESH_POINT_MIN);
      rtscheck.RTS_SndData(max_value * 1000, MESH_POINT_MAX);
      rtscheck.RTS_SndData(deviation * 1000, MESH_POINT_DEVIATION);
      RTS_SendLang(AUTO_LEVELING_START_TITLE_VP);

      if(!printingIsActive()){
        queue.enqueue_now_P(PSTR("M420 S1"));
      }else{
        RTS_LoadMainsiteIcons();
      }
    }

}

void RTS_CleanPrintAndSelectFile(void)
{
  for (int j = 0; j < 55; j++) {
    RTS_ResetSingleVP(SELECT_FILE_TEXT_VP + j);
  }
  for (int j = 0; j < 25; j++) {  
    RTS_ResetSingleVP(PRINT_FILE_TEXT_VP + j);
  }
}

void RTS_CleanPrintFile(void)
{
  for (int j = 0; j < 25; j++) {
    RTS_ResetSingleVP(PRINT_FILE_TEXT_VP + j);
  }    
}

void RTS_LoadMainsiteIcons(void)
{
  rtscheck.RTS_SndData(recovery.enabled ? 101 : 102, POWERCONTINUE_CONTROL_ICON_VP);
  rtscheck.RTS_SndData(runout.enabled ? 101 : 102, FILAMENT_CONTROL_ICON_VP);
  rtscheck.RTS_SndData(lcd_rts_settings.external_m73 ? 206 : 205, EXTERNAL_M73_ICON_VP);
  rtscheck.RTS_SndData(bedlevel.mesh_is_valid() ? 213 : 212, MESH_SIZE_ICON_ONOFF_VP);
}

void RTS_SendLevelingSiteData(uint8_t axis)
{
  if (axis == 0){
  rtscheck.RTS_SndData(lcd_rts_settings.total_probing , PROBE_COUNT_VP);
  rtscheck.RTS_SndData(lcd_rts_settings.max_points, SET_GRID_MAX_POINTS_VP);
  rtscheck.RTS_SndData(lcd_rts_settings.max_points * lcd_rts_settings.max_points, AUTO_BED_LEVEL_END_POINT);
  }
  if (axis == 0 || axis == 1){
    rtscheck.RTS_SndData(lcd_rts_settings.probe_margin_x, PROBE_MARGIN_X_VP);
  }
  if (axis == 0 || axis == 2){    
    rtscheck.RTS_SndData(lcd_rts_settings.probe_margin_y_back, PROBE_MARGIN_Y_VP);
  }
}

void RTS_ShowPage(uint8_t pageNumber)
{
  rtscheck.RTS_SndData(ExchangePageBase + pageNumber, ExchangepageAddr);
  change_page_font = pageNumber;
}

void RTS_ShowPreviewImage(bool status)
{
  #if ENABLED(GCODE_PREVIEW_ENABLED)
    gcodePicDisplayOnOff(DEFAULT_PRINT_MODEL_VP, status);
  #endif
}

void RTS_ShowMotorFreeIcon(bool status)
{
  rtscheck.RTS_SndData(status, MOTOR_FREE_ICON_VP);
  rtscheck.RTS_SndData(status, MOTOR_FREE_ICON_MAIN_VP);
}

void RTS_SendCurrentPosition(uint8_t axis)
{
  if(axis == 4 || axis == 1){
    rtscheck.RTS_SndData(10 * current_position[X_AXIS], AXIS_X_COORD_VP);
  }else if (axis == 4 || axis == 2){
    rtscheck.RTS_SndData(10 * current_position[Y_AXIS], AXIS_Y_COORD_VP);
  }else if (axis == 4 || axis == 3){
    rtscheck.RTS_SndData(10 * current_position[Z_AXIS], AXIS_Z_COORD_VP);
  }
}

void RTS_ResetHeadAndBedSetTemp(void)
{
  RTS_ResetSingleVP(HEAD_SET_TEMP_VP);
  RTS_ResetSingleVP(BED_SET_TEMP_VP);
}

void RTS_SendMachineData(void)
{
  rtscheck.RTS_SndData(MACHINE_TYPE, MACHINE_TYPE_ABOUT_TEXT_VP);
  rtscheck.RTS_SndData(FIRMWARE_VERSION, FIRMWARE_VERSION_ABOUT_TEXT_VP);
  rtscheck.RTS_SndData(HARDWARE_VERSION, HARDWARE_VERSION_ABOUT_TEXT_VP);
  rtscheck.RTS_SndData(PRINT_SIZE, PRINTER_PRINTSIZE_TEXT_VP);
  rtscheck.RTS_SndData(lang == 1 ? CORP_WEBSITE_C : CORP_WEBSITE_E, WEBSITE_ABOUT_TEXT_VP);
}

void RTS_SendBedTemp()
{
  thermalManager.setTargetBed(thermalManager.temp_bed.target);
  rtscheck.RTS_SndData(thermalManager.temp_bed.target, BED_SET_TEMP_VP);
}

void RTS_SendHeadTemp()
{
  thermalManager.setTargetHotend(thermalManager.temp_hotend[0].target, 0);
  rtscheck.RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
}

void RTS_SendHeadCurrentTemp()
{
  rtscheck.RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD_CURRENT_TEMP_VP);
  rtscheck.RTS_SndData(thermalManager.temp_hotend[0].target, HEAD_SET_TEMP_VP);
}

void RTS_SendMoveaxisUnitIcon(uint8_t icon)
{
  rtscheck.RTS_SndData(icon, MOVEAXIS_UNIT_ICON_VP);
}

void RTS_SendDefaultRates()
{
  rtscheck.RTS_SndData(default_max_feedrate[X_AXIS], MAX_VELOCITY_XAXIS_DATA_VP);
  rtscheck.RTS_SndData(default_max_feedrate[Y_AXIS], MAX_VELOCITY_YAXIS_DATA_VP);
  rtscheck.RTS_SndData(default_max_feedrate[Z_AXIS], MAX_VELOCITY_ZAXIS_DATA_VP);
  rtscheck.RTS_SndData(default_max_feedrate[E_AXIS], MAX_VELOCITY_EAXIS_DATA_VP); 
  delay(20);
  rtscheck.RTS_SndData(default_max_acceleration[X_AXIS], MAX_ACCEL_XAXIS_DATA_VP);
  rtscheck.RTS_SndData(default_max_acceleration[Y_AXIS], MAX_ACCEL_YAXIS_DATA_VP);
  rtscheck.RTS_SndData(default_max_acceleration[Z_AXIS], MAX_ACCEL_ZAXIS_DATA_VP);
  rtscheck.RTS_SndData(default_max_acceleration[E_AXIS], MAX_ACCEL_EAXIS_DATA_VP);
  delay(20);
  rtscheck.RTS_SndData(default_max_jerk[X_AXIS] * 100, MAX_JERK_XAXIS_DATA_VP);
  rtscheck.RTS_SndData(default_max_jerk[Y_AXIS] * 100, MAX_JERK_YAXIS_DATA_VP);
  rtscheck.RTS_SndData(default_max_jerk[Z_AXIS] * 100, MAX_JERK_ZAXIS_DATA_VP);
  rtscheck.RTS_SndData(default_max_jerk[E_AXIS] * 100, MAX_JERK_EAXIS_DATA_VP);
  delay(20);
  rtscheck.RTS_SndData(default_axis_steps_per_unit[X_AXIS] * 10, MAX_STEPSMM_XAXIS_DATA_VP);
  rtscheck.RTS_SndData(default_axis_steps_per_unit[Y_AXIS] * 10, MAX_STEPSMM_YAXIS_DATA_VP);
  rtscheck.RTS_SndData(default_axis_steps_per_unit[Z_AXIS] * 10, MAX_STEPSMM_ZAXIS_DATA_VP);
  rtscheck.RTS_SndData(default_axis_steps_per_unit[E_AXIS] * 10, MAX_STEPSMM_EAXIS_DATA_VP);
  delay(20);
  rtscheck.RTS_SndData(default_nozzle_ptemp * 100, NOZZLE_TEMP_P_DATA_VP);
  rtscheck.RTS_SndData(default_nozzle_itemp * 100, NOZZLE_TEMP_I_DATA_VP);
  rtscheck.RTS_SndData(default_nozzle_dtemp * 100, NOZZLE_TEMP_D_DATA_VP);
  delay(20);
  rtscheck.RTS_SndData(default_hotbed_ptemp * 100, HOTBED_TEMP_P_DATA_VP);
  rtscheck.RTS_SndData(default_hotbed_itemp * 100, HOTBED_TEMP_I_DATA_VP);
  rtscheck.RTS_SndData(default_hotbed_dtemp * 10, HOTBED_TEMP_D_DATA_VP);
}

void RTSSHOW::RTS_SendLoadedData(uint8_t loadpart)
{
  if(loadpart == 255 || loadpart == 1){
    rtscheck.RTS_SndData(thermalManager.temp_hotend[0].celsius, HEAD_CURRENT_TEMP_VP);
    rtscheck.RTS_SndData(thermalManager.temp_bed.celsius, BED_CURRENT_TEMP_VP);
    rtscheck.RTS_SndData(thermalManager.fan_speed[0] , PRINTER_FAN_SPEED_DATA_VP);
    GRID_USED_POINTS_X = lcd_rts_settings.max_points;
    GRID_USED_POINTS_Y = lcd_rts_settings.max_points;
  }
  if(loadpart == 255 || loadpart == 2){
    // M203
    rtscheck.RTS_SndData(planner.settings.max_feedrate_mm_s[X_AXIS], MAX_VELOCITY_XAXIS_DATA_VP);
    rtscheck.RTS_SndData(planner.settings.max_feedrate_mm_s[Y_AXIS], MAX_VELOCITY_YAXIS_DATA_VP);
    rtscheck.RTS_SndData(planner.settings.max_feedrate_mm_s[Z_AXIS], MAX_VELOCITY_ZAXIS_DATA_VP);
    rtscheck.RTS_SndData(planner.settings.max_feedrate_mm_s[E_AXIS], MAX_VELOCITY_EAXIS_DATA_VP);
    // M201
    rtscheck.RTS_SndData(planner.settings.max_acceleration_mm_per_s2[0], MAX_ACCEL_XAXIS_DATA_VP);
    rtscheck.RTS_SndData(planner.settings.max_acceleration_mm_per_s2[1], MAX_ACCEL_YAXIS_DATA_VP);
    rtscheck.RTS_SndData(planner.settings.max_acceleration_mm_per_s2[2], MAX_ACCEL_ZAXIS_DATA_VP);
    rtscheck.RTS_SndData(planner.settings.max_acceleration_mm_per_s2[3], MAX_ACCEL_EAXIS_DATA_VP);
    // M92
    rtscheck.RTS_SndData(planner.settings.axis_steps_per_mm[0] * 10, MAX_STEPSMM_XAXIS_DATA_VP);
    rtscheck.RTS_SndData(planner.settings.axis_steps_per_mm[1] * 10, MAX_STEPSMM_YAXIS_DATA_VP);
    rtscheck.RTS_SndData(planner.settings.axis_steps_per_mm[2] * 10, MAX_STEPSMM_ZAXIS_DATA_VP);
    rtscheck.RTS_SndData(planner.settings.axis_steps_per_mm[3] * 10, MAX_STEPSMM_EAXIS_DATA_VP);
  }
  if(loadpart == 255 || loadpart == 3){
    //M205
    rtscheck.RTS_SndData(planner.max_jerk.x * 100, MAX_JERK_XAXIS_DATA_VP);
    rtscheck.RTS_SndData(planner.max_jerk.y * 100, MAX_JERK_YAXIS_DATA_VP);
    rtscheck.RTS_SndData(planner.max_jerk.z * 100, MAX_JERK_ZAXIS_DATA_VP);
    rtscheck.RTS_SndData(planner.max_jerk.e * 100, MAX_JERK_EAXIS_DATA_VP);
  }
  if(loadpart == 255 || loadpart == 4){
    rtscheck.RTS_SndData(thermalManager.temp_hotend[0].pid.p() * 100, NOZZLE_TEMP_P_DATA_VP);
    rtscheck.RTS_SndData(thermalManager.temp_hotend[0].pid.i() * 100, NOZZLE_TEMP_I_DATA_VP);
    rtscheck.RTS_SndData(thermalManager.temp_hotend[0].pid.d() * 100, NOZZLE_TEMP_D_DATA_VP);
    rtscheck.RTS_SndData(thermalManager.temp_bed.pid.p() * 100, HOTBED_TEMP_P_DATA_VP);
    rtscheck.RTS_SndData(thermalManager.temp_bed.pid.i() * 100, HOTBED_TEMP_I_DATA_VP);
    rtscheck.RTS_SndData(thermalManager.temp_bed.pid.d() * 10, HOTBED_TEMP_D_DATA_VP);
  }
  if(loadpart == 255 || loadpart == 5){
    rtscheck.RTS_SndData(stepper.get_shaping_frequency(X_AXIS) * 100, SHAPING_X_FREQUENCY_VP);
    rtscheck.RTS_SndData(stepper.get_shaping_frequency(Y_AXIS) * 100, SHAPING_Y_FREQUENCY_VP);
    rtscheck.RTS_SndData(stepper.get_shaping_damping_ratio(X_AXIS) * 100, SHAPING_X_ZETA_VP);
    rtscheck.RTS_SndData(stepper.get_shaping_damping_ratio(Y_AXIS) * 100, SHAPING_Y_ZETA_VP);
    rtscheck.RTS_SndData(planner.extruder_advance_K[0] * 1000, ADVANCE_K_SET);  
  }
  if(loadpart == 255 || loadpart == 6){   
    rtscheck.RTS_SndData(planner.flow_percentage[0], E0_SET_FLOW_VP);  
  }
}

void RTS_GetRemainTime(void)
{
  rtscheck.RTS_SndData(ui.get_remaining_time() / 3600, PRINT_REMAIN_TIME_HOUR_VP);
  rtscheck.RTS_SndData((ui.get_remaining_time() % 3600) / 60, PRINT_REMAIN_TIME_MIN_VP);
  RTS_SendProgress((unsigned char) ui.get_progress_percent());
}

void RTS_ResetHotendBed(void)
{
  thermalManager.setTargetHotend(0, 0);
  thermalManager.setTargetBed(0);
}

void RTS_G28MoveNow(void)
{
  RTS_ShowPage(40);
  queue.enqueue_now_P(PSTR("G28"));
  RTS_ShowMotorFreeIcon(false);
  Update_Time_Value = 0;  
}

void RTS_G28MoveOne(void)
{
  queue.enqueue_one_P(PSTR("G28"));
  RTS_ShowPage(40);
}

void RTS_TrammingPosition(uint8_t xx, uint8_t xy, uint8_t yx, uint8_t yy)
{
  queue.enqueue_now_P(PSTR("G1 F600 Z3"));
  sprintf_P(cmdbuf, "G1 X%d Y%d F3000", manual_level_5position[xx][xy],manual_level_5position[yx][yy]);
  queue.enqueue_now_P(cmdbuf);  
  rtscheck.RTS_SndData(10 * manual_level_5position[xx][xy], AXIS_X_COORD_VP);
  rtscheck.RTS_SndData(10 * manual_level_5position[yx][yy], AXIS_Y_COORD_VP); 
  queue.enqueue_now_P(PSTR("G1 F600 Z0"));
  RTS_ResetSingleVP(AXIS_Z_COORD_VP);
}

void RTS_SetOneToVP(int vpaddress)
{
  rtscheck.RTS_SndData(1, vpaddress);
}

void RTS_SendZoffsetFeedratePercentage(bool sendzoffset)
{
  if(sendzoffset){
  rtscheck.RTS_SndData(zprobe_zoffset * 100, AUTO_BED_LEVEL_ZOFFSET_VP);
  }
  rtscheck.RTS_SndData(feedrate_percentage, PRINT_SPEED_RATE_VP);  
}

void RTS_AxisZCoord()
{
  queue.enqueue_now_P(PSTR("G1 F600 Z0"));
  rtscheck.RTS_SndData(10 * 0, AXIS_Z_COORD_VP);
}

void RTS_SendProgress(uint8_t progresspercent)
{
  rtscheck.RTS_SndData(progresspercent, PRINT_PROCESS_VP);
  rtscheck.RTS_SndData(progresspercent, PRINT_PROCESS_ICON_VP);  
}

void RTS_ResetPrintData(bool defaultpic)
{
  Percentrecord = 0;
  RTS_ResetSingleVP(PRINT_LAYERS_VP);
  RTS_ResetSingleVP(PRINT_LAYERS_DONE_VP);
  RTS_ResetSingleVP(PRINT_CURRENT_Z_VP);
  RTS_ResetSingleVP(PRINT_FILAMENT_M_VP);
  RTS_ResetSingleVP(PRINT_FILAMENT_M_TODO_VP);
  RTS_ResetSingleVP(PRINT_FILAMENT_G_VP);
  RTS_ResetSingleVP(PRINT_FILAMENT_G_TODO_VP);
  RTS_ResetSingleVP(PRINT_LAYER_HEIGHT_VP);
  RTS_ResetSingleVP(PRINT_PROCESS_ICON_VP);
  RTS_ResetSingleVP(PRINT_PROCESS_ICON_VP);  
  RTS_ResetSingleVP(PRINT_PROCESS_VP);
  RTS_ResetSingleVP(PRINT_REMAIN_TIME_HOUR_VP);
  RTS_ResetSingleVP(PRINT_REMAIN_TIME_MIN_VP);
  RTS_ResetSingleVP(PRINT_TIME_HOUR_VP);
  RTS_ResetSingleVP(PRINT_TIME_MIN_VP);
  if(defaultpic){
    RTS_ShowPreviewImage(true);
  }else{
    RTS_ShowPreviewImage(false);    
  }

}

void RTS_ResetProgress()
{
  Percentrecord = 0;
  RTS_ResetSingleVP(PRINT_PROCESS_ICON_VP);
  RTS_ResetSingleVP(PRINT_PROCESS_ICON_VP);  
  RTS_ResetSingleVP(PRINT_PROCESS_VP);
  RTS_ResetSingleVP(PRINT_REMAIN_TIME_HOUR_VP);
  RTS_ResetSingleVP(PRINT_REMAIN_TIME_MIN_VP);
  RTS_ResetSingleVP(PRINT_TIME_HOUR_VP);
  RTS_ResetSingleVP(PRINT_TIME_MIN_VP);  
}

void RTS_SendPrintData(void)
{
  rtscheck.RTS_SndData(picLayers, PRINT_LAYERS_VP);
  rtscheck.RTS_SndData(picFilament_m, PRINT_FILAMENT_M_VP);
  rtscheck.RTS_SndData(picFilament_m, PRINT_FILAMENT_M_TODO_VP);
  rtscheck.RTS_SndData(picFilament_g, PRINT_FILAMENT_G_VP);
  rtscheck.RTS_SndData(picFilament_g, PRINT_FILAMENT_G_TODO_VP);              
  rtscheck.RTS_SndData(picLayerHeight * 100, PRINT_LAYER_HEIGHT_VP);
}

void RTS_ResetSingleVP(int vpaddress)
{
  rtscheck.RTS_SndData(0, vpaddress);  
}

void RTS_SetProbeCount(uint8_t probescount, uint8_t m19load)
{
  if (probescount < 1){
    probescount = 1;
  }
  if (probescount > 5){
    probescount = 5;
  }
  lcd_rts_settings.total_probing = probescount;
  rtscheck.RTS_SndData(probescount, PROBE_COUNT_VP);
  RTS_SetBltouchHSMode();
}

void RTS_SetProbeMarginX(uint8_t marginx, uint8_t m19load)
{
  probe_offset_x_temp = fabs(probe.offset_xy.x);
  int max_reachable_pos_x = X_MAX_POS - custom_ceil(probe_offset_x_temp);
  int min_calc_margin_x = X_BED_SIZE - max_reachable_pos_x;
  min_calc_margin_x = fabs(min_calc_margin_x); // Ensure it's positive
  if(min_calc_margin_x > marginx){
  marginx = min_calc_margin_x;
  }
  #if ENABLED(ENDER_3S1_PLUS)
    if(marginx <= 27){
      marginx= 27;
    }
  #endif
  lcd_rts_settings.probe_margin_x = marginx;
  #if ENABLED(LCD_RTS_DEBUG_MARGIN_X)
    SERIAL_ECHO_MSG("SetX min_calc_margin_x: ", min_calc_margin_x);
    SERIAL_ECHO_MSG("SetX lcd_rts_settings.probe_margin_x: ", lcd_rts_settings.probe_margin_x);
  #endif           
  RTS_SendLevelingSiteData(1);
  if (m19load == 0){    
    settings.save();
  }
}

void RTS_SetProbeMarginY(uint8_t marginy, uint8_t m19load)
{
  probe_offset_y_temp = fabs(probe.offset_xy.y);
  int max_reachable_pos_y = Y_MAX_POS - custom_ceil(probe_offset_y_temp);
  int min_calc_margin_y = Y_BED_SIZE - max_reachable_pos_y;
  if(min_calc_margin_y <= 10){
    min_calc_margin_y=10;
  }
  if(min_calc_margin_y > marginy){
    if(lcd_rts_settings.probe_margin_x <= min_calc_margin_y){      
      lcd_rts_settings.probe_margin_y_front = lcd_rts_settings.probe_margin_x;
      lcd_rts_settings.probe_margin_y_back = min_calc_margin_y;
    }else{
      lcd_rts_settings.probe_margin_y_front = min_calc_margin_y;
      lcd_rts_settings.probe_margin_y_back = min_calc_margin_y;
    }
  }else if(min_calc_margin_y < marginy){
    if(lcd_rts_settings.probe_margin_x <= marginy){
      lcd_rts_settings.probe_margin_y_front = lcd_rts_settings.probe_margin_x;
      lcd_rts_settings.probe_margin_y_back = marginy;
    }else{    
      lcd_rts_settings.probe_margin_y_front = marginy;
      lcd_rts_settings.probe_margin_y_back = marginy;
    }
  }else if(min_calc_margin_y == marginy){
    lcd_rts_settings.probe_margin_y_front = marginy;
    lcd_rts_settings.probe_margin_y_back = marginy;
  }
  #if ENABLED(LCD_RTS_DEBUG_MARGIN_Y)
    SERIAL_ECHO_MSG("SetX min_calc_margin_y: ", min_calc_margin_y);
    SERIAL_ECHO_MSG("SetX lcd_rts_settings.probe_margin_y_front: ", lcd_rts_settings.probe_margin_y_front);
    SERIAL_ECHO_MSG("SetX lcd_rts_settings.probe_margin_y_back: ", lcd_rts_settings.probe_margin_y_back);
  #endif    
  RTS_SendLevelingSiteData(2);
  if (m19load == 0){
    settings.save();
  }
}

void RTS_SetGridMaxPoints(uint8_t gridmaxpoints, uint8_t m19load)
{
  if (gridmaxpoints == 5 || gridmaxpoints == 7 || gridmaxpoints == 10){
    RTS_ResetMesh();
    lcd_rts_settings.max_points = gridmaxpoints;
    RTS_LoadMeshPointOffsets();
    RTS_SetBltouchHSMode();
    rtscheck.RTS_SndData(lcd_rts_settings.max_points * lcd_rts_settings.max_points, AUTO_BED_LEVEL_END_POINT);
    rtscheck.RTS_SndData(gridmaxpoints, SET_GRID_MAX_POINTS_VP);
    bedlevel.max_points.x = gridmaxpoints;
    bedlevel.max_points.y = gridmaxpoints;
    if (settingsload != 1) queue.enqueue_now_P(PSTR("M84"));
    queue.enqueue_now_P(PSTR("G92.9Z0"));
    if (m19load == 0){
      rtscheck.RTS_ChangeLevelingPage();
    }
    RTS_ShowMotorFreeIcon(true);
  }
} 

void RTS_SendLang(int vpaddress)
{
  rtscheck.RTS_SndData(lang, vpaddress);
}

void RTS_MoveAxisHoming(void)
{
  if(waitway == 2)
  {
    RTS_ShowPage(1);
    waitway = 0;
  }
  else if(waitway == 4)
  {
    rtscheck.RTS_SndData(ExchangePageBase + 16 + (AxisUnitMode - 1), ExchangepageAddr);
    change_page_font = 16;
    waitway = 0;
  }  
  else if(waitway == 6)
  {
    waitway = 0;
  }
  else if(waitway == 7)
  {
    // Click Print finish
    RTS_ShowPage(1);
    RTS_ShowPreviewImage(true);
    waitway = 0;
  }else if(waitway == 8)
  {
    rtscheck.RTS_SndData(ExchangePageBase + 78 + (AxisUnitMode - 1), ExchangepageAddr);
    change_page_font = 78 + (AxisUnitMode - 1);
    waitway = 0;
  }else if(waitway == 9)
  {
    rtscheck.RTS_SndData(ExchangePageBase + 70 + (AxisUnitMode - 1), ExchangepageAddr);
    change_page_font = 70 + (AxisUnitMode - 1);
    waitway = 0;
  }else if(waitway == 10){
    RTS_ShowPage(51);
    waitway = 0;
  }else if(waitway == 14)
  {
    rtscheck.RTS_SndData(ExchangePageBase + 86 + (AxisUnitMode - 1), ExchangepageAddr);
    change_page_font = 86;
    waitway = 0;
  }
  else if(waitway == 15)
    {
      waitway = 0;
      RTS_AutoBedLevelPage();      
    }      
  else if(waitway == 16)
    {
      RTS_ShowPage(25);
      waitway = 0;
    } 
  else if(waitway == 17)
    {
      RTS_ShowPage(89);
      waitway = 0;
    }
  else if(waitway == 18)
    {
      RTS_ShowPage(98);
      waitway = 0;
    }           

  #if HAS_CUTTER
    if(laser_device.is_laser_device())
    {
      
    }else
  #endif
  {
    RTS_SendCurrentPosition(1);
    RTS_SendCurrentPosition(2);
  }
  RTS_SendCurrentPosition(3);
}

void RTS_CommandPause(void)
{
  if(printingIsActive())
  {
        RTS_LoadMainsiteIcons();
        RTS_ShowPage(13);
  }
}

void RTS_SendM600Icon(bool icon)
{
  rtscheck.RTS_SndData(icon ? 208 : 207, EXTERNAL_M600_ICON_VP);
}

void RTS_SendM73Icon(bool icon)
{
  rtscheck.RTS_SndData(icon ? 206 : 205, EXTERNAL_M73_ICON_VP);
}

void ErrorHanding(void)
{
  // No more operations
  if(errorway == 1)
  {
    errorway = errornum = 0;
  }
  else if(errorway == 2)
  {
    // Z axis home failed
    home_errornum ++;
    if(home_errornum <= 3)
    {
      errorway = 0;
      waitway  = 4;
      RTS_G28MoveNow();
    }
    else
    {
      // After three failed returns to home, it will report the failure interface
      home_errornum = 0;
      errorway = 0;
      RTS_ShowPage(41);
      // Z axis home failed
      rtscheck.RTS_SndData(Error_202, ABNORMAL_PAGE_TEXT_VP);
      if(printingIsActive())
      {
        rtscheck.RTS_SDcard_Stop();
        RTS_ShowPage(1);
        Update_Time_Value = 0;
      }
    }
  }
  else if(errorway == 3)
  {
    // No more operations
    reset_bed_level();
    errorway = 0;
    errornum = 0;
  }
  else if(errorway == 4)
  {

  }
}

#endif
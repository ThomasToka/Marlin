#ifndef LCD_RTS_H
#define LCD_RTS_H
#include "../../../sd/cardreader.h"
#include "string.h"
//#include <arduino.h>
#include "../../../libs/BL24CXX.h"
#include "../../../inc/MarlinConfig.h"
//#include "lcd_rts_defines.h"

extern bool power_off_type_yes;

// #pragma once
/*********************************/

#define PID_PARAM(F,H) _PID_##F(TERN(PID_PARAMS_PER_HOTEND, H, 0 & H)) // Always use 'H' to suppress warning
#define _PID_Kp(H) TERN(PIDTEMP, Temperature::temp_hotend[H].pid.Kp, NAN)
#define _PID_Ki(H) TERN(PIDTEMP, Temperature::temp_hotend[H].pid.Ki, NAN)
#define _PID_Kd(H) TERN(PIDTEMP, Temperature::temp_hotend[H].pid.Kd, NAN)
#if ENABLED(PIDTEMP)
  #define _PID_Kc(H) TERN(PID_EXTRUSION_SCALING, Temperature::temp_hotend[H].pid.Kc, 1)
  #define _PID_Kf(H) TERN(PID_FAN_SCALING,       Temperature::temp_hotend[H].pid.Kf, 0)
#else
  #define _PID_Kc(H) 1
  #define _PID_Kf(H) 0
#endif

#if ENABLED(ENDER_3S1_PLUS) || ENABLED(ENDER_3S1_PRO)
#define FHONE   (0x5A)
#endif
#define FHTWO   (0xA5)
#define FHLENG  (0x06)
#define TEXTBYTELEN     55
#define MaxFileNumber   40
//#if ENABLED(LCD_RTS_SOFTWARE_AUTOSCROLL) 
#define MaxFilenameLength   128
//#endif
#define VALUE_INVALID                       0xFFFF
#define VALUE_INVALID_8BIT                  0xFF

#define AUTO_BED_LEVEL_PREHEAT  0

#define FileNum             MaxFileNumber
#define FileNameLen         TEXTBYTELEN
#define RTS_UPDATE_INTERVAL 500
#define RTS_UPDATE_VALUE    RTS_UPDATE_INTERVAL

#define SizeofDatabuf       26

/*************Register and Variable addr*****************/
#define RegAddr_W   0x80
#define RegAddr_R   0x81
#define VarAddr_W   0x82
#define VarAddr_R   0x83
#define ExchangePageBase    ((unsigned long)0x5A010000)
#define StartSoundSet       ((unsigned long)0x060480A0)

#define WRITE_CURVE_DDR_CMD ((unsigned long)0x030B)

const uint8_t DGUS_READVAR = 0x83;
const uint16_t DGUS_VERSION = 0x000F;

/*Error value*/
#define Error_201   "201 (Command Timeout)"   // The command too much inactive time
#define Error_202   "202 (Homing Failed)"     // Homing Failed
#define Error_203   "203 (Probing Failed)"    // Probing Failed
#define Error_204   "204 (Click Reboot)"      // SD Read Error

/*variable addr*/
#define ExchangepageAddr      0x0084
#define SoundAddr             0x00A0

#define PAGE_STATUS_TEXT_CURRENT_VP        0x10CC
#define PAGE_STATUS_TEXT_TOTAL_VP          0x10D0

#define SELECT_FILE_TEXT_VP                0x219A // uses space up to 0x21D5 next free 0x21D6
#define RECTANGLE_VP                       0x2490

#define START_PROCESS_ICON_VP              0x1000
#define PRINT_SPEED_RATE_VP                0x1006
#define PRINT_PROCESS_ICON_VP              0x100E
#define PRINT_TIME_HOUR_VP                 0x1010
#define PRINT_TIME_MIN_VP                  0x1012
#define PRINT_PROCESS_VP                   0x1016
#define PRINT_LAYERS_VP                    0x1018 // 0x1018, 0x1019, next free is 0x101A
#define PRINT_LAYERS_DONE_VP               0x101A // 0x101A, 0x101B, next free is 0x101C
#define PRINT_FILAMENT_G_VP                0x101C // 0x101C, 0x101D, next free is 0x101E
#define PRINT_FILAMENT_G_TODO_VP           0x101E // 0x101E, 0x101F, next free is 0x1020
#define PRINT_FILAMENT_M_VP                0x1020 // 0x1020, 0x1021, next free is 0x1022
#define PRINT_FILAMENT_M_TODO_VP           0x1022 // 0x1022, 0x1023, next free is 0x1024
#define PRINT_LAYER_HEIGHT_VP              0x169A // 0x169A, 0x169B, 0x169C, 0x169D next free is 0x169E
#define PRINT_CURRENT_Z_VP                 0x169E // 0x169E, 0x169F, 0x16A0, 0x16A1 next free is 0x16A2
#define AUTO_BED_LEVEL_ZOFFSET_VP          0x1026
#define AUTO_BED_LEVEL_ZOFFSET005_VP       0x2213

#define HEAD_SET_TEMP_VP                   0x1034
#define HEAD_CURRENT_TEMP_VP               0x1036
#define BED_SET_TEMP_VP                    0x103A
#define BED_CURRENT_TEMP_VP                0x103C
#define AXIS_X_COORD_VP                    0x1048
#define AXIS_Y_COORD_VP                    0x104A
#define AXIS_Z_COORD_VP                    0x104C
#define HEAD_FILAMENT_LOAD_DATA_VP         0x1052
#define HEAD_FILAMENT_UNLOAD_DATA_VP       0x1054
#define AUTO_BED_PREHEAT_HEAD_DATA_VP      0x108A
#define AUTO_BED_LEVEL_TITLE_VP            0x108D
#define FILAMENT_LOAD_ICON_VP              0x108E
#define PREHEAT_PLA_SET_NOZZLE_TEMP_VP     0x1090
#define PREHEAT_PLA_SET_BED_TEMP_VP        0x1092
#define PREHEAT_ABS_SET_NOZZLE_TEMP_VP     0x1094
#define PREHEAT_ABS_SET_BED_TEMP_VP        0x1096
#define FAN_SPEED_CONTROL_DATA_VP          0x109A
#define AUTO_LEVELING_PERCENT_DATA_VP      0x109C

#define MAX_VELOCITY_XAXIS_DATA_VP         0x109E
#define MAX_VELOCITY_YAXIS_DATA_VP         0x10A0
#define MAX_VELOCITY_ZAXIS_DATA_VP         0x10A2
#define MAX_VELOCITY_EAXIS_DATA_VP         0x10A4

#define MAX_ACCEL_XAXIS_DATA_VP            0x10A6
#define MAX_ACCEL_YAXIS_DATA_VP            0x10A8
#define MAX_ACCEL_ZAXIS_DATA_VP            0x10AA
#define MAX_ACCEL_EAXIS_DATA_VP            0x10AC

#define MAX_JERK_XAXIS_DATA_VP             0x10AE
#define MAX_JERK_YAXIS_DATA_VP             0x10B0
#define MAX_JERK_ZAXIS_DATA_VP             0x10B2
#define MAX_JERK_EAXIS_DATA_VP             0x10B4

#define MAX_STEPSMM_XAXIS_DATA_VP          0x10B6
#define MAX_STEPSMM_YAXIS_DATA_VP          0x10B8
#define MAX_STEPSMM_ZAXIS_DATA_VP          0x10BA
#define MAX_STEPSMM_EAXIS_DATA_VP          0x10BC

#define NOZZLE_TEMP_P_DATA_VP              0x10BE
#define NOZZLE_TEMP_I_DATA_VP              0x10C0
#define NOZZLE_TEMP_D_DATA_VP              0x10C2
#define HOTBED_TEMP_P_DATA_VP              0x10C4
#define HOTBED_TEMP_I_DATA_VP              0x10C6
#define HOTBED_TEMP_D_DATA_VP              0x10C8
#define PRINTER_FAN_SPEED_DATA_VP          0x10CA

#define PID_TUNING_RUNNING_VP             0x1001
#define PID_ICON_MODE_VP                  0x1003
#define PID_TEXT_OUT_VP                   0x205B

#define PRINT_REMAIN_TIME_HOUR_VP          0x1171
#define PRINT_REMAIN_TIME_MIN_VP           0x1173

#define AUTO_BED_LEVEL_1POINT_NEW_VP       0x4000

#define AUTO_BED_LEVEL_CUR_POINT_VP        0x0110
#define AUTO_BED_LEVEL_END_POINT           0x0112
#define AUTO_BED_LEVEL_MESH_VP             0x0111

#define AUTO_BED_LEVEL_1POINT_VP           0x1100
#define AUTO_BED_LEVEL_2POINT_VP           0x1102
#define AUTO_BED_LEVEL_3POINT_VP           0x1104
#define AUTO_BED_LEVEL_4POINT_VP           0x1106
#define AUTO_BED_LEVEL_5POINT_VP           0x1108
#define AUTO_BED_LEVEL_6POINT_VP           0x110A
#define AUTO_BED_LEVEL_7POINT_VP           0x110C
#define AUTO_BED_LEVEL_8POINT_VP           0x110E
#define AUTO_BED_LEVEL_9POINT_VP           0x1110
#define AUTO_BED_LEVEL_10POINT_VP          0x1112
#define AUTO_BED_LEVEL_11POINT_VP          0x1114
#define AUTO_BED_LEVEL_12POINT_VP          0x1116
#define AUTO_BED_LEVEL_13POINT_VP          0x1118
#define AUTO_BED_LEVEL_14POINT_VP          0x111A
#define AUTO_BED_LEVEL_15POINT_VP          0x111C
#define AUTO_BED_LEVEL_16POINT_VP          0x111E
#define AUTO_BED_LEVEL_17POINT_VP          0x110E
#define AUTO_BED_LEVEL_18POINT_VP          0x1110
#define AUTO_BED_LEVEL_19POINT_VP          0x1112
#define AUTO_BED_LEVEL_20POINT_VP          0x1114
#define AUTO_BED_LEVEL_21POINT_VP          0x1116
#define AUTO_BED_LEVEL_22POINT_VP          0x1118
#define AUTO_BED_LEVEL_23POINT_VP          0x111A
#define AUTO_BED_LEVEL_24POINT_VP          0x111C
#define AUTO_BED_LEVEL_25POINT_VP          0x111E

#define MOVEAXIS_UNIT_ICON_VP              0x116A
#define PREHAEAT_NOZZLE_ICON_VP            0x116B
#define PREHAEAT_HOTBED_ICON_VP            0x116C
#define FILAMENT_CONTROL_ICON_VP           0x116D
#define POWERCONTINUE_CONTROL_ICON_VP      0x116E
#define CHANGE_SDCARD_ICON_VP              0x116F

#define PRINT_FINISH_ICON_VP               0x1170
#define MOTOR_FREE_ICON_VP                 0x1200
#define MOTOR_FREE_ICON_MAIN_VP            0x1202
#define FILE1_SELECT_ICON_VP               0x1221
#define FILE2_SELECT_ICON_VP               0x1222
#define FILE3_SELECT_ICON_VP               0x1223
#define FILE4_SELECT_ICON_VP               0x1224
#define FILE5_SELECT_ICON_VP               0x1225
// used for file icons as of v017
#define FILE6_SELECT_ICON_VP               0x1226
#define FILE7_SELECT_ICON_VP               0x1227
#define FILE8_SELECT_ICON_VP               0x1228
#define FILE9_SELECT_ICON_VP               0x1229
#define FILE10_SELECT_ICON_VP              0x122A
#define FILE11_SELECT_ICON_VP              0x122B
#define FILE12_SELECT_ICON_VP              0x122C
#define FILE13_SELECT_ICON_VP              0x122D
#define FILE14_SELECT_ICON_VP              0x122E
#define FILE15_SELECT_ICON_VP              0x122F
#define FILE16_SELECT_ICON_VP              0x1230
#define FILE17_SELECT_ICON_VP              0x1231
#define FILE18_SELECT_ICON_VP              0x1232
#define FILE19_SELECT_ICON_VP              0x1233
#define FILE20_SELECT_ICON_VP              0x1234

#define FILE1_TEXT_VP                      0x200A
#define FILE2_TEXT_VP                      0x201E
#define FILE3_TEXT_VP                      0x2032
#define FILE4_TEXT_VP                      0x2046
#define FILE5_TEXT_VP                      0x205A
#define FILE6_TEXT_VP                      0x206E
#define FILE7_TEXT_VP                      0x2082
#define FILE8_TEXT_VP                      0x2096
#define FILE9_TEXT_VP                      0x20AA
#define FILE10_TEXT_VP                     0x20BE
#define FILE11_TEXT_VP                     0x20D2
#define FILE12_TEXT_VP                     0x20E6
#define FILE13_TEXT_VP                     0x20FA
#define FILE14_TEXT_VP                     0x210E
#define FILE15_TEXT_VP                     0x2122
#define FILE16_TEXT_VP                     0x2136
#define FILE17_TEXT_VP                     0x214A
#define FILE18_TEXT_VP                     0x215E
#define FILE19_TEXT_VP                     0x2172
#define FILE20_TEXT_VP                     0x2186

// uses sp space from User 0x6C9D up to 0x6CB5 next free is 0x6CB6
#define PRINT_FILE_TEXT_VP                 0x2238 // 25 chars up to 0x2250 next free 0x2251
#define ABNORMAL_PAGE_TEXT_VP              0x21D6 // with 30 chars up to 0x21F3 next free 0x21F4


#define MAIN_PAGE_BLUE_TITLE_VP            0x1300
#define SELECT_FILE_BLUE_TITLE_VP          0x1301
#define PREPARE_PAGE_BLUE_TITLE_VP         0x1302
#define SETTING_PAGE_BLUE_TITLE_VP         0x1303
#define MAIN_PAGE_BLACK_TITLE_VP           0x1304
#define SELECT_FILE_BLACK_TITLE_VP         0x1305
#define PREPARE_PAGE_BLACK_TITLE_VP        0x1306
#define SETTING_PAGE_BLACK_TITLE_VP        0x1307

#define PRINT_ADJUST_MENT_TITLE_VP         0x130D
#define PRINT_SPEED_TITLE_VP               0x130E
#define HEAD_SET_TITLE_VP                  0x130F
#define BED_SET_TITLE_VP                   0x1310
#define LEVEL_ZOFFSET_TITLE_VP             0x1311
#define FAN_CONTROL_TITLE_VP               0x1312
#define LED_CONTROL_TITLE_VP               0x1313

#define MOVE_AXIS_ENTER_GREY_TITLE_VP      0x1314
#define CHANGE_FILAMENT_GREY_TITLE_VP      0x1315
#define PREHAET_PAGE_GREY_TITLE_VP         0x1316
#define MOVE_AXIS_ENTER_BLACK_TITLE_VP     0x1317
#define CHANGE_FILAMENT_BLACK_TITLE_VP     0x1318
#define PREHAET_PAGE_BLACK_TITLE_VP        0x1319

#define PREHEAT_PLA_BUTTON_TITLE_VP        0x131A
#define PREHEAT_ABS_BUTTON_TITLE_VP        0x131B
#define COOL_DOWN_BUTTON_TITLE_VP          0x131C

#define FILAMENT_LOAD_BUTTON_TITLE_VP      0x1321
#define FILAMENT_UNLOAD_BUTTON_TITLE_VP    0x1322

#define LANGUAGE_SELECT_ENTER_VP           0x1323
#define FACTORY_DEFAULT_ENTER_TITLE_VP     0x1324
#define LEVELING_PAGE_TITLE_VP             0x1325

#define PRINTER_DEVICE_GREY_TITLE_VP       0x1326
#define PRINTER_ADVINFO_GREY_TITLE_VP      0x1327
#define PRINTER_INFO_ENTER_GREY_TITLE_VP   0x1328
#define PRINTER_DEVICE_BLACK_TITLE_VP      0x1329
#define PRINTER_ADVINFO_BLACK_TITLE_VP     0x132A
#define PRINTER_INFO_ENTER_BLACK_TITLE_VP  0x132B

#define PREHEAT_PLA_SET_TITLE_VP           0x132D
#define PREHEAT_ABS_SET_TITLE_VP           0x132E

#define STORE_MEMORY_CONFIRM_TITLE_VP      0x1332
#define STORE_MEMORY_CANCEL_TITLE_VP       0x1333

#define FILAMENT_UNLOAD_IGNORE_TITLE_VP    0x133E
#define FILAMENT_USEUP_TITLE_VP            0x133F
#define BUTTON_CHECK_CONFIRM_TITLE_VP      0x1340
#define BUTTON_CHECK_CANCEL_TITLE_VP       0x1341
#define FILAMENT_LOAD_TITLE_VP             0x1342
#define FILAMENT_LOAD_RESUME_TITLE_VP      0x1343
#define PAUSE_PRINT_POP_TITLE_VP           0x1344
#define STOP_PRINT_POP_TITLE_VP            0x1347
#define POWERCONTINUE_POP_TITLE_VP         0x1348
#define AUTO_HOME_WAITING_POP_TITLE_VP     0x1349

#define BEDLEVELING_WAIT_TITLE_VP          0x134B
#define RESTORE_FACTORY_TITLE_VP           0x134D
#define RESET_WIFI_SETTING_TITLE_VP        0x134E
#define KILL_THERMAL_RUNAWAY_TITLE_VP      0x134F
#define KILL_HEATING_FAIL_TITLE_VP         0x1350
#define KILL_THERMISTOR_ERROR_TITLE_VP     0x1351
#define WIND_AUTO_SHUTDOWN_TITLE_VP        0x1352
#define RESET_WIFI_SETTING_BUTTON_VP       0x1353
#define PRINTER_AUTO_SHUTDOWN_TITLE_VP     0x1354
#define WIND_AUTO_SHUTDOWN_PAGE_VP         0x1355
#define AUTO_LEVELING_START_TITLE_VP       0x1356
#define AUX_LEVELING_GREY_TITLE_VP         0x1357
#define AUTO_LEVELING_GREY_TITLE_VP        0x1358
#define AUX_LEVELING_BLACK_TITLE_VP        0x1359
#define AUTO_LEVELING_BLACK_TITLE_VP       0x135A
#define LANGUAGE_SELECT_PAGE_TITLE_VP      0x135B
#define ADV_SETTING_MOTION_TITLE_VP        0x135C
#define ADV_SETTING_PID_TITLE_VP           0x135D
#define ADV_SETTING_WIFI_TITLE_VP          0x135E

#define MOTION_SETTING_TITLE_VP            0x135F
#define MOTION_SETTING_STEPSMM_TITLE_VP    0x1360
#define MOTION_SETTING_ACCEL_TITLE_VP      0x1361
#define MOTION_SETTING_JERK_TITLE_VP       0x1362
#define MOTION_SETTING_VELOCITY_TITLE_VP   0x1363

#define MAX_VELOCITY_SETTING_TITLE_VP      0x1364
#define MAX_VELOCITY_XAXIS_TITLE_VP        0x1365
#define MAX_VELOCITY_YAXIS_TITLE_VP        0x1366
#define MAX_VELOCITY_ZAXIS_TITLE_VP        0x1367
#define MAX_VELOCITY_EAXIS_TITLE_VP        0x1368

#define MAX_ACCEL_SETTING_TITLE_VP         0x1369
#define MAX_ACCEL_XAXIS_TITLE_VP           0x136A
#define MAX_ACCEL_YAXIS_TITLE_VP           0x136B
#define MAX_ACCEL_ZAXIS_TITLE_VP           0x136C
#define MAX_ACCEL_EAXIS_TITLE_VP           0x136D

#define MAX_JERK_SETTING_TITLE_VP          0x136E
#define MAX_JERK_XAXIS_TITLE_VP            0x136F
#define MAX_JERK_YAXIS_TITLE_VP            0x1370
#define MAX_JERK_ZAXIS_TITLE_VP            0x1371
#define MAX_JERK_EAXIS_TITLE_VP            0x1372

#define MAX_STEPSMM_SETTING_TITLE_VP       0x1373
#define MAX_STEPSMM_XAXIS_TITLE_VP         0x1374
#define MAX_STEPSMM_YAXIS_TITLE_VP         0x1375
#define MAX_STEPSMM_ZAXIS_TITLE_VP         0x1376
#define MAX_STEPSMM_EAXIS_TITLE_VP         0x1377

#define TEMP_PID_SETTING_TITLE_VP          0x1378
#define NOZZLE_TEMP_P_TITLE_VP             0x1379
#define NOZZLE_TEMP_I_TITLE_VP             0x137A
#define NOZZLE_TEMP_D_TITLE_VP             0x137B
#define HOTBED_TEMP_P_TITLE_VP             0x137C
#define HOTBED_TEMP_I_TITLE_VP             0x137D
#define HOTBED_TEMP_D_TITLE_VP             0x137E

#define FILAMENT_CONTROL_TITLE_VP          0x137F
#define POWERCONTINUE_CONTROL_TITLE_VP     0x1380

#define PLA_SETTINGS_TITLE_VP              0x1382
#define ABS_SETTINGS_TITLE_VP              0x1384

#define LEVELING_WAY_TITLE_VP              0x1386

#define MACHINE_TYPE_ABOUT_CHAR_VP         0x1400
#define FIRMWARE_VERSION_ABOUT_CHAR_VP     0x1401
#define PRINTER_DISPLAY_VERSION_TITLE_VP   0x1402
#define HARDWARE_VERSION_ABOUT_TITLE_VP    0x1403

#define WEBSITE_ABOUT_CHAR_VP              0x1405
#define PRINTER_PRINTSIZE_TITLE_VP         0x1406

#define LANGUAGE_CHINESE_TITLE_VP          0x1411
#define LANGUAGE_ENGLISH_TITLE_VP          0x1412
#define LANGUAGE_GERMAN_TITLE_VP           0x1413
#define LANGUAGE_SPANISH_TITLE_VP          0x1414
#define LANGUAGE_FRENCH_TITLE_VP           0x1415
#define LANGUAGE_ITALIAN_TITLE_VP          0x1416
#define LANGUAGE_PORTUGUESE_TITLE_VP       0x1417
#define LANGUAGE_RUSSIAN_TITLE_VP          0x1418
#define LANGUAGE_TURKISH_TITLE_VP          0x1419

#define DEFAULT_PRINT_MODEL_VP             0x1420
#define DOWNLOAD_PREVIEW_VP                0x1421

#define SOUND_SETTING_VP                   0x1422
#define SOUND_SETTING_OFF_ON_VP            0x1423

#define AUTO_PID_INLET_VP                  0x1424

#define AUTO_PID_NOZZLE_INLET_VP           0x1427
#define AUTO_PID_NOZZLE_TIS_VP             0x1428
#define AUTO_PID_RUN_NOZZLE_TIS_VP         0x142A
#define AUTO_PID_FINISH_NOZZLE_TIS_VP      0x143A
#define AUTO_PID_START_NOZZLE_VP           0x144A
#define AUTO_PID_SET_NOZZLE_TEMP           0x2209
#define AUTO_PID_SET_NOZZLE_CYCLES         0x2210
#define AUTO_PID_NOZZLE_TEMP               0x147A
#define AUTO_PID_NOZZLE_CYCLES             0x148A
#define PID_TEXT_OUT_CUR_CYCLE_NOZZLE_VP   0x157A

#define AUTO_PID_HOTBED_INLET_VP           0x1425
#define AUTO_PID_HOTBED_TIS_VP             0x1426
#define AUTO_PID_RUN_HOTBED_TIS_VP         0x149A
#define AUTO_PID_FINISH_HOTBED_TIS_VP      0x150A
#define AUTO_PID_START_HOTBED_VP           0x151A 
#define AUTO_PID_SET_HOTBED_TEMP           0x2211
#define AUTO_PID_SET_HOTBED_CYCLES         0x2212
#define AUTO_PID_HOTBED_TEMP               0x154A 
#define AUTO_PID_HOTBED_CYCLES             0x155A
#define PID_TEXT_OUT_CUR_CYCLE_HOTBED_VP   0x158A 

#define MESH_LEVELING_BLACK_TITLE_VP       0x156A
#define ADVANCE_K_SET                      0x2214
#define HOTEND_X_ZOFFSET_VP                0x163A
#define HOTEND_Y_ZOFFSET_VP                0x164A

//#define X_MIN_POS_EEPROM_VP                0x165A
//#define Y_MIN_POS_EEPROM_VP                0x166A
// prepare for hotend_fan
//#define HOTEND_FAN_SPEED_DATA_VP         0x167A
//#define Y_MIN_POS_VP                       0x168A

//#define X_BEDSIZE_VP                       0x169A
//#define Y_BEDSIZE_VP                       0x170A 

//#define HOME_X_OFFSET_NEW_VP               0x171A
//#define HOME_Y_OFFSET_NEW_VP               0x172A

#define SHAPING_X_FREQUENCY_VP             0x173A
#define SHAPING_Y_FREQUENCY_VP             0x174A

#define PREHEAT_PETG_SET_NOZZLE_TEMP_VP    0x175A
#define PREHEAT_PETG_SET_BED_TEMP_VP       0x176A
#define PREHEAT_CUST_SET_NOZZLE_TEMP_VP    0x177A
#define PREHEAT_CUST_SET_BED_TEMP_VP       0x178A

#define PREHEAT_PETG_BUTTON_TITLE_VP       0x179A
#define PREHEAT_CUST_BUTTON_TITLE_VP       0x180A
#define PREHEAT_PETG_SET_TITLE_VP          0x181A
#define PREHEAT_CUST_SET_TITLE_VP          0x182A

#define PETG_SETTINGS_TITLE_VP             0x183A
#define CUST_SETTINGS_TITLE_VP             0x184A

#define SHAPING_X_ZETA_VP                  0x185A
#define SHAPING_Y_ZETA_VP                  0x186A
#define SHAPING_X_TITLE_VP                 0x187A
#define SHAPING_Y_TITLE_VP                 0x188A

#define HOME_X_OFFSET_VP                   0x189A
#define HOME_Y_OFFSET_VP                   0x190A

#define HOME_X_OFFSET_SET_VP               0x191A
#define HOME_Y_OFFSET_SET_VP               0x192A

#define E0_SET_FLOW_VP                     0x193A

// CRTOUCH_TRAMMING_POINT_1_VP to CRTOUCH_TRAMMING_POINT_9_VP with 1adr offset 0x230A, 0x230B, 0x230C, ... to 0x2312
#define CRTOUCH_TRAMMING_POINT_1_VP        0x230A
// ASSISTED_TRAMMING_POINT_1_VP with 1adr offset 0x2313, 0x2314, 0x2315, 0x2316
#define ASSISTED_TRAMMING_POINT_1_VP       0x2313
// ASSISTED_TRAMMING_POINT_TEXT_VP with 26adr offset 0x4110, 0x412A, 0x4144, 0x415E. next free 0x4178 
#define ASSISTED_TRAMMING_POINT_TEXT_VP    0x4110

#define MACHINE_TYPE_ABOUT_TEXT_VP         0x17B0
#define FIRMWARE_VERSION_ABOUT_TEXT_VP     0x17C4
#define PRINTER_DISPLAY_VERSION_TEXT_VP    0x17D8
#define HARDWARE_VERSION_ABOUT_TEXT_VP     0x17EC
#define PRINTER_PRINTSIZE_TEXT_VP          0x1800
#define WEBSITE_ABOUT_TEXT_VP              0x1814

#define FilenameNature                     0x6003
#define TrammingpointNature                0x6153

//#define QR_CODE_1_VP                       0x6C40 // uses space up to 0x6C67
// 0x6C68 up to 0x6C9C is sp space for SELECT_PRINT_FILE_VP next free 0x6C9D
#define MESH_RECTANGLE_BASE_VP             0x7000

#define ABNORMAL_PAGE_TEXT_VP_SIZE         30  

#if HAS_CUTTER
  #define SELECT_LASER_WARNING_TIPS_VP      0x1381
  #define SELECT_FDM_WARNING_TIPS_VP        0x1382  
  #define PRINT_MOVE_AXIS_VP                0x1383
  #define PRINT_DIRECT_ENGRAV_VP            0x1384
  #define PRINT_RUN_RANGE_VP                0x1385
  #define PRINT_RETURN_VP                   0x1386
  #define PRINT_WARNING_TIPS_VP             0x1387
  #define DEVICE_SWITCH_LASER_VP            0x1388
  #define FIRST_SELECT_DEVICE_TYPE          0x1389
  #define HOME_LASER_ENGRAVE_VP             0x138A
  #define PREPARE_ADJUST_FOCUS_VP           0x138B
  #define PREPARE_SWITCH_FDM_VP             0x138C
  #define FIRST_DEVICE_FDM                  0x138D
  #define FIRST_DEVICE_LASER                0x138E
  #define FOCUS_SET_FOCUS_TIPS              0x138F
  #define SW_FOCUS_Z_VP                     0x2207
#endif

#define VOLUME_DISPLAY                      0x1140
#define DISPLAY_BRIGHTNESS                  0x1142
#define DISPLAYSTANDBY_BRIGHTNESS           0x1144
#define DISPLAYSTANDBY_ENABLEINDICATOR      0x1146
#define DISPLAYSTANDBY_SECONDS              0x1148
#define VolumeIcon                          0x1150
#define SoundIcon                           0x1152
#define BrightnessIcon                      0x1154
#define SET_GRID_MAX_POINTS_VP              0x1156
#define PROBE_MARGIN_X_VP                   0x1158
#define PROBE_MARGIN_Y_VP                   0x1160
#define PROBE_COUNT_VP                      0x1162
#define CURRENT_MESH_POINT                  0x2220
#define MESH_POINT_MIN                      0x2222
#define MESH_POINT_MAX                      0x2224
#define MESH_POINT_DEVIATION                0x2226
#define EXTERNAL_M_VP                       0x2228
#define EXTERNAL_M73_ICON_VP                0x2230
#define EXTERNAL_M600_ICON_VP               0x2232
#define MESH_SIZE_ICON_VP                   0x2234
#define MESH_SIZE_ICON_ONOFF_VP             0x2236

/************struct**************/
typedef struct DataBuf
{
  unsigned char len;
  unsigned char head[2];
  unsigned char command;
  unsigned long addr;
  unsigned long bytelen;
  unsigned short data[32];
  unsigned char reserv[4];
} DB;

typedef struct CardRecord
{
  int recordcount;
  int Filesum;
  unsigned long addr[FileNum];
  char Cardshowfilename[FileNum][FileNameLen];
  //#if ENABLED(LCD_RTS_SOFTWARE_AUTOSCROLL)   
    char* Cardshowlongfilename[FileNum];
    int filenamelen[MaxFilenameLength];
  //#endif
  char Cardfilename[FileNum][FileNameLen];
  bool selectFlag;
}CRec;

extern CRec CardRecbuf;

struct lcd_rts_settings_t { // use bit fields to save space, max 48 bytes
size_t settings_size;
uint8_t settings_version;
bool display_sound;
int16_t display_volume;
uint8_t screen_brightness;
bool display_standby;
uint8_t standby_brightness;
int16_t standby_time_seconds;  
uint8_t max_points;
uint8_t probe_margin_x;
uint8_t probe_margin_y_front;
uint8_t probe_margin_y_back;
bool external_m73;
uint8_t extra_probing;
uint8_t total_probing;
uint8_t plr_zraise;
bool boot_zraise;
//uint8_t hotend_fan;
};

static constexpr size_t eeprom_data_size = sizeof(lcd_rts_settings_t);

class RTSSHOW
{
  public:
    RTSSHOW(void);
    static void EachMomentUpdate(void);
    int RTS_RecData(void);
    void RTS_SDCardInit(void);
    bool RTS_SD_Detected(void);
    void RTS_SDCardUpdate(void);
    void languagedisplayUpdate(void);
    void RTS_SndData(void);
    void RTS_SndData(const String &, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(const char[], unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(char, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(unsigned char*, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(int, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(float, unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(unsigned int,unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(long,unsigned long, unsigned char = VarAddr_W);
    void RTS_SndData(unsigned long,unsigned long, unsigned char = VarAddr_W);
    void RTS_SDcard_Stop(void);
    void RTS_HandleData(void);
    void RTS_Init(void);
    void RTS_SendCurveData(uint8_t channel, uint16_t *vaule, uint8_t size);
    #if HAS_CUTTER
      void RTS_HandleData_Laser(void);
      void RTS_SDcard_Stop_laser(void);
    #endif
    void writeVariable(const uint16_t adr, const void * const values, uint8_t valueslen, const bool isstr=false, const char fillChar=' ');    
    void setTouchScreenConfiguration();    
    void sendRectangleCommand(uint16_t vpAddress, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
    void sendOneFilledRectangle(uint16_t baseAddress, uint16_t showcount, uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
    void RTS_ChangeLevelingPage(void);
    void RTS_SendLoadedData(uint8_t loadpart);
    //void sendQRCodeCommand(uint16_t vpAddress, const char* url);    
    void calculateProbePoints(uint8_t current_point, uint8_t& x_probe_point, uint8_t& y_probe_point);
    //String RTS_ReadTextField(uint16_t address);
    //void sendPacketAndReceiveResponse(uint16_t packetValue);
    //bool readDisplayVersion(uint8_t &guiVersion, uint8_t &osVersion);
    static void RTS_SndText(const char string[], unsigned long addr, uint8_t textSize = 30);
  
    static DB recdat;
    static DB snddat;
  private:
    unsigned char databuf[SizeofDatabuf];
};

extern RTSSHOW rtscheck;

typedef enum PROC_COM : int8_t {  
  MainEnterKey              = 0,
  AdjustEnterKey            = 1,
  PrintSpeedEnterKey        = 2,
  StopPrintKey              = 3,
  PausePrintKey             = 4,
  ResumePrintKey            = 5,
  ZoffsetEnterKey           = 6,
  TempControlKey            = 7,
  CoolDownKey               = 8,
  HeaterTempEnterKey        = 9,
  HotBedTempEnterKey        = 10,
  PrepareEnterKey           = 11,
  BedLevelKey               = 12,
  AutoHomeKey               = 13,
  XaxismoveKey              = 14,
  YaxismoveKey              = 15,
  ZaxismoveKey              = 16,
  HeaterLoadEnterKey        = 17,
  HeaterUnLoadEnterKey      = 18,
  HeaterLoadStartKey        = 19,
  SelectLanguageKey         = 20,
  PowerContinuePrintKey     = 21,
  PLAHeadSetEnterKey        = 22,
  PLABedSetEnterKey         = 23,
  ABSHeadSetEnterKey        = 24,
  ABSBedSetEnterKey         = 25,
  StoreMemoryKey            = 26,
  FanSpeedEnterKey          = 27,
  VelocityXaxisEnterKey     = 28,
  VelocityYaxisEnterKey     = 29,
  VelocityZaxisEnterKey     = 30,
  VelocityEaxisEnterKey     = 31,
  AccelXaxisEnterKey        = 32,
  AccelYaxisEnterKey        = 33,
  AccelZaxisEnterKey        = 34,
  AccelEaxisEnterKey        = 35,
  JerkXaxisEnterKey         = 36,
  JerkYaxisEnterKey         = 37,
  JerkZaxisEnterKey         = 38,
  JerkEaxisEnterKey         = 39,
  StepsmmXaxisEnterKey      = 40,
  StepsmmYaxisEnterKey      = 41,
  StepsmmZaxisEnterKey      = 42,
  StepsmmEaxisEnterKey      = 43,
  NozzlePTempEnterKey       = 44,
  NozzleITempEnterKey       = 45,
  NozzleDTempEnterKey       = 46,
  HotbedPTempEnterKey       = 47,
  HotbedITempEnterKey       = 48,
  HotbedDTempEnterKey       = 49,
  PrintFanSpeedkey          = 50,
  ChangePageKey             = 51,
  ErrorKey                  = 52,
  StartFileKey              = 53,
  SelectFileKey             = 54,
  #if HAS_CUTTER || ENABLED(E3S1PRO_RTS)
    SwitchDeviceKey         = 55,
    PauseEngraveingKey      = 56,
    EngraveWarningKey       = 57,
    AdjustFocusKey          = 58, 
    SwAdjustFocusKey        = 59,
    LaserMoveAxis           = 60,
    FocusZAxisKey           = 61,
  #endif
   AutopidSetNozzleTemp     = 62,
   AutopidSetNozzleCycles   = 63,   
   AutopidSetHotbedTemp     = 64,
   AutopidSetHotbedCycles   = 65,
   Zoffset005EnterKey       = 66,   
   Advance_K_Key            = 67,
   XoffsetEnterKey          = 68,
   YoffsetEnterKey          = 69,   
   PETGHeadSetEnterKey      = 70,
   PETGBedSetEnterKey       = 71,
   CUSTHeadSetEnterKey      = 72,
   CUSTBedSetEnterKey       = 73,   
   XShapingFreqsetEnterKey  = 74,
   YShapingFreqsetEnterKey  = 75,   
   XShapingZetasetEnterKey  = 76,
   YShapingZetasetEnterKey  = 77,
   XMinPosEepromEnterKey    = 78,
   YMinPosEepromEnterKey    = 79,
   XaxismoveKeyHomeOffset   = 80,
   YaxismoveKeyHomeOffset   = 81,
   E0FlowKey                = 82,
   Volume                   = 83,
   DisplayBrightness        = 84,
   DisplayStandbyBrightness = 85,
   VolumeDisplay            = 86,   
   DisplayStandbySeconds    = 87,
   SetGridMaxPoints         = 88,
   SetProbeMarginX          = 89,
   ExternalMToggle          = 90,
   SetProbeMarginY          = 91,
   EditMeshpoint            = 92,
   CurrentMeshpoint         = 93,
   SetProbeCount            = 94,
   SaveM503Settings         = 95,
   HotendFanSpeedkey      = 96
} proc_command_t; 

const unsigned long Addrbuf[] = 
{
  0x1002,
  0x1004,
  0x1006,
  0x1008,
  0x100A,   // 4
  0x100C,
  0x1026,
  0x1030,
  0x1032,
  0x1034,   // 9
  0x103A,
  0x103E,
  0x1044,
  0x1046,
  0x1048,   // 14
  0x104A,
  0x104C,
  0x1052,
  0x1054,
  0x1056,   // 19
  0x105C,
  0x105F,
  0x1090,
  0x1092,
  0x1094,   // 24
  0x1096,
  0x1098,
  0x109A,
  0x109E,
  0x10A0,   // 29
  0x10A2,
  0x10A4,
  0x10A6,
  0x10A8,
  0x10AA,   // 34
  0x10AC,
  0x10AE,
  0x10B0,
  0x10B2,
  0x10B4,   // 39
  0x10B6,
  0x10B8,
  0x10BA,
  0x10BC,
  0x10BE,   // 44
  0x10C0,
  0x10C2,
  0x10C4,
  0x10C6,
  0x10C8,   // 49
  0x10CA,
  0x110E,
  0x111A,
  0x2198,
  0x2199,  // 54
  #if HAS_CUTTER || ENABLED(E3S1PRO_RTS)
    0x2201, //SwitchDeviceKey
    0x2202,
    0x2203, //EngraveWarningKey
    0x2204,
    0x2205,
    0x2206,
    0x2207,
  #endif  
   0x2209, //pidsetkey nozzle temp
   0x2210, //pidsetkey nozzle cycles
   0x2211, //pidsetkey bed temp
   0x2212, //pidsetkey bed cycle
   0x2213, // Zoffset005enterkey   
   0x2214, // Advance_K_set 
   0x163A, // XoffsetEnterKey
   0x164A, // YoffsetEnterKey 
   0x175A, // PETG NozzleTemp
   0x176A, // PETG BedTemp
   0x177A, // CUST NozzleTemp
   0x178A, // CUST BedTemp 
   0x173A, // X Frequency
   0x174A, // Y Frequency
   0x185A, // X Zeta
   0x186A, // Y Zeta
   0x189A, // x_min_pos
   0x190A, // y_min_pos
   0x191A, // y_min_pos_homeoffset
   0x192A, // y_min_pos_homeoffset
   0x193A, // Flow Key
   0x194A, // Volume
   0x1142, // Screen Brightness
   0x1144, // Screen Standby Brightness
   0x1146, // VolumeDisplayEnableIndicator
   0x1148, // DisplayStandbySeconds
   0x1156, // SetGridMaxPoints
   0x1158, // SetProbeMarginX
   0x2228, // ExternalMToggle
   0x1160, // SetProbeMarginY 
   0x2218, // EditMeshpoint
   0x2220, // CurrentMeshpoint
   0x1162, // SetProbeCount
   0x166A, // SaveM503Settings
   0x167A, // HotendFanSpeedKey
  0
};

extern int EndsWith(const char*, const char*);
void ErrorHanding(void);
extern void RTS_Update(void);
//#if ENABLED(LCD_RTS_SOFTWARE_AUTOSCROLL)
//  extern void RTS_Update_SCROLLING(void);
//#endif
extern void RTSInit(void);
#if HAS_CUTTER
  extern void RTS_UpdateLaser(void);
#endif
extern int touchscreen_requested_mesh;
extern float zprobe_zoffset;
extern int16_t temphot;
extern char waitway;
extern int change_page_font;
extern uint8_t language_change_font;
extern uint8_t lang;
extern int Update_Time_Value;
extern bool PoweroffContinue;
extern bool sdcard_pause_check;
extern bool pause_action_flag;
extern bool SD_Card_status;
extern bool home_flag;
extern bool heat_flag;
extern char commandbuf[30];
extern char errorway;
extern char errornum;
extern float z_offset;
extern uint32_t last_start_time;
extern bool eeprom_save_flag;

#define EEPROM_SAVE_LANGUAGE()      {if(eeprom_save_flag) { settings.save(); eeprom_save_flag = false; }}

void RTS_PauseMoveAxisPage(void);
void RTS_AutoBedLevelPage(void);
void RTS_LoadMeshPointOffsets(void);
void RTS_ResetMesh(void);
void RTS_LoadMesh(void);
void RTS_CleanPrintAndSelectFile(void);
void RTS_CleanPrintFile(void);
void RTS_ShowPage(uint8_t pageNumber);
void RTS_ShowPreviewImage(bool status);
void RTS_ShowMotorFreeIcon(bool status);
void RTS_ResetHeadAndBedSetTemp(void);
void RTS_ResetSingleVP(int vpaddress);
void RTS_SendMachineData(void);
void RTS_SendCurrentPosition(uint8_t axis);
void RTS_LoadMainsiteIcons(void);
void RTS_SendLevelingSiteData(uint8_t axis);
void RTS_SendPrintData(void);
void RTS_SendBedTemp(void);
void RTS_SendHeadTemp(void);
void RTS_SendHeadCurrentTemp();
void RTS_SendMoveaxisUnitIcon(uint8_t icon);
void RTS_SendDefaultRates();
float* getM503_settings();
void RTS_SetProbeCount(uint8_t probescount, uint8_t m19load);
void RTS_SetProbeMarginX(uint8_t marginx, uint8_t m19load);
void RTS_SetProbeMarginY(uint8_t marginy, uint8_t m19load);
void RTS_SetGridMaxPoints(uint8_t gridmaxpoints, uint8_t m19load);
void RTS_SendLang(int vpaddress);
void RTS_GetRemainTime(void);
void RTS_SendProgress(uint8_t progresspercent);

void RTS_G28MoveNow(void);
void RTS_G28MoveOne(void);
void RTS_ResetHotendBed(void);
void RTS_SetBltouchHSMode(void);
void RTS_SendZoffsetFeedratePercentage(bool sendzoffset);
void RTS_AxisZCoord();
void RTS_ResetPrintData(bool defaultpic);
void RTS_SetOneToVP(int vpaddress);
void RTS_ResetProgress();
void RTS_TrammingPosition(uint8_t xx, uint8_t xy, uint8_t yx, uint8_t yy);
void RTS_MoveAxisHoming(void);
void RTS_SetMeshPage();
void RTS_SendM600Icon(bool icon);
void RTS_SendM73Icon(bool icon);
void RTS_MoveParkNozzle(void);
void RTS_CommandPause(void);

extern int8_t g_uiAutoPIDRunningDiff;
extern int16_t g_uiCurveDataCnt;
extern uint8_t leveling_running;
extern uint8_t color_sp_offset;
extern uint8_t min_margin_y_front;
extern uint8_t min_margin_y_back;
extern uint8_t min_margin_x;
extern unsigned int picLayers;    // picture layers
extern unsigned int picFilament_m;
extern unsigned int picFilament_g;
extern float picLayerHeight;
extern uint8_t settingsload;
//#if ENABLED(LCD_RTS_SOFTWARE_AUTOSCROLL)
//  void lcd_rts_scrolling();
//#endif
extern lcd_rts_settings_t lcd_rts_settings;
void saveSettings(char * const buff);
void loadSettings(const char * const buff);
void resetSettings();
int custom_ceil(float x); 
#endif
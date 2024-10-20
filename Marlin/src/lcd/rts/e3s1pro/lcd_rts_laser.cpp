#include <Wstring.h>
#include <stdio.h>
#include <string.h>
#include <Arduino.h>
// #include <libmaple/usart.h>
#include "lcd_rts.h"
#include "../../../MarlinCore.h"
#include "../../../inc/MarlinConfig.h"
#include "../../../module/settings.h"
#include "../../../core/serial.h"
#include "../../../core/macros.h"
#include "../../utf8.h"
#include "../../marlinui.h"
#include "../../../sd/cardreader.h"
#include "../../../feature/babystep.h"
#include "../../../module/temperature.h"
#include "../../../module/printcounter.h"
#include "../../../module/motion.h"
#include "../../../module/planner.h"
#include "../../../gcode/queue.h"
#include "../../../gcode/gcode.h"
#include "../../../module/probe.h"
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

#if ENABLED(LASER_FEATURE)

#if HAS_CUTTER
  #include "lcd_rts_laser.h"
  #include "../../../feature/spindle_laser.h"
#endif

#if ENABLED(GCODE_PREVIEW_ENABLED)
  #include "preview.h"
#endif

#ifdef LCD_SERIAL_PORT
  #define LCDSERIAL LCD_SERIAL
#elif SERIAL_PORT_2
  #define LCDSERIAL MYSERIAL2
#endif

#if ENABLED(E3S1PRO_RTS)

extern void RTS_line_to_current(AxisEnum axis);
constexpr float default_max_feedrate[]        = DEFAULT_MAX_FEEDRATE;
constexpr float default_max_acceleration[]    = DEFAULT_MAX_ACCELERATION;
constexpr float default_max_jerk[]            = { DEFAULT_XJERK, DEFAULT_YJERK, DEFAULT_ZJERK, DEFAULT_EJERK };
constexpr float default_axis_steps_per_unit[] = DEFAULT_AXIS_STEPS_PER_UNIT;
static bool first_start_laser = true;
bool laser_axes_should_home = false;
int16_t fileCnt_laser = 0;
uint8_t file_current_page_laser = 1;
uint8_t file_total_page_laser = 1;
uint8_t page_total_file_laser = 0;

static void RTS_line_to_filelist_laser() {
  char statStr1[4];
  snprintf(statStr1, sizeof(statStr1), "%d", file_current_page_laser);
  char statStr2[4];
  snprintf(statStr2, sizeof(statStr2), "%d", file_total_page_laser);
  for (int h = 0; h < 2; h++) {
  RTS_ResetSingleVP(PAGE_STATUS_TEXT_CURRENT_VP);
  RTS_ResetSingleVP(PAGE_STATUS_TEXT_TOTAL_VP);    
  }

  rtscheck.RTS_SndData(statStr1, PAGE_STATUS_TEXT_CURRENT_VP);
  rtscheck.RTS_SndData(statStr2, PAGE_STATUS_TEXT_TOTAL_VP);

  for (int i = 0; i < 20; i += 20) {
    for (int j = 0; j < 60; j++) {
      RTS_ResetSingleVP(FILE1_TEXT_VP + i + j);
    }
  }  
  // clean filename Icon
  //uint8_t file_current_page = 1;
  for (int j = 0; j < 5; j++){
    for (int i = 0; i < TEXTBYTELEN; i++){
      RTS_ResetSingleVP(CardRecbuf.addr[j] + i);
    }
  }

  memset(&CardRecbuf, 0, sizeof(CardRecbuf));

  int num = 0;
  for (int16_t i = (file_current_page_laser - 1) * 5; i < (file_current_page_laser * 5); i++) {  
      card.selectFileByIndexSorted(i);
      #if ENABLED(LCD_RTS_DEBUG_SDCARD)    
          SERIAL_ECHO_MSG("card.longFilename ", card.longFilename); 
      #endif
      char *pointFilename = card.longFilename;
      int filenamelen = strlen(card.longFilename);

      if (!CardRecbuf.Cardshowlongfilename[num]) {
          CardRecbuf.Cardshowlongfilename[num] = new char[filenamelen + 1];
          strcpy(CardRecbuf.Cardshowlongfilename[num], card.longFilename);
      }

      int j = 1;
      while ((strncmp(&pointFilename[j], ".gcode", 6) != 0 && strncmp(&pointFilename[j], ".GCODE", 6) != 0 && strncmp(&pointFilename[j], ".GCO", 4) != 0 && strncmp(&pointFilename[j], ".gco", 4) != 0) && (j++ < filenamelen));

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
  page_total_file_laser = CardRecbuf.Filesum;
  CardRecbuf.Filesum = ((file_total_page_laser - 1) * 5) + page_total_file_laser;
}

void RTSSHOW::RTS_SDcard_Stop_laser(void)
{
  card.flag.abort_sd_printing = true;  //card.flag.abort_sd_printing
  queue.clear();
  if(home_flag) planner.synchronize();

  quickstop_stepper();
  print_job_timer.stop();

  print_job_timer.reset();
  laser_device.quick_stop();

  PoweroffContinue = false;
  if(CardReader::flag.mounted)
  {
    #if ENABLED(SDSUPPORT) && ENABLED(POWER_LOSS_RECOVERY)
      card.removeJobRecoveryFile();
    #endif
  }

  RTS_ShowMotorFreeIcon(true);
  RTS_ResetSingleVP(PRINT_PROCESS_ICON_VP);
  RTS_ResetSingleVP(PRINT_PROCESS_VP);
  delay(2);
  RTS_CleanPrintAndSelectFile();
  planner.synchronize();
}

void RTSSHOW::RTS_HandleData_Laser(void)
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
  
  // SERIAL_ECHOPAIR("\nCheckkey=", Checkkey, "  recdat.data[0]=", recdat.data[0]);

  switch(Checkkey)
  {
    case MainEnterKey:
      if(recdat.data[0] == 1)
      {

        CardUpdate = true;
        CardRecbuf.recordcount = -1;
        #if ENABLED(LCD_RTS_DEBUG_SDCARD)
        SERIAL_ECHOPGM("Working dir is: ");
        SERIAL_ECHO(card.getWorkDirName());
        SERIAL_ECHOLN("");
        #endif
        std::string currentdir;
        currentdir = card.getWorkDirName();
        if (card.getWorkDirName() != std::string("/")) {
        card.cdup();
        #if ENABLED(LCD_RTS_DEBUG_SDCARD)
        SERIAL_ECHO("chroot done to:");
        SERIAL_ECHO(card.getWorkDirName());
        SERIAL_ECHOLN("");
        #endif
        }

        if (card.flag.mounted)
        {
        int16_t fileCnt_laser = card.get_num_items();

        if (fileCnt_laser > 0) {
          file_total_page_laser = fileCnt_laser / 5;
          if (fileCnt_laser % 5 > 0) { // Add an extra page only if there are leftover files
            file_total_page_laser++;
          }
          if (file_total_page_laser > 8) file_total_page_laser = 8; // Limit the maximum number of pages
        }
        else {
          file_total_page_laser = 1;
        }

        RTS_SndData(file_total_page_laser, PAGE_STATUS_TEXT_TOTAL_VP);
        file_current_page_laser = 1;
        RTS_SndData(file_current_page_laser, PAGE_STATUS_TEXT_CURRENT_VP);

        RTS_ShowPage(52);
        if (IS_SD_INSERTED()) RTS_line_to_filelist_laser();
        }
        CardUpdate = false;        
        EEPROM_SAVE_LANGUAGE();
      }
      else if(recdat.data[0] == 2)
      {
        AxisUnitMode = 1;
        axis_unit = 10.0;
        
        if(!laser_axes_should_home) {
          laser_axes_should_home = true;
          waitway = 9;
          queue.enqueue_now_P(HOME_LASER);
          RTS_ShowPage(40);
          
          RTS_ResetSingleVP(AXIS_X_COORD_VP);
          RTS_SndData(10*10, AXIS_Y_COORD_VP);

        }else{
          RTS_ShowPage(70);
          RTS_SendCurrentPosition(2);
          RTS_SendCurrentPosition(3);
        }
        EEPROM_SAVE_LANGUAGE();
        //RTS_SetOneToVP(FILAMENT_CONTROL_ICON_VP);
      }
      else if(recdat.data[0] == 3)
      {
        RTS_ShowPage(64);
        EEPROM_SAVE_LANGUAGE();
      }
      else if(recdat.data[0] == 4)
      {
        RTS_ShowPage(25);
        planner.synchronize();
        queue.enqueue_now_P(PSTR("G28\nG1 F200 Z0.0"));
        //RTS_SetOneToVP(AUTO_BED_LEVEL_TITLE_VP);
        RTS_ShowMotorFreeIcon(false);
      }
      else if(recdat.data[0] == 5)
      {
        // card.flag.abort_sd_printing = true;  
        queue.clear();
        quickstop_stepper();
        print_job_timer.stop();
        RTS_ShowMotorFreeIcon(false); // 激光时锁定
        RTS_ResetPrintData(true);
        print_job_timer.reset();
        RTS_CleanPrintAndSelectFile();
        RTS_ShowPage(51);
      }
      else if(recdat.data[0] == 6)
      {
        waitway = 3;
        RTS_SetOneToVP(AUTO_BED_LEVEL_TITLE_VP);
        RTS_SndData(AUTO_BED_LEVEL_PREHEAT, AUTO_BED_PREHEAT_HEAD_DATA_VP);
        RTS_ResetSingleVP(AUTO_LEVELING_PERCENT_DATA_VP);  
        thermalManager.setTargetHotend(AUTO_BED_LEVEL_PREHEAT, 0);
        RTS_SndData(AUTO_BED_LEVEL_PREHEAT, HEAD_SET_TEMP_VP);
        if(thermalManager.temp_hotend[0].celsius < (AUTO_BED_LEVEL_PREHEAT - 5))
        {
          queue.enqueue_now_P(PSTR("G4 S40"));
        }

        if(axes_should_home()) queue.enqueue_one_P(PSTR("G28"));
        queue.enqueue_one_P(PSTR("G29"));
        RTS_ShowMotorFreeIcon(false);
      }
      else if(recdat.data[0] == 7)
      {
        if(errorway == 1)
        {

        }
        else if(errorway == 2)
        {
          // auto home fail
        }
        else if(errorway == 3)
        {
          // bed leveling fail
        }
        else if(errorway == 4) 
        {

        }
      }
      else if(recdat.data[0] == 8)
      {
        RTS_ShowPage(51);
        EEPROM_SAVE_LANGUAGE();
      }
      break;

    case AdjustEnterKey:
      if(recdat.data[0] == 1)
      {
        RTS_ShowPage(74);
      }
      else if(recdat.data[0] == 2)
      {
        if(card.isPrinting())
        {
          RTS_ShowPage(59);
        }
        else
        {
          RTS_ShowPage(61);
        }
        // settings.save();
      }
      else if(recdat.data[0] == 5)
      {
        RTS_ShowPage(15);
      }
      else if(recdat.data[0] == 7)
      {
        RTS_ShowPage(74);
        settings.save();
      }
      break;

    case PrintSpeedEnterKey:
      feedrate_percentage = recdat.data[0];
      RTS_SendZoffsetFeedratePercentage(false);
      break;

    case StopPrintKey:
      if(recdat.data[0] == 1)
      {
        RTS_ShowPage(58);
      }
      else if(recdat.data[0] == 2)
      {
        waitway = 10;

        //card.flag.abort_sd_printing = true;  //card.flag.abort_sd_printing
        RTS_ResetSingleVP(PRINT_TIME_HOUR_VP);
        RTS_ResetSingleVP(PRINT_TIME_MIN_VP);
        Update_Time_Value = 0;

        //runout.reset();

        RTS_ShowPage(40);
        RTS_SDcard_Stop_laser();
        cutter.apply_power(0);//关闭激光
      }
      else if(recdat.data[0] == 3)
      {
        if(card.isPrinting())
        {
          RTS_ShowPage(59);
        }
        else
        {
          RTS_ShowPage(61);
        }
      }
      break;

    case PausePrintKey: 
      if(recdat.data[0] == 1)
      {
        if(card.isPrinting())// && (thermalManager.temp_hotend[0].celsius > (thermalManager.temp_hotend[0].target - 5)) && (thermalManager.temp_bed.celsius > (thermalManager.temp_bed.target - 3)))
        {
          RTS_ShowPage(62);
        }
        else 
        {
          RTS_ShowPage(59);
        }
      }
      else if(recdat.data[0] == 2)
      {
        waitway = 1;
        // pause_e = current_position[E_AXIS];
        card.pauseSDPrint();
        print_job_timer.pause();
        pause_action_flag = true;
        Update_Time_Value = 0;
        RTS_ShowPage(61);
        planner.synchronize();
        sdcard_pause_check = false;
        // queue.inject((char*)"M25");
      }
      else if(recdat.data[0] == 3)
      {
        if(card.isPrinting())
        {
          RTS_ShowPage(59);
        }
        else
        {
          RTS_ShowPage(61);
        }
      }
      break;

    case ResumePrintKey:
      if(recdat.data[0] == 1)
      {
        RTS_ShowPage(59);
        cutter.apply_power(laser_device.power);
        card.startOrResumeFilePrinting();
        print_job_timer.start();
        Update_Time_Value = 0;
        sdcard_pause_check = true;
        //queue.inject((char*)"M24");
      }
      else if(recdat.data[0] == 2)
      {

      }
      else if(recdat.data[0] == 3)
      {
        // queue.inject_P(PSTR("M108"));
        wait_for_user = false;
        // runout.filament_ran_out = false;
        //runout.reset();

        RTS_ShowPage(10);

        // card.startFileprint();
        print_job_timer.start();
        Update_Time_Value = 0;
        sdcard_pause_check = true;
      }
      break;

    case ZoffsetEnterKey:
      
      last_zoffset = zprobe_zoffset;
      if(recdat.data[0] >= 32768)
      {
        //zprobe_zoffset = ((float)recdat.data[0] - 65536) / 100;
        zprobe_zoffset = (signed short)recdat.data[0]/100.0;
        zprobe_zoffset -= 0.001;
      }
      else
      {
        zprobe_zoffset = ((float)recdat.data[0]) / 100;
        zprobe_zoffset += 0.001;
      }
      if(WITHIN((zprobe_zoffset), PROBE_OFFSET_ZMIN, PROBE_OFFSET_XMAX))
      {
        babystep.add_mm_laser(Z_AXIS, zprobe_zoffset - last_zoffset);
        //SERIAL_ECHOPAIR("\nZoffset=", zprobe_zoffset - last_zoffset);
      }
      probe.offset.z = zprobe_zoffset;
      // settings.save();
      break;

    case PrepareEnterKey:
      //SERIAL_ECHOPAIR("PrepareEnterKey=", recdat.data[0]);
      if(recdat.data[0] == 1)
      {
        RTS_ShowPage(28);
      }
      else if(recdat.data[0] == 2)
      {
        RTS_ShowPage(64);
        //RTS_ShowPage(65);
        //EEPROM_SAVE_LANGUAGE();
      }
      else if(recdat.data[0] == 3)
      {
        RTS_SendCurrentPosition(4);
        delay(2);
        RTS_ShowPage(70);
      }
      else if(recdat.data[0] == 4)
      {
        RTS_ShowPage(58);
      }
      else if(recdat.data[0] == 5)
      {
        RTS_SendMachineData();
        RTS_ShowPage(66);
        EEPROM_SAVE_LANGUAGE();
      }
      else if(recdat.data[0] == 6)
      {
        queue.enqueue_now_P(PSTR("M84"));
        RTS_ShowMotorFreeIcon(true);
      }
      else if(recdat.data[0] == 7)
      {
        RTS_ShowPage(43);
      }
      else if(recdat.data[0] == 8)
      {
        //settings.save();
        //RTS_ShowPage(21);
        RTS_ShowPage(65);
        EEPROM_SAVE_LANGUAGE();        
      }
      else if(recdat.data[0] == 9)
      {
        RTS_ShowPage(1);
      }
      else if(recdat.data[0] == 0xA)
      {
        RTS_ShowPage(42);
      }
      else if(recdat.data[0] == 0xB)// 恢复出厂设置  确定
      {
         #if ENABLED(HAS_MENU_RESET_WIFI)
          WIFI_STATE = PRESSED;
          OUT_WRITE(RESET_WIFI_PIN, LOW);
        #endif
        (void)settings.reset(); 
        (void)settings.save();
        RTS_Init(); 
        RTS_ShowPage(51);
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
      else if(recdat.data[0] == 0xE) // 恢复出厂设置  取消
      {
        if(!planner.has_blocks_queued())
        {
	        RTS_ShowPage(64);
	      }
      }
      else if(recdat.data[0] == 0xF)//高级设置
      {
        RTS_ShowPage(64);
        settings.save();//delay(100);
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

    case AutoHomeKey:
      if(recdat.data[0] == 1)
      {
        AxisUnitMode = 1;
        axis_unit = 10.0;
        RTS_ShowPage(70);
        RTS_SendMoveaxisUnitIcon(3);
      }
      else if(recdat.data[0] == 2)
      {
        AxisUnitMode = 2;
        axis_unit = 1.0;
        RTS_ShowPage(71);
        RTS_SendMoveaxisUnitIcon(2);
      }
      else if(recdat.data[0] == 3)
      {
        AxisUnitMode = 3;
        axis_unit = 0.1;
        RTS_ShowPage(72);
        RTS_SendMoveaxisUnitIcon(1);
      }
      else if(recdat.data[0] == 4)
      {
        waitway = 4;
        queue.enqueue_now_P(PSTR("G28 X Y"));
        Update_Time_Value = 0;
        RTS_ShowMotorFreeIcon(false);
      }
      else if(recdat.data[0] == 5)
      {
        waitway = 4;
        queue.enqueue_now_P(PSTR("G28 Z"));
        RTS_ShowMotorFreeIcon(false);
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
      eeprom_save_flag = true;
      break;

    case PowerContinuePrintKey:
      if(recdat.data[0] == 1)
      {

      #if ENABLED(POWER_LOSS_RECOVERY)
        if(recovery.recovery_flag)
        {
          power_off_type_yes = true;
          Update_Time_Value = 0;
          RTS_ShowPage(10);
          // recovery.resume();
          queue.enqueue_now_P(PSTR("M1000"));

          PoweroffContinue = true;
          sdcard_pause_check = true;
          zprobe_zoffset = probe.offset.z;
          RTS_SendZoffsetFeedratePercentage(true);
        }
      #endif
      }
      else if(recdat.data[0] == 2)
      {
        Update_Time_Value = RTS_UPDATE_VALUE;
        RTS_ShowPage(51);
        RTS_ResetSingleVP(PRINT_TIME_HOUR_VP);
        RTS_ResetSingleVP(PRINT_TIME_MIN_VP);
        Update_Time_Value = 0;
        RTS_SDcard_Stop_laser();
      }
      break;

    case StoreMemoryKey:
      if(recdat.data[0] == 1)
      {
        RTS_ShowPage(34);
      }
      if(recdat.data[0] == 2)
      {
        //queue.enqueue_now_P(PSTR("M502"));
        RTS_ShowPage(76);
        //settings.save();
        //RTS_SendDefaultRates();
        // delay(100);
      }
      else if(recdat.data[0] == 3)
      {
        RTS_ShowPage(67);
      }
      else if(recdat.data[0] == 4)
      {
        RTS_ShowPage(68);
      }
      else if(recdat.data[0] == 5)
      {
        RTS_ShowPage(69);
      }
      else if(recdat.data[0] == 7)
      {
        RTS_ShowPage(76);
      }
      else if(recdat.data[0] == 8) // back to motion with save
      {
        RTS_ShowPage(34);
        settings.save();
        delay(100);
      }
      else if(recdat.data[0] == 9) //最大拐角速度
      {
        RTS_ShowPage(68);
      }
      else if(recdat.data[0] == 0x0A) //最大速度
      {
        RTS_ShowPage(69);
      }
      else if(recdat.data[0] == 0x0B)
      {
        RTS_ShowPage(33);
        settings.save();
        delay(100);
      }
      else if(recdat.data[0] == 0x0C)
      {
        RTS_ShowPage(34);
        settings.save();
        delay(100);
      }
      else if(recdat.data[0] == 0x0D)//返回按钮
      {
        RTS_ShowPage(64);
        // settings.save();
        // delay(100);
      }
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
      PID_PARAM(Kp, 0) = nozzle_ptemp;
      break;

    case NozzleITempEnterKey:
      float nozzle_itemp;
      nozzle_itemp = (float)recdat.data[0] / 100;
      RTS_SndData(nozzle_itemp * 100, NOZZLE_TEMP_I_DATA_VP);
      PID_PARAM(Ki, 0) = scalePID_i(nozzle_itemp);
      break;

    case NozzleDTempEnterKey:
      float nozzle_dtemp;
      nozzle_dtemp = (float)recdat.data[0] / 100;
      RTS_SndData(nozzle_dtemp * 100, NOZZLE_TEMP_D_DATA_VP);
      PID_PARAM(Kd, 0) = scalePID_d(nozzle_dtemp);
      break;

    case HotbedPTempEnterKey:
      float hotbed_ptemp;
      hotbed_ptemp = (float)recdat.data[0] / 100;
      RTS_SndData(hotbed_ptemp * 100, HOTBED_TEMP_P_DATA_VP);
      thermalManager.temp_bed.pid.Kp = hotbed_ptemp;
      break;

    case HotbedITempEnterKey:
      float hotbed_itemp;
      hotbed_itemp = (float)recdat.data[0] / 100;
      RTS_SndData(hotbed_itemp * 100, HOTBED_TEMP_I_DATA_VP);
      thermalManager.temp_bed.pid.Ki = scalePID_i(hotbed_itemp);
      break;

    case HotbedDTempEnterKey:
      float hotbed_dtemp;
      hotbed_dtemp = (float)recdat.data[0] / 10;
      RTS_SndData(hotbed_dtemp * 10, HOTBED_TEMP_D_DATA_VP);
      thermalManager.temp_bed.pid.Kd = scalePID_d(hotbed_dtemp);
      break;

    case PrintFanSpeedkey:
      uint8_t fan_speed;
      fan_speed = (uint8_t)recdat.data[0];
      RTS_SndData(fan_speed , PRINTER_FAN_SPEED_DATA_VP);
      thermalManager.set_fan_speed(0, fan_speed);
      break;

    case SelectFileKey:

      if (RTS_SD_Detected()) {
        if (recdat.data[0] > CardRecbuf.Filesum) break;
        CardRecbuf.recordcount = recdat.data[0] - 1;
        std::string filename = CardRecbuf.Cardfilename[CardRecbuf.recordcount];
        // Find the last occurrence of the '.' character in the filename
        std::size_t dot_pos = filename.find_last_of('.');

        if (dot_pos == std::string::npos) {
          card.cd(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount]);
          int16_t fileCnt_laser = card.get_num_items();
          card.getWorkDirName();
          
          if (fileCnt_laser > 0) {
            file_total_page_laser = fileCnt_laser / 5;
            if (fileCnt_laser % 5 > 0) { // Add an extra page only if there are leftover files
              file_total_page_laser++;
            }
            if (file_total_page_laser > 8) file_total_page_laser = 8; // Limit the maximum number of pages
          }
          else {
            file_total_page_laser = 1;
          }

          RTS_SndData(file_total_page_laser, PAGE_STATUS_TEXT_TOTAL_VP);
          file_current_page_laser = 1;
          RTS_SndData(file_current_page_laser, PAGE_STATUS_TEXT_CURRENT_VP);
          RTS_line_to_filelist_laser();
          CardRecbuf.selectFlag = false;
          if (PoweroffContinue /*|| print_job_timer.isRunning()*/) return;

          // clean print file
          RTS_CleanPrintAndSelectFile();
          lcd_sd_status = IS_SD_INSERTED();
        }
        else {
          CardRecbuf.selectFlag = true;
          CardRecbuf.recordcount = recdat.data[0] - 1;
          for (int j = 0; j < 60; j++) RTS_ResetSingleVP(SELECT_FILE_TEXT_VP + j);
          delay(2);
          RTS_SndData((unsigned long)0xFFFF, FilenameNature + recdat.data[0] * 16);      
          RTS_ShowPage(51);
          
          #if ENABLED(GCODE_PREVIEW_ENABLED)
            char ret;
            RTS_ShowPreviewImage(false);
            ret = gcodePicDataSendToDwin(CardRecbuf.Cardfilename[CardRecbuf.recordcount],VP_OVERLAY_PIC_PTINT,PIC_FORMAT_JPG, PIC_RESOLUTION_250_250);
            if (ret == PIC_OK) {
              RTS_ShowPreviewImage(false);
            } else {
              RTS_ShowPreviewImage(true);
            }          
          #endif
          
          rts_start_print = true;
          delay(20);
          if (CardRecbuf.filenamelen[CardRecbuf.recordcount] > 25){
            RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], SELECT_FILE_TEXT_VP);
          }else{
            RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);
          }
          RTS_ShowPage(51);
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
          //SERIAL_ECHOLNPAIR("\r\nrts_start_print: ", rts_start_print);
          break;
        }
        
        RTS_ShowPage(75);

        card.openAndPausePrintFile(CardRecbuf.Cardfilename[CardRecbuf.recordcount]);
        
        laser_device.reset_data();
        laser_device.set_read_gcode_range_on();
        laser_device.power = 0;
      }
      else if(recdat.data[0] == 2)
      {
        if (!planner.has_blocks_queued()) {
          if ((file_total_page_laser > file_current_page_laser) && (file_current_page_laser < (MaxFileNumber / 5))){
            file_current_page_laser++;
          }else{
            break;
          }
          RTS_ShowPage(52);
          if (card.flag.mounted){
            RTS_line_to_filelist_laser();              
          }       
        }
      }
      else if(recdat.data[0] == 3)
      {
        if (!planner.has_blocks_queued()) {
          if (file_current_page_laser > 1){
            file_current_page_laser--;
          }else{
            break;
          }
          RTS_ShowPage(52);
          if (card.flag.mounted){
            RTS_line_to_filelist_laser();
          }
        }        
      }
      else if(recdat.data[0] == 4)
      {

        if (!planner.has_blocks_queued()) {
          file_current_page_laser = 1;
          RTS_ShowPage(52);
          RTS_line_to_filelist_laser();
        }        
      }
      else if(recdat.data[0] == 5)
      {
        if (!planner.has_blocks_queued()) {
          file_current_page_laser = file_total_page_laser;
          RTS_ShowPage(52);
          RTS_line_to_filelist_laser();
        }        
      }
      else if(recdat.data[0] == 6)
      {
        RTS_ShowPage(52);
      }
      else if(recdat.data[0] == 7)
      {
        RTS_ShowPage(52);
      }
      else if(recdat.data[0] == 8)
      {
        RTS_ShowPage(52);
      }
      else if(recdat.data[0] == 9)
      {

        if (!planner.has_blocks_queued()) {
          file_current_page_laser = 1;
          RTS_ShowPage(52);
          if (card.flag.mounted){
            RTS_line_to_filelist_laser();
          }
        }
      }
      else if(recdat.data[0] == 0x0A)
      {
        if (!planner.has_blocks_queued()) {
          file_current_page_laser = file_total_page_laser;
          RTS_ShowPage(52);
          if (card.flag.mounted){
            RTS_line_to_filelist_laser();              
          }
        }  
      }
      break;

    case ChangePageKey:
      for(int i = 0; i < MaxFileNumber; i ++)
      {
        for (int j = 0; j < 60; j ++)
        {
          RTS_ResetSingleVP(FILE1_TEXT_VP + i * 20 + j);
        }
      }

      for (int i = 0; i < CardRecbuf.Filesum; i++)
      {
        for (int j = 0; j < 60; j++)
        {
          RTS_ResetSingleVP(CardRecbuf.addr[i] + j);
        }
        RTS_SndData((unsigned long)0xFFFF, FilenameNature + (i + 1) * 16);
      }

      RTS_CleanPrintAndSelectFile();
      // clean filename Icon
      for (int j = 0; j < 60; j ++)
      {
        RTS_ResetSingleVP(FILE1_SELECT_ICON_VP + j);
      }

      RTS_SndData(CardRecbuf.Cardshowfilename[CardRecbuf.recordcount], PRINT_FILE_TEXT_VP);

      // represents to update file list
      // if (CardUpdate && lcd_sd_status && IS_SD_INSERTED())
      if (CardUpdate && lcd_sd_status && RTS_SD_Detected())
      {
        for (uint16_t i = 0; i < CardRecbuf.Filesum; i++)
        {
          delay(3);
          RTS_SndData(CardRecbuf.Cardshowfilename[i], CardRecbuf.addr[i]);
          RTS_SndData((unsigned long)0xFFFF, FilenameNature + (i + 1) * 16);
          RTS_ResetSingleVP(FILE1_SELECT_ICON_VP + i);
        }
      }
      RTS_SendMachineData();
      Percentrecord = card.percentDone() + 1;
      if (Percentrecord <= 100)
      {
        RTS_SendProgress((unsigned char)Percentrecord);
      }
      RTS_SendProgress((unsigned char)card.percentDone());
      RTS_SendZoffsetFeedratePercentage(true);
      RTS_SendHeadTemp();
      RTS_SendBedTemp();
      languagedisplayUpdate();

      RTS_SndData(change_page_font + ExchangePageBase, ExchangepageAddr);
      break;

    case FocusZAxisKey: 
    {
      waitway = 4;
      current_position[Z_AXIS] = ((signed short)recdat.data[0])/10.0;
      RTS_line_to_current(Z_AXIS);
      RTS_SendCurrentPosition(3);
      delay(1);
      RTS_ShowMotorFreeIcon(false);
      waitway = 0;
    }
    break;

    case AdjustFocusKey:
      if(recdat.data[0] == 1)//调节激光焦距
      {
        RTS_SndData(10*current_position[Z_AXIS], SW_FOCUS_Z_VP);
        RTS_ShowPage(63);
      // }else if(recdat.data[0] == 2)// Z+
      // {

      // }else if(recdat.data[0] == 3)// Z-
      // {

      }else if(recdat.data[0] == 4)// 返回
      {
        RTS_ShowPage(64);
      }else if(recdat.data[0] == 5)// √
      {
        queue.inject_P(PSTR("G92.9 Z0"));
        RTS_ResetSingleVP(AXIS_Z_COORD_VP);
        RTS_ResetSingleVP(SW_FOCUS_Z_VP);
        
        RTS_ShowPage(64);
      }else if(recdat.data[0] == 6)// x
      {
        // queue.inject_P(PSTR("G92.9 Z0"));
        // RTS_ResetSingleVP(AXIS_Z_COORD_VP);
        // RTS_ResetSingleVP(SW_FOCUS_Z_VP);
        RTS_ShowPage(64);
      }
    break;
    case SwAdjustFocusKey:

    break;

    case EngraveWarningKey: 
      if(recdat.data[0] == 1)
      {

      }else if(recdat.data[0] == 2)// 轴移动
      {
        AxisUnitMode = 1;
        axis_unit = 10.0;
        RTS_ShowPage(78);
      }else if(recdat.data[0] == 3) // 直接雕刻
      {
        char cmd[30];
        laser_device.laser_printing = true; // 雕刻中
        strcat_P(cmd, M24_STR);
        queue.inject((char*)"M24");//cmd);

        RTS_ShowPage(59);

      }else if(recdat.data[0] == 4) //跑边框
      {
        HMI_Area_Move();
      }else if(recdat.data[0] == 5) //返回
      {
        CardRecbuf.recordcount = -1;
        print_job_timer.stop();
        //RTS_ShowMotorFreeIcon(true);
        RTS_ResetPrintData(true);
        print_job_timer.reset();
        RTS_CleanPrintAndSelectFile();
        CardUpdate = true;
        CardRecbuf.recordcount = -1;
        RTS_SDCardUpdate();
        RTS_ShowPage(52);
      }
    break;

    case SwitchDeviceKey:
      if(recdat.data[0] == 1)// FDM
      {

      }else if(recdat.data[0] == 2)// 激光
      {

      }else if(recdat.data[0] == 3)// 切到FDM 确定
      {
        uint8_t language;

        RTS_ShowPage(33);
        laser_device.set_current_device(DEVICE_FDM);

        language = language_change_font; 
        settings.reset();
        language_change_font = language;
        settings.save();
        probe.offset.z = zprobe_zoffset = 0;
        RTS_SendZoffsetFeedratePercentage(true);
        laser_device.laser_power_close();
      }else if(recdat.data[0] == 4)// 切到FDM 取消
      {
        RTS_ShowPage(64);
      }else if(recdat.data[0] == 7)// 切到熔积打印
      {
        RTS_ShowPage(57);
      }else if(recdat.data[0] == 8) //调整焦距 √
      {
        queue.inject_P(PSTR("G92.9 Z0"));
        RTS_ResetSingleVP(AXIS_Z_COORD_VP);
        RTS_ResetSingleVP(SW_FOCUS_Z_VP);

        if(change_page_font == 33){
          RTS_ShowPage(64);
        }else{
          RTS_ShowPage(51);
        }
      }
    break;

    case LaserMoveAxis:
      if(recdat.data[0] == 1)//
      {
        AxisUnitMode = 1;
        axis_unit = 10.0;
        RTS_ShowPage(78);
      }else if(recdat.data[0] == 2)//
      {
        AxisUnitMode = 2;
        axis_unit = 1.0;
        RTS_ShowPage(79);
      }else if(recdat.data[0] == 3)
      {
        AxisUnitMode = 3;
        axis_unit = 0.1;
        RTS_ShowPage(80);
      }else if(recdat.data[0] == 4)// 返回
      {
        RTS_ShowPage(75);
      }else if(recdat.data[0] == 5)// 雕刻警告界面 xy home
      {
        waitway = 8;
        RTS_ShowPage(40);

        queue.enqueue_now_P(HOME_LASER);//PSTR("G28 XY\nG0 X0 Y5"));
        Update_Time_Value = 0;
        RTS_ResetSingleVP(AXIS_X_COORD_VP);
        RTS_SndData(10*10, AXIS_Y_COORD_VP);
        delay(1);
        RTS_ShowMotorFreeIcon(false);
      }else if(recdat.data[0] == 6)// 雕刻警告界面z home
      {
        queue.enqueue_now_P(PSTR("G0 Z0"));
      }else if(recdat.data[0] == 7)// 激光 xy home
      {
        waitway = 9;
        RTS_ShowPage(40);
        queue.enqueue_now_P(HOME_LASER);//"G28 XY\n G1 X0 Y10 F3000");//EVENT_HOME_LASER);//PSTR("G28 XY\nG0 X0 Y5"));
        Update_Time_Value = 0;
        
        RTS_ResetSingleVP(AXIS_X_COORD_VP);
        RTS_SndData(10*10, AXIS_Y_COORD_VP);
        delay(1);
        RTS_ShowMotorFreeIcon(false);
      }else if(recdat.data[0] == 8)// 激光 z home
      {
        queue.enqueue_now_P(PSTR("G0 Z0"));
        RTS_ResetSingleVP(AXIS_Z_COORD_VP);
        delay(1);
        RTS_ShowMotorFreeIcon(false);
      }


    break;

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
          }

          if(errorway == 4)
          {
            // reboot
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

void EachMomentUpdateLaser(void)
{
  millis_t ms = millis();
  if(ms > next_rts_update_ms)
  {
   #if ENABLED(POWER_LOSS_RECOVERY)
    // print the file before the power is off.
    if(!power_off_type_yes && lcd_sd_status && (recovery.recovery_flag == true))
    {
      rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
      if(startprogress < 100)
      {
        rtscheck.RTS_SndData(startprogress, START_PROCESS_ICON_VP);
      }
      // delay(30);
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
    else if(!power_off_type_yes && (recovery.recovery_flag == false))
    {
      rtscheck.RTS_SndData(ExchangePageBase, ExchangepageAddr);
      if(startprogress < 100)
      {
        rtscheck.RTS_SndData(startprogress, START_PROCESS_ICON_VP);
      }
      // delay(30);
      if((startprogress += 1) > 100)
      {
        power_off_type_yes = true;
        Update_Time_Value = RTS_UPDATE_VALUE;
        
        if(laser_device.is_laser_device()){
          RTS_ShowPage(51);
        }else{
          RTS_ShowPage(1);
        }
      }
      return;
    }
    else
    {
      // need to optimize
      static unsigned char last_cardpercentValue = 100;
      if(card.isPrinting() && (last_cardpercentValue != card.percentDone()))
      {
        duration_t elapsed = print_job_timer.duration();
        
        rtscheck.RTS_SndData(elapsed.value / 3600, PRINT_TIME_HOUR_VP);
        rtscheck.RTS_SndData((elapsed.value % 3600) / 60, PRINT_TIME_MIN_VP);
        if((unsigned char) card.percentDone() > 0)
        {
          Percentrecord = card.percentDone();
          if(Percentrecord <= 100)
          {
            RTS_SendProgress((unsigned char)Percentrecord);
          }
        }
        else
        {
          RTS_ResetSingleVP(PRINT_PROCESS_ICON_VP);
        }
        RTS_SendProgress((unsigned char)card.percentDone());
        last_cardpercentValue = card.percentDone();
        RTS_SendCurrentPosition(3);
      }

      if(pause_action_flag && (false == sdcard_pause_check) && printingIsPaused() && !planner.has_blocks_queued())
      {
        pause_action_flag = false;
        //queue.enqueue_now_P(PSTR("G0 F3000 X0 Y0"));
        laser_device.power = cutter.power;
        //SERIAL_ECHOPAIR("laser_device.power=", laser_device.power);
        cutter.apply_power(0);

        RTS_ShowPage(61);
        waitway = 0;
      }

      #if ENABLED(SDSUPPORT)
        if((false == sdcard_pause_check) && (false == card.isPrinting()) && !planner.has_blocks_queued())
        {
          if(CardReader::flag.mounted)
          {
            rtscheck.RTS_SndData(51, CHANGE_SDCARD_ICON_VP);
          }
          else
          {
            RTS_ResetSingleVP(CHANGE_SDCARD_ICON_VP);
          }
        }
      #endif

      if( marlin_state == MarlinState::MF_RUNNING && first_start_laser == true)
      {
        char str_1[7],cmd[20]={0};
        first_start_laser = false;
        sprintf_P(cmd, "G92.9 Z%s\n",  dtostrf(laser_device.laser_z_axis_high, 1, 2, str_1));
        //SERIAL_ECHOPGM(cmd);
        queue.inject(cmd);

        rtscheck.RTS_SndData((float)(10 * laser_device.laser_z_axis_high), AXIS_Z_COORD_VP);
        delay(1);

      }else if(laser_device.laser_z_axis_high != current_position.z && first_start_laser == false)
      {
        laser_device.save_z_axis_high_to_eeprom(current_position.z);
      }
      
    }
   #endif

    next_rts_update_ms = ms + RTS_UPDATE_INTERVAL + Update_Time_Value;
  }
}

void RTS_UpdateLaser(void)
{
  // Check the status of card
  rtscheck.RTS_SDCardUpdate();

  EachMomentUpdateLaser();
  // wait to receive massage and response
  if (rtscheck.RTS_RecData() > 0)
  {
      rtscheck.RTS_HandleData_Laser();
  }
}

// 激光模式，跑边框
void HMI_Area_Move(void)
{
  static uint8_t MINUNITMULT = 10;
  uint16_t Move_X_scaled=0, Move_Y_scaled=0;

  laser_device.is_run_range =true; //标志正在跑边框

  float y = laser_device.get_laser_range(LASER_MAX_Y) - laser_device.get_laser_range(LASER_MIN_Y);
  float x = laser_device.get_laser_range(LASER_MAX_X) - laser_device.get_laser_range(LASER_MIN_X);
  float origin_position_x = current_position.x, origin_position_y = current_position.y; // 记录当前位置


  Move_X_scaled = current_position.x*MINUNITMULT;
  Move_Y_scaled = current_position.y*MINUNITMULT;

  Move_X_scaled += laser_device.get_laser_range(LASER_MIN_X)*MINUNITMULT;
  Move_Y_scaled += laser_device.get_laser_range(LASER_MIN_Y)*MINUNITMULT;

  LIMIT(Move_X_scaled, (X_MIN_POS)*MINUNITMULT, (X_MAX_POS)*MINUNITMULT);
  LIMIT(Move_Y_scaled, (Y_MIN_POS)*MINUNITMULT, (Y_MAX_POS)*MINUNITMULT);
  current_position.x = Move_X_scaled / MINUNITMULT;
  current_position.y = Move_Y_scaled / MINUNITMULT;

  // 超出打印区域
  if(current_position.x+x > X_MAX_POS) x = X_MAX_POS - current_position.x;
  if(current_position.y+y > Y_MAX_POS) y = Y_MAX_POS - current_position.y;

  //先跑到最小位置
  // current_position.x += laser_device.get_laser_range(LASER_MIN_X);
  // current_position.y += laser_device.get_laser_range(LASER_MIN_Y);

  //HMI_Plan_Move(homing_feedrate(Y_AXIS));
  RTS_line_to_current(Y_AXIS);
  planner.synchronize();

  //current_position.y += y;
  Move_Y_scaled += y*MINUNITMULT;
  LIMIT(Move_Y_scaled, (Y_MIN_POS)*MINUNITMULT, (Y_MAX_POS)*MINUNITMULT);
  current_position.y = Move_Y_scaled / MINUNITMULT;

  laser_device.laser_power_start(5);
  RTS_line_to_current(Y_AXIS);//HMI_Plan_Move(homing_feedrate(Y_AXIS));
  planner.synchronize();

  //current_position.x += x;
  Move_X_scaled += x*MINUNITMULT;
  LIMIT(Move_X_scaled, (X_MIN_POS)*MINUNITMULT, (X_MAX_POS)*MINUNITMULT);
  current_position.x = Move_X_scaled / MINUNITMULT;

  RTS_line_to_current(X_AXIS);//HMI_Plan_Move(homing_feedrate(X_AXIS));
  planner.synchronize();

  //current_position.y -= y;
  Move_Y_scaled -= y*MINUNITMULT;
  LIMIT(Move_Y_scaled, (Y_MIN_POS)*MINUNITMULT, (Y_MAX_POS)*MINUNITMULT);
  current_position.y = Move_Y_scaled / MINUNITMULT;

  RTS_line_to_current(Y_AXIS);//HMI_Plan_Move(homing_feedrate(Y_AXIS));
  planner.synchronize();


  //current_position.x -= x;
    Move_X_scaled -= x*MINUNITMULT;
  LIMIT(Move_X_scaled, (X_MIN_POS)*MINUNITMULT, (X_MAX_POS)*MINUNITMULT);
  current_position.x = Move_X_scaled / MINUNITMULT;

  RTS_line_to_current(X_AXIS);//HMI_Plan_Move(homing_feedrate(X_AXIS));
  planner.synchronize();


  laser_device.laser_power_stop(); //关闭激光

  //回到原点位置 107011 -20211009
  // current_position.x = origin_position_x;
  // current_position.y = origin_position_y;
  Move_X_scaled = origin_position_x*MINUNITMULT;
  Move_Y_scaled = origin_position_y*MINUNITMULT;
  LIMIT(Move_X_scaled, (X_MIN_POS)*MINUNITMULT, (X_MAX_POS)*MINUNITMULT);
  LIMIT(Move_Y_scaled, (Y_MIN_POS)*MINUNITMULT, (Y_MAX_POS)*MINUNITMULT);

  current_position.x = Move_X_scaled / MINUNITMULT;
  current_position.y = Move_Y_scaled / MINUNITMULT;

  RTS_line_to_current(X_AXIS);//HMI_Plan_Move(homing_feedrate(X_AXIS));
  planner.synchronize();

  laser_device.is_run_range =false;

}
#endif
#endif // #if ENABLE(LASER_FEATURE)

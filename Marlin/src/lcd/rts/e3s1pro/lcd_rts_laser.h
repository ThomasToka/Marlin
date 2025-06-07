#ifndef LCD_RTS_LASER_H
#define LCD_RTS_LASER_H

#include "string.h"
//#include <Arduino.h>

#include "../../../inc/MarlinConfig.h"


/*********************************/
#if ENABLED(E3S1PRO_RTS) && ENABLED(LASER_FEATURE)
#define FHONE   (0x5A)
#endif
#define FHTWO   (0xA5)
#define FHLENG  (0x06)
#define TEXTBYTELEN     55
#define MaxFileNumber   40
#define AUTO_BED_LEVEL_PREHEAT  0
#define FileNum             MaxFileNumber
#define FileNameLen         TEXTBYTELEN
#define RTS_UPDATE_INTERVAL 500
#define RTS_UPDATE_VALUE    RTS_UPDATE_INTERVAL
#define SizeofDatabuf       26

extern RTSSHOW rtscheck;
extern const unsigned long Addrbuf[];
void ErrorHanding(void);
extern void RTS_Update(void);
extern void RTSInit(void);
extern float zprobe_zoffset;
extern char waitway;
extern int change_page_font;
extern uint8_t language_change_font;
extern uint8_t lang;
extern int Update_Time_Value;
extern bool PoweroffContinue;
extern bool heat_flag;
extern char commandbuf[30];
extern bool StartPrint_flag;
extern char errorway;
extern char errornum;
extern char error_sd_num;
extern unsigned char Count_first;
extern unsigned char Count_probe; 
extern float z_offset;

/*******************************类定义*********************************/
extern RTSSHOW rtscheck;

extern CardReader card;
extern char home_errornum;
extern bool StartPrint_flag;
//babystep偏移参数，Z轴偏移参数
#if ENABLED(BABYSTEPPING)
  extern float zprobe_zoffset;
  extern float last_zoffset;
#endif
extern bool power_off_type_yes;
//喷头默认PID
extern float default_nozzle_ptemp;
extern float default_nozzle_itemp;
extern float default_nozzle_dtemp;
//热床默认PID
extern float default_hotbed_ptemp;
extern float default_hotbed_itemp;
extern float default_hotbed_dtemp;
//打印进度条
extern uint8_t startprogress;
//读取SD卡GCODE文件名的结构体参数
extern CRec CardRecbuf; 
//SD卡打印是否暂停，true需要检测暂停，false已经暂停完成
extern bool sdcard_pause_check;
//暂停的动作就是回到 X0,Y0处
extern bool pause_action_flag;
//更换耗材时的设定温度
extern float ChangeFilamentTemp;
extern int heatway;
//下一次更新数据的时间
extern millis_t next_rts_update_ms;
//上一次喷头的温度
extern int last_target_temperature[4];
//上一次热床的温度
extern int last_target_temperature_bed;
//当前的页面序列
extern unsigned char Percentrecord;   //SD卡打印百分比
// represents to update file list
//SD卡文件列表更新，标志位
extern bool CardUpdate;  
//当前的语言标志位
extern uint8_t lang; 
// represents SD-card status, true means SD is available, false means opposite.
//表示 SD 卡状态，true 表示 SD 可用，false 表示相反。
extern bool lcd_sd_status;
extern char cmdbuf[20];

// 1 for 10mm, 2 for 1mm, 3 for 0.1mm
//移动轴的单位选择标志 1 代表 10mm，2 代表 1mm，3 代表 0.1mm
extern unsigned char AxisUnitMode;

//移动轴的每个单位参数
extern float axis_unit;
extern int Update_Time_Value;
//extern int temphot;
extern bool rts_start_print;  
void RTS_PauseMoveAxisPage(void);
void RTS_AutoBedLevelPage(void);
void RTS_MoveAxisHoming(void);
void RTS_MoveParkNozzle(void);
void RTS_CommandPause(void);
void HMI_Area_Move(void);
#endif

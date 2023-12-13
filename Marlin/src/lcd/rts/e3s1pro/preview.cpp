#include "stdio.h"
#include <stdio.h>
#include <arduino.h>
#include <wstring.h>
#include "lcd_rts.h"
#include "../../../inc/MarlinConfig.h"
#include "../../../sd/cardreader.h"
#include "../../../gcode/queue.h"
#include "../../../libs/duration_t.h"
#include "../../../module/settings.h"
#include "../../../core/serial.h"
#include "../../../module/temperature.h"
#include "../../../module/motion.h"
#include "../../../module/planner.h"
#include "../../../module/printcounter.h"
#include "../../../module/probe.h"
#include "base64.h"
#include "utf8_unicode.h"
#include "lcd_rts.h"
#include "preview.h"
#define BRIGHTNESS_PRINT_HIGH    250        // 进度条的总高度
#define BRIGHTNESS_PRINT_WIDTH   250        // 进度条的总宽度
#define BRIGHTNESS_PRINT_LEFT_HIGH_X   123  // 进度条的左上角-X
#define BRIGHTNESS_PRINT_LEFT_HIGH_Y   269  // 进度条的左上角-Y
#define BRIGHTNESS_PRINT    120             // 亮度值（0最暗）
#define FORMAT_JPG_HEADER "jpg begin"
#define FORMAT_JPG_HEADER_PRUSA "thumbnail_JPG begin"
#define FORMAT_JPG_HEADER_CURA "thumbnail begin"
#define FORMAT_JPG "jpg"
#define FORMAT_JPG_PRUSA "thumbnail_JPG"
#define FORMAT_JPG_CURA "thumbnail"
DwinBrightness_t printBri; // 预览图结构体
#define JPG_BYTES_PER_FRAME 240   // 每一帧发送的字节数（图片数据）
#define JPG_WORD_PER_FRAME  (JPG_BYTES_PER_FRAME/2)   // 每一帧发送的字数（图片数据）
#define SizeofDatabuf2		300
#define USER_LOGIC_DEUBG 0

#ifdef LCD_SERIAL_PORT
  #define LCDSERIAL LCD_SERIAL
#elif SERIAL_PORT_2
  #define LCDSERIAL MYSERIAL2
#endif

/**
 * 亮度调节
 * brightless addr
 * in range: 0x8800 ~ 0x8FFF
 */
#define BRIGHTNESS_ADDR_PRINT               0x8800
unsigned char databuf[SizeofDatabuf2];

// 向指定地址空间写两个字节
void DWIN_WriteOneWord(unsigned long addr, unsigned int data)
{
  rtscheck.RTS_SndData(data, addr, VarAddr_W);
}

void dwin_uart_write(unsigned char *buf, int len)
{
  for (uint8_t n = 0; n < len; ++n) { LCDSERIAL.write(buf[n]); }
}

// 发送jpg图片的一帧数据
void RTS_SendJpegDate(const char *str, unsigned long addr, unsigned char cmd/*= VarAddr_W*/)
{
  int len = JPG_BYTES_PER_FRAME;//strlen(str);
  if( len > 0)
  {
    databuf[0] = FHONE;
    databuf[1] = FHTWO;
    databuf[2] = 3+len;
    databuf[3] = cmd;
    databuf[4] = addr >> 8;
    databuf[5] = addr & 0x00FF;
    for(int i =0;i <len ;i++)
    {
      databuf[6 + i] = str[i];
    }

    dwin_uart_write(databuf, len + 6);

    memset(databuf,0, sizeof(databuf));
  }
}
// 显示jpg图片
void DWIN_DisplayJpeg(unsigned long addr, unsigned long vp)
{
  unsigned char buf[10];
  buf[0] = 0x5A;
  buf[1] = 0xA5;
  buf[2] = 0x07;
  buf[3] = 0x82;
  buf[4] = addr >> 8;     //控件地址
  buf[5] = addr & 0x00FF;
  buf[6] = 0x5A;
  buf[7] = 0xA5;
  buf[8] = vp >> 8;       //图片存储地址
  buf[9] = vp & 0x00FF;

  dwin_uart_write(buf, 10);
}
  
/**
 * Sends JPEG data to a specified address.
 *
 * @param jpeg Pointer to the JPEG data.
 * @param size The size of the JPEG data.
 * @param jpgAddr The address to send the JPEG data to.
 */
void DWIN_SendJpegDate(char *jpeg, unsigned long size, unsigned long jpgAddr)
{
  uint32_t MS = millis();

  int jpgSize = size;

  char buf[JPG_BYTES_PER_FRAME];
  int i,j;
  
  for (i = 0; i < jpgSize / JPG_BYTES_PER_FRAME; i++) {
    // delay(20);
    //memset(buf, 0, JPG_BYTES_PER_FRAME);
    memcpy(buf, &jpeg[i * JPG_BYTES_PER_FRAME], JPG_BYTES_PER_FRAME);
    hal.watchdog_refresh();

    // Send image data to the specified address
    RTS_SendJpegDate(buf, (jpgAddr + (JPG_WORD_PER_FRAME * i)), 0x82);

    if (ENABLED(DWIN_DEBUG)) {
      for (j = 0; j < JPG_BYTES_PER_FRAME; j++) {
        //SERIAL_ECHOPAIR(" ", j,
        //                " = ", buf[j]);
        //if ((j+1) % 8 == 0) SERIAL_ECHO("\r\n");
      }
    }
    
    // Dwin to color, Dwin's 7-inch screen, after sending a frame of preview image, there is no return value
    // So don't judge the exception first
    #define CMD_TAILA 0x03824F4B // Frame tail (currently effective for 4.3 inch dwin screen)
    uint8_t receivedbyte = 0;
    uint8_t cmd_pos = 0;    // Current instruction pointer state
    uint32_t cmd_state = 0; // Queue frame tail detection status

    char buffer[20] = {0};
    MS = millis();
    while(1)
    {
      if (LCDSERIAL.available())
      {

        // Take one data
        receivedbyte = LCDSERIAL.read();
        //SERIAL_ECHO_MSG("receivedbyte = ", receivedbyte);
        if (cmd_pos == 0 && receivedbyte != 0x5A) // The first byte of the instruction must be the frame header
        {
          continue;
        }

        if(cmd_pos < 20){ 
          buffer[cmd_pos++] = receivedbyte; // Prevent Overflow
        }else{
          break;
        }

        cmd_state = ((cmd_state << 8) | receivedbyte); // Concatenate the last 4 bytes to form the last 32-bit integer

        // Frame tail judgment
        if(cmd_state==CMD_TAILA)
        {
          break;
        }
      }
      if (millis() - MS >= 25)
      { // According to the data manual, delay 20ms, there is a probability that the preview image cannot be brushed out!!!
        //   PRINT_LOG("more than 25ms");
        break;
      } 
    }

  }

  if (jpgSize % JPG_BYTES_PER_FRAME)
  {
    memset(buf, 0, JPG_BYTES_PER_FRAME);
    memcpy(buf, &jpeg[i * JPG_BYTES_PER_FRAME], (jpgSize - i * JPG_BYTES_PER_FRAME));
    hal.watchdog_refresh();

    // Send image data to the specified address
    RTS_SendJpegDate(buf, (jpgAddr + (JPG_WORD_PER_FRAME * i)), 0x82);

    if (ENABLED(DWIN_DEBUG)) {
      for (j = 0; j < JPG_BYTES_PER_FRAME; j++) {
        //SERIAL_ECHOPAIR(" ", j,
        //                " = ", buf[j]);
        //if ((j+1) % 8 == 0) SERIAL_ECHO("\r\n");
      }
    }
  delay(25); // According to the data manual, delay 20ms, there is a chance of not being able to display the preview image!!!
  }
}

/**
 * Read and decode base64 encoded image data and save it into a buffer.
 *
 * @param buf the buffer to save the decoded data
 * @param picLen the length of the picture data
 * @param resetFlag a flag indicating whether to reset the data -- Because after Base64 decoding it is a multiple of 3 (4 Base64 characters are decoded into 4 byte data),
 * but the input parameter 'picLen' may not be a multiple of 3.
 * So after a single call, the remaining unused byte data is saved in "base64_out", and its length is "deCodeBase64Cnt".
 * After displaying the first picture, when displaying the second picture,
 * you need to clear these two data to prevent affecting the display of the second picture.
 *
 * @return true if the decoding process is successful, false otherwise
 */
bool gcodePicGetDataFormBase64(char * buf, unsigned long picLen, bool resetFlag)
{
  char base64_in[4];                          // Save base64 encoded array
  static unsigned char base64_out[3] = {'0'}; // Save base64 decoded array
  int getBase64Cnt = 0;                       // Data obtained from the USB flash drive, base64 encoded
  static int deCodeBase64Cnt = 0;             // Data that has been decoded
  unsigned long deCodePicLenCnt = 0;          // Save the obtained picture data

    //static char lCmdBuf[100];
    bool getPicEndFlag = false;

  //  Clear the last record
    if (resetFlag)
    {
      for (unsigned int i = 0; i < sizeof(base64_out); i++){
        //base64_out[i] = '0x00';
        base64_out[i] = '\0';
      }
      deCodeBase64Cnt = 0;
      return true;
    }

    if ((deCodeBase64Cnt > 0) && (deCodePicLenCnt < picLen))
    {
      for (int deCode = deCodeBase64Cnt; deCode > 0; deCode--)
      {
          if (deCodePicLenCnt < picLen)
          {
              buf[deCodePicLenCnt++] = base64_out[3 - deCode];
          } else {
              break;
          }
      }
    }
    
    while(deCodePicLenCnt < picLen)
    {
      char j, ret;
      for ( j = 0; j < 20; j++)
      {
        ret = card.get(); // Get a character from the USB flash drive
        if (ret == ';' || ret == ' ' || ret == '\r' || ret == '\n'){
          continue;
        }

        base64_in[getBase64Cnt++] = ret;
        if (getBase64Cnt >= 4){
          getBase64Cnt = 0;
          break;
        }
            
      }
        
      memset(base64_out, 0, sizeof(base64_out));
      deCodeBase64Cnt = base64_decode(base64_in, 4, base64_out);
      for(int i = deCodeBase64Cnt; i < 3; i++)
          base64_out[i] = 0;
      deCodeBase64Cnt = 3; // Here is forcibly given 3, because it is always 4 --> 3 characters

      int test = deCodeBase64Cnt;
      for (int deCode = 0; deCode < test; deCode++)
      {
        if (deCodePicLenCnt < picLen)
        {
          // Special treatment of the end character, exit after finding FF D9
          if (getPicEndFlag){
            buf[deCodePicLenCnt++] = 0;
          }else{
            buf[deCodePicLenCnt++] = base64_out[deCode];
          }

          if (deCodePicLenCnt > 2 && ((buf[deCodePicLenCnt-1] == 0xD9 && buf[deCodePicLenCnt-2] == 0xFF) || (buf[deCodePicLenCnt-1] == 0xd9 && buf[deCodePicLenCnt-2] == 0xff)))
          {
            getPicEndFlag = true;
          }

          deCodeBase64Cnt--;
        } else {
          break;
        }
      }

      hal.watchdog_refresh();
    }

    return true;
}

/**
 * Reading a JPEG image from gcode: 1. Send it to the screen for display; 2. Let the pointer skip this image and go find the next one.
 *
 * @param picLenth The length of the base64 encoded picture.
 * @param isDisplay Indicates whether to display the picture.
 * @param jpgAddr The address of the JPEG data.
 *
 * @return True if the function executed successfully.
 */

bool gcodePicDataRead(unsigned long picLenth, char isDisplay, unsigned long jpgAddr)
{
// Time consumed in ms: 96*96  200*200
//   * 2  :             1780   8900
//   * 4  :             940    4490
//   * 8  :             518    2010
//   * 12 :             435    1300
//   * 16 :             420    1130
#define PIN_BUG_LEN_DACAI 2048
#define PIN_BUG_LEN_DWIN (JPG_BYTES_PER_FRAME * 12)
#define PIN_DATA_LEN_DWIN (PIN_BUG_LEN_DWIN / 2)

  static char picBuf[PIN_BUG_LEN_DWIN + 1]; // Take this MAX(PIN_BUG_LEN_DACAI, PIN_BUG_LEN_DWIN)

  unsigned long picLen; // Picture length (length after decoding)
    unsigned long j;

    picLen = picLenth;//(picLenth / 4) * 3; 

    gcodePicGetDataFormBase64(picBuf, 0, true);

    // DWIN
    // First write 0 to the starting address, otherwise DWIN will freeze (or crash).
    DWIN_WriteOneWord(jpgAddr, 0);

    // Start reading
    for (j = 0; j < (picLen / PIN_BUG_LEN_DWIN); j++)
    {
      memset(picBuf, 0, sizeof(picBuf));
      // card.read(picBuf, PIN_BUG_LEN_DWIN);
      gcodePicGetDataFormBase64(picBuf, PIN_BUG_LEN_DWIN, false);
      rtscheck.RTS_SndData((j % 8) + 1, DOWNLOAD_PREVIEW_VP); // Loading image appears
      // Send image data to the specified address
      if (isDisplay) {
        DWIN_SendJpegDate(picBuf, PIN_BUG_LEN_DWIN, (2 + jpgAddr + PIN_DATA_LEN_DWIN * j));
      }
    }
    rtscheck.RTS_SndData(0, DOWNLOAD_PREVIEW_VP);
    // Process the remaining data that is less than 240 characters, according to DWIN's processing content
    // watchdog_refresh();
    if (picLen % PIN_BUG_LEN_DWIN != 0)
    {
      memset(picBuf, 0, sizeof(picBuf));
      // card.read(picBuf, (picLen - PIN_BUG_LEN_DWIN * j));  
      gcodePicGetDataFormBase64(picBuf, (picLen - PIN_BUG_LEN_DWIN * j), false);
      // Send image data to the specified address
      if (isDisplay) {
        DWIN_SendJpegDate(picBuf, (picLen - PIN_BUG_LEN_DWIN * j), (2 + jpgAddr + PIN_DATA_LEN_DWIN * j));
      }
    }
    // delay(25);
    // Used to display jpg images
    if (isDisplay){
      DWIN_DisplayJpeg(jpgAddr, picLen);
    }

    return true;
}

static uint32_t msTest;

/**
 * This function checks if a picture file exists in the given format and resolution at the specified address.
 * It does this by reading data from the file and checking specific details about the picture.
 *
 * @param fileName Pointer to the name of the picture file.
 * @param targetPicAddr The address where the picture is supposed to be located.
 * @param targetPicFormat The expected format of the picture.
 * @param targetPicResolution The expected resolution of the picture.
 *
 * @return Returns PIC_MISS_ERR if the picture is not found or if it does not meet the required specifications.
 *         Otherwise, it returns the character read from the file.
 *
 * @throws Does not throw any exceptions but returns PIC_MISS_ERR in case of any errors.
 */
char gcodePicExistjudge(char *fileName, unsigned int targitPicAddr, const char targitPicFormat, const char targitPicResolution)
{
  #define STRING_MAX_LEN      80
					
  // unsigned char picFormat = PIC_FORMAT_MAX;		// picture format
  unsigned char picResolution = PIC_RESOLITION_MAX; // picture resolution
	unsigned char ret;
  unsigned char strBuf[STRING_MAX_LEN] = {0};
  unsigned char bufIndex = 0;
  // char lCmdBuf[20];
  char *picMsgP; 
  char lPicFormar[20];
  char lPicHeder[20];
  // char lPicResolution[20];
  unsigned long picLen = 0;      // picture data length
  unsigned int picStartLine = 0; // picture start line
  unsigned int picEndLine = 0;   // picture end line
  unsigned int picHigh = 0;      // picture model height    

  // read a string, separated by spaces
  #define GET_STRING_ON_GCODE()
  {
    // read a line, separated by newline character
    memset(strBuf, 0, sizeof(strBuf));
    int strLenMax;
    bool strStartFg = false;
    uint8_t curBufLen = 0;
    uint8_t inquireYimes = 0; // number of searches
    do {
      for (strLenMax = 0; strLenMax < STRING_MAX_LEN; strLenMax++)
      {
        ret = card.get(); // get a character from the U disk
        if (ret != ';' && strStartFg == false) { // reading ';' is the start of a line
          continue;
        } else{
          strStartFg = true;
        }                        
        if ((ret == '\r' || ret == '\n') && bufIndex != 0) break;   // exit on reading a newline character
        strBuf[bufIndex++] = ret;
      }
      if (strLenMax >= STRING_MAX_LEN) {
        return PIC_MISS_ERR;
      }
      curBufLen = sizeof(strBuf);
      if (inquireYimes++ >= 5)
      {
        return PIC_MISS_ERR;
      }
    }while(curBufLen < 20);

    //SERIAL_ECHO_MSG("strBuf = ", strBuf);
    //SERIAL_ECHO_MSG("curBufLen = ", curBufLen);
  }

  // 1. read a line of data
    GET_STRING_ON_GCODE();

  // 2. judge the format of the picture (jpg, png), if it does not match, exit directly
    if (targitPicFormat == PIC_FORMAT_JPG) {
      if (strstr((const char *)strBuf, FORMAT_JPG_HEADER) == NULL) {
        if (strstr((const char *)strBuf, FORMAT_JPG_HEADER_PRUSA) == NULL) {
          if (strstr((const char *)strBuf, FORMAT_JPG_HEADER_CURA) == NULL) {
            return PIC_MISS_ERR;
          }
        }
      }
    }
    else
    {
      if ( strstr((const char *)strBuf, FORMAT_JPG_HEADER ) == NULL){
        return PIC_MISS_ERR;
      }
    }

  // 3. get the picture format content of the string
    picMsgP = strtok((char *)strBuf, (const char *)" ");
    do {
      if ( ENABLED(USER_LOGIC_DEUBG) )  SERIAL_ECHO_MSG("3.picMsgP = ", picMsgP);

      if ( picMsgP == NULL )
      {
        return PIC_MISS_ERR;
      }

      if (picMsgP != NULL && (strstr((const char *)picMsgP, FORMAT_JPG) != NULL || strstr((const char *)picMsgP, FORMAT_JPG_PRUSA) != NULL || strstr((const char *)picMsgP, FORMAT_JPG_CURA) != NULL)) break;

      picMsgP = strtok(NULL, (const char *)" ");
    }while(1);

  // 4. get the "start" field
    picMsgP = strtok(NULL, (const char *)" ");
    if ( ENABLED(USER_LOGIC_DEUBG) )  SERIAL_ECHO_MSG("4.picMsgP = ", picMsgP,  " strlen(picMsgP) = ", strlen(picMsgP));
    if ( picMsgP != NULL )
    {
      memset(lPicHeder, 0, sizeof(lPicHeder));
      memcpy(lPicHeder, picMsgP, strlen(picMsgP));
    }

  // 5. get the picture size field 200*200, 300*300, etc.
    picMsgP = strtok(NULL, (const char *)" ");
    if ( ENABLED(USER_LOGIC_DEUBG) )  SERIAL_ECHO_MSG("5.picMsgP = ", picMsgP, " strlen(picMsgP) = ", strlen(picMsgP));
    if ( picMsgP != NULL )
    {
      picResolution = PIC_RESOLITION_MAX;
      if (strcmp(picMsgP, RESOLITION_250_250) == 0 || strcmp(picMsgP, RESOLITION_250_250_PRUSA) == 0) {
        picResolution = PIC_RESOLITION_250_250;            
      }
    }
    

  // 6. get the picture data length
    picMsgP = strtok(NULL, (const char *)" ");
    if ( ENABLED(USER_LOGIC_DEUBG) )  SERIAL_ECHO_MSG("6.picMsgP = ", picMsgP);
    if ( picMsgP != NULL )
    {
      picLen = atoi(picMsgP);

      if (picLen > 24500) {
        return PIC_MISS_ERR;  // Define PICLEN_ERR similar to other error codes
      }
    }

  // 7. get the start line of the picture
    picMsgP = strtok(NULL, (const char *)" ");
    if ( ENABLED(USER_LOGIC_DEUBG) )  SERIAL_ECHO_MSG("7.picMsgP = ", picMsgP);
    if ( picMsgP != NULL )
    {
      picStartLine = atoi(picMsgP);
    }

  // 8. get the end line of the picture
    picMsgP = strtok(NULL, (const char *)" ");
    if ( ENABLED(USER_LOGIC_DEUBG) )  SERIAL_ECHO_MSG("8.picMsgP = ", picMsgP);
    if ( picMsgP != NULL )
    {
      picEndLine = atoi(picMsgP);
    }

  // 9. get the height of the model
    picMsgP = strtok(NULL, (const char *)" ");
    if ( ENABLED(USER_LOGIC_DEUBG) )  SERIAL_ECHO_MSG("9.picMsgP = ", picMsgP);
    if ( picMsgP != NULL )
    {
      picHigh = atoi(picMsgP);
    }

    if (ENABLED(USER_LOGIC_DEUBG)) 
    {
      SERIAL_ECHO_MSG("lPicFormar = ", lPicFormar);
      SERIAL_ECHO_MSG("lPicHeder = ", lPicHeder);
      SERIAL_ECHO_MSG("picResolution = ", picResolution);
      SERIAL_ECHO_MSG("picLen = ", picLen);
      SERIAL_ECHO_MSG("picStartLine = ", picStartLine);
      SERIAL_ECHO_MSG("picEndLine = ", picEndLine);
      SERIAL_ECHO_MSG("picHigh = ", picHigh);
    }
    

    if (ENABLED(USER_LOGIC_DEUBG)) 
    {
      //SERIAL_ECHOPAIR("\r\n gcode pic time test 1 msTest = ", (millis() - msTest));
      msTest = millis();
    }

  // read the picture data from the gcode, and judge whether it needs to be sent to the screen based on whether the selected is a predetermined format or predetermined size picture
  // determine whether the required resolution is needed
    if ( picResolution == targitPicResolution )
    {
      gcodePicDataRead(picLen, true, targitPicAddr);
    }
    else
    {
      // directly move the pointer and skip invalid pictures
      // The protocol stipulates a complete line of data: ';' + ' ' + 'Data' + '\n' 1+1+76+1 = 79 bytes
      // The last line is '; png end\r' or '; jpg end\r',
      uint32_t index1 = card.getFileCurPosition();//card.getIndex();
      uint32_t targitPicpicLen = 0;
      if ( picLen % 3 == 0 ) {
        targitPicpicLen = picLen / 3 * 4;
      } else {
        targitPicpicLen = (picLen / 3 + 1) * 4; 
      }
      uint32_t indexAdd = (targitPicpicLen / 76) * 3 + targitPicpicLen + 10;
      if ( (targitPicpicLen % 76 ) != 0) {
        indexAdd += 3;
      }

      card.setIndex((index1 + indexAdd));
      if ( ENABLED(USER_LOGIC_DEUBG) ) 
      {
        //SERIAL_ECHOLNPAIR("\r\n ...old_index1 = ", index1,
        //                  "\r\n ...indexAdd = ", indexAdd);
      }

      if ( picResolution != targitPicResolution ){
        return PIC_RESOLITION_ERR;
      } else {
        return PIC_FORMAT_ERR;
      }
    }

    //card.closefile();
    if ( ENABLED(USER_LOGIC_DEUBG) ) 
	  //SERIAL_ECHOPAIR("\r\n gcode pic time test 3 msTest = ", (millis() - msTest));
    msTest = millis();
	return PIC_OK;
}

/**
 * Checks if the given picture exists in the gcode and validates its format and resolution.
 *
 * @param fileName the name of the file containing the picture
 * @param jpgAddr the address of the picture in memory
 * @param jpgFormat the format of the picture
 * @param jpgResolution the resolution of the picture
 *
 * @return the result of the operation: PIC_MISS_ERR if the picture is missing, PIC_OK if the picture is valid
 *
 * @throws None
 */
char gcodePicDataSendToDwin(char *fileName, unsigned int jpgAddr, unsigned char jpgFormat, unsigned char jpgResolution)
{
  char ret;
  char returyCnt = 0;
  card.openFileRead(fileName);
  msTest = millis();
  while (1)
  {
    ret = gcodePicExistjudge(fileName, jpgAddr, jpgFormat, jpgResolution);
    if (ret == PIC_MISS_ERR) // When there is no pic in the gcode, return directly
    {
      card.closefile();
      return PIC_MISS_ERR;
    }
    else if ((ret == PIC_FORMAT_ERR) || (ret == PIC_RESOLITION_ERR)) // When there is a format or size error, continue to judge further
    {
      if (++returyCnt >= 3)
      {
        card.closefile();
        return PIC_MISS_ERR;
      } else {
        continue;
      }
    }
    else 
    {
      card.closefile();
      return PIC_OK;
    }
  }

}

/**
 * Toggles the display of a Gcode picture preview.
 *
 * @param jpgAddr the address of the Gcode picture
 * @param onoff a boolean indicating whether to show the Gcode preview (true==show, false==hide)
 */

void gcodePicDisplayOnOff(unsigned int jpgAddr, bool onoff)
{
  if (onoff) {
    rtscheck.RTS_SndData(1, jpgAddr);  
  } else {
    rtscheck.RTS_SndData(0, jpgAddr);
  }
}

/**
 * Controls the brightness of the DWIN display.
 *
 * @param brightness the brightness values for different corners of the display
 */
void DWIN_BrightnessCtrl(DwinBrightness_t brightness)
{
  unsigned int buf[10];
  buf[0] = brightness.LeftUp_X; // brightness
  buf[1] = brightness.LeftUp_Y;
  buf[2] = brightness.RightDown_X; // brightness
  buf[3] = brightness.RightDown_Y;

  // Display Area Modification
  DWIN_WriteOneWord(brightness.spAddr, buf[0]);
  DWIN_WriteOneWord(brightness.spAddr + 1, buf[1]);
  DWIN_WriteOneWord(brightness.spAddr + 2, buf[2]);
  DWIN_WriteOneWord(brightness.spAddr + 3, buf[3]);

  // Brightness Adjustment
  DWIN_WriteOneWord(brightness.addr, brightness.brightness);
}

/**
 * Refreshes the display brightness of the G-code preview image during printing.
 *
 * @param persent the percentage value to calculate the brightness
 */
void RefreshBrightnessAtPrint(uint16_t persent)
{
  printBri.brightness = BRIGHTNESS_PRINT;
  printBri.addr = BRIGHTNESS_ADDR_PRINT;
  printBri.spAddr = SP_ADDR_BRIGHTNESS_PRINT + 1;
  printBri.LeftUp_X = BRIGHTNESS_PRINT_LEFT_HIGH_X;
  printBri.LeftUp_Y = BRIGHTNESS_PRINT_LEFT_HIGH_Y;
  printBri.RightDown_X = BRIGHTNESS_PRINT_LEFT_HIGH_X + BRIGHTNESS_PRINT_WIDTH;
  printBri.RightDown_Y = BRIGHTNESS_PRINT_LEFT_HIGH_Y + (100 - persent) * BRIGHTNESS_PRINT_HIGH / 100;

  DWIN_BrightnessCtrl(printBri);
}

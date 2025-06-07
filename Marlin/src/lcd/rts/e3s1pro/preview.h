#ifndef PREWIEW_H
#define PREWIEW_H

/*****************  gcode embedded preview image read related definition (start)  ******************/
// Picture file identification bit (length 4 bytes)
#define PIC_HEADER "begin"
// #define PIC_PRESENT_LEN	4
// Picture length bit (length 4 bytes)
// #define PIC_DATA_LEN	4
// Picture type (length 1 byte)
// #define PIC_FORMAT_LEN	1		// Picture format length (bytes)
enum
{
  PIC_FORMAT_JPG = 0x00,       // jpg format picture
  PIC_FORMAT_MAX = 0x01,       //
  PIC_FORMAT_JPG_PRUSA = 0x02, // jpg format picture
};

#define FORMAT_JPG "jpg"
#define FORMAT_JPG_PRUSA "thumbnail_JPG"

// Resolution (length 1 byte)
// #define PIC_RESOLUTION_LEN		1		// Picture resolution length (bytes)
enum
{
  PIC_RESOLUTION_250_250 = 0x00, // Resolution = 250*250
  PIC_RESOLUTION_MAX,            // gcode without picture
};

typedef struct
{
  unsigned long addr;       // Variable address
  unsigned long spAddr;     // Description pointer
  unsigned int brightness;  // Brightness (0x0000 - 0x0100, unit is 1/256)
  unsigned int LeftUp_X;    // Display area's top-left corner X
  unsigned int LeftUp_Y;    // Display area's top-left corner Y
  unsigned int RightDown_X; // Display area's bottom-right corner X
  unsigned int RightDown_Y; // Display area's bottom-right corner Y
} DwinBrightness_t;

#define RESOLUTION_250_250 "250*250"
#define RESOLUTION_250_250_PRUSA "250x250"
#define VP_BRIGHTNESS_PRINT 0x8800
#define VP_OVERLAY_PIC_PTINT 0xA000 /* Preview image of the print interface */

/* Brightness of the model in the print interface, used to represent a countdown method (occupies 40 bytes, 0xA000 ~ 0xA01F) */
#define SP_ADDR_BRIGHTNESS_PRINT 0x9000
// // Model row start bit (2 bytes)
// #define PIC_START_LINE_LEN		2		// Picture start row length (bytes)
// // Model row end bit (2 bytes)
// #define PIC_END_LINE_LEN		2		// Picture end row length (bytes)
// // Model height (2 bytes)
// #define PIC_HIGH_LINE_LEN		2		// Picture height length (bytes)
// // Data leading bit before picture
// #define DATA_BEFOR_PIC_LENTH	(PIC_FORMAT_LEN + PIC_RESOLUTION_LEN + PIC_START_LINE_LEN + PIC_END_LINE_LEN + PIC_HIGH_LINE_LEN)
// Function return information
enum
{
  PIC_OK,             // Picture display ok
  PIC_FORMAT_ERR,     // Picture format error
  PIC_RESOLUTION_ERR, // Picture resolution error
  PIC_MISS_ERR,       // gcode without picture
};

#define PRIWIEW_PIC_FORMAT_NEED PIC_FORMAT_JPG
#define PRINT_PIC_RESOLUTION_NEED PIC_RESOLUTION_250_250

/*****************  gcode embedded preview image read related definition (end)  ******************/
void RefreshBrightnessAtPrint(uint16_t persent);
bool gcodePicGetDataFormBase64(char * buf, unsigned long picLen, bool resetFlag);
bool gcodePicDataRead(unsigned long picLength, char isDisplay, unsigned long jpgAddr);
char gcodePicExistjudge(char *fileName, unsigned int targetPicAddr, const char targetPicFormat, const char targetPicResolution);
char gcodePicDataSendToDwin(char *fileName, unsigned int jpgAddr, unsigned char jpgFormat, unsigned char jpgResolution);
char gcodePicDataOctoPrintSendToDwin(char *fileName, unsigned int jpgAddr, unsigned char jpgFormat, unsigned char jpgResolution);
void gcodePicDisplayOnOff(unsigned int jpgAddr, bool onoff);
extern unsigned long picLen;      // picture data length
extern unsigned int picStartLine; // picture start line
extern unsigned int picEndLine;   // picture end line

#endif

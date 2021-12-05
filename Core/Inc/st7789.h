#ifndef __ST7789_H
#define __ST7789_H
#include "main.h"
#define WIDTH		240
#define HEIGHT		240
#define BPP		16

/* Init script function */
struct st7789_function {
  uint8_t cmd;
  uint16_t data;
};

/* Init script commands */
enum st7789_cmd {
  ST7789_START,
  ST7789_END,
  ST7789_CMD,
  ST7789_DATA,
  ST7789_DELAY
};

/* ST7789 Commands */
#define ST7789_CASET	0x2A
#define ST7789_RASET	0x2B
#define ST7789_RAMWR	0x2C
#define ST7789_RAMRD	0x2E

int st7789_init();
void myLcdWriteReg(uint8_t Data);
void myLcdWriteData(uint8_t Data);
void myLcdWriteDataMultiple(uint8_t * pData, int NumItems);

void ST7789H2_WritePixel(uint16_t Xpos, uint16_t Ypos, uint16_t RGBCode);
void ST7789H2_WriteLine(uint16_t Xpos, uint16_t Ypos, uint16_t *RGBCode, uint16_t pointNum);
void BSP_LCD_Clear(uint16_t Color);
#endif /* __ST7789_H */

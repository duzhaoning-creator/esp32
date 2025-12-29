// #ifndef _ST7789_DRIVER_H_
// #define _ST7789_DRIVER_H_
// #include "driver/gpio.h"
// #include "esp_err.h"
// #include "driver/spi_master.h"
// #include "driver/gpio.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "font.h"  // 字库头文件（下文提供）
// #include <stdio.h>
// #include <inttypes.h>
// #include "sdkconfig.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_chip_info.h"
// #include "esp_flash.h"
// #include "esp_system.h"

// // 屏幕参数
// #define TFT_WIDTH  128
// #define TFT_HEIGHT 160
// #define TFT_COLOR_DEPTH 16  // RGB565

// // 引脚定义（可修改）
// #define TFT_SPI_HOST    SPI2_HOST
// #define TFT_SCK_PIN     GPIO_NUM_12
// #define TFT_MOSI_PIN    GPIO_NUM_11
// #define TFT_DC_PIN      GPIO_NUM_40
// #define TFT_CS_PIN      GPIO_NUM_21
// #define TFT_RST_PIN     GPIO_NUM_42
// #define TFT_BL_PIN      GPIO_NUM_2

// // RGB565 颜色定义
// #define COLOR_BLACK     0x0000
// #define COLOR_WHITE     0xFFFF
// #define COLOR_RED       0xF800
// #define COLOR_GREEN     0x07E0
// #define COLOR_BLUE      0x001F
// #define COLOR_YELLOW    0xFFE0
// #define COLOR_PURPLE    0xF81F

// // 表情位图（示例：笑脸，16×16 像素）
// #define FACE_WIDTH  16
// #define FACE_HEIGHT 16

// #define ST7735_GREENTAB160x80 // For 160 x 80 display (BGR, inverted, 26 / 1 offset) 0.96 tft
// #define COLSTART            26
// #define ROWSTART            1

// // Delay between some initialisation commands
// #define TFT_INIT_DELAY      0x80

// #define TFT_INVOFF          0x20
// #define TFT_INVON           0x21

// // ST7735 specific commands used in init
// #define ST7735_NOP          0x00
// #define ST7735_SWRESET      0x01
// #define ST7735_RDDID        0x04
// #define ST7735_RDDST        0x09

// #define ST7735_SLPIN        0x10
// #define ST7735_SLPOUT       0x11
// #define ST7735_PTLON        0x12
// #define ST7735_NORON        0x13

// #define ST7735_INVOFF       0x20
// #define ST7735_INVON        0x21
// #define ST7735_DISPOFF      0x28
// #define ST7735_DISPON       0x29
// #define ST7735_CASET        0x2A
// #define ST7735_RASET        0x2B
// #define ST7735_RAMWR        0x2C
// #define ST7735_RAMRD        0x2E

// #define ST7735_PTLAR        0x30
// #define ST7735_VSCRDEF      0x33
// #define ST7735_COLMOD       0x3A
// #define ST7735_MADCTL       0x36
// #define ST7735_VSCRSADD     0x37

// #define ST7735_FRMCTR1      0xB1
// #define ST7735_FRMCTR2      0xB2
// #define ST7735_FRMCTR3      0xB3
// #define ST7735_INVCTR       0xB4
// #define ST7735_DISSET5      0xB6

// #define ST7735_PWCTR1       0xC0
// #define ST7735_PWCTR2       0xC1
// #define ST7735_PWCTR3       0xC2
// #define ST7735_PWCTR4       0xC3
// #define ST7735_PWCTR5       0xC4
// #define ST7735_VMCTR1       0xC5

// #define ST7735_RDID1        0xDA
// #define ST7735_RDID2        0xDB
// #define ST7735_RDID3        0xDC
// #define ST7735_RDID4        0xDD

// #define ST7735_PWCTR6       0xFC

// #define ST7735_GMCTRP1      0xE0
// #define ST7735_GMCTRN1      0xE1


// typedef void(*lcd_flush_done_cb)(void* param);

// typedef struct
// {
//     gpio_num_t  mosi;       //数据
//     gpio_num_t  clk;        //时钟
//     gpio_num_t  cs;         //片选
//     gpio_num_t  dc;         //命令
//     gpio_num_t  rst;        //复位
//     gpio_num_t  bl;         //背光
//     uint32_t    spi_fre;    //spi总线速率
//     uint16_t    width;      //宽
//     uint16_t    height;     //长
//     uint8_t     spin;       //选择方向(0不旋转，1顺时针旋转90, 2旋转180，3顺时针旋转270)
//     lcd_flush_done_cb   done_cb;    //数据传输完成回调函数
//     void*       cb_param;   //回调函数参数
// }st7789_cfg_t;


// /** st7789初始化
//  * @param st7789_cfg_t  接口参数
//  * @return 成功或失败
// */
// esp_err_t st7789_driver_hw_init();

// /** st7789写入显示数据
//  * @param x1,x2,y1,y2:显示区域
//  * @return 无
// */
// void st7789_flush(int x1,int x2,int y1,int y2,void *data);
// void st7735_draw_rect();
// /** 控制背光
//  * @param enable 是否使能背光
//  * @return 无
// */
// void st7789_lcd_backlight(bool enable);   

// #endif


#ifndef ST7735_H
#define ST7735_H

#include "driver/spi_master.h"
#include <stdint.h>
#include <stddef.h>
#include "font.h"

extern spi_device_handle_t spi;

#define PIN_NUM_MISO -1
#define PIN_NUM_MOSI 11
#define PIN_NUM_CLK  12
#define PIN_NUM_CS   21
#define PIN_NUM_DC   40
#define PIN_NUM_RST  42
#define PIN_NUM_BCKL 2

#define LCD_HOST    SPI2_HOST
#define DMA_CHAN    SPI_DMA_CH_AUTO

#define xOffset 0
#define yOffset 0






// ST7735 Commands
#define ST7735_SLPOUT   0x11
#define ST7735_FRMCTR1  0xB1
#define ST7735_FRMCTR2  0xB2
#define ST7735_FRMCTR3  0xB3
#define ST7735_INVCTR   0xB4
#define ST7735_PWCTR1   0xC0
#define ST7735_PWCTR2   0xC1
#define ST7735_PWCTR3   0xC2
#define ST7735_PWCTR4   0xC3
#define ST7735_PWCTR5   0xC4
#define ST7735_VMCTR1   0xC5
#define ST7735_COLMOD   0x3A
#define ST7735_GMCTRP1  0xE0
#define ST7735_GMCTRN1  0xE1
#define ST7735_NORON    0x13
#define ST7735_DISPON   0x29
#define ST7735_CASET    0x2A
#define ST7735_RASET    0x2B
#define ST7735_RAMWR    0x2C
#define ST7735_INVOFF   0x20
#define ST7735_INVON    0x21

#define ST7735_MADCTL     0x36
#define ST7735_MADCTL_MX  0x40
#define ST7735_MADCTL_MY  0x80
#define ST7735_MADCTL_MV  0x20

#define ST7735_WIDTH  128
#define ST7735_HEIGHT 160

#define ST7735_BLACK   0x0000
#define ST7735_BLUE    0x001F
#define ST7735_RED     0xF800
#define ST7735_GREEN   0x07E0
#define ST7735_CYAN    0x07FF
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW  0xFFE0
#define ST7735_WHITE   0xFFFF
#define ST7735_COLOR565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd);
void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len);



void ST7735_Init(void);
void ST7735_Init_DMA();
void ST7735_SetRotation(uint8_t rotation);
void ST7735_FillScreen(uint16_t color);
void ST7735_DrawRectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
void ST7735_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bgColor, const FontDef *font);
void ST7735_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bgColor, const FontDef *font);
void ST7735_DrawImage(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *image);
void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
void ST7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void ST7735_DrawBitmap_DMA(const uint16_t *bitmap, uint16_t width, uint16_t height);

#endif // ST7735_H
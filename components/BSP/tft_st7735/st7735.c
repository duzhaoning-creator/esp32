// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// #include "esp_lcd_panel_io.h"
// #include "esp_lcd_panel_vendor.h"
// #include "esp_lcd_panel_ops.h"
// #include "esp_lcd_panel_commands.h"
// #include "driver/gpio.h"
// #include "driver/spi_master.h"
// #include "esp_err.h"
// #include "esp_log.h"
// #include "st7789_driver.h"
// #include "font.h"  // 字库头文件（下文提供）
// #include "esp_err.h"

// #define LCD_SPI_HOST    SPI2_HOST

// static const char* TAG = "st7789";

// //lcd操作句柄
// static esp_lcd_panel_io_handle_t lcd_io_handle = NULL;

// //刷新完成回调函数
// static lcd_flush_done_cb    s_flush_done_cb = NULL;



// static bool notify_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
// {
//     if(s_flush_done_cb)
//         s_flush_done_cb(user_ctx);
//     return false;
// }


// /** st7789初始化
//  * @param st7789_cfg_t  接口参数
//  * @return 成功或失败
// */
// esp_err_t st7789_driver_hw_init()
// {
//     //初始化SPI
//     spi_bus_config_t buscfg = {
//         .sclk_io_num = TFT_SCK_PIN,        //SCLK引脚
//         .mosi_io_num = TFT_MOSI_PIN,       //MOSI引脚
//         .miso_io_num = -1,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//         .flags = SPICOMMON_BUSFLAG_MASTER , //SPI主模式
//         .max_transfer_sz = TFT_WIDTH * 40 * sizeof(uint16_t),  //DMA单次传输最大字节，最大32768
//     };
//     ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    

    
//     //初始化GPIO(BL)
//     gpio_config_t bl_gpio_cfg = 
//     {
//         .pull_up_en = GPIO_PULLUP_DISABLE,          //禁止上拉
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,      //禁止下拉
//         .mode = GPIO_MODE_OUTPUT,                   //输出模式
//         .intr_type = GPIO_INTR_DISABLE,             //禁止中断
//         .pin_bit_mask = (1<<TFT_BL_PIN)                //GPIO脚
//     };
//     gpio_config(&bl_gpio_cfg);


//     //初始化复位脚
//     if(TFT_RST_PIN > 0)
//     {
//         gpio_config_t rst_gpio_cfg = 
//         {
//             .pull_up_en = GPIO_PULLUP_DISABLE,          //禁止上拉
//             .pull_down_en = GPIO_PULLDOWN_DISABLE,      //禁止下拉
//             .mode = GPIO_MODE_OUTPUT,                   //输出模式
//             .intr_type = GPIO_INTR_DISABLE,             //禁止中断
//             .pin_bit_mask = (1<<TFT_RST_PIN)                //GPIO脚
//         };
//         gpio_config(&rst_gpio_cfg);
//     }

//     //创建基于spi的lcd操作句柄
//     esp_lcd_panel_io_spi_config_t io_config = {
//         .dc_gpio_num = TFT_DC_PIN,         //DC引脚
//         .cs_gpio_num = TFT_CS_PIN,         //CS引脚
//         .pclk_hz = 60*1000*1000,        //SPI时钟频率
//         .lcd_cmd_bits = 8,              //命令长度
//         .lcd_param_bits = 8,            //参数长度
//         .spi_mode = 0,                  //使用SPI0模式
//         .trans_queue_depth = 10,        //表示可以缓存的spi传输事务队列深度
//         // .on_color_trans_done = notify_flush_ready,   //刷新完成回调函数
//         // .user_ctx = cfg->cb_param,                                    //回调函数参数
//         .flags = {    // 以下为 SPI 时序的相关参数，需根据 LCD 驱动 IC 的数据手册以及硬件的配置确定
//             .sio_mode = 0,    // 通过一根数据线（MOSI）读写数据，0: Interface I 型，1: Interface II 型
//         },
//     };
//     // Attach the LCD to the SPI bus
//     ESP_LOGI(TAG,"create esp_lcd_new_panel");
//     ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_config, &lcd_io_handle));
    
//     //硬件复位
//     if(TFT_RST_PIN > 0)
//     {
//         gpio_set_level(TFT_RST_PIN,0);
//         vTaskDelay(pdMS_TO_TICKS(20));
//         gpio_set_level(TFT_RST_PIN,1);
//         vTaskDelay(pdMS_TO_TICKS(150));
//     }
// static const lcd_init_cmd_t vendor_specific_init[] = {
//     	{ST7735_SWRESET, {0}, 0x80},         		// Software reset, 0 args, w/delay 150
// 		{ST7735_SLPOUT, {0}, 0x80},                 // Out of sleep mode, 0 args, w/delay 500
// 		{ST7735_FRMCTR1, {0x05, 0x3A, 0x3A}, 3},    // Frame rate ctrl - normal mode, 3 args: Rate = fosc/(1x2+40) * (LINE+2C+2D)
// 		{ST7735_FRMCTR2, {0x05, 0x3A, 0x3A}, 3},    // Frame rate control - idle mode, 3 args:Rate = fosc/(1x2+40) * (LINE+2C+2D)
// 		{ST7735_FRMCTR3, {0x05, 0x3A, 0x3A, 0x05, 0x3A, 0x3A}, 6}, //Frame rate ctrl - partial mode, 6 args:Dot inversion mode. Line inversion mode
// 		{ST7735_INVCTR, {0x03}, 1},                 // Display inversion ctrl, 1 arg, no delay:No inversion
// 		{ST7735_PWCTR1, {0x62, 0x02, 0x04}, 3},      // Power control, 3 args, no delay:-4.6V AUTO mode
// 		{ST7735_PWCTR2, {0xC0}, 1},                 // Power control, 1 arg, no delay:VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
// 		{ST7735_PWCTR3, {0x0D, 0x00}, 2},           // Power control, 2 args, no delay: Opamp current small, Boost frequency
// 		{ST7735_PWCTR4, {0x8D, 0x6A}, 2},           // Power control, 2 args, no delay: BCLK/2, Opamp current small & Medium low
// 		{ST7735_PWCTR5, {0x8D, 0xEE}, 2},           // Power control, 2 args, no delay:
// 		{ST7735_VMCTR1, {0x0E}, 1},                 // Power control, 1 arg, no delay:
// 		{ST7735_INVON, {0}, 0},                     // set inverted mode
//  		//{ST7735_INVOFF, {0}, 0},                    // set non-inverted mode
// 		{ST7735_COLMOD, {0x05}, 1},               	// set color mode, 1 arg, no delay: 16-bit color
// 		{ST7735_GMCTRP1, {0x10, 0x0E, 0x02, 0x03, 0x0E, 0x07, 0x02, 0x07, 0x0A, 0x12, 0x27, 0x37, 0x00, 0x0D, 0x0E, 0x10}, 16},           // 16 args, no delay:
// 		{ST7735_GMCTRN1, {0x10, 0x0E, 0x03, 0x03, 0x0F, 0x06, 0x02, 0x08, 0x0A, 0x13, 0x26, 0x36, 0x00, 0x0D, 0x0E, 0x10}, 16},           // 16 args, no delay:
// 		{ST7735_NORON, {0}, TFT_INIT_DELAY},       	// Normal display on, no args, w/delay 10 ms delay
// 		{ST7735_DISPON, {0}, TFT_INIT_DELAY},       // Main screen turn on, no args w/delay 100 ms delay
// 		{0, {0}, 0xff}
// };
//     esp_lcd_panel_io_tx_param(io, LCD_CMD_CASET, (uint8_t[]) {
//         (x_start >> 8) & 0xFF,
//         (x_start + COLSTART) & 0xFF,
//         ((x_end - 1) >> 8) & 0xFF,
//         (x_end - 1 + COLSTART) & 0xFF,
//     }, 4);
//     esp_lcd_panel_io_tx_param(io, LCD_CMD_RASET, (uint8_t[]) {
//         (y_start >> 8) & 0xFF,
//         (y_start + ROWSTART) & 0xFF,
//         ((y_end - 1) >> 8) & 0xFF,
//         (y_end -1 + ROWSTART) & 0xFF,
//     }, 4);

//     /*向LCD写入初始化命令*/
//     esp_lcd_panel_io_tx_param(lcd_io_handle,LCD_CMD_SWRESET,NULL,0);    //软件复位
//     vTaskDelay(pdMS_TO_TICKS(150));
//     esp_lcd_panel_io_tx_param(lcd_io_handle,LCD_CMD_SLPOUT,NULL,0);     //退出休眠模式
//     vTaskDelay(pdMS_TO_TICKS(200));
//     esp_lcd_panel_io_tx_param(lcd_io_handle,LCD_CMD_COLMOD,(uint8_t[]) {0x55,}, 1);  //选择RGB数据格式，0x55:RGB565,0x66:RGB666
//     esp_lcd_panel_io_tx_param(lcd_io_handle, 0xb0, (uint8_t[]) {0x00, 0xF0},2);

//     esp_lcd_panel_io_tx_param(lcd_io_handle,LCD_CMD_INVON,NULL,0);     //颜色翻转
//     esp_lcd_panel_io_tx_param(lcd_io_handle,LCD_CMD_NORON,NULL,0);     //普通显示模式
//     uint8_t spin_type = 0;
//     switch(0)
//     {
//         case 0:
//             spin_type = 0x00;   //不旋转
//             break;
//         case 1:
//             spin_type = 0x60;   //顺时针90
//             break;
//         case 2:
//             spin_type = 0xC0;   //180
//             break;
//         case 3:
//             spin_type = 0xA0;   //顺时针270,（逆时针90）
//             break;
//         default:break;
//     }
//     esp_lcd_panel_io_tx_param(lcd_io_handle,LCD_CMD_MADCTL,(uint8_t[]) {spin_type,}, 1);   //屏旋转方向
//     vTaskDelay(pdMS_TO_TICKS(150));
//     esp_lcd_panel_io_tx_param(lcd_io_handle,LCD_CMD_DISPON,NULL,0);    //打开显示
//     vTaskDelay(pdMS_TO_TICKS(300));
//     return ESP_OK;
// }

// /** st7789写入显示数据
//  * @param x1,x2,y1,y2:显示区域
//  * @return 无
// */
// void st7789_flush(int x1,int x2,int y1,int y2,void *data)
// {
//     // define an area of frame memory where MCU can access
//     if(x2 <= x1 || y2 <= y1)
//     {
//         if(s_flush_done_cb)
//             s_flush_done_cb(NULL);
//         return;
//     }
//     esp_lcd_panel_io_tx_param(lcd_io_handle, LCD_CMD_CASET, (uint8_t[]) {
//         (x1 >> 8) & 0xFF,
//         x1 & 0xFF,
//         ((x2 - 1) >> 8) & 0xFF,
//         (x2 - 1) & 0xFF,
//     }, 4);
//     esp_lcd_panel_io_tx_param(lcd_io_handle, LCD_CMD_RASET, (uint8_t[]) {
//         (y1 >> 8) & 0xFF,
//         y1 & 0xFF,
//         ((y2 - 1) >> 8) & 0xFF,
//         (y2 - 1) & 0xFF,
//     }, 4);
//     // transfer frame buffer
//     size_t len = (x2 - x1) * (y2 - y1) * 2;
//     esp_lcd_panel_io_tx_color(lcd_io_handle, LCD_CMD_RAMWR, data, len);
//     return ;
// }
// // 示例：刷新指定小区域（比如左上角 20×20 红色方块）
// void st7735_draw_rect(void) {
//     uint8_t buf[20*20*2];  // 20×20 像素，每个像素2字节
//     uint16_t red = 0xF800; // RGB565 红色
//     uint8_t r_h = red >> 8;
//     uint8_t r_l = red & 0xFF;

//     // 填充红色数据
//     for (int i = 0; i < 20*20; i++) {
//         buf[i*2] = r_h;
//         buf[i*2+1] = r_l;
//     }

//     // 刷新区域：x1=0, x2=20, y1=0, y2=20
//     st7789_flush(0, 20, 0, 20, buf);
// }
// /** 控制背光
//  * @param enable 是否使能背光
//  * @return 无
// */
// void st7789_lcd_backlight(bool enable)
// {
//     if(enable)
//     {
//         gpio_set_level(TFT_BL_PIN,1);
//     }
//     else
//     {
//         gpio_set_level(TFT_BL_PIN,0);
//     }
// }


#include "st7735.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "font.h"

spi_device_handle_t spi;

void lcd_cmd(spi_device_handle_t spi, const uint8_t cmd) {

    // spi_transaction_t t;
    // memset(&t, 0, sizeof(t));
    // t.length = 8;
    // t.tx_data[0] = cmd;
    // t.flags = SPI_TRANS_USE_TXDATA;
    // gpio_set_level(PIN_NUM_DC, 0);
    // spi_device_transmit(spi, &t);

    // 1. 零初始化结构体（替代memset，更高效）
    spi_transaction_t t = {0};
    t.length = 8;                  // 1字节命令 = 8比特
    t.tx_data[0] = cmd;            // 命令写入tx_data（1字节）
    t.flags = SPI_TRANS_USE_TXDATA;// 启用tx_data，无需tx_buffer
    gpio_set_level(PIN_NUM_DC, 0); // DC=0：标识发送命令

    // 2. 发送命令并检查返回值（关键：定位通信故障）
    esp_err_t ret = spi_device_transmit(spi, &t);
    if (ret != ESP_OK) {
        printf("rrrrrraaaaa1");
    }


}



void lcd_data(spi_device_handle_t spi, const uint8_t *data, int len) {

    // spi_transaction_t t;
    // if (len == 0) return;
    // memset(&t, 0, sizeof(t));
    // t.length = len * 8;
    // t.tx_buffer = data;
    // gpio_set_level(PIN_NUM_DC, 1);
    // spi_device_transmit(spi, &t);

 

    spi_transaction_t t = {0}; // 零初始化，替代memset更高效
    t.length = len * 8;        // 总比特数
    gpio_set_level(PIN_NUM_DC, 1); // DC=1：数据模式

    // 短数据（≤4字节）用tx_data，长数据用tx_buffer
    if (len <= 4) {
        t.flags = SPI_TRANS_USE_TXDATA;
        memcpy(t.tx_data, data, len);
    } else {
        // DMA模式下检查32位对齐
      
        if ((uint32_t)data % 4 != 0) {
            printf("rrrrrraaaaa2");
        }
        
        t.tx_buffer = data;
    }

    // 发送事务并检查返回值
    esp_err_t ret = spi_device_transmit(spi, &t);
    if (ret != ESP_OK) {
        printf("rrrrrraaaaa3");
    }

}

void ST7735_Reset(void) {
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(150));
}

void ST7735_Init(void) {
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    ret = spi_bus_initialize(LCD_HOST, &buscfg, DMA_CHAN);
    ESP_ERROR_CHECK(ret);

    ret = spi_bus_add_device(LCD_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);

    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    ST7735_Reset();

    // Initialization sequence for ST7735
    const uint8_t init_cmds[] = {
        0x01, 0, // Software reset
        0x11, 0, // Sleep out
        0xB1, 3, 0x01, 0x2C, 0x2D, // Frame rate control
        0xB2, 3, 0x01, 0x2C, 0x2D, // Frame rate control
        0xB3, 6, 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D, // Frame rate control
        0xB4, 1, 0x07, // Display inversion control
        0xC0, 3, 0xA2, 0x02, 0x84, // Power control 1
        0xC1, 1, 0xC5, // Power control 2
        0xC2, 2, 0x0A, 0x00, // Power control 3
        0xC3, 2, 0x8A, 0x2A, // Power control 4
        0xC4, 2, 0x8A, 0xEE, // Power control 5
        0xC5, 1, 0x0E, // VCOM control 1
        0x36, 1, 0xC8, // Memory data access control
        0x3A, 1, 0x05, // Set color mode to 16-bit
        0xE0, 16, 0x02, 0x1C, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2D, 0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10, // Positive gamma correction
        0xE1, 16, 0x03, 0x1D, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D, 0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10, // Negative gamma correction
        0x29, 0, // Display on
    };

    size_t cmd = 0;
    while (cmd < sizeof(init_cmds)) {
        lcd_cmd(spi, init_cmds[cmd++]);
        int len = init_cmds[cmd++];
        lcd_data(spi, &init_cmds[cmd], len);
        cmd += len;
    }

    // Enable backlight
    gpio_set_level(PIN_NUM_BCKL, 1);

    ST7735_SetRotation(1);
    ST7735_FillScreen(ST7735_BLACK);
}

void ST7735_Init_DMA() {
    spi_bus_config_t buscfg = {
        .miso_io_num = -1,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
        // .dma_chan = DMA_CHAN  // 显式分配 DMA 通道
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 8 * 1000 * 1000,  // 提升时钟速度
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 7,
        .flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY,
        // .dma_chan = DMA_CHAN
    };

    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, DMA_CHAN));
    ESP_ERROR_CHECK(spi_bus_add_device(LCD_HOST, &devcfg, &spi));

    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    gpio_set_level(PIN_NUM_DC, 1);


    ST7735_Reset();
    // Enable backlight
    gpio_set_level(PIN_NUM_BCKL, 1);
    vTaskDelay(pdMS_TO_TICKS(150));
    // Initialization sequence for ST7735
    // const uint8_t init_cmds[] = {
    //     // 0x01, 0, // Software reset
    //     0x11, 0, // Sleep out
    //     // 0xB1, 3, 0x05, 0x3C, 0x3C, // Frame rate control
    //     // 0xB2, 3, 0x05, 0x3C, 0x3C, // Frame rate control
    //     // 0xB3, 6, 0x05, 0x3C, 0x3C, 0x05, 0x3C, 0x3C, // Frame rate control
    //     // 0xB4, 1, 0x03, // Display inversion control
    //     // 0xC0, 3, 0x28, 0x08, 0x04, // Power control 1
    //     // 0xC1, 1, 0xC0, // Power control 2
    //     // 0xC2, 2, 0x0D, 0x00, // Power control 3
    //     // 0xC3, 2, 0x8D, 0x2A, // Power control 4
    //     // 0xC4, 2, 0x8D, 0xEE, // Power control 5
    //     // 0xC5, 1, 0x1A, // VCOM control 1
    //     // 0x36, 1, 0xA0, // Memory data access control
    //     // 0x3A, 1, 0x05, // Set color mode to 16-bit```
    //     // 0xE0, 16, 0x04, 0x22, 0x07, 0x0A,0x2E,0x30,0x25,0x2A,0x28,0x26,0x2E,0x3A,0x00,0x01,0x03,0x13,
    //     // 0xE1, 16, 0xE1,0x04,0x16,0x06,0x0D,0x2D,0x26,0x23,0x27,0x27,0x25,0x2D,0x3B,0x00,0x01,0x04,0x13,
    //     0x3A, 1, 0x05,
    //     0x29, 0, // Display on
    // };
    lcd_cmd(spi, 0x11);
    vTaskDelay(pdMS_TO_TICKS(150));
    uint8_t a = 0x3A;
    lcd_data(spi, &a, 1);
    lcd_cmd(spi, 0x29);
    // size_t cmd = 0;
    // while (cmd < sizeof(init_cmds)) {
    //     lcd_cmd(spi, init_cmds[cmd++]);
    //     if (init_cmds[cmd] == 0x11){
    //         vTaskDelay(pdMS_TO_TICKS(150));
    //     }

    //     int len = init_cmds[cmd++];
    //     lcd_data(spi, &init_cmds[cmd], len);
    //     cmd += len;
    // }

    

    // ST7735_SetRotation(1);

}

void ST7735_DrawBitmap_DMA(const uint16_t *bitmap, uint16_t width, uint16_t height) {
    ST7735_SetAddressWindow(0, 0, width - 1, height - 1);
    lcd_cmd(spi, ST7735_RAMWR);

    uint32_t total_pixels = width * height;
    uint32_t max_block_size = 2046; // 4092字节 / 2（每像素2字节）
    uint32_t sent_pixels = 0;

    while (sent_pixels < total_pixels) {
        uint32_t remaining = total_pixels - sent_pixels;
        uint32_t block_size = (remaining > max_block_size) ? max_block_size : remaining;
        
        lcd_data(spi, (uint8_t*)(bitmap + sent_pixels), block_size * sizeof(uint16_t));
        sent_pixels += block_size;
    }
}


void ST7735_SetRotation(uint8_t rotation) {
    lcd_cmd(spi, ST7735_MADCTL);
    switch (rotation) {
        case 0:
            lcd_data(spi, (uint8_t[]){ST7735_MADCTL_MX | ST7735_MADCTL_MY}, 1);
            break;
        case 1:
            lcd_data(spi, (uint8_t[]){ST7735_MADCTL_MY | ST7735_MADCTL_MV}, 1);
            break;
        case 2:
            lcd_data(spi, (uint8_t[]){0x00}, 1);
            break;
        case 3:
            lcd_data(spi, (uint8_t[]){ST7735_MADCTL_MX | ST7735_MADCTL_MV}, 1);
            break;
    }
}

void ST7735_BitMap(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint8_t *bitmap) {
    if ((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT) || ((x + width - 1) >= ST7735_WIDTH) || ((y + height - 1) >= ST7735_HEIGHT))
        return;

    ST7735_SetAddressWindow(x, y, x + width - 1, y + height - 1);
    lcd_cmd(spi, ST7735_RAMWR);

    for (uint16_t i = 0; i < height; i++) {
        for (uint16_t j = 0; j < width; j++) {
            uint8_t color = bitmap[i * width + j];
            uint8_t data[] = {color >> 8, color & 0xFF};
            lcd_data(spi, data, sizeof(data));
        }
    }
}

void ST7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {
    uint8_t data[] = {x0 + 2, x1 + 2};
    lcd_cmd(spi, ST7735_CASET);
    lcd_data(spi, data, sizeof(data));
    uint8_t data1[] = {y0 + 1, y1 + 1};
    lcd_cmd(spi, ST7735_RASET);
    lcd_data(spi, data1, sizeof(data1));

    lcd_cmd(spi, ST7735_RAMWR);
}

void ST7735_DrawChar(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bgColor, const FontDef *font) {
    if ((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT) || ((x + font->width - 1) >= ST7735_WIDTH) || ((y + font->height - 1) >= ST7735_HEIGHT))
        return;

    ST7735_SetAddressWindow(x, y, x + font->width - 1, y + font->height - 1);
    lcd_cmd(spi, ST7735_RAMWR);

    for (uint8_t i = 0; i < font->height; i++) {
        uint32_t line = font->data[(font == &Font_Custom ? (c - 46) : (c - 32)) * font->height + i];
        for (uint8_t j = 0; j < font->width; j++) {
            if ((line << j) & (font->width > 16 ? 0x80000000 : 0x8000)) {
                uint8_t data[] = { color >> 8, color & 0xFF };
                lcd_data(spi, data, sizeof(data));
            } else {
                uint8_t data[] = { bgColor >> 8, bgColor & 0xFF };
                lcd_data(spi, data, sizeof(data));
            }
        }
    }
}

void ST7735_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bgColor, const FontDef *font) {
    while (*str) {
        if (x + font->width > ST7735_WIDTH) {
            x = 0;
            y += font->height;
        }

        if (y + font->height > ST7735_HEIGHT) {
            break;
        }

        ST7735_DrawChar(x, y, *str, color, bgColor, font);
        x += font->width;
        str++;
    }
}

// void ST7735_FillScreen(uint16_t color) {
//     uint8_t data[2] = {color >> 8, color & 0xFF};
//     ST7735_SetAddressWindow(0, 0, ST7735_WIDTH - 1, ST7735_HEIGHT - 1);
//     for(int i=0;i<ST7735_HEIGHT;i++)
// 	{													   	 	
// 		for(int j=0;j<ST7735_WIDTH;j++)
// 		{
			
//             lcd_data(spi, data, sizeof(data));
// 		}
// 	} 		

//     // 使用批量传输
//     uint32_t size = ST7735_WIDTH * ST7735_HEIGHT;
//     uint8_t buffer[64]; // 每次传输64个像素
//     for (int i = 0; i < 64; i += 2) {
//         buffer[i] = data[0];
//         buffer[i + 1] = data[1];
//     }

//     while (size > 0) {
//         int chunk_size = size > 32 ? 32 : size; // 每次传输32个像素
//         lcd_data(spi, buffer, chunk_size * 2);
//         size -= chunk_size;
//     }
// }


void ST7735_DrawRectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color) {
    uint8_t data[2] = {color >> 8, color & 0xFF};
    ST7735_SetAddressWindow(x, y, x + width - 1, y + height - 1);
    for (uint16_t i = 0; i < width * height; i++) {
        lcd_data(spi, data, sizeof(data));
    }
}

void ST7735_DrawPixel(uint16_t x, uint16_t y, uint16_t color) {
    if ((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT))
        return;

    ST7735_SetAddressWindow(x, y, x, y);
    uint8_t data[2] = {color >> 8, color & 0xFF};
    lcd_data(spi, data, sizeof(data));
}


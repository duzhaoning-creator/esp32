/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"

#include "st7735.h"
#include "my_camera_ov2640.h"
#include "wifi_connect.h"

static const char *TAG = "HELLO_WORLD_MAIN";

void app_main(void)
{
    esp_err_t ret = wifi_init_sta();
    if (ret != ESP_OK) {
        printf("[%s][ERROR] WiFi初始化失败，程序退出\n", TAG);
        return;
    }

    // WiFi连接成功后的业务逻辑
    while (1) {
        // 周期性检查WiFi连接状态
        if (wifi_is_connected()) {
            printf("[%s][INFO] WiFi保持连接中...\n", TAG);
        } else {
            printf("[%s][ERROR] WiFi已断开！\n", TAG);
        }
        vTaskDelay(pdMS_TO_TICKS(5000)); // 5秒检查一次
    }
    
    printf("Hello world!\n");
    // camera_init();
    // gpio_set_direction(2, GPIO_MODE_OUTPUT);
    // while (1) {
    //     vTaskDelay(pdMS_TO_TICKS(2000));
    //     camera_capture();
    //     // 循环切换颜色，验证通信
    //     gpio_set_level(2, 1);
    //     vTaskDelay(pdMS_TO_TICKS(2000));
    //     gpio_set_level(2, 0);
    // }
    // tft_init();
    // /* Print chip information */
    // esp_chip_info_t chip_info;
    // uint32_t flash_size;
    // esp_chip_info(&chip_info);
    // printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
    //        CONFIG_IDF_TARGET,
    //        chip_info.cores,
    //        (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
    //        (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
    //        (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
    //        (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    // unsigned major_rev = chip_info.revision / 100;
    // unsigned minor_rev = chip_info.revision % 100;
    // printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    // if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
    //     printf("Get flash size failed");
    //     return;
    // }

    // printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
    //        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    // printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // for (int i = 10; i >= 0; i--) {
    //     printf("Restarting in %d seconds...\n", i);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }
    // printf("Restarting now.\n");
    // fflush(stdout);
    // esp_restart();
    // printf("TFT init start (128×160)...\n");
    // ST7735_Init();

    // ST7735_Init_DMA();
    // // 强制填充红色
    // ST7735_DrawRectangle(0,0,100,100,ST7735_BLUE);
    
    // while (1) {
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    //     // 循环切换颜色，验证通信
    //     ST7735_DrawRectangle(0,0,100,100,ST7735_RED);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    //     ST7735_DrawRectangle(0,0,100,100,ST7735_BLUE);
    // }

    // 初始化 TFT 屏幕
    //tft_init();
    
    // 清屏（白色背景）
    // tft_clear(COLOR_BLUE);
    
    // // // 显示文字（适配 160 高度，可显示更多行）
    // tft_show_string(10, 20, "Hello TFT!", COLOR_RED, COLOR_WHITE, 1);
    // tft_show_string(10, 40, "ESP32-S3", COLOR_BLUE, COLOR_WHITE, 1);
    // tft_show_string(10, 60, "128×160 LCD", COLOR_GREEN, COLOR_WHITE, 1);
    // tft_show_string(10, 80, "Test Line 4", COLOR_PURPLE, COLOR_WHITE, 1);
    
    // // 显示表情（笑脸，位置适配 160 高度）
    // tft_show_face(80, 20);
    
    // printf("TFT display init success!\n");
    
}

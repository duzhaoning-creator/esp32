#ifndef MY_CAMERA_OV2640_H
#define MY_CAMERA_OV2640_H
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"

#include "esp_camera.h" 


//WROVER-KIT PIN Map
#define CAM_PIN_PWDN    17 //power down is not used
#define CAM_PIN_RESET   -1 //software reset will be performed
#define CAM_PIN_XCLK    -1
#define CAM_PIN_SIOD    10
#define CAM_PIN_SIOC    47

#define CAM_PIN_D7      16
#define CAM_PIN_D6      8
#define CAM_PIN_D5      15
#define CAM_PIN_D4      3
#define CAM_PIN_D3      7
#define CAM_PIN_D2      46
#define CAM_PIN_D1       6
#define CAM_PIN_D0       9
#define CAM_PIN_VSYNC    4
#define CAM_PIN_HREF     5
#define CAM_PIN_PCLK    18

esp_err_t camera_capture();
esp_err_t camera_init();

#endif
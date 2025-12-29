
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/netif.h"
#include "esp_err.h"

#include "wifi_connect.h"

// 日志标签（保留便于区分模块）
static const char *TAG = "WIFI_CONNECT";

// 事件组，用于标记WiFi连接状态
static EventGroupHandle_t s_wifi_event_group = NULL;
// 重连次数计数器
static int s_retry_num = 0;

/**
 * @brief WiFi事件处理函数
 */
static void event_handler(void* arg, esp_event_base_t event_base,
                          int32_t event_id, void* event_data)
{
    // WiFi启动事件
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        printf("[%s][INFO] 正在连接WiFi: %s\n", TAG, WIFI_SSID);
    }
    // WiFi断开连接事件
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAX_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            printf("[%s][INFO] WiFi断开，正在重连(%d/%d)\n", TAG, s_retry_num, WIFI_MAX_RETRY);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            printf("[%s][ERROR] WiFi连接失败，达到最大重连次数\n", TAG);
        }
    }
    // IP获取成功事件（连接成功）
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        printf("[%s][INFO] WiFi连接成功，IP地址: %d.%d.%d.%d\n", 
               TAG,
               IP2STR(&event->ip_info.ip)); // 解析IP地址为十进制格式
        s_retry_num = 0;  // 重置重连计数器
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief 初始化WiFi STA模式并连接网络
 */
esp_err_t wifi_init_sta(void)
{
    // 1. 创建事件组
    if (s_wifi_event_group == NULL) {
        s_wifi_event_group = xEventGroupCreate();
        if (s_wifi_event_group == NULL) {
            printf("[%s][ERROR] 创建WiFi事件组失败\n", TAG);
            return ESP_ERR_NO_MEM;
        }
    }

    // 2. 初始化NVS（WiFi需要NVS存储配置）
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        printf("[%s][ERROR] NVS初始化失败: %s\n", TAG, esp_err_to_name(ret));
        return ret;
    }

    // 3. 初始化TCP/IP协议栈
    ret = esp_netif_init();
    if (ret != ESP_OK) {
        printf("[%s][ERROR] TCP/IP协议栈初始化失败: %s\n", TAG, esp_err_to_name(ret));
        return ret;
    }

    // 4. 创建默认事件循环
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK) {
        printf("[%s][ERROR] 事件循环创建失败: %s\n", TAG, esp_err_to_name(ret));
        return ret;
    }

    // 5. 创建STA模式的网络接口
    esp_netif_create_default_wifi_sta();

    // 6. 配置WiFi参数
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        printf("[%s][ERROR] WiFi初始化失败: %s\n", TAG, esp_err_to_name(ret));
        return ret;
    }

    // 7. 注册WiFi事件和IP事件处理函数
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ret = esp_event_handler_instance_register(WIFI_EVENT,
                                              ESP_EVENT_ANY_ID,
                                              &event_handler,
                                              NULL,
                                              &instance_any_id);
    if (ret != ESP_OK) {
        printf("[%s][ERROR] 注册WiFi事件处理函数失败: %s\n", TAG, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_event_handler_instance_register(IP_EVENT,
                                              IP_EVENT_STA_GOT_IP,
                                              &event_handler,
                                              NULL,
                                              &instance_got_ip);
    if (ret != ESP_OK) {
        printf("[%s][ERROR] 注册IP事件处理函数失败: %s\n", TAG, esp_err_to_name(ret));
        return ret;
    }

    // 8. 配置STA模式参数
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,  // 适配WPA2加密
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    // 如果WiFi密码为空，配置为开放模式
    if (strlen(WIFI_PASS) == 0) {
        wifi_config.sta.threshold.authmode = WIFI_AUTH_OPEN;
    }

    // 9. 设置WiFi工作模式为STA
    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        printf("[%s][ERROR] 设置WiFi模式失败: %s\n", TAG, esp_err_to_name(ret));
        return ret;
    }

    // 10. 配置WiFi参数
    ret = esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    if (ret != ESP_OK) {
        printf("[%s][ERROR] 配置WiFi参数失败: %s\n", TAG, esp_err_to_name(ret));
        return ret;
    }

    // 11. 启动WiFi
    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        printf("[%s][ERROR] 启动WiFi失败: %s\n", TAG, esp_err_to_name(ret));
        return ret;
    }

    printf("[%s][INFO] WiFi初始化完成，等待连接...\n", TAG);

    // 等待WiFi连接结果（成功/失败）
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    // 处理连接结果
    if (bits & WIFI_CONNECTED_BIT) {
        printf("[%s][INFO] WiFi连接成功！\n", TAG);
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        printf("[%s][ERROR] WiFi连接失败！\n", TAG);
        return ESP_FAIL;
    } else {
        printf("[%s][ERROR] WiFi连接出现未知错误！\n", TAG);
        return ESP_ERR_INVALID_STATE;
    }
}

/**
 * @brief 检查WiFi当前连接状态
 */
bool wifi_is_connected(void)
{
    if (s_wifi_event_group == NULL) {
        return false;
    }

    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
    return (bits & WIFI_CONNECTED_BIT) != 0;
}
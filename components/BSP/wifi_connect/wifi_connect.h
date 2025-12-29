#ifndef WIFI_CONNECT_H
#define WIFI_CONNECT_H

#include "esp_err.h"
#include <stdbool.h> 

/**
 * @brief WiFi配置参数（需根据实际环境修改）
 */
#define WIFI_SSID               "duzhaoning"  // WiFi名称
#define WIFI_PASS               "123456987"  // WiFi密码
#define WIFI_MAX_RETRY          5               // 最大重连次数
#define WIFI_CONNECTED_BIT      BIT0            // WiFi连接成功标志位
#define WIFI_FAIL_BIT           BIT1            // WiFi连接失败标志位

/**
 * @brief 初始化WiFi STA模式并连接网络
 * 
 * @return esp_err_t 初始化结果
 *         - ESP_OK: 初始化成功
 *         - 其他: 初始化失败
 */
esp_err_t wifi_init_sta(void);

/**
 * @brief 检查WiFi当前连接状态
 * 
 * @return true 已连接
 * @return false 未连接
 */
bool wifi_is_connected(void);

#endif // WIFI_CONNECT_H
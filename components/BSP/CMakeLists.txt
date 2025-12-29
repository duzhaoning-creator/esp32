# 组件编译配置（ESP-IDF 5.x 标准写法）


set(src_dirs 
            tft_st7735 
            font
            wifi_connect
            my_camera_ov2640
            
            )
set(include_dirs 
                tft_st7735 
                font
                wifi_connect
                my_camera_ov2640
                )

set(requires 
            driver 
            spi_flash
            esp_system
            esp_lcd
            espressif__esp32-camera
            esp_wifi
            esp_netif
            esp_event 
            nvs_flash
            
) 

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS  
${include_dirs} REQUIRES ${requires}) 

component_compile_options(-ffast-math 
        -O3 
        -Wno-error=format
        -Wno-format) 
#  -ffast-math: 允许编译器进行某些可能减少数学运算精度的优化，以提高性能。 
#  -O3: 这是一个优化级别选项，指示编译器尽可能地进行高级优化以生成更高效的代码。 
#  -Wno-error=format: 这将编译器关于格式字符串不匹配的警告从错误降级为警告。 
#  -Wno-format: 这将完全禁用关于格式字符串的警告。 

/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "servo.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#define BUF_SIZE (1024)

#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

static const char *TAG = "360_servo";

// Please consult the datasheet of your servo before changing the following parameters
#define SERVO_NEUTRAL_US 1500 // Minimum pulse width in microsecond
#define SERVO_MIN_US 1000     // Maximum pulse width in microsecond
#define SERVO_MAX_US 2000

#define SERVO_PULSE_GPIO 15                  // GPIO connects to the PWM signal line
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD 20000          // 20000 ticks, 20ms

static mcpwm_cmpr_handle_t comparator = NULL;

void mg50s_set_speed(int8_t speed_percent)
{
    // 速度百分比: -100% (全速反转) 到 +100% (全速正转)
    if (speed_percent < -100)
        speed_percent = -100;
    if (speed_percent > 100)
        speed_percent = 100;

    uint32_t pulse_width;
    if (speed_percent == 0)
    {
        // 停止状态
        pulse_width = SERVO_NEUTRAL_US; // 1500us
    }
    else if (speed_percent > 0)
    {
        // 正转：1500us ~ 2000us
        pulse_width = SERVO_NEUTRAL_US +
                      (speed_percent * (SERVO_MAX_US - SERVO_NEUTRAL_US) / 100);
    }
    else
    {
        // 反转：1000us ~ 1500us
        pulse_width = SERVO_NEUTRAL_US +
                      (speed_percent * (SERVO_NEUTRAL_US - SERVO_MIN_US) / 100);
    }
    // 设置比较器值（脉冲宽度）
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, pulse_width));

    ESP_LOGI(TAG, "Set speed: %d%%, Pulse width: %ldus", speed_percent, pulse_width);
}

void mg50s_stop(void)
{
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, SERVO_NEUTRAL_US));
    ESP_LOGI(TAG, "Servo stopped");
}

void uart_control_task(void *pvParameters)
{
    // UART配置
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0));

    char input_buffer[32];
    int buffer_index = 0;

    printf("MG50S Continuous Servo Control Ready!\n");
    printf("Enter speed (-100 to 100):\n");
    while (1)
    {
        // 读取串口数据
        uint8_t byte;
        int len = uart_read_bytes(UART_NUM_0, &byte, 1, 100 / portTICK_PERIOD_MS);
        if (len > 0)
        {
            if (byte == '\r' || byte == '\n')
            {
                // 回车或换行，处理输入
                if (buffer_index > 0)
                {
                    input_buffer[buffer_index] = '\0'; // 终止字符串
                    printf("Received command: %s\n", input_buffer);
                    int speed = atoi((char *)input_buffer);
                    if (speed < -100 || speed > 100)
                    {
                        printf("Invalid speed! Please enter a value between -100 and 100.\n");
                    }
                    else
                    {
                        mg50s_set_speed(speed);
                        printf("Set speed to %d%%\n", speed);
                    }
                    buffer_index = 0; // 重置索引
                    printf("Enter speed (-100 to 100):\n");
                }
            }
            else if (byte == '\b' || byte == 0x7F)
            {
                if (buffer_index > 0)
                {
                    buffer_index--;
                    printf("\b \b"); // 回显退格
                }
            }
            else if (buffer_index < sizeof(input_buffer) - 1)
            {
                // 存储输入字符
                input_buffer[buffer_index++] = byte;
                printf("%c", byte); // 回显字符
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

esp_err_t servo_mcpwm_init(void)
{
    esp_err_t ret = ESP_OK;
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {

        .group_id = 0,
        .intr_priority = 0, // default interrupt priority
                            // timer must be in the same group to the operator
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .flags = {
            .update_period_on_empty = true, // allow update period on timer count empty event
            .update_period_on_sync = false,
        }};
    ret = mcpwm_new_timer(&timer_config, &timer);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create timer: 0x%x", ret);
        return ret;
    }

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = {
        .group_id = 0,      // operator must be in the same group to the timer
        .intr_priority = 0, // default interrupt priority
    };
    ret = mcpwm_new_operator(&operator_config, &oper);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create operator: 0x%x", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Connect timer and operator");
    ret = mcpwm_operator_connect_timer(oper, timer);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to connect timer and operator: 0x%x", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    // mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = {
        .flags = {
            .update_cmp_on_tez = true, // allow update compare on timer count equal to zero event
            .update_cmp_on_tep = false,
            .update_cmp_on_sync = false,
        },
        .intr_priority = 0, // default interrupt priority

        // comparator must be in the same group to the operator
    };
    ret = mcpwm_new_comparator(oper, &comparator_config, &comparator);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create comparator: 0x%x", ret);
        return ret;
    }

    mcpwm_gen_handle_t generator = NULL;
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_PULSE_GPIO,
    };
    ret = mcpwm_new_generator(oper, &generator_config, &generator);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create generator: 0x%x", ret);
        return ret;
    }
    // set the initial compare value, so that the servo will spin to the center position
    // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ret = mcpwm_generator_set_action_on_timer_event(generator,
                                                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                                                                 MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set timer event action: 0x%x", ret);
        return ret;
    } // go low on compare threshold
    ret = mcpwm_generator_set_action_on_compare_event(generator,
                                                      MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW));
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set compare event action: 0x%x", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Enable and start timer");
    ret = mcpwm_timer_enable(timer);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to enable timer: 0x%x", ret);
        return ret;
    }

    ret = mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start timer: 0x%x", ret);
        return ret;
    }

    ESP_LOGI(TAG, "Servo MCPWM initialized successfully");
    return ESP_OK;
}

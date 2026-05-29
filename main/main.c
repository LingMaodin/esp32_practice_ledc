#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/gpio_filter.h"
#include "driver/pulse_cnt.h"
#include "driver/ledc.h"

#define TAG "main"
#define PCNT_S1_NUM GPIO_NUM_1
#define PCNT_S2_NUM GPIO_NUM_2
#define TIMER_PERIOD_US 10*1000 //定时器周期
#define PWM_HZ 20000
#define BASE_PWM_DUTY (7.4/12.0*(float)((1<<10)-1))
#define PWM_NUM GPIO_NUM_9
#define AIN1_NUM GPIO_NUM_10
#define AIN2_NUM GPIO_NUM_11
#define ENCODER_PPR 13.0//每转脉冲数
#define TARGET_RPM 300.0//目标转速
#define GEAR_RATIO 20.049//减速比
#define TARGET_PULSES_PER_PERIOD (TARGET_RPM*GEAR_RATIO*ENCODER_PPR*(TIMER_PERIOD_US/1e6)/60.0)//目标每周期脉冲数
#define PID_KP 2.0
#define PID_KI 0.02

static pcnt_unit_handle_t pcnt_unit_hdl=NULL;//霍尔编码器单元句柄
static pcnt_channel_handle_t pcnt_channel_s1_hdl=NULL;//霍尔编码器通道1句柄
static pcnt_channel_handle_t pcnt_channel_s2_hdl=NULL;//霍尔编码器通道2句柄
static gpio_glitch_filter_handle_t pcnt_button_filter_hdl=NULL;//霍尔编码器毛刺过滤器句柄
static esp_timer_handle_t esptimer_hdl=NULL;//esp定时器句柄
static QueueHandle_t pulse_count_queue=NULL;//计数值队列句柄

void gpio_init()//GPIO初始化
{
    gpio_config_t gpio_conf={
        .mode=GPIO_MODE_OUTPUT,//输出模式
        .pin_bit_mask=(1ULL<<AIN1_NUM)|(1ULL<<AIN2_NUM),//配置AIN1_NUM、AIN2_NUM为输出
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));//配置GPIO并判断是否成功
    return;
}

void pcnt_init()//pcnt初始化
{
    //s1通道：上升沿+1，下降沿-1，高电平翻转，低电平保持
    //s2通道：上升沿+1，下降沿-1，高电平保持，低电平翻转
    //每次旋转s1、s2各接受一个上升沿一个下降沿，计数单元共计+4或-4
    pcnt_unit_config_t pcnt_unit_cfg={//配置pcnt单元
        .low_limit=-100,//最小计数值
        .high_limit=100,//最大计数值
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&pcnt_unit_cfg, &pcnt_unit_hdl));//新建pcnt单元并判断是否成功

    pcnt_chan_config_t pcnt_channel_s1_cfg={//配置pcnt通道1
        .edge_gpio_num=PCNT_S1_NUM,//输入s1边沿
        .level_gpio_num=PCNT_S2_NUM,//输入s2电平
    };
    pcnt_chan_config_t pcnt_channel_s2_cfg={//配置pcnt通道2
        .edge_gpio_num=PCNT_S2_NUM,//输入s2边沿
        .level_gpio_num=PCNT_S1_NUM,//输入s1电平
    };
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_hdl,&pcnt_channel_s1_cfg,&pcnt_channel_s1_hdl));//新建pcnt通道1并判断是否成功
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_hdl,&pcnt_channel_s2_cfg,&pcnt_channel_s2_hdl));//新建pcnt通道2并判断是否成功

    pcnt_glitch_filter_config_t filte_cfg={//配置毛刺过滤器
        .max_glitch_ns=1000,//过滤1000ns以下毛刺
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit_hdl, &filte_cfg));//新建毛刺过滤器并判断是否成功
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_channel_s1_hdl,
                                                PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                PCNT_CHANNEL_EDGE_ACTION_DECREASE));//上升沿+1，下降沿-1                                           
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_channel_s1_hdl,
                                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
                                                PCNT_CHANNEL_LEVEL_ACTION_KEEP));//高电平翻转，低电平保持                                            
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_channel_s2_hdl,
                                                PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                PCNT_CHANNEL_EDGE_ACTION_DECREASE));//上升沿+1，下降沿-1                                          
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_channel_s2_hdl,
                                                PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                                                PCNT_CHANNEL_LEVEL_ACTION_INVERSE));//高电平保持，低电平翻转

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_hdl));//使能pcnt单元
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_hdl));//清除计数值
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_hdl));//启动pcnt单元
    return;
}

void esptimer_cb(void *arg)
{
    int16_t pulse_count=0;
    pcnt_unit_get_count(pcnt_unit_hdl,&pulse_count);
    pcnt_unit_clear_count(pcnt_unit_hdl);
    float speed_error=(float)TARGET_PULSES_PER_PERIOD-(float)pulse_count;
    xQueueSend(pulse_count_queue,&speed_error,pdMS_TO_TICKS(10));
}

void esptimer_init()
{
    esp_timer_create_args_t esptimer_cfg={
        .callback=esptimer_cb,
        .name="pulse_count_reader",
        .arg=NULL,
        .dispatch_method=ESP_TIMER_TASK,
    };
    ESP_ERROR_CHECK(esp_timer_create(&esptimer_cfg,&esptimer_hdl));
    ESP_ERROR_CHECK(esp_timer_start_periodic(esptimer_hdl,TIMER_PERIOD_US));
}

void ledc_init()
{
    ledc_timer_config_t ledc_timer_cfg={
        .speed_mode=LEDC_LOW_SPEED_MODE,
        .timer_num=LEDC_TIMER_0,
        .freq_hz=PWM_HZ,
        .duty_resolution=LEDC_TIMER_10_BIT,
        .clk_cfg=LEDC_APB_CLK,
    };
    ledc_timer_config(&ledc_timer_cfg);
    ledc_channel_config_t ledc_channel_cfg={
        .gpio_num=PWM_NUM,
        .speed_mode=LEDC_LOW_SPEED_MODE,
        .channel=LEDC_CHANNEL_0,
        .intr_type=LEDC_INTR_DISABLE,
        .timer_sel=LEDC_TIMER_0,
        .duty=0,
        .hpoint=0,
    };
    ledc_channel_config(&ledc_channel_cfg);
}

void motor_control_task(void *arg)
{
    float speed_error=0,speed_error_integral=0;
    while(1) {
        if(xQueueReceive(pulse_count_queue,&speed_error,portMAX_DELAY))
        {
            speed_error_integral+=speed_error;
            double duty=BASE_PWM_DUTY;
            duty+=PID_KP*speed_error+PID_KI*speed_error_integral;
            if (duty>1023) duty=1023;
            if (duty<0) duty=0;
            ledc_set_duty_and_update(LEDC_LOW_SPEED_MODE,LEDC_CHANNEL_0,(uint32_t)duty,0);
        }
    }
}

void app_main(void)
{
    gpio_init();//GPIO初始化
    pcnt_init();//pcnt初始化
    esptimer_init();//esptimer初始化
    ledc_init();//ledc初始化
    pulse_count_queue=xQueueCreate(5,sizeof(float));
    xTaskCreate(motor_control_task,"motor_control_task",4096,NULL,5,NULL);
}

/**************************************************************
 * 文件名:  battery_detect.c
 * 创建时间: 2026年3月7日 下午4:02:39
 * 说明: 电池电压检测 (ADC分压)
 **************************************************************/

#define VOLTAGE_PORT    ADC0_CH11_A11 // 电池检测通道

#include "LED.h"
#include "battery_detect.h"

float voltage_now = 0;

void voltage_detect_init(){
    adc_init(VOLTAGE_PORT, ADC_12BIT);
    pit_ms_init(CCU61_CH0,500);
}

void detect_main(){
    int voltage_adc = 0;


    voltage_adc = adc_convert(VOLTAGE_PORT);
    voltage_now = (36.3f * (float)voltage_adc / 4096.0f) - 0.3f;

}

/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       voltage_task.c/h
  * @brief      24v power voltage ADC task, get voltage and calculate electricity
  *             percentage.24电源电压ADC任务,获取电压并且计算电量百分比.
  * @note       when power is not derectly link to delelopment, please change VOLTAGE_DROP
  *             当电源不直连开发板,请修改VOLTAGE_DROP
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Nov-11-2019     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef VOLTAGE_TASK_H
#define VOLTAGE_TASK_H
#include "struct_typedef.h"



/**
  * @brief          power ADC and calculate electricity percentage
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          电源采样和计算电源百分比
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void battery_voltage_task(void const * argument);

/**
  * @brief          get electricity percentage
  * @param[in]      void
  * @retval         electricity percentage, unit 1, 1 = 1%
  */
/**
  * @brief          获取电量
  * @param[in]      void
  * @retval         电量, 单位 1, 1 = 1%
  */
extern uint16_t get_battery_percentage(void);
#endif

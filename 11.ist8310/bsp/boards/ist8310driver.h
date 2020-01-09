
/**
  * @file       IST8310driver.c/h
  * @brief      ist8310 is a 3-axis digital magnetometer, the file includes initialization function,
  *             read magnetic field strength data function.
  * @note       IST8310 only support I2C
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  */

#ifndef IST8310DRIVER_H
#define IST8310DRIVER_H
#include "struct_typedef.h"

#define IST8310_DATA_READY_BIT 2

#define IST8310_NO_ERROR 0x00
#define IST8310_NO_SENSOR 0x40

typedef struct ist8310_real_data_t
{
  uint8_t status;
  fp32 mag[3];
} ist8310_real_data_t;

/**
  * @brief          initialize ist8310
  * @param[in]      none
  * @retval         error value
  */
/**
  * @brief          初始化IST8310
  * @param[in]      none
  * @retval         error value
  */ 
extern uint8_t ist8310_init(void);

/**
  * @brief          if you have read the data from STAT1 to DATAZL usaully by I2C DMA , you can use the function to solve. 
  * @param[in]      status_buf:the data point from the STAT1(0x02) register of IST8310 to the DATAZL(0x08) register 
  * @param[out]     ist8310_real_data:ist8310 data struct 
  * @retval         none
  */
/**
  * @brief          如果已经通过I2C的DMA方式读取到了从STAT1到DATAZL的数据，可以使用这个函数进行处理
  * @param[in]      status_buf:数据指针,从STAT1(0x02) 寄存器到 DATAZL(0x08)寄存器 
  * @param[out]     ist8310_real_data:ist8310的数据结构
  * @retval         none
  */
extern void ist8310_read_over(uint8_t *status_buf, ist8310_real_data_t *ist8310_real_data);

/**
  * @brief          read mag magnetic field strength data of IST8310 by I2C
  * @param[out]     mag variable
  * @retval         none
  */
/**
  * @brief          通过读取磁场数据
  * @param[out]     磁场数组
  * @retval         none
  */
extern void ist8310_read_mag(fp32 mag[3]);
#endif

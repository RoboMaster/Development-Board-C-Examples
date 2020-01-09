
#include "red_led_task.h"

#include "cmsis_os.h"
#include "main.h"


/**
  * @brief          red led task
  * @param[in]      argument: NULL
  * @retval         none
  */
/**
  * @brief          ºìµÆÈÎÎñ
  * @param[in]      argument: NULL
  * @retval         none
  */
void red_led_task(void const * argument)
{

    while(1)
    {
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
        osDelay(500);
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
        osDelay(500);
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
        osDelay(500);

    }
}



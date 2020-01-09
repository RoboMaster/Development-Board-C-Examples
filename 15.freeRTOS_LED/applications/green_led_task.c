
#include "green_led_task.h"

#include "cmsis_os.h"
#include "main.h"


/**
  * @brief          green led task
  * @param[in]      argument: NULL
  * @retval         none
  */
/**
  * @brief          ÂÌµÆÈÎÎñ
  * @param[in]      argument: NULL
  * @retval         none
  */
void green_led_task(void const * argument)
{

    while(1)
    {

        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        osDelay(500);
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
        osDelay(500);
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
        osDelay(500);


    }
}



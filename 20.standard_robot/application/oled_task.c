/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       oled_task.c/h
  * @brief      OLED show error value.oledÆÁÄ»ÏÔÊ¾´íÎóÂë
  * @note       
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
#include "oled_task.h"
#include "main.h"
#include "oled.h"

#include "cmsis_os.h"
#include "detect_task.h"
#include "voltage_task.h"

#define OLED_CONTROL_TIME 10
#define REFRESH_RATE    10

const error_t *error_list_local;

uint8_t other_toe_name[4][4] = {"GYR\0","ACC\0","MAG\0","REF\0"};

uint8_t last_oled_error = 0;
uint8_t now_oled_errror = 0;
static uint8_t refresh_tick = 0;


/**
  * @brief          oled task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          oledÈÎÎñ
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void oled_task(void const * argument)
{
    uint8_t i;
    uint8_t show_col, show_row;
    error_list_local = get_error_list_point();
    osDelay(1000);
    OLED_init();
    OLED_LOGO();
    i = 100;
    while(i--)
    {
        if(OLED_check_ack())
        {
            detect_hook(OLED_TOE);
        }
        osDelay(10);
    }
    while(1)
    {
        //use i2c ack to check the oled
        if(OLED_check_ack())
        {
            detect_hook(OLED_TOE);
        }

        now_oled_errror = toe_is_error(OLED_TOE);
        //oled init
        if(last_oled_error == 1 && now_oled_errror == 0)
        {
            OLED_init();
        }

        if(now_oled_errror == 0)
        {
            refresh_tick++;
            //10Hz refresh
            if(refresh_tick > configTICK_RATE_HZ / (OLED_CONTROL_TIME * REFRESH_RATE))
            {
                refresh_tick = 0;
                OLED_operate_gram(PEN_CLEAR);
                OLED_show_graphic(0, 1, &battery_box);

                if(get_battery_percentage() < 10)
                {
                    OLED_printf(9, 2, "%d", get_battery_percentage());
                }
                else if(get_battery_percentage() < 100)
                {
                    OLED_printf(6, 2, "%d", get_battery_percentage());
                }
                else
                {
                    OLED_printf(3, 2, "%d", get_battery_percentage());
                }

                OLED_show_string(90, 27, "DBUS");
                OLED_show_graphic(115, 27, &check_box[error_list_local[DBUS_TOE].error_exist]);
                for(i = CHASSIS_MOTOR1_TOE; i < TRIGGER_MOTOR_TOE + 1; i++)
                {
                    show_col = ((i-1) * 32) % 128;
                    show_row = 15 + (i-1) / 4 * 12;
                    OLED_show_char(show_col, show_row, 'M');
                    OLED_show_char(show_col + 6, show_row, '0'+i);
                    OLED_show_graphic(show_col + 12, show_row, &check_box[error_list_local[i].error_exist]);
                }

                for(i = BOARD_GYRO_TOE; i < REFEREE_TOE + 1; i++)
                {
                    show_col = (i * 32) % 128;
                    show_row = 15 + i / 4 * 12;
                    OLED_show_string(show_col, show_row, other_toe_name[i - BOARD_GYRO_TOE]);
                    OLED_show_graphic(show_col + 18, show_row, &check_box[error_list_local[i].error_exist]);

                }

                OLED_refresh_gram();
            }
        }



        last_oled_error = now_oled_errror;
        osDelay(OLED_CONTROL_TIME);
    }
}


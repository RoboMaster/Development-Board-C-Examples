/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       oled.h
 * @brief      the head file of oled.c ,define the I2C address of oled ,declare function of oled
 * @note         
 * @Version    V1.0.0
 * @Date       Oct-7-2019      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */


#ifndef OLED_H
#define OLED_H
#include "struct_typedef.h"
#include "oledfont.h"


// the I2C address of oled
#define OLED_I2C_ADDRESS    0x78        //or be 0x7A 
#define OLED_I2C    I2C2


//the resolution of oled   128*64
#define MAX_COLUMN      128
#define MAX_ROW         64

#define X_WIDTH         MAX_COLUMN
#define Y_WIDTH         MAX_ROW

#define OLED_CMD        0x00
#define OLED_DATA       0x01

#define CHAR_SIZE_WIDTH 6
#define CHAR_SIZE_HIGHT 12


typedef enum
{
    PEN_CLEAR = 0x00,
    PEN_WRITE = 0x01,
    PEN_INVERSION= 0x02,
}pen_typedef;


typedef  __packed struct  
{
    uint8_t cmd_data;
    uint8_t OLED_GRAM[8][128];
}OLED_GRAM_strutct_t;



extern void OLED_com_reset(void);


/**
 * @brief   initialize the oled device
 * @param   none
 * @retval  none
 */
extern void OLED_init(void);



extern bool_t OLED_check_ack(void);





/**
 * @brief   turn on OLED display
 * @param   None
 * @param   None
 * @retval  
 */
extern void OLED_display_on(void);


/**
 * @brief   turn off OLED display
 * @param   None
 * @param   None
 * @retval  
 */
extern void OLED_display_off(void);


/**
 * @brief   operate the graphic ram(size: 128*8 char)
 * @param   pen: the type of operate.
            PEN_CLEAR: set ram to 0x00
            PEN_WRITE: set ram to 0xff
            PEN_INVERSION: bit inversion 
 * @retval  none
 */
extern void OLED_operate_gram(pen_typedef pen);


/**
 * @brief   cursor set to (x,y) point
 * @param   x:X-axis, from 0 to X_WIDTH-1
 * @param   y:Y-axis, from 0 to Y_WIDTH-1
 * @retval  none
 */
extern void OLED_set_pos(uint8_t x, uint8_t y);


/**
 * @brief   draw one bit of graphic raw, operate one point of screan(128*64)
 * @param   x: x-axis, [0, X_WIDTH-1]
 * @param   y: y-axis, [0, Y_WIDTH-1]
 * @param   pen: type of operation,
            PEN_CLEAR: set (x,y) to 0
            PEN_WRITE: set (x,y) to 1
            PEN_INVERSION: (x,y) value inversion 
 * @retval  none
 */
extern void OLED_draw_point(uint8_t x, uint8_t y, pen_typedef pen);

/**
 * @brief   draw a line from (x1, y1) to (x2, y2)
 * @param   x1, y1: the start point of line
 * @param   x2, y2: the end of line
 * @param   pen: Pen_Clear, Pen_Write, Pen_Inversion @Pen_Typedef
 * @retval  None
 */
extern void OLED_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, pen_typedef pen);


/**
 * @brief   show a character
 * @param   row: row of character
 * @param   col: column of character
 * @param   chr: the character ready to show
 * @retval  None
 */
extern void OLED_show_char(uint8_t row, uint8_t col, uint8_t chr);

/**
 * @brief   show a character string
 * @param   row: row of character string begin
 * @param   col: column of character string begin
 * @param   chr: the pointer to character string
 * @retval  None
 */
extern void OLED_show_string(uint8_t row, uint8_t col, uint8_t *chr);

/**
 * @brief   formatted output in oled 128*64
 * @param   row: row of character string begin, 0 <= row <= 4;
 * @param   col: column of character string begin, 0 <= col <= 20;
 * @param   *fmt: the pointer to format character string
 * @retval  None
 * @note    if the character length is more than one row at a time, the extra characters will be truncated
 */
extern void OLED_printf(uint8_t row, uint8_t col, const char *fmt,...);

/**
 * @brief   send the data of gram to oled sreen
 * @param   none
 * @retval  none
 */
extern void OLED_refresh_gram(void);




extern void OLED_show_graphic(uint8_t x, uint8_t y, const picture_t *graphic);



/**
 * @brief   show the logo of robomaster
 * @param   none
 * @retval  none
 */
extern void OLED_LOGO(void);
#endif

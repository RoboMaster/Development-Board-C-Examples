#include "OLED.h" 
#include "oledfont.h"
#include "bsp_i2c.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"

//#define OLED_ONE_COLOR
#define OLED_TWO_COLOR


OLED_GRAM_strutct_t oled_gram;



void OLED_com_reset(void)
{
    static uint16_t time = 0;
    time++;
    if(time > 100)
    {
        bsp_I2C_reset(OLED_I2C);
        time = 0;
    }
}



/**
 * @brief   write data/command to OLED, if you use spi, please rewrite the function
 * @param   dat: the data ready to write
 * @param   cmd: OLED_CMD means command; OLED_DATA means data
 * @retval  none
 */
 void oled_write_byte(uint8_t dat, uint8_t cmd)
{
    static uint8_t cmd_data[2];
    if(cmd == OLED_CMD)
    {
        cmd_data[0] = 0x00;
    }
    else
    {
        cmd_data[0] = 0x40;
    }
    cmd_data[1] = dat;
    bsp_I2C_master_transmit(OLED_I2C, OLED_I2C_ADDRESS, cmd_data, 2);
}

/**
 * @brief   initialize the oled device
 * @param   none
 * @retval  none
 */
void OLED_init(void)
{
    I2C2_tx_DMA_init();

#if defined(OLED_ONE_COLOR)
    oled_write_byte(0xAE, OLED_CMD);    //display off
    oled_write_byte(0x40, OLED_CMD);    //--set start line address
    oled_write_byte(0x81, OLED_CMD);    //--set contrast control register
    oled_write_byte(0xFF, OLED_CMD);    //brightness 0x00~0xff
    oled_write_byte(0xa4, OLED_CMD);    //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    oled_write_byte(0xa6, OLED_CMD);    //--set normal display
    oled_write_byte(0x20, OLED_CMD);    //Set Memory Addressing Mode	
    oled_write_byte(0x00, OLED_CMD);    //00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
    oled_write_byte(0x21, OLED_CMD);
    oled_write_byte(0x00, OLED_CMD);
    oled_write_byte(0x7F, OLED_CMD);
    oled_write_byte(0x22, OLED_CMD);
    oled_write_byte(0x00, OLED_CMD);
    oled_write_byte(0x07, OLED_CMD);
    oled_write_byte(0x3F, OLED_CMD);    //
    oled_write_byte(0xc8, OLED_CMD);    //Set COM Output Scan Direction
    oled_write_byte(0xa1, OLED_CMD);    //--set segment re-map 0 to 127
    oled_write_byte(0xa8, OLED_CMD);    //--set multiplex ratio(1 to 64)
    oled_write_byte(0x00, OLED_CMD);    //
    oled_write_byte(0xd3, OLED_CMD);    //-set display offset
    oled_write_byte(0x00, OLED_CMD);    //-not offset
    oled_write_byte(0xd5, OLED_CMD);    //--set display clock divide ratio/oscillator frequency
    oled_write_byte(0xF0, OLED_CMD);    //--set divide ratio
    oled_write_byte(0xd9, OLED_CMD);    //--set pre-charge period
    oled_write_byte(0x22, OLED_CMD);    //
    oled_write_byte(0xda, OLED_CMD);    //--set com pins hardware configuration
    oled_write_byte(0x12, OLED_CMD);
    oled_write_byte(0xdb, OLED_CMD);    //--set vcomh
    oled_write_byte(0x20, OLED_CMD);    //0x20,0.77xVcc
    oled_write_byte(0x8d, OLED_CMD);    //--set DC-DC enable
    oled_write_byte(0x14, OLED_CMD);    //
    oled_write_byte(0xaf, OLED_CMD);    //--turn on oled panel
#elif defined(OLED_TWO_COLOR)

    oled_write_byte(0xAE, OLED_CMD);
    oled_write_byte(0x20, OLED_CMD);
    oled_write_byte(0x00, OLED_CMD);
    oled_write_byte(0x21, OLED_CMD);
    oled_write_byte(0x00, OLED_CMD);
    oled_write_byte(0x7F, OLED_CMD);
    oled_write_byte(0x22, OLED_CMD);
    oled_write_byte(0x00, OLED_CMD);
    oled_write_byte(0x07, OLED_CMD);
    oled_write_byte(0x3F, OLED_CMD);
    oled_write_byte(0x81, OLED_CMD);
    oled_write_byte(0xFF, OLED_CMD);
    oled_write_byte(0xA1, OLED_CMD);
    oled_write_byte(0xA6, OLED_CMD);
    oled_write_byte(0xA8, OLED_CMD);
    oled_write_byte(0x3F, OLED_CMD);
    oled_write_byte(0xC8, OLED_CMD);
    oled_write_byte(0xD3, OLED_CMD);
    oled_write_byte(0x00, OLED_CMD);
    oled_write_byte(0xD5, OLED_CMD);
    oled_write_byte(0x80, OLED_CMD);
    oled_write_byte(0xD9, OLED_CMD);
    oled_write_byte(0x1F, OLED_CMD);
    oled_write_byte(0xDA, OLED_CMD);
    oled_write_byte(0x12, OLED_CMD);
    oled_write_byte(0xDB, OLED_CMD);
    oled_write_byte(0x30, OLED_CMD);
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x14, OLED_CMD);
    oled_write_byte(0xAF, OLED_CMD);

#endif
}

bool_t OLED_check_ack(void)
{
    return bsp_I2C_check_ack(OLED_I2C, OLED_I2C_ADDRESS);
}


/**
 * @brief   turn on OLED display
 * @param   None
 * @param   None
 * @retval  
 */
void OLED_display_on(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x14, OLED_CMD);
    oled_write_byte(0xaf, OLED_CMD);
}

/**
 * @brief   turn off OLED display
 * @param   None
 * @param   None
 * @retval  
 */
void OLED_display_off(void)
{
    oled_write_byte(0x8d, OLED_CMD);
    oled_write_byte(0x10, OLED_CMD);
    oled_write_byte(0xae, OLED_CMD);
}


/**
 * @brief   operate the graphic ram(size: 128*8 char)
 * @param   pen: the type of operate.
            PEN_CLEAR: set ram to 0x00
            PEN_WRITE: set ram to 0xff
            PEN_INVERSION: bit inversion 
 * @retval  none
 */
void OLED_operate_gram(pen_typedef pen)
{
    uint8_t i, n;

    for (i = 0; i < 8; i++)
    {
        for (n = 0; n < 128; n++)
        {
            if (pen == PEN_WRITE)
            {
                oled_gram.OLED_GRAM[i][n] = 0xff;
            }
            else if (pen == PEN_CLEAR)
            {
                oled_gram.OLED_GRAM[i][n] = 0x00;
            }
            else
            {
                oled_gram.OLED_GRAM[i][n] = 0xff - oled_gram.OLED_GRAM[i][n];
            }
        }
    }
}

/**
 * @brief   cursor set to (x,y) point
 * @param   x:X-axis, from 0 to 127
 * @param   y:Y-axis, from 0 to 7
 * @retval  none
 */
void OLED_set_pos(uint8_t x, uint8_t y)
{
    x &= 0x7F;
    y &= 0x07;

    oled_write_byte(0x21, OLED_CMD);
    oled_write_byte(0x00 + x, OLED_CMD);
    oled_write_byte(0x7F, OLED_CMD);

    oled_write_byte(0x22, OLED_CMD);
    oled_write_byte(0x00 + y, OLED_CMD);
    oled_write_byte(0x07, OLED_CMD);
}

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
void OLED_draw_point(uint8_t x, uint8_t y, pen_typedef pen)
{
    uint8_t page = 0, row = 0;

    /* check the corrdinate */
    if ( (x > (X_WIDTH - 1)) || (y > (Y_WIDTH - 1)))
    {
        return;
    }
    page = y / 8;
    row = y % 8;

    if (pen == PEN_WRITE)
    {
        oled_gram.OLED_GRAM[page][x] |= 1 << row;
    }
    else if (pen == PEN_INVERSION)
    {
        oled_gram.OLED_GRAM[page][x] ^= 1 << row;
    }
    else
    {
        oled_gram.OLED_GRAM[page][x] &= ~(1 << row);
    }
}



/**
 * @brief   draw a line from (x1, y1) to (x2, y2)
 * @param   x1, y1: the start point of line
 * @param   x2, y2: the end of line
 * @param   pen: Pen_Clear, Pen_Write, Pen_Inversion @Pen_Typedef
 * @retval  None
 */
void OLED_draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, pen_typedef pen)
{
    uint8_t col = 0, row = 0;
    uint8_t x_st = 0, x_ed = 0, y_st = 0, y_ed = 0;
    float k = 0.0f, b = 0.0f;

    if (y1 == y2)
    {
        (x1 <= x2) ? (x_st = x1):(x_st = x2);
        (x1 <= x2) ? (x_ed = x2):(x_ed = x1);

        for (col = x_st; col <= x_ed; col++)
        {
            OLED_draw_point(col, y1, pen);
        }
    }
    else if (x1 == x2)
    {
        (y1 <= y2) ? (y_st = y1):(y_st = y2);
        (y1 <= y2) ? (y_ed = y2):(y_ed = y1);

        for (row = y_st; row <= y_ed; row++)
        {
            OLED_draw_point(x1, row, pen);
        }
    }
    else
    {
        k = ((float)(y2 - y1)) / (x2 - x1);
        b = (float)y1 - k * x1;

        (x1 <= x2) ? (x_st = x1):(x_st = x2);
        (x1 <= x2) ? (x_ed = x2):(x_ed = x2);

        for (col = x_st; col <= x_ed; col++)
        {
            OLED_draw_point(col, (uint8_t)(col * k + b), pen);
        }
    }
}


/**
 * @brief   show a character
 * @param   row: row of character
 * @param   col: column of character
 * @param   chr: the character ready to show
 * @retval  None
 */
void OLED_show_char(uint8_t col, uint8_t row, uint8_t chr)
{
    uint8_t x = col;
    uint8_t y = row ;
    uint8_t temp, t, t1;
    uint8_t y0 = y;
    chr = chr - ' ';

    for (t = 0; t < 12; t++)
    {
        temp = asc2_1206[chr][t];

        for (t1 = 0; t1 < 8; t1++)
        {
            if (temp&0x80)
                OLED_draw_point(x, y, PEN_WRITE);
            else
                OLED_draw_point(x, y, PEN_CLEAR);

            temp <<= 1;
            y++;
            if ((y - y0) == 12)
            {
                y = y0;
                x++;
                break;
            }
        }
    }
}


/**
 * @brief   show a character string
 * @param   row: row of character string begin
 * @param   col: column of character string begin
 * @param   chr: the pointer to character string
 * @retval  None
 */
void OLED_show_string(uint8_t col, uint8_t row, uint8_t *chr)
{
    uint8_t n =0;

    while (chr[n] != '\0')
    {
        OLED_show_char(col, row, chr[n]);
        col+=6;

        if (col > X_WIDTH - 6)
        {
            col = 0;
            row += 12;
        }
        n++;
    }
}


/**
 * @brief   formatted output in oled 128*64
 * @param   row: row of character string begin, 0 <= row <= 4;
 * @param   col: column of character string begin, 0 <= col <= 20;
 * @param   *fmt: the pointer to format character string
 * @retval  None
 * @note    if the character length is more than one row at a time, the extra characters will be truncated
 */
void OLED_printf(uint8_t col, uint8_t row, const char *fmt,...)
{
    static uint8_t LCD_BUF[22] = {0};
    static va_list ap;
    uint16_t remain_size = 0;

    va_start(ap, fmt);

    remain_size = vsprintf((char *)LCD_BUF, fmt, ap);

    va_end(ap);



    LCD_BUF[remain_size] = '\0';

    OLED_show_string(col, row, LCD_BUF);
}

/**
 * @brief   send the data of gram to oled sreen
 * @param   none
 * @retval  none
 */
void OLED_refresh_gram(void)
{
    OLED_set_pos(0, 0);
    oled_gram.cmd_data = 0x40;
    I2C2_DMA_transmit(OLED_I2C_ADDRESS, (uint8_t*)&oled_gram, 1025);
}





/**
 * @brief   show the logo of robomaster
 * @param   none
 * @retval  none
 */
void OLED_LOGO(void)
{
    memcpy(oled_gram.OLED_GRAM, RM_LOGO_BMP_TRANS, 1024);
//    OLED_show_graphic(0, 0, &rm_logo);
    OLED_refresh_gram();
}




void OLED_show_graphic(uint8_t x, uint8_t y, const picture_t *graphic)
{
    uint8_t col, row;
    uint8_t temp_char, t;
    uint16_t i = 0;

    for(col = 0; col < graphic->length; col++)
    {
        for(row = 0; row < graphic->width; )
        {
            temp_char = graphic->data[i];
            i++;
            for(t = 0; t < 8; t++)
            {
                if(temp_char & 0x80)
                {
                    OLED_draw_point(x + col, y + row,PEN_WRITE);
                }
                else
                {
                    OLED_draw_point(x + col, y + row,PEN_CLEAR);
                }
                temp_char <<= 1;
                row++;
                if(row == graphic->width)
                {
                    break;
                }
            }
        }
    }
}




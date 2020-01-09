/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       AHRS_MiddleWare.c/h
  * @brief      姿态解算中间层，为姿态解算提供相关函数
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "AHRS_MiddleWare.h"
#include "AHRS.h"
#include "arm_math.h"
#include "main.h"
/**
 * @brief          用于获取当前高度
 * @author         RM
 * @param[in]      高度的指针，fp32
 * @retval         返回空
 */

void AHRS_get_height(fp32* high)
{
    if (high != NULL)
    {
        *high = 0.0f;
    }
}

/**
 * @brief          用于获取当前纬度
 * @author         RM
 * @param[in]      纬度的指针，fp32
 * @retval         返回空
 */

void AHRS_get_latitude(fp32* latitude)
{
    if (latitude != NULL)
    {
        *latitude = 22.0f;
    }
}

/**
 * @brief          快速开方函数，
 * @author         RM
 * @param[in]      输入需要开方的浮点数，fp32
 * @retval         返回1/sqrt 开方后的倒数
 */

fp32 AHRS_invSqrt(fp32 num)
{
    return 1/sqrtf(num);

//    fp32 halfnum = 0.5f * num;
//    fp32 y = num;
//    long i = *(long*)&y;
//    i = 0x5f3759df - (i >> 1);
//    y = *(fp32*)&i;
//    y = y * (1.5f - (halfnum * y * y));
//    y = y * (1.5f - (halfnum * y * y));
//    return y;
}

/**
 * @brief          sin函数
 * @author         RM
 * @param[in]      角度 单位 rad
 * @retval         返回对应角度的sin值
 */

fp32 AHRS_sinf(fp32 angle)
{
    return arm_sin_f32(angle);
}
/**
 * @brief          cos函数
 * @author         RM
 * @param[in]      角度 单位 rad
 * @retval         返回对应角度的cos值
 */

fp32 AHRS_cosf(fp32 angle)
{
    return arm_cos_f32(angle);
}

/**
 * @brief          tan函数
 * @author         RM
 * @param[in]      角度 单位 rad
 * @retval         返回对应角度的tan值
 */

fp32 AHRS_tanf(fp32 angle)
{
    return tanf(angle);
}
/**
 * @brief          用于32位浮点数的反三角函数 asin函数
 * @author         RM
 * @param[in]      输入sin值，最大1.0f，最小-1.0f
 * @retval         返回角度 单位弧度
 */

fp32 AHRS_asinf(fp32 sin)
{

    return asinf(sin);
}

/**
 * @brief          反三角函数acos函数
 * @author         RM
 * @param[in]      输入cos值，最大1.0f，最小-1.0f
 * @retval         返回对应的角度 单位弧度
 */

fp32 AHRS_acosf(fp32 cos)
{

    return acosf(cos);
}

/**
 * @brief          反三角函数atan函数
 * @author         RM
 * @param[in]      输入tan值中的y值 最大正无穷，最小负无穷
 * @param[in]      输入tan值中的x值 最大正无穷，最小负无穷
 * @retval         返回对应的角度 单位弧度
 */

fp32 AHRS_atan2f(fp32 y, fp32 x)
{
    return atan2f(y, x);
}

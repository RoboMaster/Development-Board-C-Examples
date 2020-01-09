#include "bsp_i2c.h"
#include "main.h"


extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;


void bsp_I2C_master_transmit(I2C_TypeDef *I2C, uint16_t I2C_address, uint8_t *data, uint16_t len)
{
    if(I2C == I2C1)
    {
        HAL_I2C_Master_Transmit(&hi2c1, I2C_address, data, len, 100);
    }
    else if(I2C == I2C2)
    {
        HAL_I2C_Master_Transmit(&hi2c2, I2C_address, data, len, 100);
    }
}


void bsp_I2C_reset(I2C_TypeDef *I2C)
{
    I2C_HandleTypeDef *hi2c;
    if(I2C == I2C1)
    {
        hi2c = &hi2c1;
    }
    else if(I2C == I2C2)
    {
        hi2c = &hi2c2;
    }

    SET_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_SWRST);
    if (HAL_I2C_Init(hi2c) != HAL_OK)
    {
        Error_Handler();
    }
}



bool_t bsp_I2C_check_ack(I2C_TypeDef *I2C, uint16_t I2C_address)
{
    I2C_HandleTypeDef *hi2c;
    if(I2C == I2C1)
    {
        hi2c = &hi2c1;
    }
    else if(I2C == I2C2)
    {
        hi2c = &hi2c2;
    }

    if((hi2c->Instance->CR2 & I2C_CR2_DMAEN) && ((hi2c->hdmatx != NULL && hi2c->hdmatx->Instance->NDTR != 0) || (hi2c->hdmarx != NULL && hi2c->hdmarx->Instance->NDTR != 0)))
    {
        return I2C_ACK;
    }
    else
    {
        uint16_t timeout = 0;

        timeout = 0;
        while(hi2c->Instance->SR2 & 0x02)
        {
            timeout ++;
            if(timeout > 100)
            {
                SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
                return I2C_NO_ACK;
            }
        }

        CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_POS);

        SET_BIT(hi2c->Instance->CR1, I2C_CR1_START);

        timeout = 0;
        while(!(hi2c->Instance->SR1 & 0x01))
        {
            timeout ++;
            if(timeout > 100)
            {
                SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
                return I2C_NO_ACK;
            }
        }

        hi2c->Instance->DR = I2C_7BIT_ADD_WRITE(I2C_address);

        timeout = 0;
        while(!(hi2c->Instance->SR1 & 0x02))
        {
            timeout ++;
            if(timeout > 500)
            {
                SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
                return I2C_NO_ACK;
            }
        }

        do{
            __IO uint32_t tmpreg = 0x00U;
            tmpreg = hi2c->Instance->SR1;
            tmpreg = hi2c->Instance->SR2;
            UNUSED(tmpreg);
        } while(0);

        timeout = 0;
        while(!(hi2c->Instance->SR1 & 0x80))
        {
            timeout ++;
            if(timeout > 500)
            {
                SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
                return I2C_NO_ACK;
            }
        }

        SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);

        return I2C_ACK;
    }


}



void DMA1_Stream7_IRQHandler(void)
{
    if(DMA1->HISR & (1<<27))
    {
        __HAL_DMA_CLEAR_FLAG(hi2c2.hdmatx, DMA_HISR_TCIF7);
        SET_BIT(hi2c2.Instance->CR1, I2C_CR1_STOP);
    }
}



void I2C2_tx_DMA_init(void)
{

    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(hi2c2.hdmatx);
    
    while(hi2c2.hdmatx->Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(hi2c2.hdmatx);
    }
    hi2c2.hdmatx->Instance->PAR = (uint32_t)(&I2C2->DR);
    __HAL_DMA_CLEAR_FLAG(hi2c2.hdmatx, DMA_HISR_TCIF7);
    __HAL_DMA_ENABLE_IT(hi2c2.hdmatx, DMA_IT_TC);



}

void I2C2_tx_DMA_enable(uint32_t tx_buf, uint16_t ndtr)
{
    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(hi2c2.hdmatx);

    while(hi2c2.hdmatx->Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(hi2c2.hdmatx);
    }

    __HAL_DMA_CLEAR_FLAG(hi2c2.hdmatx, DMA_HISR_TCIF7);


    hi2c2.hdmatx->Instance->M0AR = tx_buf;
    __HAL_DMA_SET_COUNTER(hi2c2.hdmatx, ndtr);

    __HAL_DMA_ENABLE(hi2c2.hdmatx);

}

HAL_StatusTypeDef I2C_TX_DMA_START(I2C_HandleTypeDef *hi2c, uint16_t DevAddress)
{
    uint16_t timeout = 0;

    if ((hi2c->Instance->CR1 & I2C_CR1_PE) != I2C_CR1_PE)
    {
        hi2c->Instance->CR1 |= I2C_CR1_PE;
    }


    timeout = 0;
    while(hi2c->Instance->SR2 & 0x02)
    {
        timeout ++;
        if(timeout > 100)
        {
            SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
            return HAL_BUSY;
        }
    }


    CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_POS);

    SET_BIT(hi2c->Instance->CR1, I2C_CR1_START);

    timeout = 0;
    while(!(hi2c->Instance->SR1 & 0x01))
    {
        timeout ++;
        if(timeout > 100)
        {
            SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
            return HAL_ERROR;
        }
    }


    hi2c->Instance->DR = I2C_7BIT_ADD_WRITE(DevAddress);


    timeout = 0;
    while(!(hi2c->Instance->SR1 & 0x02))
    {
        timeout ++;
        if(timeout > 500)
        {
            SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
            return HAL_ERROR;
        }
    }

    do{
        __IO uint32_t tmpreg = 0x00U;
        tmpreg = hi2c->Instance->SR1;
        tmpreg = hi2c->Instance->SR2;
        UNUSED(tmpreg);
    } while(0);



    timeout = 0;
    while(!(hi2c->Instance->SR1 & 0x80))
    {
        timeout ++;
        if(timeout > 500)
        {
            SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}
void I2C2_DMA_transmit(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
    if( I2C_TX_DMA_START(&hi2c2, DevAddress) == HAL_OK)
    {
        I2C2_tx_DMA_enable((uint32_t)pData, Size);
    }
}








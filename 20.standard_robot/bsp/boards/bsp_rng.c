#include "bsp_rng.h"
#include "main.h"

extern RNG_HandleTypeDef hrng;

uint32_t RNG_get_random_num(void)
{
    static uint32_t rng;
    HAL_RNG_GenerateRandomNumber(&hrng, &rng);
    return rng;
}

int32_t RNG_get_random_rangle(int min, int max)
{
    static int32_t random;
    random = (RNG_get_random_num() % (max - min + 1)) + min;
    return random;
}




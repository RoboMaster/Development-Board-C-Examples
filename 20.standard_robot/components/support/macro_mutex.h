#ifndef __MACRO_MUTEX_H
#define __MACRO_MUTEX_H

#include "stm32f4xx_hal.h"

#define USE_CRITICAL_LOCK

#ifdef USE_CRITICAL_LOCK
/* using disable interrupt to protect */
#define MUTEX_DECLARE(mutex) unsigned long mutex
#define MUTEX_INIT(mutex) \
  do                      \
  { /* nothing */         \
  } while (0)
#define MUTEX_LOCK(mutex)    \
  do                         \
  {                          \
    mutex = __get_PRIMASK(); \
    __disable_irq();         \
  } while (0)
#define MUTEX_UNLOCK(mutex) \
  do                        \
  {                         \
    __set_PRIMASK(mutex);   \
  } while (0)

#else
/* no protect */
#define MUTEX_DECLARE(mutex)
#define MUTEX_INIT(mutex) \
  do                      \
  { /* nothing */         \
  } while (0)
#define MUTEX_LOCK(mutex) \
  do                      \
  { /* nothing */         \
  } while (0)
#define MUTEX_UNLOCK(mutex) \
  do                        \
  { /* nothing */           \
  } while (0)
#endif

#endif /* __MACRO_MUTEX_H */

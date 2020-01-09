
#include "fifo.h"

//******************************************************************************************
//!                     ASSERT MACRO
//******************************************************************************************
#ifndef ASSERT

#ifdef NDEBUG
#define ASSERT(x)
#else
#define ASSERT(x)                                     \
  do                                                  \
  {                                                   \
    if (!(x))                                         \
      printf("[assert]: %s, %d", __FILE__, __LINE__); \
    while (!(x))                                      \
      ;                                               \
  } while (0)
#endif

#endif // ASSERT

#ifdef USE_DYNAMIC_MEMORY
//******************************************************************************************
//
//! \brief  Create An New FIFO Instance(in Single Mode).
//! This function allocate enought room for N blocks fifo elements, then return the pointer
//! of FIFO.
//!
//! \param  [in] uint_cnt is count of fifo elements.
//! \retval The Pointer of FIFO instance, return NULL is failure to allocate memory.
//!
//! \note   -# You must enable USE_MEMORY_ALLOC macro and ensure your system have <stdlib.h>
//!            Header file before use this function.
//! \note   -# Functions FIFO_Create and FIFO_Destory must be used in pairs.
//!
//******************************************************************************************
fifo_s_t *fifo_s_create(int uint_cnt)
{
  fifo_s_t *p_fifo = NULL;  //!< FIFO Pointer
  char *p_base_addr = NULL; //!< Memory Base Address

  //! Check input parameters.
  ASSERT(uint_cnt);

  //! Allocate Memory for pointer of new FIFO Control Block
  p_fifo = (fifo_s_t *)malloc(sizeof(fifo_s_t));
  if (NULL == p_fifo)
  {
    //! Allocate Failure, exit now
    return (NULL);
  }
  //! Allocate Memory for pointer of new FIFO
  p_base_addr = malloc(uint_cnt);
  if (NULL == p_base_addr)
  {
    //! Allocate Failure, exit now
    free(p_fifo);
    return (NULL);
  }
  //! Initialize General FIFO Module
  fifo_s_init(p_fifo, p_base_addr, uint_cnt);

  return (p_fifo);
}

//******************************************************************************************
//
//! \brief  Destory FIFO Instance(in Single Mode).
//!  This function release memory, then reinit the FIFO struct.
//!
//! \param  [in] p_fifo is the pointer of FIFO instance
//! \retval None.
//!
//! \note   -# You must enable USE_MEMORY_ALLOC macro and ensure your system have <stdlib.h>
//!            Header file before use this function.
//
//******************************************************************************************
void fifo_s_destroy(fifo_s_t *p_fifo)
{
  //! Check input parameters.
  ASSERT(p_fifo);
  ASSERT(p_fifo->p_start_addr);

  //! Free FIFO memory
  free(p_fifo->p_start_addr);
  //! Free FIFO Control Block memory
  free(p_fifo);

  return; //!< Success
}

#endif // USE_DYNAMIC_MEMORY

//******************************************************************************************
//
//! \brief  Initialize an static FIFO struct(in single mode).
//!
//! \param  [in] p_fifo is the pointer of valid FIFO instance.
//! \param  [in] p_base_addr is the base address of pre-allocate memory, such as array.
//! \param  [in] uint_cnt is count of fifo elements.
//! \retval 0 if initialize successfully, otherwise return -1.
//
//******************************************************************************************
int fifo_s_init(fifo_s_t *p_fifo, void *p_base_addr, int uint_cnt)
{
  //! Check input parameters.
  ASSERT(p_fifo);
  ASSERT(p_base_addr);
  ASSERT(uint_cnt);

  //! Initialize FIFO Control Block.
  p_fifo->p_start_addr = (char *)p_base_addr;
  p_fifo->p_end_addr = (char *)p_base_addr + uint_cnt - 1;
  p_fifo->free_num = uint_cnt;
  p_fifo->used_num = 0;
  p_fifo->p_read_addr = (char *)p_base_addr;
  p_fifo->p_write_addr = (char *)p_base_addr;

  return (0);
}

//******************************************************************************************
//
//! \brief  Put an element into FIFO(in single mode).
//!
//! \param  [in]  p_fifo is the pointer of valid FIFO.
//! \param  [in]  element is the data element you want to put
//!
//! \retval 0 if operate successfully, otherwise return -1.
//
//******************************************************************************************
int fifo_s_put(fifo_s_t *p_fifo, char element)
{
  FIFO_CPU_SR_TYPE cpu_sr;

  //! Check input parameters.
  ASSERT(p_fifo);

  //Interrupt off
  cpu_sr = FIFO_GET_CPU_SR();
  FIFO_ENTER_CRITICAL();

  if (0 == p_fifo->free_num)
  {
    //! Error, FIFO is full!
      goto end;
  }

  if (p_fifo->p_write_addr > p_fifo->p_end_addr)
    p_fifo->p_write_addr = p_fifo->p_start_addr;

  *(p_fifo->p_write_addr) = element;
  p_fifo->p_write_addr++;
  p_fifo->free_num--;
  p_fifo->used_num++;

  //Interrupt On
  FIFO_RESTORE_CPU_SR(cpu_sr);

  return (0);
end:
  //Interrupt On
  FIFO_RESTORE_CPU_SR(cpu_sr);
  return (-1);
}

//******************************************************************************************
//
//! \brief  Put some elements into FIFO(in single mode).
//!
//! \param  [in]  p_fifo is the pointer of valid FIFO.
//! \param  [in]  p_source is the data element you want to put
//! \param  [in]  the number of elements
//! \retval the number of really write data, otherwise return -1.
//
//******************************************************************************************
int fifo_s_puts(fifo_s_t *p_fifo, char *p_source, int len)
{
  FIFO_CPU_SR_TYPE cpu_sr;

  int retval;
  int len_to_end;
  int len_from_start;

  ASSERT(p_fifo);
  
  //Interrupt Off;
  cpu_sr = FIFO_GET_CPU_SR();
  FIFO_ENTER_CRITICAL();

  if (NULL == p_source)
  {
      goto end;
  }

  if (0 == p_fifo->free_num)
  {
      goto end;
  }

  if (p_fifo->p_write_addr > p_fifo->p_end_addr)
    p_fifo->p_write_addr = p_fifo->p_start_addr;

  len = (len < p_fifo->free_num) ? len : p_fifo->free_num;
  len_to_end = p_fifo->p_end_addr - p_fifo->p_write_addr + 1;

  if (len_to_end >= len) //no rollback
  {
    len_to_end = len;
    memcpy(p_fifo->p_write_addr, p_source, len_to_end);
    p_fifo->p_write_addr += len_to_end;
  }
  else //rollback
  {
    len_from_start = len - len_to_end;
    memcpy(p_fifo->p_write_addr, p_source, len_to_end);
    memcpy(p_fifo->p_start_addr, p_source + len_to_end, len_from_start);
    p_fifo->p_write_addr = p_fifo->p_start_addr + len_from_start;
  }

  p_fifo->free_num -= len;
  p_fifo->used_num += len;
  retval = len;

  //Interrupt On
  FIFO_RESTORE_CPU_SR(cpu_sr);

  return retval;
end:
  //Interrupt On
  FIFO_RESTORE_CPU_SR(cpu_sr);
  return (-1);
}

//******************************************************************************************
//
//! \brief  Put some elements into FIFO, ingnore the interrupt
//!
//! \param  [in]  p_fifo is the pointer of valid FIFO.
//! \param  [in]  p_source is the data element you want to put
//! \param  [in]  the number of elements
//! \retval the number of really write data, otherwise return -1.
//
//******************************************************************************************
int fifo_s_puts_noprotect(fifo_s_t *p_fifo, char *p_source, int len)
{
  int retval;
  int len_to_end;
  int len_from_start;

  ASSERT(p_fifo);

  if (NULL == p_source)
    return -1;

  if (0 == p_fifo->free_num)
    return 0;

  if (p_fifo->p_write_addr > p_fifo->p_end_addr)
    p_fifo->p_write_addr = p_fifo->p_start_addr;

  len = (len < p_fifo->free_num) ? len : p_fifo->free_num;
  len_to_end = p_fifo->p_end_addr - p_fifo->p_write_addr + 1;

  if (len_to_end >= len) //no rollback
  {
    len_to_end = len;
    memcpy(p_fifo->p_write_addr, p_source, len_to_end);
    p_fifo->p_write_addr += len_to_end;
  }
  else //rollback
  {
    len_from_start = len - len_to_end;
    memcpy(p_fifo->p_write_addr, p_source, len_to_end);
    memcpy(p_fifo->p_start_addr, p_source + len_to_end, len_from_start);
    p_fifo->p_write_addr = p_fifo->p_start_addr + len_from_start;
  }

  p_fifo->free_num -= len;
  p_fifo->used_num += len;
  retval = len;

  return retval;
}

//******************************************************************************************
//
//! \brief  Get an element from FIFO(in single mode).
//!
//! \param  [in]  p_fifo is the pointer of valid FIFO.
//!
//! \retval the data element of FIFO.
//
//******************************************************************************************
char fifo_s_get(fifo_s_t *p_fifo)
{
  FIFO_CPU_SR_TYPE cpu_sr;
  char retval = 0;

  //! Check input parameters.
  ASSERT(p_fifo);

  //Interrupt Off;
  cpu_sr = FIFO_GET_CPU_SR();
  FIFO_ENTER_CRITICAL();

  if (p_fifo->p_read_addr > p_fifo->p_end_addr)
    p_fifo->p_read_addr = p_fifo->p_start_addr;

  retval = *p_fifo->p_read_addr;
  // Update information
  p_fifo->p_read_addr++;
  p_fifo->free_num++;
  p_fifo->used_num--;
  //Interrupt On
  FIFO_RESTORE_CPU_SR(cpu_sr);
  return (retval);
}

//******************************************************************************************
//
//! \brief  Get some element from FIFO(in single mode).
//!
//! \param  [in]  p_fifo is the pointer of valid FIFO.
//!
//! \retval the number of really read data.
//
//******************************************************************************************
int fifo_s_gets(fifo_s_t *p_fifo, char *p_dest, int len)
{
  FIFO_CPU_SR_TYPE cpu_sr;
  int retval;
  int len_to_end;
  int len_from_start;

  ASSERT(p_fifo);

  //Interrupt Off;
  cpu_sr = FIFO_GET_CPU_SR();
  FIFO_ENTER_CRITICAL();

  if (NULL == p_dest)
  {
    goto end;
  }

  if (0 == p_fifo->used_num)
  {
    goto end;
  }

  if (p_fifo->p_read_addr > p_fifo->p_end_addr)
    p_fifo->p_read_addr = p_fifo->p_start_addr;

  len = (len < p_fifo->used_num) ? len : p_fifo->used_num;
  len_to_end = p_fifo->p_end_addr - p_fifo->p_read_addr + 1;

  if (len_to_end >= len) //no rollback
  {
    len_to_end = len;
    memcpy(p_dest, p_fifo->p_read_addr, len_to_end);
    p_fifo->p_read_addr += len_to_end;
  }
  else //rollback
  {
    len_from_start = len - len_to_end;
    memcpy(p_dest, p_fifo->p_read_addr, len_to_end);
    memcpy(p_dest + len_to_end, p_fifo->p_start_addr, len_from_start);
    p_fifo->p_read_addr = p_fifo->p_start_addr + len_from_start;
  }

  p_fifo->free_num += len;
  p_fifo->used_num -= len;
  retval = len;
  //Interrupt On
  FIFO_RESTORE_CPU_SR(cpu_sr);

  return retval;
end:
  //Interrupt On
  FIFO_RESTORE_CPU_SR(cpu_sr);
  return (-1);
}

//******************************************************************************************
//
//! \brief  Get some element from FIFO(in single mode).
//!
//! \param  [in]  p_fifo is the pointer of valid FIFO.
//!
//! \retval the number of really read data.
//
//******************************************************************************************
int fifo_s_gets_noprotect(fifo_s_t *p_fifo, char *p_dest, int len)
{
  int retval;
  int len_to_end;
  int len_from_start;

  ASSERT(p_fifo);

  if (NULL == p_dest)
    return -1;

  if (0 == p_fifo->used_num)
    return 0;

  if (p_fifo->p_read_addr > p_fifo->p_end_addr)
    p_fifo->p_read_addr = p_fifo->p_start_addr;

  len = (len < p_fifo->used_num) ? len : p_fifo->used_num;
  len_to_end = p_fifo->p_end_addr - p_fifo->p_read_addr + 1;

  if (len_to_end >= len) //no rollback
  {
    len_to_end = len;
    memcpy(p_dest, p_fifo->p_read_addr, len_to_end);
    p_fifo->p_read_addr += len_to_end;
  }
  else //rollback
  {
    len_from_start = len - len_to_end;
    memcpy(p_dest, p_fifo->p_read_addr, len_to_end);
    memcpy(p_dest + len_to_end, p_fifo->p_start_addr, len_from_start);
    p_fifo->p_read_addr = p_fifo->p_start_addr + len_from_start;
  }

  p_fifo->free_num += len;
  p_fifo->used_num -= len;
  retval = len;

  return retval;
}

//******************************************************************************************
//
//! \brief  Pre-Read an element from FIFO(in single mode).
//!
//! \param  [in]  p_fifo is the pointer of valid FIFO.
//! \param  [in]  Offset is the offset from current pointer.
//!
//! \retval the data element of FIFO.
//
//******************************************************************************************
char fifo_s_preread(fifo_s_t *p_fifo, int offset)
{
  char *tmp_read_addr;

  //! Check input parameters.
  ASSERT(p_fifo);

  if (offset > p_fifo->used_num)
  {
    return 0;
  }
  else
  {
    // Move Read Pointer to right position
    tmp_read_addr = p_fifo->p_read_addr + offset;
    if (tmp_read_addr > p_fifo->p_end_addr)
      tmp_read_addr = tmp_read_addr - p_fifo->p_end_addr + p_fifo->p_start_addr - 1;

    return *tmp_read_addr;
  }
}

/*
 *
 *
 *
 *
 */
int fifo_s_prereads(fifo_s_t *p_fifo, char *p_dest, int offset, int len)
{
  FIFO_CPU_SR_TYPE cpu_sr;
  int retval;
  char *tmp_read_addr;
  int len_to_end;
  int len_from_start;

  ASSERT(p_fifo);

  //Interrupt Off;
  cpu_sr = FIFO_GET_CPU_SR();
  FIFO_ENTER_CRITICAL();

  if (NULL == p_dest)
    goto end;

  if (0 == p_fifo->used_num)
    goto end;

  if (offset >= p_fifo->used_num)
    goto end;

  tmp_read_addr = p_fifo->p_read_addr + offset;
  if (tmp_read_addr > p_fifo->p_end_addr)
    tmp_read_addr = tmp_read_addr - p_fifo->p_end_addr + p_fifo->p_start_addr - 1;

  len = (len < (p_fifo->used_num - offset)) ? len : (p_fifo->used_num - offset);
  len_to_end = p_fifo->p_end_addr - tmp_read_addr + 1;

  if (len_to_end >= len) //no rollback
  {
    len_to_end = len;
    memcpy(p_dest, tmp_read_addr, len_to_end);
  }
  else //rollback
  {
    len_from_start = len - len_to_end;
    memcpy(p_dest, tmp_read_addr, len_to_end);
    memcpy(p_dest + len_to_end, p_fifo->p_start_addr, len_from_start);
  }

  retval = len;
  //Interrupt On
  FIFO_RESTORE_CPU_SR(cpu_sr);

  return retval;
end:
  //Interrupt On
  FIFO_RESTORE_CPU_SR(cpu_sr);
  return (-1);
}

//******************************************************************************************
//
//! \brief  FIFO is empty (in single mode)?
//!
//! \param  [in] p_fifo is the pointer of valid FIFO.
//!
//! \retval - None-zero(true) if empty.
//!         - Zero(false) if not empty.
//
//******************************************************************************************
char fifo_s_isempty(fifo_s_t *p_fifo)
{
  //! Check input parameter.
  ASSERT(p_fifo);
  return (p_fifo->used_num ? 0 : 1);
}

//******************************************************************************************
//
//! \brief  FIFO is full (in single mode)?
//!
//! \param  [in] p_fifo is the pointer of valid FIFO.
//!
//! \retval - None-zero(true) if full.
//!         - Zero(false) if not full.
//
//******************************************************************************************
char fifo_s_isfull(fifo_s_t *p_fifo)
{
  //! Check input parameter.
  ASSERT(p_fifo);
  return (p_fifo->free_num ? 0 : 1);
}

//******************************************************************************************
//
//! \brief  Get FIFO the number of elements(in single mode)?
//!
//! \param  [in] p_fifo is the pointer of valid FIFO.
//!
//! \retval The number of elements in FIFO.
//
//******************************************************************************************
int fifo_s_used(fifo_s_t *p_fifo)
{
  //! Check input parameter.
  ASSERT(p_fifo);
  return p_fifo->used_num;
}

//******************************************************************************************
//
//! \brief  Get FIFO the number of elements(in single mode)?
//!
//! \param  [in] p_fifo is the pointer of valid FIFO.
//!
//! \retval The number of elements in FIFO.
//
//******************************************************************************************
int fifo_s_free(fifo_s_t *p_fifo)
{
  //! Check input parameter.
  ASSERT(p_fifo);
  return p_fifo->free_num;
}

//******************************************************************************************
//
//! \brief  Flush the content of FIFO.
//!
//! \param  [in] p_fifo is the pointer of valid FIFO.
//!
//! \retval 0 if success, -1 if failure.
//
//******************************************************************************************
void fifo_s_flush(fifo_s_t *p_fifo)
{
  FIFO_CPU_SR_TYPE cpu_sr;

  //! Check input parameters.
  ASSERT(p_fifo);
  //! Initialize FIFO Control Block.
  //Interrupt Off;
  cpu_sr = FIFO_GET_CPU_SR();
  FIFO_ENTER_CRITICAL();

  p_fifo->free_num = p_fifo->p_end_addr - p_fifo->p_start_addr + 1;
  p_fifo->used_num = 0;
  p_fifo->p_read_addr = p_fifo->p_start_addr;
  p_fifo->p_write_addr = p_fifo->p_start_addr;
  //Interrupt On
  FIFO_RESTORE_CPU_SR(cpu_sr);
}

int fifo_s_discard(fifo_s_t *p_fifo, int len)
{
  FIFO_CPU_SR_TYPE cpu_sr;
  //! Check input parameters.
  char *tmp_index;
  ASSERT(p_fifo);

  //Interrupt Off;
  cpu_sr = FIFO_GET_CPU_SR();
  FIFO_ENTER_CRITICAL();

  if (len > p_fifo->used_num)
    len = p_fifo->used_num;

  tmp_index = len + p_fifo->p_read_addr;
  if (tmp_index > p_fifo->p_end_addr)
    tmp_index = tmp_index - p_fifo->p_end_addr + p_fifo->p_start_addr - 1;
  p_fifo->p_read_addr = tmp_index;
  p_fifo->free_num += len;
  p_fifo->used_num -= len;
  //Interrupt On
  FIFO_RESTORE_CPU_SR(cpu_sr);

  return len;
}

#ifdef USE_DYNAMIC_MEMORY
//******************************************************************************************
//
//! \brief  Create An New FIFO Instance.
//! This function allocate enought room for N blocks fifo elements, then return the pointer
//! of FIFO.
//!
//! \param  [in] UnitSize is fifo element size.
//! \param  [in] UnitCnt is count of fifo elements.
//! \retval The Pointer of FIFO instance, return NULL is failure to allocate memory.
//!
//! \note   -# You must enable USE_MEMORY_ALLOC macro and ensure your system have <stdlib.h>
//!            Header file before use this function.
//! \note   -# Functions FIFO_Create and FIFO_Destory must be used in pairs.
//!
//******************************************************************************************
fifo_t *fifo_create(char unit_size, int unit_cnt)
{
  fifo_t *p_fifo = NULL;    //!< FIFO Pointer
  char *p_base_addr = NULL; //!< Memory Base Address

  //! Check input parameters.
  ASSERT(unit_size);
  ASSERT(unit_cnt);

  //! Allocate Memory for pointer of new FIFO Control Block.
  p_fifo = (fifo_t *)malloc(sizeof(fifo_t));
  if (NULL == p_fifo)
  {
    //! Allocate Failure, exit now.
    return (NULL);
  }

  //! Allocate memory for FIFO.
  p_base_addr = malloc(unit_size * unit_cnt);
  if (NULL == p_base_addr)
  {
    //! Allocate Failure, exit now.
    free(p_fifo);
    return (NULL);
  }

  //! Initialize General FIFO Module.
  fifo_init(p_fifo, p_base_addr, unit_size, unit_cnt);

  return (p_fifo);
}

//******************************************************************************************
//
//! \brief  Destory FIFO Instance.
//!  This function release memory, then reinit the FIFO struct.
//!
//! \param  [in] pFIFO is the pointer of FIFO instance
//! \retval None.
//!
//! \note   -# You must enable USE_MEMORY_ALLOC macro and ensure your system have <stdlib.h>
//!            Header file before use this function.
//
//******************************************************************************************
void fifo_destory(fifo_t *p_fifo)
{
  //! Check input parameters.
  ASSERT(p_fifo);
  ASSERT(p_fifo->p_start_addr);

  //! Free FIFO memory
  free(p_fifo->p_start_addr);
  //! Free FIFO Control Block memory.
  free(p_fifo);

  return; //!< Success
}

#endif // USE_DYNAMIC_MEMORY

//******************************************************************************************
//
//! \brief  Initialize an static FIFO struct.
//!
//! \param  [in] pFIFO is the pointer of valid FIFO instance.
//! \param  [in] pBaseAddr is the base address of pre-allocate memory, such as array.
//! \param  [in] UnitSize is fifo element size.
//! \param  [in] UnitCnt is count of fifo elements.
//! \retval 0 if initialize successfully, otherwise return -1.
//
//******************************************************************************************
int fifo_init(fifo_t *p_fifo, void *p_base_addr, char unit_size, int unit_cnt)
{
  //! Check input parameters.
  ASSERT(p_fifo);
  ASSERT(p_base_addr);
  ASSERT(unit_size);
  ASSERT(unit_cnt);

  //! Initialize FIFO Control Block.
  p_fifo->p_start_addr = (char *)p_base_addr;
  p_fifo->p_end_addr = (char *)p_base_addr + unit_size * unit_cnt - 1;
  p_fifo->free_num = unit_cnt;
  p_fifo->used_num = 0;
  p_fifo->unit_size = unit_size;
  p_fifo->p_read_addr = (char *)p_base_addr;
  p_fifo->p_write_addr = (char *)p_base_addr;

  return (0);
}

//******************************************************************************************
//
//! \brief  Put an element into FIFO.
//!
//! \param  [in]  pFIFO is the pointer of valid FIFO.
//! \param  [in]  pElement is the address of element you want to put
//!
//! \retval 0 if operate successfully, otherwise return -1.
//
//******************************************************************************************
int fifo_put(fifo_t *p_fifo, void *p_element)
{
  //! Check input parameters.
  FIFO_CPU_SR_TYPE cpu_sr;
  ASSERT(p_fifo);
  ASSERT(p_element);

    //Interrupt Off;
  cpu_sr = FIFO_GET_CPU_SR();
  FIFO_ENTER_CRITICAL();

  // Full ?
  if (0 == p_fifo->free_num)
  {
    //! Error, FIFO is full!
    goto end;
  }

  //! Copy Data

  if (p_fifo->p_write_addr > p_fifo->p_end_addr)
    p_fifo->p_write_addr = p_fifo->p_start_addr;

  memcpy(p_fifo->p_write_addr, p_element, p_fifo->unit_size);
  p_fifo->p_write_addr += p_fifo->unit_size;
  p_fifo->free_num--;
  p_fifo->used_num++;
  //Interrupt On
  FIFO_RESTORE_CPU_SR(cpu_sr);
  return (0);
end:
  //Interrupt On
  FIFO_RESTORE_CPU_SR(cpu_sr);
  return (-1);
}

//******************************************************************************************
//
//! \brief  Put an element into FIFO.
//!
//! \param  [in]  pFIFO is the pointer of valid FIFO.
//! \param  [in]  pElement is the address of element you want to put
//!
//! \retval 0 if operate successfully, otherwise return -1.
//
//******************************************************************************************
int fifo_put_noprotect(fifo_t *p_fifo, void *p_element)
{
  //! Check input parameters.
  ASSERT(p_fifo);
  ASSERT(p_element);

  // Full ?
  if (0 == p_fifo->free_num)
  {
    //! Error, FIFO is full!
    return (-1);
  }

  //! Copy Data
  if (p_fifo->p_write_addr > p_fifo->p_end_addr)
    p_fifo->p_write_addr = p_fifo->p_start_addr;

  memcpy(p_fifo->p_write_addr, p_element, p_fifo->unit_size);
  p_fifo->p_write_addr += p_fifo->unit_size;
  p_fifo->free_num--;
  p_fifo->used_num++;

  return (0);
}

//******************************************************************************************
//
//! \brief  Get an element from FIFO.
//!
//! \param  [in]  pFIFO is the pointer of valid FIFO.
//! \param  [out] pElement is the address of element you want to get
//!
//! \retval 0 if operate successfully, otherwise return -1.
//
//******************************************************************************************
int fifo_get(fifo_t *p_fifo, void *p_element)
{
  FIFO_CPU_SR_TYPE cpu_sr;
  //! Check input parameters.
  ASSERT(p_fifo);
  ASSERT(p_element);
  
      //Interrupt Off;
  cpu_sr = FIFO_GET_CPU_SR();
  FIFO_ENTER_CRITICAL();

  // Empty ?
  if (0 == p_fifo->used_num)
  {
    //! Error, FIFO is Empty!
    goto end;
  }

  //! Copy Data

  if (p_fifo->p_read_addr > p_fifo->p_end_addr)
  {
    p_fifo->p_read_addr = p_fifo->p_start_addr;
  }
  memcpy(p_element, p_fifo->p_read_addr, p_fifo->unit_size);
  p_fifo->p_read_addr += p_fifo->unit_size;
  p_fifo->free_num++;
  p_fifo->used_num--;
  //Interrupt On
  FIFO_RESTORE_CPU_SR(cpu_sr);
  return (0);
end:
  //Interrupt On
  FIFO_RESTORE_CPU_SR(cpu_sr);
  return (-1);
}

//******************************************************************************************
//
//! \brief  Get an element from FIFO.
//!
//! \param  [in]  pFIFO is the pointer of valid FIFO.
//! \param  [out] pElement is the address of element you want to get
//!
//! \retval 0 if operate successfully, otherwise return -1.
//
//******************************************************************************************
int fifo_get_noprotect(fifo_t *p_fifo, void *p_element)
{
  //! Check input parameters.
  ASSERT(p_fifo);
  ASSERT(p_element);

  // Empty ?
  if (0 == p_fifo->used_num)
  {
    //! Error, FIFO is Empty!
    return (-1);
  }

  //! Copy Data
  if (p_fifo->p_read_addr > p_fifo->p_end_addr)
  {
    p_fifo->p_read_addr = p_fifo->p_start_addr;
  }
  memcpy(p_element, p_fifo->p_read_addr, p_fifo->unit_size);
  p_fifo->p_read_addr += p_fifo->unit_size;
  p_fifo->free_num++;
  p_fifo->used_num--;

  return (0);
}

//******************************************************************************************
//
//! \brief  Pre-Read an element from FIFO.
//!
//! \param  [in]  pFIFO is the pointer of valid FIFO.
//! \param  [in]  Offset is the offset from current pointer.
//! \param  [out] pElement is the address of element you want to get
//!
//! \retval 0 if operate successfully, otherwise return -1.
//
//******************************************************************************************
int fifo_pre_read(fifo_t *p_fifo, char offset, void *p_element)
{
  char *_pre_red_index = (void *)0;

  //! Check input parameters.
  ASSERT(p_fifo);
  ASSERT(p_element);

  // OverFlow ?
  if (offset >= p_fifo->used_num)
  {
    return (-1);
  }

  // Move Read Pointer to right position
  _pre_red_index = p_fifo->p_read_addr + p_fifo->unit_size * offset;
  while (_pre_red_index > p_fifo->p_end_addr)
  {
    _pre_red_index = _pre_red_index - p_fifo->p_end_addr + p_fifo->p_start_addr - 1;
  }
  //! Copy Data
  memcpy(p_element, _pre_red_index, p_fifo->unit_size);

  return (0);
}

//******************************************************************************************
//
//! \brief  FIFO is empty ?
//!
//! \param  [in] pFIFO is the pointer of valid FIFO.
//!
//! \retval - None-zero(true) if empty.
//!         - Zero(false) if not empty.
//
//******************************************************************************************
int fifo_is_empty(fifo_t *p_fifo)
{
  //! Check input parameter.
  ASSERT(p_fifo);

  return (0 == p_fifo->used_num);
}

//******************************************************************************************
//
//! \brief  FIFO is full ?
//!
//! \param  [in] pFIFO is the pointer of valid FIFO.
//!
//! \retval - None-zero(true) if full.
//!         - Zero(false) if not full.
//
//******************************************************************************************
int fifo_is_full(fifo_t *p_fifo)
{
  //! Check input parameter.
  ASSERT(p_fifo);

  return (0 == p_fifo->free_num);
}

//******************************************************************************************
//
//! \brief  Get FIFO the number of elements?
//!
//! \param  [in] pFIFO is the pointer of valid FIFO.
//!
//! \retval The number of elements in FIFO.
//
//******************************************************************************************
int fifo_used(fifo_t *p_fifo)
{
  //! Check input parameter.
  ASSERT(p_fifo);

  return (p_fifo->used_num);
}

//******************************************************************************************
//
//! \brief  Get FIFO the number of elements?
//!
//! \param  [in] pFIFO is the pointer of valid FIFO.
//!
//! \retval The number of elements in FIFO.
//
//******************************************************************************************
int fifo_free(fifo_t *p_fifo)
{
  //! Check input parameter.
  ASSERT(p_fifo);

  return (p_fifo->free_num);
}

//******************************************************************************************
//
//! \brief  Flush the content of FIFO.
//!
//! \param  [in] pFIFO is the pointer of valid FIFO.
//!
//! \retval 0 if success, -1 if failure.
//
//******************************************************************************************
int fifo_flush(fifo_t *p_fifo)
{
  FIFO_CPU_SR_TYPE cpu_sr;
  //! Check input parameters.
  ASSERT(p_fifo);

  //! Initialize FIFO Control Block.
  //Interrupt Off;
  cpu_sr = FIFO_GET_CPU_SR();
  FIFO_ENTER_CRITICAL();

  p_fifo->free_num = (p_fifo->p_end_addr - p_fifo->p_start_addr) / (p_fifo->unit_size);
  p_fifo->used_num = 0;
  p_fifo->p_read_addr = p_fifo->p_start_addr;
  p_fifo->p_write_addr = p_fifo->p_start_addr;
  //Interrupt On
  FIFO_RESTORE_CPU_SR(cpu_sr);

  return (0);
}

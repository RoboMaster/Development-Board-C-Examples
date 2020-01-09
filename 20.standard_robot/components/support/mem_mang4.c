
#include "mem_mang.h"

MUTEX_DECLARE(mem_mutex);
/* Define the linked list structure.  This is used to link free blocks in order
of their memory address. */
typedef struct _block_link
{
  struct _block_link *next_free; /*<< The next free block in the list. */
  uint32_t block_size;           /*<< The size of the free block. */
} block_link_t;

/* Block sizes must not get too small. */
#define MINIMUM_BLOCK_SIZE ((uint32_t)(STRUCT_SIZE << 1))
/* Assumes 8bit bytes! */
#define BITS_PER_BYTE ((uint32_t)8)

/* The size of the structure placed at the beginning of each allocated memory
block must by correctly byte aligned. */
static const uint32_t STRUCT_SIZE = (sizeof(block_link_t) +
                                     ((uint32_t)(BYTE_ALIGNMENT - 1))) &
                                    ~((uint32_t)BYTE_ALIGNMENT_MASK);

/* Allocate the memory for the heap. */
static uint8_t heap[TOTAL_HEAP_SIZE];

static uint8_t mutex_init;

/*-----------------------------------------------------------*/

/*
 * Inserts a block of memory that is being freed into the correct position in
 * the list of free memory blocks.  The block being freed will be merged with
 * the block in front it and/or the block behind it if the memory blocks are
 * adjacent to each other.
 */
static void insert_into_free_list(block_link_t *block_to_insert);

/*
 * Called automatically to setup the required heap structures the first time
 * heap_malloc() is called.
 */
static void heap_init(void);

/*-----------------------------------------------------------*/

/* Create a couple of list links to mark the start and end of the list. */
static block_link_t start, *end = NULL;

/* Keeps track of the number of free bytes remaining, but says nothing about
fragmentation. */
static uint32_t free_bytes_remain = 0U;
static uint32_t ever_free_bytes_remain = 0U;

/* Gets set to the top bit of an uint32_t type.  When this bit in the block_size
member of an block_link_t structure is set then the block belongs to the
application.  When the bit is free the block is still part of the free heap
space. */
static uint32_t block_allocated_bit = 0;

/*-----------------------------------------------------------*/

void *heap_malloc(uint32_t wanted_size)
{
  block_link_t *block, *prev_block, *new_block;
  void *reval = NULL;

  if (mutex_init == 0)
  {
    mutex_init = 1;
    MUTEX_INIT(mem_mutex);
  }

  MUTEX_LOCK(mem_mutex);
  {
    /* If this is the first call to malloc then the heap will require
        initialisation to setup the list of free blocks. */
    if (end == NULL)
    {
      heap_init();
    }

    /* Check the requested block size is not so large that the top bit is
        set.  The top bit of the block size member of the block_link_t structure
        is used to determine who owns the block - the application or the
        kernel, so it must be free. */
    if ((wanted_size & block_allocated_bit) == 0)
    {
      /* The wanted size is increased so it can contain a block_link_t
            structure in addition to the requested amount of bytes. */
      if (wanted_size > 0)
      {
        wanted_size += STRUCT_SIZE;

        /* Ensure that blocks are always aligned to the required number
                of bytes. */
        if ((wanted_size & BYTE_ALIGNMENT_MASK) != 0x00)
        {
          /* Byte alignment required. */
          wanted_size += (BYTE_ALIGNMENT - (wanted_size & BYTE_ALIGNMENT_MASK));
          HEAP_ASSERT((wanted_size & BYTE_ALIGNMENT_MASK) == 0);
        }
      }

      if ((wanted_size > 0) && (wanted_size <= free_bytes_remain))
      {
        /* Traverse the list from the start    (lowest address) block until
                one    of adequate size is found. */
        prev_block = &start;
        block = start.next_free;
        while ((block->block_size < wanted_size) && (block->next_free != NULL))
        {
          prev_block = block;
          block = block->next_free;
        }

        /* If the end marker was reached then a block of adequate size
                was    not found. */
        if (block != end)
        {
          /* Return the memory space pointed to - jumping over the
                    block_link_t structure at its start. */
          reval = (void *)(((uint8_t *)prev_block->next_free) + STRUCT_SIZE);

          /* This block is being returned for use so must be taken out
                    of the list of free blocks. */
          prev_block->next_free = block->next_free;

          /* If the block is larger than required it can be split into
                    two. */
          if ((block->block_size - wanted_size) > MINIMUM_BLOCK_SIZE)
          {
            /* This block is to be split into two.  Create a new
                        block following the number of bytes requested. The void
                        cast is used to prevent byte alignment warnings from the
                        compiler. */
            new_block = (void *)(((uint8_t *)block) + wanted_size);
            HEAP_ASSERT((((uint32_t)new_block) & BYTE_ALIGNMENT_MASK) == 0);

            /* Calculate the sizes of two blocks split from the
                        single block. */
            new_block->block_size = block->block_size - wanted_size;
            block->block_size = wanted_size;

            /* Insert the new block into the list of free blocks. */
            insert_into_free_list(new_block);
          }

          free_bytes_remain -= block->block_size;

          if (free_bytes_remain < ever_free_bytes_remain)
          {
            ever_free_bytes_remain = free_bytes_remain;
          }

          /* The block is being returned - it is allocated and owned
                    by the application and has no "next" block. */
          block->block_size |= block_allocated_bit;
          block->next_free = NULL;
        }
      }
    }
  }
  MUTEX_UNLOCK(mem_mutex);

  HEAP_ASSERT((((uint32_t)reval) & (uint32_t)BYTE_ALIGNMENT_MASK) == 0);
  return reval;
}
/*-----------------------------------------------------------*/

void heap_free(void *pv)
{
  uint8_t *puc = (uint8_t *)pv;
  block_link_t *block;

  MUTEX_LOCK(mem_mutex);

  if (pv != NULL)
  {
    /* The memory being freed will have an block_link_t structure immediately
        before it. */
    puc -= STRUCT_SIZE;

    /* This casting is to keep the compiler from issuing warnings. */
    block = (void *)puc;

    /* Check the block is actually allocated. */
    HEAP_ASSERT((block->block_size & block_allocated_bit) != 0);
    HEAP_ASSERT(block->next_free == NULL);

    if ((block->block_size & block_allocated_bit) != 0)
    {
      if (block->next_free == NULL)
      {
        /* The block is being returned to the heap - it is no longer
                allocated. */
        block->block_size &= ~block_allocated_bit;

        {
          /* Add this block to the list of free blocks. */
          free_bytes_remain += block->block_size;
          insert_into_free_list(((block_link_t *)block));
        }
      }
    }
  }
  MUTEX_UNLOCK(mem_mutex);
}
/*-----------------------------------------------------------*/

uint32_t heap_get_free(void)
{
  return free_bytes_remain;
}
/*-----------------------------------------------------------*/

uint32_t heap_get_ever_free(void)
{
  return ever_free_bytes_remain;
}
/*-----------------------------------------------------------*/

static void heap_init(void)
{
  block_link_t *first_free_block;
  uint8_t *aligned_heap;
  uint32_t address;
  uint32_t total_heap_size = TOTAL_HEAP_SIZE;

  /* Ensure the heap starts on a correctly aligned boundary. */
  address = (uint32_t)heap;

  if ((address & BYTE_ALIGNMENT_MASK) != 0)
  {
    address += (BYTE_ALIGNMENT - 1);
    address &= ~((uint32_t)BYTE_ALIGNMENT_MASK);
    total_heap_size -= address - (uint32_t)heap;
  }

  aligned_heap = (uint8_t *)address;

  /* start is used to hold a pointer to the first item in the list of free
    blocks.  The void cast is used to prevent compiler warnings. */
  start.next_free = (void *)aligned_heap;
  start.block_size = (uint32_t)0;

  /* end is used to mark the end of the list of free blocks and is inserted
    at the end of the heap space. */
  address = ((uint32_t)aligned_heap) + total_heap_size;
  address -= STRUCT_SIZE;
  address &= ~((uint32_t)BYTE_ALIGNMENT_MASK);
  end = (void *)address;
  end->block_size = 0;
  end->next_free = NULL;

  /* To start with there is a single free block that is sized to take up the
    entire heap space, minus the space taken by end. */
  first_free_block = (void *)aligned_heap;
  first_free_block->block_size = address - (uint32_t)first_free_block;
  first_free_block->next_free = end;

  /* Only one block exists - and it covers the entire usable heap space. */
  ever_free_bytes_remain = first_free_block->block_size;
  free_bytes_remain = first_free_block->block_size;

  /* Work out the position of the top bit in a uint32_t variable. */
  block_allocated_bit = ((uint32_t)1) << ((sizeof(uint32_t) * BITS_PER_BYTE) - 1);
}
/*-----------------------------------------------------------*/

static void insert_into_free_list(block_link_t *block_to_insert)
{
  block_link_t *iterator;
  uint8_t *puc;

  /* Iterate through the list until a block is found that has a higher address
    than the block being inserted. */
  for (iterator = &start; iterator->next_free < block_to_insert; iterator = iterator->next_free)
  {

    /* Nothing to do here, just iterate to the right position. */
  }

  /* Do the block being inserted, and the block it is being inserted after
    make a contiguous block of memory? */
  puc = (uint8_t *)iterator;
  if ((puc + iterator->block_size) == (uint8_t *)block_to_insert)
  {
    iterator->block_size += block_to_insert->block_size;
    block_to_insert = iterator;
  }

  /* Do the block being inserted, and the block it is being inserted before
    make a contiguous block of memory? */
  puc = (uint8_t *)block_to_insert;
  if ((puc + block_to_insert->block_size) == (uint8_t *)iterator->next_free)
  {
    if (iterator->next_free != end)
    {
      /* Form one big block from the two blocks. */
      block_to_insert->block_size += iterator->next_free->block_size;
      block_to_insert->next_free = iterator->next_free->next_free;
    }
    else
    {
      block_to_insert->next_free = end;
    }
  }
  else
  {
    block_to_insert->next_free = iterator->next_free;
  }

  /* If the block being inserted plugged a gab, so was merged with the block
    before and the block after, then it's next_free pointer will have
    already been set, and should not be set here as that would make it point
    to itself. */
  if (iterator != block_to_insert)
  {
    iterator->next_free = block_to_insert;
  }
}

void heap_print_block(void)
{
  uint32_t block_num = 0;
  block_link_t *block;

  for (block = start.next_free; block != end; block = block->next_free)
  {
    block_num++;
    mem_printf("block num: %3d, block size: %d\n", block_num, block->block_size);
  }
  mem_printf("free: %4d  ever free: %d\n", heap_get_free(), heap_get_ever_free());
}

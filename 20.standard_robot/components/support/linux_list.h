
#ifndef _LINX_LIST_H
#define _LINX_LIST_H

/*
 * Simple doubly linked list implementation.
 *
 * Some of the internal functions ("__xxx") are useful when
 * manipulating whole lists rather than single entries, as
 * sometimes we already know the next/prev entries and we can
 * generate better code by using them directly rather than
 * using the generic single-entry routines.
 */

/*
 *    #define LIST_HEAD(name)
 *    
 *    static __inline void INIT_LIST_HEAD(list_t *list)
 *    static __inline void list_add(list_t *new, list_t *head)
 *    static __inline void list_add_tail(list_t *new, list_t *head)
 *    static __inline void list_del(list_t *entry)
 *    static __inline void list_del_init(list_t *entry)
 *    static __inline void list_replace(list_t *old, list_t *new)
 *    static __inline void list_move(list_t *list, list_t *head)
 *    static __inline void list_move_tail(list_t *list, list_t *head)
 *    static __inline int list_is_last(const list_t *list, const list_t *head)
 *    static __inline int list_empty(const list_t *head)
 *    static __inline void list_rotate_left(list_t *head)
 *    static __inline int list_is_singular(const list_t *head)
 *    static __inline void list_cut_position(list_t *list, list_t *head, list_t *entry)
 *    static __inline void list_splice(list_t *list, list_t *head)
 *    static __inline void list_splice_tail(list_t *list, list_t *head)
 *    static __inline void list_splice_init(list_t *list, list_t *head)
 *    static __inline void list_splice_tail_init(list_t *list, list_t *head)
 *    
 *    #define list_entry(ptr, type, member)
 *    #define list_first_entry(ptr, type, member) 
 *    #define list_last_entry(ptr, type, member)
 *    #define list_first_entry_or_null(ptr, type, member)
 *    #define list_for_each(pos, head)
 *    #define list_for_each_prev(pos, head)
 *    #define list_for_each_safe(pos, n, head)
 *    #define list_for_each_prev_safe(pos, n, head) 
 */

typedef struct _list_t
{
  struct _list_t *next, *prev;
} list_t;

#define LIST_HEAD_INIT(name) \
  {                          \
    &(name), &(name)         \
  }

#define LIST_HEAD(name) \
  list_t name = LIST_HEAD_INIT(name)

static __inline void INIT_LIST_HEAD(list_t *list)
{
  list->next = list;
  list->prev = list;
}

/*
 * Insert a new entry between two known consecutive entries.
 *
 * This is only for internal list manipulation where we know
 * the prev/next entries already!
 */
static __inline void __list_add(list_t *new,
                                list_t *prev,
                                list_t *next)
{
  next->prev = new;
  new->next = next;
  new->prev = prev;
  prev->next = new;
}

/**
 * list_add - add a new entry
 * @new: new entry to be added
 * @head: list head to add it after
 *
 * Insert a new entry after the specified head.
 * This is good for implementing stacks.
 */
static __inline void list_add(list_t *new, list_t *head)
{
  __list_add(new, head, head->next);
}

/**
 * list_add_tail - add a new entry
 * @new: new entry to be added
 * @head: list head to add it before
 *
 * Insert a new entry before the specified head.
 * This is useful for implementing queues.
 */
static __inline void list_add_tail(list_t *new, list_t *head)
{
  __list_add(new, head->prev, head);
}

/*
 * Delete a list entry by making the prev/next entries
 * point to each other.
 *
 * This is only for internal list manipulation where we know
 * the prev/next entries already!
 */
static __inline void __list_del(list_t *prev, list_t *next)
{
  next->prev = prev;
  prev->next = next;
}

/**
 * list_del - deletes entry from list.
 * @entry: the element to delete from the list.
 * Note: list_empty() on entry does not return true after this, the entry is
 * in an undefined state.
 */
static __inline void list_del(list_t *entry)
{
  __list_del(entry->prev, entry->next);
}

/**
 * list_del_init - deletes entry from list and reinitialize it.
 * @entry: the element to delete from the list.
 */
static __inline void list_del_init(list_t *entry)
{
  __list_del(entry->prev, entry->next);
  INIT_LIST_HEAD(entry);
}

/**
 * list_replace - replace old entry by new one
 * @old : the element to be replaced
 * @new : the new element to insert
 *
 * If @old was empty, it will be overwritten.
 */
static __inline void list_replace(list_t *old, list_t *new)
{
  new->next = old->next;
  new->next->prev = new;
  new->prev = old->prev;
  new->prev->next = new;
}

/**
 * list_move - delete from one list and add as another's head
 * @list: the entry to move
 * @head: the head that will precede our entry
 */
static __inline void list_move(list_t *list, list_t *head)
{
  __list_del(list->prev, list->next);
  list_add(list, head);
}

/**
 * list_move_tail - delete from one list and add as another's tail
 * @list: the entry to move
 * @head: the head that will follow our entry
 */
static __inline void list_move_tail(list_t *list,
                                    list_t *head)
{
  __list_del(list->prev, list->next);
  list_add_tail(list, head);
}

/**
 * list_is_last - tests whether @list is the last entry in list @head
 * @list: the entry to test
 * @head: the head of the list
 */
static __inline int list_is_last(const list_t *list,
                                 const list_t *head)
{
  return list->next == head;
}

/**
 * list_empty - tests whether a list is empty
 * @head: the list to test.
 */
static __inline int list_empty(const list_t *head)
{
  return head->next == head;
}

/**
 * list_rotate_left - rotate the list to the left
 * @head: the head of the list
 */
static __inline void list_rotate_left(list_t *head)
{
  list_t *first;

  if (!list_empty(head))
  {
    first = head->next;
    list_move_tail(first, head);
  }
}

/**
 * list_is_singular - tests whether a list has just one entry.
 * @head: the list to test.
 */
static __inline int list_is_singular(const list_t *head)
{
  return !list_empty(head) && (head->next == head->prev);
}

static __inline void __list_cut_position(list_t *list, list_t *head, list_t *entry)
{
  list_t *new_first = entry->next;
  list->next = head->next;
  list->next->prev = list;
  list->prev = entry;
  entry->next = list;
  head->next = new_first;
  new_first->prev = head;
}

/**
 * list_cut_position - cut a list into two
 * @list: a new list to add all removed entries
 * @head: a list with entries
 * @entry: an entry within head, could be the head itself
 *    and if so we won't cut the list
 *
 * This helper moves the iniial part of @head, up to and
 * including @entry, from @head to @list. You should
 * pass on @entry an element you know is on @head. @list
 * should be an empty list or a list you do not care about
 * losing its data.
 *
 */
static __inline void list_cut_position(list_t *list, list_t *head, list_t *entry)
{
  if (list_empty(head))
    return;
  if (list_is_singular(head) &&
      (head->next != entry && head != entry))
    return;
  if (entry == head)
    INIT_LIST_HEAD(list);
  else
    __list_cut_position(list, head, entry);
}

static __inline void __list_splice(const list_t *list, list_t *prev, list_t *next)
{
  list_t *first = list->next;
  list_t *last = list->prev;

  first->prev = prev;
  prev->next = first;

  last->next = next;
  next->prev = last;
}

/**
 * list_splice - join two lists
 * @list: the new list to add.
 * @head: the place to add it in the first list.
 */
static __inline void list_splice(list_t *list, list_t *head)
{
  if (!list_empty(list))
    __list_splice(list, head, head->next);
}

/**
 * list_splice_tail - join two lists, each list being a queue
 * @list: the new list to add.
 * @head: the place to add it in the first list.
 */
static __inline void list_splice_tail(list_t *list,
                                      list_t *head)
{
  if (!list_empty(list))
    __list_splice(list, head->prev, head);
}

/**
 * list_splice_init - join two lists and reinitialise the emptied list.
 * @list: the new list to add.
 * @head: the place to add it in the first list.
 *
 * The list at @list is reinitialised
 */
static __inline void list_splice_init(list_t *list, list_t *head)
{
  if (!list_empty(list))
  {
    __list_splice(list, head, head->next);
    INIT_LIST_HEAD(list);
  }
}

/**
 * list_splice_tail_init - join two lists and reinitialise the emptied list
 * @list: the new list to add.
 * @head: the place to add it in the first list.
 *
 * Each of the lists is a queue.
 * The list at @list is reinitialised
 */
static __inline void list_splice_tail_init(list_t *list,
                                           list_t *head)
{
  if (!list_empty(list))
  {
    __list_splice(list, head->prev, head);
    INIT_LIST_HEAD(list);
  }
}

/**
 * list_entry - get the struct for this entry
 * @ptr:    the &list_t pointer.
 * @type:    the type of the struct this is embedded in.
 * @member:    the name of the list_struct within the struct.
 */
#define list_entry(ptr, type, member) \
  ((type *)((char *)(ptr) - (unsigned long)(&((type *)0)->member)))

/**
 * list_first_entry - get the first element from a list
 * @ptr:    the list head to take the element from.
 * @type:    the type of the struct this is embedded in.
 * @member:    the name of the list_head within the struct.
 *
 * Note, that list is expected to be not empty.
 */
#define list_first_entry(ptr, type, member) \
  list_entry((ptr)->next, type, member)

/**
 * list_last_entry - get the last element from a list
 * @ptr:    the list head to take the element from.
 * @type:    the type of the struct this is embedded in.
 * @member:    the name of the list_head within the struct.
 *
 * Note, that list is expected to be not empty.
 */
#define list_last_entry(ptr, type, member) \
  list_entry((ptr)->prev, type, member)

/**
 * list_first_entry_or_null - get the first element from a list
 * @ptr:    the list head to take the element from.
 * @type:    the type of the struct this is embedded in.
 * @member:    the name of the list_head within the struct.
 *
 * Note that if the list is empty, it returns NULL.
 */
#define list_first_entry_or_null(ptr, type, member) ({      \
  list_t *head__ = (ptr);                                   \
  list_t *pos__ = (head__->next);                           \
  pos__ != head__ ? list_entry(pos__, type, member) : NULL; \
})

/**
 * list_for_each    -    iterate over a list
 * @pos:    the &list_t to use as a loop cursor.
 * @head:    the head for your list.
 *
 * This variant differs from list_for_each() in that it's the
 * simplest possible list iteration code, no prefetching is done.
 * Use this for code that knows the list to be very short (empty
 * or 1 entry) most of the time.
 */
#define list_for_each(pos, head) \
  for (pos = (head)->next; pos != (head); pos = pos->next)

/**
 * list_for_each_prev    -    iterate over a list backwards
 * @pos:    the &list_t to use as a loop counter.
 * @head:    the head for your list.
 */
#define list_for_each_prev(pos, head) \
  for (pos = (head)->prev; pos != (head); pos = pos->prev)

/**
 * list_for_each_safe    -    iterate over a list safe against removal of list entry
 * @pos:    the &list_t to use as a loop counter.
 * @n:        another &list_t to use as temporary storage
 * @head:    the head for your list.
 */
#define list_for_each_safe(pos, n, head)  \
  for (pos = (head)->next, n = pos->next; \
       pos != (head);                     \
       pos = n, n = pos->next)

/**
 * list_for_each_prev_safe - iterate over a list backwards safe against removal of list entry
 * @pos:    the &list_t to use as a loop cursor.
 * @n:        another &list_t to use as temporary storage
 * @head:    the head for your list.
 */
#define list_for_each_prev_safe(pos, n, head) \
  for (pos = (head)->prev, n = pos->prev;     \
       pos != (head);                         \
       pos = n, n = pos->prev)

#endif

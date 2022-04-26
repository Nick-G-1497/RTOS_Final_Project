#include <stdio.h>
#include <stdlib.h>
#ifndef __QUEUE__
#define __QUEUE__

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Structure which holds the event information
//----------------------------------------------------------------------------------------------------------------------------------
struct event_t {

    /// Button Event variable
    uint8_t button_event;
};

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Structure which holds the Queue's Node information
//----------------------------------------------------------------------------------------------------------------------------------
struct node_t {
    // event information
    struct event_t* event;

    // Pointer to the next node in the queue
    struct node_t* next;
};

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Creates a queue.
///
/// @param[in] event The event information
/// @param[in] size The size of the event array
///
/// @return the head of the new queue
//----------------------------------------------------------------------------------------------------------------------------------
struct node_t* create_queue(struct event_t* event, int size);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Create a new node for the queue
///
/// @param event The event information
///
/// @return a newly allocated event
//----------------------------------------------------------------------------------------------------------------------------------
struct node_t* create_new_node(uint8_t event);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Returns the top node in the queue
///
/// @param head The head of the queue
///
/// @return the event at the top of the queue
//----------------------------------------------------------------------------------------------------------------------------------
struct event_t* peek(struct node_t** head);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Removes the element at the top of the queue.
///
/// @param head The head of the queue.
//----------------------------------------------------------------------------------------------------------------------------------
void pop(struct node_t** head);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Push a new event into the queue
///
/// @param head The head of the queue
/// @param event The event to be put into the queue
//----------------------------------------------------------------------------------------------------------------------------------
void push(struct node_t** head, uint8_t event);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Determines whether the specified head is empty.
///
/// @param head The head of the Queue
///
/// @return True if the specified head is empty, False otherwise.
//----------------------------------------------------------------------------------------------------------------------------------
int is_empty(struct node_t** head);

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Remove all items from the queue
///
/// @param head The head of the queue
//----------------------------------------------------------------------------------------------------------------------------------
void empty_queue(struct node_t** head);

#endif // __QUEUE__

/*
 * queue.c
 *
 *  Created on: Mar 1, 2022
 *      Author: nickg
 */


#include <stdio.h>
#include <stdlib.h>
#include "queue.h"

struct node_t* create_queue(struct event_t* event, int size) {
    // Check that we don't have any erroneous input
    if ( (event == NULL) || (size == 0))
        return NULL;

    // Create the head of the queue
    int event_index = 0;
    struct node_t* prev_node;
    struct node_t* current_node = create_new_node(event);
    struct node_t* head = current_node;

    // Itterate through the list keeping track of two pointers
    while (event_index < (size - 1) )
    {
        event_index += 1;
        push(&head, &event[event_index]);
    }
    return head;
}

struct node_t* create_new_node(uint8_t event) {

    // Create a new node on the heap
    struct event_t* event_struct = malloc(sizeof(struct event_t));
    struct node_t* new_node = malloc(sizeof(struct node_t));
    event_struct->button_event = event;
    new_node->event = event_struct;
    new_node->next = NULL;

    // Return the pointer to the heap
    return new_node;
}

struct event_t* peek(struct node_t** head) {
    // look at the element at the head of the linked list
    return (*head)->event;
}

void pop(struct node_t** head) {

    if ( *head == NULL )
    {
        return;
    }

    struct node_t* old_head = *head;

    if (old_head->next == NULL)
    {
        *head = NULL;
    }

    struct node_t* new_head = old_head->next;

    *head = new_head;

    free(old_head->event);
    free(old_head);

}

void push(struct node_t** head, uint8_t event) {

    struct node_t* current_node = *head;

    if ( current_node == NULL )
    {
        *head = create_new_node(event);
        return;
    }

    while (current_node->next != NULL)
    {
        current_node = current_node->next;
    }

    struct node_t* new_node = create_new_node(event);
    current_node->next = new_node;

}

int is_empty(struct node_t** head) {
    // return 0 so it compiles
    struct node_t* head_node = *head;
    if (head_node == NULL)
        return 1;
    else
        return 0;
}

void empty_queue(struct node_t** head) {

    struct node_t* current_node = *head;
    struct node_t* next_node;

    while( current_node->next != NULL )
    {
        next_node = current_node->next;
        free(current_node);
        current_node = next_node;
    }

    free(current_node);
    *head = NULL;

}

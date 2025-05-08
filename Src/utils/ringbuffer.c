/*
 * ringbuffer.c
 *
 *  Created on: May 7, 2025
 *      Author: gaspr
 */

#include <stdint.h>
#include "utils/ringbuffer.h"

void ringbuf_init(ringbuf_t *c)
{
	c->head = 0;
	c->tail = 0;
}


int ringbuf_pop(ringbuf_t *c, uint8_t *data)
{
    int next;
    const uint8_t maxlen = RING_BUFFER_SIZE;

    if (c->head == c->tail)  // if the head == tail, we don't have any data
        return -1;

    next = c->tail + 1;  // next is where tail will point to after this read.
    if(next >= maxlen)
        next = 0;

    *data = c->buffer[c->tail];  // Read data and then move
    c->tail = next;              // tail to next offset.
    return 0;  // return success to indicate successful push.
}


int ringbuf_push(ringbuf_t *c, uint8_t data)
{
    int next;
    const uint8_t maxlen = RING_BUFFER_SIZE;

    next = c->head + 1;  // next is where head will point to after this write.
    if (next >= maxlen)
        next = 0;

    if (next == c->tail)  // if the head + 1 == tail, circular buffer is full
        return -1;

    c->buffer[c->head] = data;  // Load data and then move
    c->head = next;             // head to next data offset.
    return 0;  // return success to indicate successful push.
}

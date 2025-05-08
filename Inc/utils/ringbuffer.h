/*
 * ringbuffer.h
 *
 *  Created on: May 7, 2025
 *      Author: gaspr
 */

#ifndef UTILS_RINGBUFFER_H_
#define UTILS_RINGBUFFER_H_

#define RING_BUFFER_SIZE 	32

typedef struct {
    uint8_t buffer[RING_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;

} ringbuf_t;


void ringbuf_init(ringbuf_t *c);
int ringbuf_pop(ringbuf_t *c, uint8_t *data);
int ringbuf_push(ringbuf_t *c, uint8_t data);



#endif /* UTILS_RINGBUFFER_H_ */

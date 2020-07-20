/*
 * fifo.h
 *
 *  Created on: 15.07.2020
 *      Author: Andre Herms
 */

#ifndef TEENSY4_UTIL_FIFO_H_
#define TEENSY4_UTIL_FIFO_H_

#include <stdint.h>

#define FIFO_INIT(cap) (struct fifo_cnt){ cap, 0 ,0 }

struct fifo_cnt {
	size_t capacity;
	volatile size_t read_cnt;
	volatile size_t write_cnt;
};

static inline
size_t fifo_level(struct fifo_cnt *f) {
	return f->write_cnt - f->read_cnt;
}

static inline
void fifo_add(struct fifo_cnt *f, size_t num) {
	f->write_cnt += num;
}

static inline
void fifo_remove(struct fifo_cnt *f, size_t num) {
	f->read_cnt += num;
}

static inline
size_t fifo_read_index(struct fifo_cnt *f) {
	return f->read_cnt % f->capacity;
}

static inline
size_t fifo_write_index(struct fifo_cnt *f) {
	return f->write_cnt % f->capacity;
}



#endif /* TEENSY4_UTIL_FIFO_H_ */

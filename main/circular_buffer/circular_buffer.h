#ifndef _CBUFFER_
#define _CBUFFER_

typedef struct CircularBuffer {
    int read_pointer;
    int write_pointer;
    int counter;
    int size;
    float *values;
};

void init_buffer(struct CircularBuffer *queue, int size);
void free_buffer(struct CircularBuffer *buffer);
int buffer_elements(struct CircularBuffer *queue);
float get_element(struct CircularBuffer *queue);
void add_element(struct CircularBuffer *queue, float element);

#endif
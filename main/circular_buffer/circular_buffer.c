#include "freertos/FreeRTOS.h"
#include "circular_buffer.h"

void init_buffer(struct CircularBuffer *buffer, int size){
    buffer->size = size;
    buffer->values = (float *) malloc(buffer->size * sizeof(float));
    buffer->counter = 0;
    buffer->read_pointer = 0;
    buffer->write_pointer = 0;

    for(int i = 0; i < buffer->size; i++)
        buffer->values[i] = 0;
}

void free_buffer(struct CircularBuffer *buffer){
    free(buffer->values);
}

int buffer_elements(struct CircularBuffer *buffer){
    return buffer->counter;
}

float get_element(struct CircularBuffer *buffer){
    buffer->read_pointer = (buffer->read_pointer + 1) % buffer->counter;
    return buffer->values[buffer->read_pointer];
}

void add_element(struct CircularBuffer *buffer, float element){
    if(buffer->counter < buffer->size)
        buffer->counter++;

    buffer->values[buffer->write_pointer] = element;
    buffer->write_pointer = (buffer->write_pointer + 1) % buffer->size;

    // if the new element overwrite the elder one, then move the read pointer
    if(buffer->read_pointer > buffer->write_pointer || ((buffer->read_pointer == buffer->size-1) && (buffer->write_pointer == 0)))
        buffer->read_pointer = (buffer->read_pointer + 1) % buffer->counter;
}

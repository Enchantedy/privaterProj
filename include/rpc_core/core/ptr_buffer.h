#ifndef _PTR_BUFFER_H_
#define _PTR_BUFFER_H_
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    unsigned char* buffer;
	unsigned int buflen;
    unsigned int readpos;
    unsigned int writepos;
} ptr_buffer_t;

void ptr_buf_init(ptr_buffer_t *pb, unsigned char *buffer, unsigned int buflen);
void ptr_buf_reset(ptr_buffer_t *pb);

unsigned int ptr_buf_write(ptr_buffer_t *pb, const void *buf, unsigned int len);
unsigned int ptr_buf_read(ptr_buffer_t *pb, void *buf, unsigned int len);

void* ptr_buf_get_readptr(ptr_buffer_t *pb);
unsigned int ptr_buf_get_readable_len(ptr_buffer_t *pb);

void* ptr_buf_get_writeptr(ptr_buffer_t *pb);
void ptr_buf_add_write_pos(ptr_buffer_t *pb, unsigned int len);
unsigned int ptr_buf_get_writeable_len(ptr_buffer_t *pb);

#ifdef __cplusplus
}
#endif

#endif /* _PTR_BUFFER_H_ */

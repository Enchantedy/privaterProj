#ifndef _NETMSG_PACKER_H_
#define _NETMSG_PACKER_H_

#include <core/ptr_buffer.h>

#ifdef __cplusplus
extern "C" {
#endif

#define NETMSG_CONTINUE (-2)
#define NETMSG_FALSE (-1)

typedef struct
{
	unsigned int package_len;
	unsigned int package_header_len;
	unsigned int cmdid;
} netmsg_header_t;

void make_netmsg(int cmdid, const unsigned char *buf, int len, ptr_buffer_t *pbuf);
int unmake_netmsg(const void *pbuf, const unsigned int len, int *cmdid, unsigned int *package_len, ptr_buffer_t *pbuf_body);

void make_netmsg_new(int cmdid, const unsigned char *buf1, int len1, const unsigned char *buf2, int len2, ptr_buffer_t *pbuf);

#ifdef __cplusplus
}
#endif

#endif /* _NETMSG_PACKER_H_ */

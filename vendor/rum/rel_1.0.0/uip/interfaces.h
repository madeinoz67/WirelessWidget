#ifndef __tuip_INTERFACE_H__
#define __tuip_INTERFACE_H__

#include <stdint.h>

#define NUM_INTERFACES 2

extern uint8_t interface_number;

//#define tuip_interface_set(x)   interface_number = x; printf("Using interface: %d\n", x)
#define tuip_interface_set(x)   interface_number = x

#define INTERFACE_ETHERNET 0
#define INTERFACE_802154   1

#define INTERFACE_LIST {INTERFACE_ETHERNET, INTERFACE_802154 }
#define INTERFACE_LLADDR_SIZES {6, 2}

uint8_t tuip_panid[2]; /* PanID stored MSB LSB */

typedef struct {
	uint8_t panid[2]; /* MSB then LSB */
	uint8_t zeros[2]; /* 0, 0 */
	uint8_t shortaddr[2]; /* MSB then LSB */
} __attribute__((packed)) tuip_48bit_iid_t;

extern uint8_t  tuip_lladdr_len[NUM_INTERFACES];

#define UIP_LLADDR_LEN  tuip_lladdr_len[interface_number]


#endif

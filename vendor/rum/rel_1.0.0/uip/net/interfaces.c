
#include "../interfaces.h"
#include "config.h"

#if IPV6LOWPAN || defined(DOXYGEN)

uint8_t interface_number;

uint8_t tuip_lladdr_len[NUM_INTERFACES] = INTERFACE_LLADDR_SIZES;

#endif

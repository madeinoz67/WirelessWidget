#ifndef __RANDOM_H__
#define __RANDOM_H__

#include <stdlib.h>

/*
 * Initialize the pseudo-random generator.
 *
 */
#define random_init srand

/*
 * Calculate a pseudo random number between 0 and 65535.
 *
 * \return A pseudo-random number between 0 and 65535.
 */
#define random_rand rand

#define RANDOM_MAX 65535U

#endif /* __RANDOM_H__ */

#ifndef INV_DMP_UNCOMPRESS_H
#define INV_DMP_UNCOMPRESS_H

#define UNCOMPRESSED_DMP_CODE_SIZE (3062)
#define COMPRESSED_DMP_CODE_SIZE (2666)

#include <Arduino.h>

/* return 1 byte */
uint8_t inv_dmp_uncompress(void);

/* restart */
void inv_dmp_uncompress_reset(void);

#endif

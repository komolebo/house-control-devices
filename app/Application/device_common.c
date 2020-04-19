/*
 * device_common.c
 *
 *  Created on: 18 квіт. 2020 р.
 *      Author: Oleh
 */

#include <device_common.h>
#include <stdint.h>


const uint8_t DevicesName[DEVICE_COUNT][DEVINFO_STR_ATTR_LEN] =
{
    [DEVICE_MOTION]     = "MOTION",
    [DEVICE_SMOKE]      = "SMOKE",
    [DEVICE_GAS]        = "GAS",
};


/*********************************************************************
 * @fn     util_arrtohex
 *
 * @brief   Convert {0x01, 0x02} to "01:02"
 *
 * @param   src - source byte-array
 * @param   src_len - length of array
 * @param   dst - destination string-array
 * @param   dst_len - length of array
 *
 * @return  array as string
 */
char * util_arrtohex(uint8_t const *src, uint8_t src_len,
                     uint8_t *dst, uint8_t dst_len, uint8_t reverse)
{
    char hex[] = "0123456789ABCDEF";
    uint8_t *pStr = dst;
    uint8_t avail = dst_len - 1;
    int8_t inc = 1;
    if(reverse)
    {
        src = src + src_len - 1;
        inc = -1;
    }

    memset(dst, 0, avail);

    while(src_len && avail > 3)
    {
        if(avail < dst_len - 1)
        {
            *pStr++ = ':';
            avail -= 1;
        }

        *pStr++ = hex[*src >> 4];
        *pStr++ = hex[*src & 0x0F];
        src += inc;
        avail -= 2;
        src_len--;
    }

    if(src_len && avail)
    {
        *pStr++ = ':'; // Indicate not all data fit on line.
    }
    return((char *)dst);
}

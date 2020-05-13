/******************************************************************************

 @file  bim_onchip.c

 @brief This module contains the definitions for the supporting functionality of
        a Boot Image Manager.

 Group: WCS, BTS
 Target Device: cc2640r2

 ******************************************************************************
 
 Copyright (c) 2012-2020, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 
 
 *****************************************************************************/

/* -----------------------------------------------------------------------------
 *                                          Includes
 * -----------------------------------------------------------------------------
 */
#include "hal_flash.h"
#include "hal_types.h"
#include "oad_image_header.h"
#include "flash_interface.h"
#include "bim_util.h"
#include "bim_onchip.h"
#include "sign_util.h"
#include "oad.h"

/*******************************************************************************
 *                                          Constants
 */

#define DEFAULT_RAM             0
#define OAD_IMG_ID_LEN          8
#define BIM_VER                0x1     /* Supported BIM version */
#define META_VER               0x1     /* Supported metadata version */

#ifdef SECURITY

#define SHA_BUF_SZ             EFL_PAGE_SIZE

/*********************************************************************
 * LOCAL VARIABLES
 */

#if defined(__IAR_SYSTEMS_ICC__)
__no_init uint8_t shaBuf[SHA_BUF_SZ];
#elif defined(__TI_COMPILER_VERSION__)
uint8_t shaBuf[SHA_BUF_SZ];
#endif

/* Cert element stored in flash where public keys in Little endian format*/
#ifdef __TI_COMPILER_VERSION__
#pragma DATA_SECTION(_secureCertElement, ".cert_element")
#pragma RETAIN(_secureCertElement)
const certElement_t _secureCertElement =
#elif  defined(__IAR_SYSTEMS_ICC__)
#pragma location=".cert_element"
const certElement_t _secureCertElement @ ".cert_element" =
#endif
{
  .version    = SECURE_SIGN_TYPE,
  .len        = SECURE_CERT_LENGTH,
  .options    = SECURE_CERT_OPTIONS,
  .signerInfo = {0xb0,0x17,0x7d,0x51,0x1d,0xec,0x10,0x8b},
  .certPayload.eccKey.pubKeyX = {0xd8,0x51,0xbc,0xa2,0xed,0x3d,0x9e,0x19,0xb7,0x33,0xa5,0x2f,0x33,0xda,0x05,0x40,0x4d,0x13,0x76,0x50,0x3d,0x88,0xdf,0x5c,0xd0,0xe2,0xf2,0x58,0x30,0x53,0xc4,0x2a},
  .certPayload.eccKey.pubKeyY = {0xb9,0x2a,0xbe,0xef,0x66,0x5f,0xec,0xcf,0x56,0x16,0xcc,0x36,0xef,0x2d,0xc9,0x5e,0x46,0x2b,0x7c,0x3b,0x09,0xc1,0x99,0x56,0xd9,0xaf,0x95,0x81,0x63,0x23,0x7b,0xe7}
 };

uint8_t *secureHash;
int8_t internalVerifyStatus = 0;
uint8_t securityPresence = 0;
uint32_t eccWorkzone[SECURE_FW_ECC_NIST_P256_WORKZONE_LEN_IN_BYTES + SECURE_FW_ECC_BUF_TOTAL_LEN(SECURE_FW_ECC_NIST_P256_KEY_LEN_IN_BYTES)*5] = {0};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8_t Bim_signVerify(uint32_t startAddr);

#endif /* ifdef SECURITY */
/*******************************************************************************
 * @fn
 *
 * @brief       This function sets the value of local variable based on the
 *              content of RAM varaible bimVar
 *
 * @param       bimVar - RAM varaible.
 * @param       FN - pointer to local variable flash page number
 * @param       IT - pointer to local variable image type
 *
 * @return      None.
 */
void setSearchVar(uint16_t bimVar, uint8_t *FN, uint8_t *IT)
{
    if((bimVar == DEFAULT_RAM) || ((bimVar & 0xFF) > 1))
    {
        *FN = 0;
        *IT = 1;
    }
    else
    {
        if((bimVar & 0xFF) == 0)    /* Valid flash page number */
        {
            *FN = (bimVar & 0xFF00) >> 8;
            *IT = 0xFF;
        }
        else /* Valid Image Type */
        {
            *IT = (bimVar & 0xFF00) >> 8;
            *FN = 0xFF; /* start with flash page 0 */
        }
    }
}

/*******************************************************************************
 * @fn     copyFlashImage
 *
 * @brief  Copies firmware image into the executable flash area.
 *
 * @param  imgStart - starting address of image in external flash.
 * @param  imgLen   - size of image in bytes
 * @param  dstAddr  - destination address within internal flash.
 *
 * @return Zero when successful. Non-zero, otherwise.
 */
int8_t copyFlashImage(uint32_t imgStart, uint32_t imgLen, uint32_t dstAddr)
{
    if(!imgLen)
    {
        return(-1);
    }

    uint8_t page = (imgStart)?(FLASH_PAGE(imgStart)):0;
    uint8_t pageEnd = (imgLen)?(FLASH_PAGE(imgStart + imgLen - 1)):0;
    pageEnd += page;

    uint8_t dstPageStart = FLASH_PAGE(dstAddr);

    /* Change image size into word unit */
    imgLen += 3;
    imgLen >>= 2;

    if (dstAddr & 3)
    {
        /* Not an aligned address */
        return(-1);
    }

    if (page > pageEnd || pageEnd > 30)
    {
        return(-1);
    }

    for (; page <= pageEnd; page++)
    {
        uint32_t buf;
        uint16_t count = (HAL_FLASH_PAGE_SIZE / 4);

        if (page == pageEnd)
        {
            /* Count could be shorter */
            count = imgLen;
        }

        /* Erase destination flash page */
        eraseFlashPg(dstPageStart++);

        /* Erase the page */
        while (count-- > 0)
        {
            /* Read word from flash */
            readFlash(imgStart, (uint8_t *)&buf, 4);

            /* Write word to flash */
            writeFlash(dstAddr, (uint8_t *)&buf, 4);
            imgStart += 4;
            dstAddr += 4;
            imgLen--;
        }
    } /* for (; page <= pageEnd; page++) */
    return(0);
}

/*******************************************************************************
* @fn         checkForSec
*
*  @brief      Check for Security Payload. Reads through the headers in the .bin
*              file. If a security header is found the function checks to see if
*              the header has a populated payload.
*
*  @param       startAddr - The start address in internal flash of the binary image
*
*  @return      0  - security not found
*  @return      1  - security found
*
*/
#ifdef SECURITY
bool checkForSec(uint32_t startAddr, uint32_t imgLen)
{
    bool securityFound = false;
    uint8_t endOfSegment = 0;
    uint8_t segmentType = DEFAULT_STATE;
    uint32_t segmentLength = 0;
    uint32_t searchAddr =  startAddr+OAD_IMG_HDR_LEN;

    while(!endOfSegment)
    {
        readFlash(searchAddr, &segmentType, 1);

        if(segmentType == IMG_SECURITY_SEG_ID)
        {
            /* In this version of BIM, the security header will ALWAYS be present
               But the paylond will sometimes not be there. If this finds the
               header, the payload is also checked for existance. */
            searchAddr += SIG_OFFSET;
            uint32_t sigVal = 0;
            readFlash(searchAddr, (uint8_t *)&sigVal, sizeof(uint32_t));

            if(sigVal != 0) //Indicates the presence of a signature
            {
                endOfSegment = 1;
                securityFound = true;
            }
            else
            {
                break;
            }
        }
        else
        {
            searchAddr += SEG_LEN_OFFSET;
            if((searchAddr + sizeof(uint32_t)) > (startAddr + imgLen))
            {
                break;
            }
            readFlash(searchAddr, (uint8_t *)&segmentLength, sizeof(uint32_t));

            searchAddr += (segmentLength - SEG_LEN_OFFSET);
            if((searchAddr) > (startAddr + imgLen))
            {
                break;
            }
        }
    }

    return securityFound;
}


/*******************************************************************************
* @fn          verifyImage
*
*  @brief      Check for Security Payload. Reads through the headers in the .bin
*              file. If a security header is found the function checks to see if
*              the header has a populated payload.
*              Note: This api shouldn't be called from application space as it
*              can blow up system stack beacuse of ECC workzone space
*  @param      startAddr - The start address in internal flash of the binary image
*
*  @return      0  - security not found
*  @return      1  - security found
*
*/
uint8_t verifyImage(const uint32_t startAddr)
{
    uint8_t verifyStatus = FAIL;
    uint8_t readSecurityByte[SEC_VERIF_STAT_OFFSET + 1];

    /* Read in the header to check if the signature has already been denied */
    readFlash(startAddr, readSecurityByte, (SEC_VERIF_STAT_OFFSET + 1));

    if(readSecurityByte[SEC_VERIF_STAT_OFFSET] == DEFAULT_STATE)
    {
        verifyStatus = Bim_signVerify(startAddr);

        /* If the signature is invalid, mark the image as invalid */
        if((uint8_t)verifyStatus != SUCCESS)
        {
            readSecurityByte[SEC_VERIF_STAT_OFFSET] = VERIFY_FAIL;
            writeFlash((startAddr+SEC_VERIF_STAT_OFFSET),
                        &readSecurityByte[SEC_VERIF_STAT_OFFSET], 1);
        }
   }
   /* Note :: By default verification will be done every execution */
   return verifyStatus;
}

/*******************************************************************************
 * @fn      Bim_signVerify
 *
 * @brief   Verifies the image stored on external flash using ECDSA-SHA256
 *
 * @param   startAddr - image's flash address of the image to be verified.
 *
 * @return  Zero when successful. Non-zero, otherwise..
 */
static uint8_t Bim_signVerify(uint32_t startAddr)
{
    uint8_t headerBuf[129];
    uint8_t verifyStatus = FAIL;

    uint32_t *eccPayloadWorkzone = (uint32_t *)eccWorkzone;
    memset(eccPayloadWorkzone, 0, sizeof(eccWorkzone));

    /* Read in the image header to get the image signature */
    readFlash(startAddr, headerBuf, HDR_LEN_WITH_SECURITY_INFO);

    // First verify signerInfo
    verifyStatus = verifyCertElement(&headerBuf[SEG_SIGERINFO_OFFSET]);
    if(verifyStatus != SUCCESS)
    {
      return verifyStatus;
    }

    // Get the hash of the image
    uint8_t *dataHash = computeSha2Hash(startAddr, shaBuf, SHA_BUF_SZ, false);
    if(dataHash == NULL)
    {
      return FAIL;
    }
    

    // Create temp buffer used for ECDSA sign verify, it should 6*ECDSA_KEY_LEN
    uint8_t tempWorkzone[ECDSA_SHA_TEMPWORKZONE_LEN];

    // Verify the hash
    verifyStatus = bimVerifyImage_ecc(_secureCertElement.certPayload.eccKey.pubKeyX,
                                      _secureCertElement.certPayload.eccKey.pubKeyY,
                                       dataHash,
                                       &headerBuf[SEG_SIGNR_OFFSET],
                                       &headerBuf[SEG_SIGNS_OFFSET],
                                       eccPayloadWorkzone,
                                       tempWorkzone);

    if(verifyStatus == SECURE_FW_ECC_STATUS_VALID_SIGNATURE)
    {
       verifyStatus = SUCCESS;
    }
    return verifyStatus;
}
#endif /* if defined(SECURITY) */

/**************************************************************************************************
*/

#include "sign_util.h"
#include "flash_interface.h"
#include "ext_flash.h"
uint8_t finalHash[ECDSA_KEY_LEN] = {0};

extern const certElement_t _secureCertElement;

/*********************************************************************
 * GLOBAL FUNCTION REFERENCES
 ********************************************************************/
extern uint8_t eccRom_verifyHash(uint32_t *, uint32_t *, uint32_t *, uint32_t *,
                                 uint32_t *);

void eccInit(ECCROMCC26XX_Params *pParams)
{
    /* Initialize Curve to NIST P-256 with window size 3 by default */
    pParams->curve.keyLen      = SECURE_FW_ECC_NIST_P256_KEY_LEN_IN_BYTES;
    pParams->curve.workzoneLen = SECURE_FW_ECC_NIST_P256_WORKZONE_LEN_IN_BYTES;
    pParams->curve.windowSize  = SECURE_FW_ECC_WINDOW_SIZE;
    pParams->curve.param_p     = &NIST_Curve_P256_p;
    pParams->curve.param_r     = &NIST_Curve_P256_r;
    pParams->curve.param_a     = &NIST_Curve_P256_a;
    pParams->curve.param_b     = &NIST_Curve_P256_b;
    pParams->curve.param_gx    = &NIST_Curve_P256_Gx;
    pParams->curve.param_gy    = &NIST_Curve_P256_Gy;
}

/*********************************************************************
 * @fn         initEccGlobals
 * @brief      Initializa global variables needed for ECC verify operation
 *
 * @param      pCurve - pointer to curve parameters for the curve used for ECC
 *                      operations
 */
static void initEccGlobals(ECCROMCC26XX_CurveParams *pCurve)
{
  /* Store client parameters into ECC ROM parameters */
  eccRom_param_p  = pCurve->param_p;
  eccRom_param_r  = pCurve->param_r;
  eccRom_param_a  = pCurve->param_a;
  eccRom_param_b  = pCurve->param_b;
  eccRom_param_Gx = pCurve->param_gx;
  eccRom_param_Gy = pCurve->param_gy;

  /* Initialize window size */
  eccRom_windowSize = pCurve->windowSize;
}

/*********************************************************************
 * @fn         reverseOrder
 * @brief      Reverse the byte order and copy to output buffer
 *
 * @param      pBufIn - pointer to input buffer
 * @param      pBufOut - pointer to output buffer
 */
static void reverseOrder(const uint8_t *pBufIn,uint8_t *pBufOut)
{
  uint8_t i=0;
  for(i=0;i<SECURE_FW_SIGN_LEN;i++)
  {
    pBufOut[i] = pBufIn[SECURE_FW_SIGN_LEN-1-i];
  }
}

/*********************************************************************
 * @fn         copyBytes
 * @brief      Copy data between memory locatins
 *
 * @param      pDst - pointer to destination buffer
 * @param      pSrc - pointer to source buffer
 * @param      len  - length of data to be copied
 */
static void copyBytes(uint8_t *pDst, const uint8_t *pSrc, uint32_t len)
{
  uint32_t i;
  for(i=0; i<len; i++)
    pDst[i]=pSrc[i];
}

/*!
 Utility function to compare the content of two memory locations

 Public function defined in secure_fw.h
 */
/*********************************************************************
 * @fn         compareBytes
 * @brief      Compare the content of two memory locations
 *
 * @param      pData1 - pointer to first memory location
 * @param      pData2 - pointer to second memory location
 * @param      len  - length of data to be compared
 */

int compareBytes(uint8_t *pData1, const uint8_t *pData2, uint8_t len)
{
  uint8_t i;
  for(i=0; i<len; i++) if(pData1[i]!=pData2[i]) return (1);
  return (0);
}

/*!
 Check the validity of cert element

 Public function defined in secure_fw.h
 */
uint8_t verifyCertElement(uint8_t *signerInfo)
{
  /* read type in sign element and compare with type in cert element */
  return compareBytes(signerInfo,
                               _secureCertElement.signerInfo,8);
}

/**
* @brief Check for Security Payload
*
*  Reads through the headers in the .bin file. If a security header is found
*  the function checks to see if the header has a populated payload.
*
*
*  @param       eFlStrAddr - The start address in external flash of the binary image
*
*  @return      0  - security not found
*  @return      1  - security found
*
*/
int8_t  bimVerifyImage_ecc(const uint8_t *publicKeyX, const uint8_t *publicKeyY,
                           uint8_t *hash, uint8_t *sign1, uint8_t *sign2, uint32_t *eccWorkzone,
                           uint8_t *tempWorkzone)
{
    uint8_t *publicKeyXBuf;
    uint8_t *publicKeyYBuf;
    uint8_t *hashBuf;
    uint8_t *sign1Buf;
    uint8_t *sign2Buf;

    // Variables to be allocated on the tempworkzone,
    /* Split allocated memory into buffers */
    uint8_t *reversedHash = tempWorkzone;
    uint8_t *reversedPubKeyX = reversedHash + ECDSA_KEY_LEN;
    uint8_t *reversedPubKeyY = reversedPubKeyX + ECDSA_KEY_LEN;
    uint8_t *reversedSign1 = reversedPubKeyY + ECDSA_KEY_LEN;
    uint8_t *reversedSign2 = reversedSign1 + ECDSA_KEY_LEN;

    ECCROMCC26XX_Params params;
    eccInit(&params);
    reverseOrder(hash, reversedHash);
    reverseOrder(publicKeyX, reversedPubKeyX);
    reverseOrder(publicKeyY, reversedPubKeyY);
    reverseOrder(sign1, reversedSign1);
    reverseOrder(sign2, reversedSign2);

    /*total memory for operation: workzone and 5 key buffers*/
    eccRom_workzone = &eccWorkzone[0];

    /* Split allocated memory into buffers */
    publicKeyXBuf = (uint8_t *)eccRom_workzone +
                 SECURE_FW_ECC_NIST_P256_WORKZONE_LEN_IN_BYTES;
    publicKeyYBuf = publicKeyXBuf +
                 SECURE_FW_ECC_BUF_TOTAL_LEN(SECURE_FW_ECC_NIST_P256_KEY_LEN_IN_BYTES);
    hashBuf =  publicKeyYBuf +
               SECURE_FW_ECC_BUF_TOTAL_LEN(SECURE_FW_ECC_NIST_P256_KEY_LEN_IN_BYTES);
    sign1Buf  = hashBuf +
             SECURE_FW_ECC_BUF_TOTAL_LEN(SECURE_FW_ECC_NIST_P256_KEY_LEN_IN_BYTES);
    sign2Buf  = sign1Buf +
             SECURE_FW_ECC_BUF_TOTAL_LEN(SECURE_FW_ECC_NIST_P256_KEY_LEN_IN_BYTES);


    initEccGlobals(&params.curve);

    /* Set length of keys in words in the first word of each buffer*/
    *((uint32_t *)&publicKeyXBuf[SECURE_FW_ECC_KEY_LEN_OFFSET]) =
      (uint32_t)(SECURE_FW_ECC_UINT32_BLK_LEN(SECURE_FW_ECC_NIST_P256_KEY_LEN_IN_BYTES));

    *((uint32_t *)&publicKeyYBuf[SECURE_FW_ECC_KEY_LEN_OFFSET]) =
     (uint32_t)(SECURE_FW_ECC_UINT32_BLK_LEN(SECURE_FW_ECC_NIST_P256_KEY_LEN_IN_BYTES));

    *((uint32_t *)&hashBuf[SECURE_FW_ECC_KEY_LEN_OFFSET]) =
      (uint32_t)(SECURE_FW_ECC_UINT32_BLK_LEN(SECURE_FW_ECC_NIST_P256_KEY_LEN_IN_BYTES));

    *((uint32_t *)&sign1Buf[SECURE_FW_ECC_KEY_LEN_OFFSET]) =
      (uint32_t)(SECURE_FW_ECC_UINT32_BLK_LEN(SECURE_FW_ECC_NIST_P256_KEY_LEN_IN_BYTES));

    *((uint32_t *)&sign2Buf[SECURE_FW_ECC_KEY_LEN_OFFSET]) =
      (uint32_t)(SECURE_FW_ECC_UINT32_BLK_LEN(SECURE_FW_ECC_NIST_P256_KEY_LEN_IN_BYTES));

    /* Copy input key into buffer */
    copyBytes( publicKeyXBuf + SECURE_FW_ECC_KEY_OFFSET,
               reversedPubKeyX,
               SECURE_FW_ECC_NIST_P256_KEY_LEN_IN_BYTES);
    copyBytes( publicKeyYBuf + SECURE_FW_ECC_KEY_OFFSET,
               reversedPubKeyY,
               SECURE_FW_ECC_NIST_P256_KEY_LEN_IN_BYTES);
     /* copy hash into buffer */
    copyBytes( hashBuf + SECURE_FW_ECC_KEY_OFFSET,reversedHash,
              SECURE_FW_ECC_NIST_P256_KEY_LEN_IN_BYTES);

    copyBytes( sign1Buf + SECURE_FW_ECC_KEY_OFFSET,
               reversedSign1,
               SECURE_FW_ECC_NIST_P256_KEY_LEN_IN_BYTES);
    copyBytes( sign2Buf + SECURE_FW_ECC_KEY_OFFSET,
               reversedSign2,
               SECURE_FW_ECC_NIST_P256_KEY_LEN_IN_BYTES);

    int8_t status = eccRom_verifyHash((uint32_t *)publicKeyXBuf,
                             (uint32_t *)publicKeyYBuf,
                             (uint32_t *)hashBuf,
                             (uint32_t *)sign1Buf,
                             (uint32_t *)sign2Buf);

    return status;
}

#ifdef SECURITY /* The functions below are are only used in performing Secure OAD */

#ifdef OAD_ONCHIP
/*********************************************************************
 * @fn         computeSha2Hash
 * @brief      Computes SHA256 hash
 *
 * @param      imgStartAddr - start address of the image binary
 * @param      useExtFl - is image binary is stored on external flash
 * @param      finalHash - pointer to output buffer containing computed hash value
 *
 * @return     None
 */
uint8_t *computeSha2Hash(uint32_t imgStartAddr, uint8_t *SHABuff, uint16_t SHABuffLen, bool useExtFl)
{
    imgHdr_t *pImgHdr;

    /* Read first page of the image into the buffer. */
    readFlash((uint8_t)(imgStartAddr), SHABuff, SHABuffLen); //(shaBuf, (uint8_t *)(imgStartAddr), SHA_BUF_SZ);
    pImgHdr = (imgHdr_t *)(imgStartAddr);

    //pImgHdr->fixedHdr.len = pImgHdr->fixedHdr.imgEndAddr - pImgHdr->imgPayload.startAddr +1;
    uint32_t addrRead = imgStartAddr + SHABuffLen;
    uint32_t secHdrLen = HDR_LEN_WITH_SECURITY_INFO;

    SHA256_memory_t workzone;
    SHA2CC26XX_initialize(&workzone);

    SHA2CC26XX_execute(&workzone, &SHABuff[12], 4); //Start after the ID + CRC and go until CRC Status
    SHA2CC26XX_execute(&workzone, &SHABuff[18], 47); //Start after CRC status and go to signature
    SHA2CC26XX_execute(&workzone, &SHABuff[secHdrLen], (SHABuffLen - secHdrLen));

    uint32_t imgLengthLeft = pImgHdr->fixedHdr.len - SHABuffLen;
    uint32_t byteToRead = SHABuffLen;

    /* Read over image pages. */
    while(imgLengthLeft > 0)
    {
        /* Read data into the next buffer */
        memCpy(SHABuff, (uint8_t *)addrRead, byteToRead);
        SHA2CC26XX_execute(&workzone, SHABuff, byteToRead);

        imgLengthLeft -= byteToRead;
        if(imgLengthLeft > SHABuffLen)
            byteToRead = SHABuffLen;
        else
            byteToRead = imgLengthLeft;

        addrRead += SHABuffLen;
    } /* while(imgLengthLeft > 0) */

    SHA2CC26XX_output(&workzone, finalHash);
    return(finalHash);
}
#else
/*********************************************************************
 * @fn         computeSha2Hash
 * @brief      Computes SHA256 hash
 *
 * @param      imgStartAddr - start address of the image binary
 * @param      useExtFl - is image binary is stored on external flash
 *
 * @return     pointer to output buffer containing computed hash value
 */
uint8_t *computeSha2Hash(uint32_t imgStartAddr, uint8_t *SHABuff, uint16_t SHABuffLen, bool useExtFl)
{
    imgHdr_t *pImgHdr;

    /* Read first page of the image into the buffer. */
    if(!useExtFl)
    {
        memCpy(SHABuff, (uint8_t *)(imgStartAddr), SHABuffLen);
    }
    else
    {
        extFlashRead(imgStartAddr, SHABuffLen, SHABuff);
    }

    pImgHdr = (imgHdr_t *)(SHABuff);

    if(pImgHdr->fixedHdr.len  == 0)
    {
        return NULL;
    }
    uint32_t addrRead = imgStartAddr + SHABuffLen;
    uint32_t secHdrLen = HDR_LEN_WITH_SECURITY_INFO;

    SHA256_memory_t workzone;
    SHA2CC26XX_initialize(&workzone);

    SHA2CC26XX_execute(&workzone, &SHABuff[12], 4); //Start after the ID + CRC and go until CRC Status
    SHA2CC26XX_execute(&workzone, &SHABuff[18], 47); //Start after CRC status and go to signature
    SHA2CC26XX_execute(&workzone, &SHABuff[secHdrLen], SHABuffLen - secHdrLen);
    uint32_t imgLengthLeft = pImgHdr->fixedHdr.len - SHABuffLen;
    uint32_t byteToRead = SHABuffLen;

    /* Read over image pages. */
    while(imgLengthLeft > 0)
    {
        /* Read data into the next buffer */
        if(!useExtFl)
        {
            memCpy(SHABuff, (uint8_t *)addrRead, byteToRead);
        }
        else
        {
            extFlashRead(addrRead, byteToRead, SHABuff);
        }

        SHA2CC26XX_execute(&workzone, SHABuff, byteToRead);

        imgLengthLeft -= byteToRead;
        if(imgLengthLeft > SHABuffLen)
            byteToRead = SHABuffLen;
        else
            byteToRead = imgLengthLeft;

        addrRead += SHABuffLen;
    } /* while(imgLengthLeft > 0) */

    SHA2CC26XX_output(&workzone, finalHash);
    return(finalHash);
}
#endif /* ifdef OAD_ONCHIP */

#endif /*#ifdef SECURITY */

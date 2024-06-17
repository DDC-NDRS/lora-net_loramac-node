/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech
 ___ _____ _   ___ _  _____ ___  ___  ___ ___
/ __|_   _/_\ / __| |/ / __/ _ \| _ \/ __| __|
\__ \ | |/ _ \ (__| ' <| _| (_) |   / (__| _|
|___/ |_/_/ \_\___|_|\_\_| \___/|_|_\\___|___|
embedded.connectivity.solutions===============

Description: LoRa MAC layer message parser functionality implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis ( Semtech ), Gregory Cristian ( Semtech ),
            Daniel Jaeckle ( STACKFORCE ),  Johannes Bruder ( STACKFORCE )
*/
#include "LoRaMacParser.h"
#include "utilities.h"

LoRaMacParserStatus_t LoRaMacParserJoinAccept(LoRaMacMessageJoinAccept_t* macMsg) {
    if ((macMsg == 0) || (macMsg->Buffer == 0)) {
        return (LORAMAC_PARSER_ERROR_NPE);
    }

    uint8_t* Buffer = macMsg->Buffer;
    uint16_t bufItr = 0;

    macMsg->MHDR.Value = Buffer[bufItr++];

    memcpy1(macMsg->JoinNonce, &Buffer[bufItr], 3);
    bufItr = bufItr + 3;

    memcpy1(macMsg->NetID, &Buffer[bufItr], 3);
    bufItr = bufItr + 3;

    uint32_t DevAddr;
    DevAddr  =  (uint32_t)Buffer[bufItr++];
    DevAddr |= ((uint32_t)Buffer[bufItr++] << 8);
    DevAddr |= ((uint32_t)Buffer[bufItr++] << 16);
    DevAddr |= ((uint32_t)Buffer[bufItr++] << 24);
    macMsg->DevAddr = DevAddr;

    macMsg->DLSettings.Value = Buffer[bufItr++];

    macMsg->RxDelay = Buffer[bufItr++];

    uint8_t BufSize;

    BufSize = macMsg->BufSize;
    if ((BufSize - LORAMAC_MIC_FIELD_SIZE - bufItr) == LORAMAC_CF_LIST_FIELD_SIZE) {
        memcpy1(macMsg->CFList, &Buffer[bufItr], LORAMAC_CF_LIST_FIELD_SIZE);
        bufItr = bufItr + LORAMAC_CF_LIST_FIELD_SIZE;
    }
    else if ((BufSize - LORAMAC_MIC_FIELD_SIZE - bufItr) > 0) {
        return (LORAMAC_PARSER_FAIL);
    }

    uint32_t MIC;
    MIC =   (uint32_t)Buffer[bufItr++];
    MIC |= ((uint32_t)Buffer[bufItr++] <<  8);
    MIC |= ((uint32_t)Buffer[bufItr++] << 16);
    MIC |= ((uint32_t)Buffer[bufItr++] << 24);
    macMsg->MIC = MIC;

    return (LORAMAC_PARSER_SUCCESS);
}

LoRaMacParserStatus_t LoRaMacParserData(LoRaMacMessageData_t* macMsg) {
    if ((macMsg == 0) || (macMsg->Buffer == 0)) {
        return (LORAMAC_PARSER_ERROR_NPE);
    }

    uint8_t* Buffer = macMsg->Buffer;
    uint16_t bufItr = 0;

    macMsg->MHDR.Value = Buffer[bufItr++];

    uint32_t DevAddr;
    DevAddr  = Buffer[bufItr++];
    DevAddr |= ((uint32_t)Buffer[bufItr++] <<  8);
    DevAddr |= ((uint32_t)Buffer[bufItr++] << 16);
    DevAddr |= ((uint32_t)Buffer[bufItr++] << 24);
    macMsg->FHDR.DevAddr = DevAddr;

    macMsg->FHDR.FCtrl.Value = Buffer[bufItr++];

    uint16_t FCnt;
    FCnt  = Buffer[bufItr++];
    FCnt |= Buffer[bufItr++] << 8;
    macMsg->FHDR.FCnt = FCnt;

    memcpy1(macMsg->FHDR.FOpts, &Buffer[bufItr], macMsg->FHDR.FCtrl.Bits.FOptsLen);
    bufItr = bufItr + macMsg->FHDR.FCtrl.Bits.FOptsLen;

    // Initialize anyway with zero.
    macMsg->FPort = 0;
    macMsg->FRMPayloadSize = 0;

    uint8_t BufSize;

    BufSize = macMsg->BufSize;
    if ((BufSize - bufItr - LORAMAC_MIC_FIELD_SIZE) > 0) {
        macMsg->FPort = Buffer[bufItr++];

        macMsg->FRMPayloadSize = (BufSize - bufItr - LORAMAC_MIC_FIELD_SIZE);
        memcpy1(macMsg->FRMPayload, &Buffer[bufItr], macMsg->FRMPayloadSize);
        bufItr = bufItr + macMsg->FRMPayloadSize;
    }

    uint32_t MIC;
    MIC  =  (uint32_t)Buffer[(BufSize - LORAMAC_MIC_FIELD_SIZE)];
    MIC |= ((uint32_t)Buffer[(BufSize - LORAMAC_MIC_FIELD_SIZE) + 1] <<  8);
    MIC |= ((uint32_t)Buffer[(BufSize - LORAMAC_MIC_FIELD_SIZE) + 2] << 16);
    MIC |= ((uint32_t)Buffer[(BufSize - LORAMAC_MIC_FIELD_SIZE) + 3] << 24);
    macMsg->MIC = MIC;

    return (LORAMAC_PARSER_SUCCESS);
}

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

#define JOIN_ACCEPT_RX_DELAY_OFFSET         13

LoRaMacParserStatus_t LoRaMacParserJoinAccept(LoRaMacMessageJoinAccept_t* macMsg) {
    if ((macMsg == 0) || (macMsg->Buffer == 0)) {
        return (LORAMAC_PARSER_ERROR_NPE);
    }

    uint8_t* Buffer = macMsg->Buffer;
    uint16_t bufItr;

    macMsg->MHDR.Value = Buffer[0];

    memcpy1(macMsg->JoinNonce, &Buffer[1], 3);

    memcpy1(macMsg->NetID, &Buffer[4], 3);

    uint32_t DevAddr;
    DevAddr  =  (uint32_t)Buffer[7];
    DevAddr |= ((uint32_t)Buffer[8]  << 8);
    DevAddr |= ((uint32_t)Buffer[9]  << 16);
    DevAddr |= ((uint32_t)Buffer[10] << 24);
    macMsg->DevAddr = DevAddr;

    macMsg->DLSettings.Value = Buffer[11];

    macMsg->RxDelay = Buffer[12];
    bufItr = JOIN_ACCEPT_RX_DELAY_OFFSET;

    uint8_t BufSize;

    BufSize = macMsg->BufSize;
    if ((BufSize - LORAMAC_MIC_FIELD_SIZE - JOIN_ACCEPT_RX_DELAY_OFFSET) == LORAMAC_CF_LIST_FIELD_SIZE) {
        memcpy1(macMsg->CFList, &Buffer[JOIN_ACCEPT_RX_DELAY_OFFSET], LORAMAC_CF_LIST_FIELD_SIZE);
        bufItr = JOIN_ACCEPT_RX_DELAY_OFFSET + LORAMAC_CF_LIST_FIELD_SIZE;
    }
    else if ((BufSize - LORAMAC_MIC_FIELD_SIZE - JOIN_ACCEPT_RX_DELAY_OFFSET) > 0) {
        return (LORAMAC_PARSER_FAIL);
    }

    uint32_t MIC;
    MIC =   (uint32_t)Buffer[bufItr];
    MIC |= ((uint32_t)Buffer[bufItr + 1] <<  8);
    MIC |= ((uint32_t)Buffer[bufItr + 2] << 16);
    MIC |= ((uint32_t)Buffer[bufItr + 3] << 24);
    macMsg->MIC = MIC;

    return (LORAMAC_PARSER_SUCCESS);
}

LoRaMacParserStatus_t LoRaMacParserData(LoRaMacMessageData_t* macMsg) {
    if ((macMsg == 0) || (macMsg->Buffer == 0)) {
        return (LORAMAC_PARSER_ERROR_NPE);
    }

    uint8_t* Buffer = macMsg->Buffer;
    LoRaMacFrameHeader_t* FHDR = &macMsg->FHDR;
    uint16_t bufItr;

    macMsg->MHDR.Value = Buffer[0];

    uint32_t DevAddr;
    DevAddr  = Buffer[1];
    DevAddr |= ((uint32_t)Buffer[2] <<  8);
    DevAddr |= ((uint32_t)Buffer[3] << 16);
    DevAddr |= ((uint32_t)Buffer[4] << 24);
    FHDR->DevAddr = DevAddr;

    FHDR->FCtrl.Value = Buffer[5];

    uint16_t FCnt;
    FCnt  = Buffer[6];
    FCnt |= Buffer[7] << 8;
    FHDR->FCnt = FCnt;

    memcpy1(FHDR->FOpts, &Buffer[8], FHDR->FCtrl.Bits.FOptsLen);
    bufItr = (8 + FHDR->FCtrl.Bits.FOptsLen);

    // Initialize anyway with zero.
    macMsg->FPort = 0;
    macMsg->FRMPayloadSize = 0;

    uint8_t BufSize;

    BufSize = macMsg->BufSize;
    if ((BufSize - bufItr - LORAMAC_MIC_FIELD_SIZE) > 0) {
        macMsg->FPort = Buffer[bufItr];

        macMsg->FRMPayloadSize = (BufSize - (bufItr + 1) - LORAMAC_MIC_FIELD_SIZE);
        memcpy1(macMsg->FRMPayload, &Buffer[(bufItr + 1)], macMsg->FRMPayloadSize);
    }

    uint32_t MIC;
    MIC  =  (uint32_t)Buffer[(BufSize - LORAMAC_MIC_FIELD_SIZE)];
    MIC |= ((uint32_t)Buffer[(BufSize - LORAMAC_MIC_FIELD_SIZE) + 1] <<  8);
    MIC |= ((uint32_t)Buffer[(BufSize - LORAMAC_MIC_FIELD_SIZE) + 2] << 16);
    MIC |= ((uint32_t)Buffer[(BufSize - LORAMAC_MIC_FIELD_SIZE) + 3] << 24);
    macMsg->MIC = MIC;

    return (LORAMAC_PARSER_SUCCESS);
}

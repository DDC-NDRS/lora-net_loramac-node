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

Description: LoRa MAC layer message serializer functionality implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis ( Semtech ), Gregory Cristian ( Semtech ),
            Daniel Jaeckle ( STACKFORCE ),  Johannes Bruder ( STACKFORCE )
*/
#include "LoRaMacSerializer.h"
#include "utilities.h"

LoRaMacSerializerStatus_t LoRaMacSerializerJoinRequest( LoRaMacMessageJoinRequest_t* macMsg )
{
    if( ( macMsg == 0 ) || ( macMsg->Buffer == 0 ) )
    {
        return LORAMAC_SERIALIZER_ERROR_NPE;
    }

    uint8_t* Buffer = macMsg->Buffer;

    // Check macMsg->BufSize
    if( macMsg->BufSize < LORAMAC_JOIN_REQ_MSG_SIZE )
    {
        return LORAMAC_SERIALIZER_ERROR_BUF_SIZE;
    }

    Buffer[0] = macMsg->MHDR.Value;

    memcpyr( &Buffer[1], macMsg->JoinEUI, LORAMAC_JOIN_EUI_FIELD_SIZE );

    memcpyr( &Buffer[9], macMsg->DevEUI, LORAMAC_DEV_EUI_FIELD_SIZE );

    Buffer[17] = macMsg->DevNonce & 0xFF;
    Buffer[18] = ( macMsg->DevNonce >> 8 ) & 0xFF;

    Buffer[19] = macMsg->MIC & 0xFF;
    Buffer[20] = ( macMsg->MIC >> 8 ) & 0xFF;
    Buffer[21] = ( macMsg->MIC >> 16 ) & 0xFF;
    Buffer[22] = ( macMsg->MIC >> 24 ) & 0xFF;

    macMsg->BufSize = 23;                   // Equal to LORAMAC_JOIN_REQ_MSG_SIZE

    return LORAMAC_SERIALIZER_SUCCESS;
}

LoRaMacSerializerStatus_t LoRaMacSerializerReJoinType1( LoRaMacMessageReJoinType1_t* macMsg )
{
    if( ( macMsg == 0 ) || ( macMsg->Buffer == 0 ) )
    {
        return LORAMAC_SERIALIZER_ERROR_NPE;
    }

    uint8_t* Buffer = macMsg->Buffer;

    // Check macMsg->BufSize
    if( macMsg->BufSize < LORAMAC_RE_JOIN_1_MSG_SIZE )
    {
        return LORAMAC_SERIALIZER_ERROR_BUF_SIZE;
    }

    Buffer[0] = macMsg->MHDR.Value;
    Buffer[1] = macMsg->ReJoinType;

    memcpyr( &Buffer[2], macMsg->JoinEUI, LORAMAC_JOIN_EUI_FIELD_SIZE );

    memcpyr( &Buffer[10], macMsg->DevEUI, LORAMAC_DEV_EUI_FIELD_SIZE );

    Buffer[18] = macMsg->RJcount1 & 0xFF;
    Buffer[19] = ( macMsg->RJcount1 >> 8 ) & 0xFF;

    Buffer[20] = macMsg->MIC & 0xFF;
    Buffer[21] = ( macMsg->MIC >> 8 ) & 0xFF;
    Buffer[22] = ( macMsg->MIC >> 16 ) & 0xFF;
    Buffer[23] = ( macMsg->MIC >> 24 ) & 0xFF;

    macMsg->BufSize = 24;                   // Equal to LORAMAC_RE_JOIN_1_MSG_SIZE

    return LORAMAC_SERIALIZER_SUCCESS;
}

LoRaMacSerializerStatus_t LoRaMacSerializerReJoinType0or2( LoRaMacMessageReJoinType0or2_t* macMsg )
{
    if( ( macMsg == 0 ) || ( macMsg->Buffer == 0 ) )
    {
        return LORAMAC_SERIALIZER_ERROR_NPE;
    }

    uint8_t* Buffer = macMsg->Buffer;

    // Check macMsg->BufSize
    if( macMsg->BufSize < LORAMAC_RE_JOIN_0_2_MSG_SIZE )
    {
        return LORAMAC_SERIALIZER_ERROR_BUF_SIZE;
    }

    Buffer[0] = macMsg->MHDR.Value;
    Buffer[1] = macMsg->ReJoinType;

    memcpy1( &Buffer[2], macMsg->NetID, LORAMAC_NET_ID_FIELD_SIZE );

    memcpyr( &Buffer[5], macMsg->DevEUI, LORAMAC_DEV_EUI_FIELD_SIZE );

    Buffer[13] = macMsg->RJcount0 & 0xFF;
    Buffer[14] = ( macMsg->RJcount0 >> 8 ) & 0xFF;

    Buffer[15] = macMsg->MIC & 0xFF;
    Buffer[16] = ( macMsg->MIC >> 8 ) & 0xFF;
    Buffer[17] = ( macMsg->MIC >> 16 ) & 0xFF;
    Buffer[18] = ( macMsg->MIC >> 24 ) & 0xFF;

    macMsg->BufSize = 19;                   // Equal to LORAMAC_RE_JOIN_0_2_MSG_SIZE

    return LORAMAC_SERIALIZER_SUCCESS;
}

LoRaMacSerializerStatus_t LoRaMacSerializerData( LoRaMacMessageData_t* macMsg )
{
    if( ( macMsg == 0 ) || ( macMsg->Buffer == 0 ) )
    {
        return LORAMAC_SERIALIZER_ERROR_NPE;
    }

    uint8_t* Buffer = macMsg->Buffer;

    // Check macMsg->BufSize
    uint16_t computedBufSize =   LORAMAC_MHDR_FIELD_SIZE
                               + LORAMAC_FHDR_DEV_ADDR_FIELD_SIZE
                               + LORAMAC_FHDR_F_CTRL_FIELD_SIZE
                               + LORAMAC_FHDR_F_CNT_FIELD_SIZE;

    computedBufSize += macMsg->FHDR.FCtrl.Bits.FOptsLen;

    if( macMsg->FRMPayloadSize > 0 )
    {
        computedBufSize += LORAMAC_F_PORT_FIELD_SIZE;
    }

    computedBufSize += macMsg->FRMPayloadSize;
    computedBufSize += LORAMAC_MIC_FIELD_SIZE;

    if( macMsg->BufSize < computedBufSize )
    {
        return LORAMAC_SERIALIZER_ERROR_BUF_SIZE;
    }

    Buffer[0] = macMsg->MHDR.Value;

    Buffer[1] = ( macMsg->FHDR.DevAddr ) & 0xFF;
    Buffer[2] = ( macMsg->FHDR.DevAddr >> 8 ) & 0xFF;
    Buffer[3] = ( macMsg->FHDR.DevAddr >> 16 ) & 0xFF;
    Buffer[4] = ( macMsg->FHDR.DevAddr >> 24 ) & 0xFF;

    Buffer[5] = macMsg->FHDR.FCtrl.Value;

    Buffer[6] = macMsg->FHDR.FCnt & 0xFF;
    Buffer[7] = ( macMsg->FHDR.FCnt >> 8 ) & 0xFF;

    memcpy1( &Buffer[8], macMsg->FHDR.FOpts, macMsg->FHDR.FCtrl.Bits.FOptsLen );
    uint16_t bufItr = 8 + macMsg->FHDR.FCtrl.Bits.FOptsLen;

    if( macMsg->FRMPayloadSize > 0 )
    {
        Buffer[bufItr++] = macMsg->FPort;
    }

    memcpy1( &Buffer[bufItr], macMsg->FRMPayload, macMsg->FRMPayloadSize );
    bufItr = bufItr + macMsg->FRMPayloadSize;

    Buffer[bufItr] = macMsg->MIC & 0xFF;
    Buffer[bufItr + 1] = ( macMsg->MIC >> 8 ) & 0xFF;
    Buffer[bufItr + 2] = ( macMsg->MIC >> 16 ) & 0xFF;
    Buffer[bufItr + 3] = ( macMsg->MIC >> 24 ) & 0xFF;

    macMsg->BufSize = (bufItr + 4);

    return LORAMAC_SERIALIZER_SUCCESS;
}

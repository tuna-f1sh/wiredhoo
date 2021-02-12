#include "antmessage.h"

// 1+ as this is including the sync byte, which BUFFER_INDEXES ignore
#define ANT_MESSAGE_SIZE(buffer) buffer[1+BUFFER_INDEX_MESG_SIZE] + ANT_FRAME_SIZE

// Network key - B9,A5,21,FB,BD,72,C3,45
#define ANT_NETWORK_KEY_0           0xB9
#define ANT_NETWORK_KEY_1           0xA5
#define ANT_NETWORK_KEY_2           0x21
#define ANT_NETWORK_KEY_3           0xFB
#define ANT_NETWORK_KEY_4           0xBD
#define ANT_NETWORK_KEY_5           0x72
#define ANT_NETWORK_KEY_6           0xC3
#define ANT_NETWORK_KEY_7           0x45

#define FAKE_CHANNELS               0x04
#define FAKE_NETWORKS               0x04

#define DEVICE_SERIAL               133700UL

#define POWER_CH                    0
#define POWER_ID                    13370

#define POWER_FC_CH                 1
#define POWER_FC_ID                 POWER_ID + 1

typedef enum
{
  MESG_OK = 0U,
  MESG_SIZE_ERR,
  MESG_CRC_ERR,
  MESG_UNKNOWN_ID,
  MESG_INVALID_REQ,
} ANT_MessageStatus;

ANT_MessageStatus process_ant_message(uint8_t *pMessage, size_t len, uint8_t *reply);

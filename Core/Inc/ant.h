#include <stdbool.h>
#include "antmessage.h"

#define ANT_VERSION "AJX1.300"

// 1+ as this is including the sync byte, which BUFFER_INDEXES ignore
#define ANT_MESSAGE_SIZE(buffer) buffer[1+BUFFER_INDEX_MESG_SIZE] + ANT_FRAME_SIZE

// Messages not defined in antmessage.h
#define MESG_OPEN_RX_SCAN_MODE_ID   0x5B
#define MESG_CHANNEL_RADIO_POWER    0x60
#define MESG_ENABLE_LED             0x68
#define MESG_ENABLE_EXT_MESSAGE     0x66

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

#define DEVICE_SERIAL               1337UL // should be last 9 digits of string serial (if one)
#define DEVICE_ID                   1337U // should be last 5 digits of serial no.

#define ANT_RFFREQ_SPORT            57

#define ANT_TYPE_POWER              CHANNEL_TYPE_MASTER
#define ANT_DEVTYPE_POWER           11
#define ANT_PERIOD_POWER            8182
#define ANT_TXTYPE_POWER            5

#define ANT_TYPE_FEC                CHANNEL_TYPE_MASTER
#define ANT_DEVTYPE_FEC             17
#define ANT_PERIOD_FEC              8192
#define ANT_TXTYPE_FEC              5

typedef enum {
  MESG_OK = 0U,
  MESG_SIZE_ERR,
  MESG_CRC_ERR,
  MESG_UNKNOWN_ID,
  MESG_INVALID_REQ,
  MESG_SYNC_ERR,
} ANT_MessageStatus;

typedef struct {
  bool open;
  bool pairing;
  uint8_t type;
  uint8_t network_no;
  uint16_t device_no;
  uint8_t device_type;
  uint8_t transmission_type;
  uint8_t status;
  uint8_t ext_assign; // 0x01 enables background scanning mode; the slave channel will not align to first master found channel ID but continue as a wildcard
  uint16_t period;
  uint8_t timeout;
  uint8_t rf_freq;
  uint8_t transmit_power;
  uint8_t tx_power;
} ANT_Channel_t;

typedef struct {
  bool rx_scanning;
  bool ext_enabled; // extended provides channel ID in data message, intended for use with background scanning mode
  uint8_t tx_power;
} ANT_Device_t;

ANT_MessageStatus process_ant_message(uint8_t *pMessage, size_t len, uint8_t *reply);

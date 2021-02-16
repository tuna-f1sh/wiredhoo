#include <stdbool.h>
#include "antmessage.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "timers.h"

#define ANT_VERSION                 "AJX1.300"
#define ANT_MANU_ID                 0x00FF // unassigned
#define ANT_MODEL_NO                1
#define HW_VER                      1
#define SW_VER                      1

// 1+ as this is including the sync byte, which BUFFER_INDEXES ignore
#define ANT_MESSAGE_SIZE(buffer) buffer[1+BUFFER_INDEX_MESG_SIZE] + ANT_FRAME_SIZE
#define PERIOD_TO_MS(x) ((uint32_t) x * 100) / 32768 // 9.5.2.4

// Messages not defined in antmessage.h
#define MESG_OPEN_RX_SCAN_MODE_ID   0x5B
#define MESG_CHANNEL_RADIO_POWER    0x60
#define MESG_ENABLE_LED             0x68
#define MESG_ENABLE_EXT_MESSAGE     0x66
#define MESG_LOW_PRIORITY_TIMEOUT   0x63

// flag byte, device_no_l, device_no_h, device_type, trans_type
#define MESG_EXTENDED_SIZE          MESG_DATA_SIZE + 5

// Network key - B9,A5,21,FB,BD,72,C3,45
#define ANT_NETWORK_KEY_0           0xB9
#define ANT_NETWORK_KEY_1           0xA5
#define ANT_NETWORK_KEY_2           0x21
#define ANT_NETWORK_KEY_3           0xFB
#define ANT_NETWORK_KEY_4           0xBD
#define ANT_NETWORK_KEY_5           0x72
#define ANT_NETWORK_KEY_6           0xC3
#define ANT_NETWORK_KEY_7           0x45

#define FAKE_CHANNELS               0x08
#define FAKE_NETWORKS               0x08

#define DEVICE_SERIAL               1337UL // should be last 9 digits of string serial (if one)
#define DEVICE_ID                   1337U // should be last 5 digits of serial no.

#define ANT_RFFREQ_SPORT            57

#define ANT_TYPE_POWER              CHANNEL_TYPE_MASTER
#define ANT_FAKE_CH_POWER           0
#define ANT_DEVTYPE_POWER           11
#define ANT_PERIOD_POWER            8182
#define ANT_TXTYPE_POWER            5

#define ANT_TYPE_FEC                CHANNEL_TYPE_MASTER
#define ANT_FAKE_CH_FEC             1
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

// channel configuration
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
  uint8_t lp_timeout;
  uint8_t rf_freq;
  uint8_t transmit_power;
  uint8_t tx_power;
} ANT_Channel_t;

// device control (this)
typedef struct {
  bool rx_scanning;
  bool ext_enabled; // extended provides channel ID in data message, intended for use with background scanning mode
  uint8_t tx_power;
} ANT_Control_t;

// ANT device (virtual in our case)
typedef struct {
  uint16_t device_no;
  uint8_t device_type;
  uint8_t transmission_type;
  uint16_t period;
  uint8_t event_counter;
  uint8_t page_counter;
  uint8_t channel;
  TimerHandle_t *timer;
} ANT_Device_t;

ANT_MessageStatus process_ant_message(uint8_t *pMessage, size_t len, uint8_t *reply);
void ant_construct_data_message(uint8_t id, uint8_t size, ANT_Device_t *dev, uint8_t *pMsg, uint8_t *pData);
void ant_construct_message(uint8_t id, uint8_t size, uint8_t channel, uint8_t *pMsg, uint8_t *pData);
uint8_t ant_process_tx_event(uint8_t *pMsg, size_t len);
void transmit_message(uint8_t *pBuffer, size_t len, uint8_t block_tick);
void ant_generate_data_page(ANT_Device_t *dev, uint8_t *page);

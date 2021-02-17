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

// network key - this shouldn't be distributed but can be found in many libraries online or even just sniffed. We don't actually need it since the host kindly sends it to us - another hint as to where to get this!
#define ANT_NETWORK_KEY_0           0x00
#define ANT_NETWORK_KEY_1           0x00
#define ANT_NETWORK_KEY_2           0x00
#define ANT_NETWORK_KEY_3           0x00
#define ANT_NETWORK_KEY_4           0x00
#define ANT_NETWORK_KEY_5           0x00
#define ANT_NETWORK_KEY_6           0x00
#define ANT_NETWORK_KEY_7           0x00

#define FAKE_CHANNELS               0x08
#define FAKE_NETWORKS               0x08
// TODO add real channels that are forward to ANT radio

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

#define ANT_COMMON_PAGE1            0x50
#define ANT_COMMON_PAGE2            0x51
#define ANT_COMMON_REQ_PAGE         0x46
#define ANT_COMMON_CMD_STATUS       0x47
#define ANT_POWER_STANDARD_PAGE     0x10
#define ANT_POWER_CALIBRATION_PAGE  0x01
#define ANT_FEC_TRAINER_DATA_PAGE   0x19
#define ANT_FEC_GENERAL_PAGE        0x10
#define ANT_FEC_GENERAL_SET_PAGE    0x11

// calibration req is ack then update with progress interleved with general data followed by req to send calibration complete status
#define ANT_FEC_CALIBRATION_REQ     ANT_POWER_CALIBRATION_PAGE
#define ANT_FEC_CALIBRATION_PROG    0x02
// data pages are all ack broadcast
#define ANT_FEC_DP_BASIC_RESISTANCE 0x30
#define ANT_FEC_DP_TARGET_POWER     0x31
#define ANT_FEC_DP_WIND_RESISTANCE  0x32
#define ANT_FEC_DP_TRACK_RESISTANCE 0x33
#define ANT_FEC_DP_FE_CAPABILITES   0x36 // required
#define ANT_FEC_DP_USER_CONFIG      0x37
#define ANT_FEC_ZERO_OFFSET_MASK    (1 << 6)
#define ANT_FEC_SPIN_DOWN_MASK      (1 << 7)

typedef enum {
  MESG_OK = 0U,
  MESG_SIZE_ERR,
  MESG_CRC_ERR,
  MESG_UNKNOWN_ID,
  MESG_INVALID_REQ,
  MESG_SYNC_ERR,
  MESG_NO_REPLY,
} ANT_MessageStatus_t;

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
  uint8_t last_request[8];
  TimerHandle_t *timer;
} ANT_Device_t;

ANT_MessageStatus_t process_ant_message(uint8_t *pMessage, size_t len, uint8_t *reply);
void ant_construct_data_message(uint8_t id, uint8_t size, ANT_Device_t *dev, uint8_t *pMsg, uint8_t *pData);
void ant_construct_message(uint8_t id, uint8_t size, uint8_t channel, uint8_t *pMsg, uint8_t *pData);
uint8_t ant_process_tx_event(uint8_t *pMsg, size_t len);
void transmit_message(uint8_t *pBuffer, size_t len, uint8_t block_tick);
void ant_generate_data_page(ANT_Device_t *dev, uint8_t *page);
void ant_generate_common_page(uint8_t id, uint8_t *page);
uint8_t ant_start_device(ANT_Device_t *dev);
uint8_t ant_stop_device(ANT_Device_t *dev);
uint8_t ant_any_channels_open(void);

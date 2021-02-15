#include <string.h>
#include <stdbool.h>
#include "main.h"
#include "utilities.h"
#include "antmessage.h"
#include "antdefines.h"
#include "ant.h"

/* static const uint8_t sAntNetworkKey[] = { */
/*   ANT_NETWORK_KEY_0, ANT_NETWORK_KEY_1, */
/*   ANT_NETWORK_KEY_2, ANT_NETWORK_KEY_3, */
/*   ANT_NETWORK_KEY_4, ANT_NETWORK_KEY_5, */
/*   ANT_NETWORK_KEY_6, ANT_NETWORK_KEY_7, */
/* }; */
static uint8_t network_key[FAKE_NETWORKS][8] = {0};
static const uint32_t device_serial = DEVICE_SERIAL;
static ANT_Channel_t ant_channels[FAKE_CHANNELS];
static ANT_Control_t ant_device;
static const char* ant_version = ANT_VERSION;

ANT_MessageStatus process_ant_request(uint8_t *pBuffer, size_t len, uint8_t *pReply);
ANT_MessageStatus process_ant_configuration(uint8_t config, uint8_t *pBuffer, size_t len);
inline void channel_message_reply(uint8_t *pBuffer, uint8_t code, uint8_t *pReply);

ANT_MessageStatus process_ant_message(uint8_t *pMessage, size_t len, uint8_t *pReply) {
  ANT_MessageStatus ret = MESG_OK;

  // check sync byte
  if (pMessage[0] == MESG_TX_SYNC) {
    // check valid size
    if (len >= ANT_FRAME_SIZE) {
      // check valid CRC -1 to exclude actual CRC byte
      if (calculate_crc(pMessage, len - 1) == pMessage[len-1]) {
        // buffer is indexed with sync byte trimmed
        uint8_t *pBuffer = &pMessage[1];
        uint8_t msg = pBuffer[BUFFER_INDEX_MESG_ID];

        // TX sync reply
        pReply[0] = MESG_TX_SYNC;

        switch (msg) {
          // --- device control ---
          case MESG_SYSTEM_RESET_ID:
            pReply[1] = MESG_SYSTEM_RESET_SIZE;
            pReply[2] = MESG_START_UP;
            // re-initialise channels to unassigned state or maybe even reset after send?
            memset(ant_channels, 0x00, sizeof(ANT_Channel_t) * FAKE_CHANNELS);
            memset(&ant_device, 0x00, sizeof(ANT_Control_t));
            HAL_GPIO_WritePin(ANT_CONTROL_LED_PORT, ANT_CONTROL_LED, 1);
            pReply[3] = 0x20; // watchdog reset
            break;
          case MESG_OPEN_CHANNEL_ID:
          case MESG_CLOSE_CHANNEL_ID:
          case MESG_OPEN_RX_SCAN_MODE_ID:
            channel_message_reply(pBuffer, process_ant_configuration(msg, pBuffer, len - 1), pReply);
            break;
          // --- device/channel configuration messages ---
          // not all are covered here but commonly used and supported are
          case MESG_UNASSIGN_CHANNEL_ID:
          case MESG_ASSIGN_CHANNEL_ID:
          case MESG_CHANNEL_ID_ID:
          case MESG_CHANNEL_MESG_PERIOD_ID:
          case MESG_CHANNEL_SEARCH_TIMEOUT_ID:
          case MESG_LOW_PRIORITY_TIMEOUT:
          case MESG_CHANNEL_RADIO_FREQ_ID:
          case MESG_NETWORK_KEY_ID:
          case MESG_CHANNEL_RADIO_POWER: // set channel power
          case MESG_RADIO_TX_POWER_ID:
          case MESG_RADIO_CW_MODE_ID:
          case MESG_SEARCH_WAVEFORM_ID:
          case MESG_ENABLE_LED:
          case MESG_ENABLE_EXT_MESSAGE:
            channel_message_reply(pBuffer, process_ant_configuration(msg, pBuffer, len - 1), pReply);
            break;
          // --- request / response ---
          case MESG_REQUEST_ID:
            ret = process_ant_request(pBuffer, len - 1, pReply);
            break;
          default:
            ret = MESG_UNKNOWN_ID;
            break;
        }

        // stick CRC at end of message
        pReply[ANT_MESSAGE_SIZE(pReply) - 1] = calculate_crc(pReply, ANT_MESSAGE_SIZE(pReply) - 1);
      } else {
        pReply[1] = 0x01;
        pReply[2] = 0xAE; // serial error message
        pReply[3] = 0x02; // crc not match
        pReply[ANT_MESSAGE_SIZE(pReply) - 1] = calculate_crc(pReply, ANT_MESSAGE_SIZE(pReply) - 1);
        ret = MESG_CRC_ERR;
      }
    } else {
      pReply[1] = 0x01;
      pReply[2] = 0xAE; // serial error message
      pReply[3] = 0x03; // too large
      pReply[ANT_MESSAGE_SIZE(pReply) - 1] = calculate_crc(pReply, ANT_MESSAGE_SIZE(pReply) - 1);
      ret = MESG_SIZE_ERR;
    }
  } else {
    pReply[1] = 0x01;
    pReply[2] = 0xAE; // serial error message
    pReply[3] = 0x00; // sync incorrect
    pReply[ANT_MESSAGE_SIZE(pReply) - 1] = calculate_crc(pReply, ANT_MESSAGE_SIZE(pReply) - 1);
    ret = MESG_SYNC_ERR;
  }

  return ret;
}

ANT_MessageStatus process_ant_request(uint8_t *pBuffer, size_t len, uint8_t *pReply) {
  ANT_MessageStatus ret = MESG_OK;
  uint8_t channel = pBuffer[BUFFER_INDEX_CHANNEL_NUM];
  uint8_t request = pBuffer[BUFFER_INDEX_MESG_DATA];

  switch (request) {
    case MESG_CAPABILITIES_ID:
      pReply[1] = MESG_CAPABILITIES_SIZE + 1; // plus for extended options
      pReply[2] = MESG_CAPABILITIES_ID;
      pReply[3] = FAKE_CHANNELS;
      pReply[4] = FAKE_NETWORKS;
      pReply[5] = 0x00; // standard options (no exclusions)
      pReply[6] = // advanced options
        (1 << 1) | // network enabled
        (1 << 3) | // serial number enabled
        (1 << 4) | // per channel tx power
        (1 << 5) | // low priority search
        (0 << 6) | // script
        (1 << 7); // search list
      // these are extended options but basic reply of 4 bytes only required
      pReply[7] = // advanced options 2
        (1 << 0) | // led
        (1 << 1) | // extended message
        (1 << 2) | // scan mode
        (0 << 3) | // reserved
        (1 << 4) | // prox search
        (1 << 5) | // ext assign
        (0 << 6) | // ant fs
        (0 << 7); // fit1
      /* pReply[8] = 0x00; // sensRcore channels */
      /* pReply[9] = 0xDF; // advanced 3 */
      break;
    case MESG_CHANNEL_STATUS_ID:
      pReply[1] = MESG_CHANNEL_STATUS_SIZE;
      pReply[2] = MESG_CHANNEL_STATUS_ID;
      if (channel < FAKE_CHANNELS) {
        pReply[3] = channel; // channel number
        pReply[4] = (ant_channels[channel].status & 0x03) | (ant_channels[channel].network_no & 0x0C) | (ant_channels[channel].type & 0xF0);
      } else {
        ret = MESG_INVALID_REQ;
      }
      break;
    case MESG_CHANNEL_ID_ID:
      // TODO reply with channel setup info
      pReply[1] = MESG_CHANNEL_ID_SIZE;
      pReply[2] = MESG_CHANNEL_ID_ID;
      if (channel < FAKE_CHANNELS) {
        pReply[3] = channel; // channel number
        pReply[4] = LOW_BYTE(ant_channels[channel].device_no);
        pReply[5] = HIGH_BYTE(ant_channels[channel].device_no);
        pReply[6] = ant_channels[channel].device_type;
        pReply[7] = ant_channels[channel].transmission_type;
      } else {
        ret = MESG_INVALID_REQ;
      }
      break;
    case 0x61: // device serial
      pReply[1] = 0x04;
      pReply[2] = 0x61;
      memcpy(&pReply[3], &device_serial, 4);
      break;
    case MESG_VERSION_ID:
      pReply[1] = MESG_VERSION_SIZE;
      pReply[2] = MESG_VERSION_ID;
      memcpy(&pReply[3], ant_version, 9);
      break;
    default:
      ret = MESG_UNKNOWN_ID;
      break;
  }

  return ret;
}

ANT_MessageStatus process_ant_configuration(uint8_t config, uint8_t *pBuffer, size_t len) {
  uint8_t ret = RESPONSE_NO_ERROR;
  uint8_t channel = pBuffer[BUFFER_INDEX_CHANNEL_NUM];

  if (channel < FAKE_CHANNELS) {
    switch (config) {
        case MESG_UNASSIGN_CHANNEL_ID:
          if (ant_channels[channel].status != STATUS_UNASSIGNED_CHANNEL) {
            ant_channels[channel].status = STATUS_UNASSIGNED_CHANNEL;
          } else {
            ret = CHANNEL_IN_WRONG_STATE;
          }
          break;
        case MESG_ASSIGN_CHANNEL_ID:
          if (ant_channels[channel].status == STATUS_UNASSIGNED_CHANNEL) {
            ant_channels[channel].status = STATUS_ASSIGNED_CHANNEL;
            ant_channels[channel].type = pBuffer[BUFFER_INDEX_MESG_DATA];
            ant_channels[channel].network_no = pBuffer[BUFFER_INDEX_MESG_DATA + 1];
            ant_channels[channel].ext_assign = pBuffer[BUFFER_INDEX_MESG_DATA + 2];
          } else {
            ret = CHANNEL_IN_WRONG_STATE;
          }
          break;
        case MESG_CHANNEL_ID_ID:
          if (ant_channels[channel].status != STATUS_UNASSIGNED_CHANNEL) {
            ant_channels[channel].device_no = pBuffer[BUFFER_INDEX_MESG_DATA] | (pBuffer[BUFFER_INDEX_MESG_DATA + 1] << 8);
            // bit 7 is pairing request TODO pair with fake device
            ant_channels[channel].pairing = pBuffer[BUFFER_INDEX_MESG_DATA + 2] & 0x80;
            ant_channels[channel].device_type = pBuffer[BUFFER_INDEX_MESG_DATA + 2] & 0x7F;
            ant_channels[channel].transmission_type = pBuffer[BUFFER_INDEX_MESG_DATA + 3];
          } else {
            ret = CHANNEL_IN_WRONG_STATE;
          }
          break;
        case MESG_CHANNEL_MESG_PERIOD_ID:
          // TODO create task with this period to send data on broadcast
          if (ant_channels[channel].status != STATUS_UNASSIGNED_CHANNEL) {
            ant_channels[channel].period = pBuffer[BUFFER_INDEX_MESG_DATA] | (pBuffer[BUFFER_INDEX_MESG_DATA + 1] << 8);
          } else {
            ret = CHANNEL_IN_WRONG_STATE;
          }
          break;
        case MESG_CHANNEL_SEARCH_TIMEOUT_ID:
          // TODO do something with this
          if (ant_channels[channel].status != STATUS_UNASSIGNED_CHANNEL) {
            ant_channels[channel].timeout = pBuffer[BUFFER_INDEX_MESG_DATA];
          } else {
            ret = CHANNEL_IN_WRONG_STATE;
          }
          break;
        case MESG_LOW_PRIORITY_TIMEOUT:
          // TODO do something with this
          if (ant_channels[channel].status != STATUS_UNASSIGNED_CHANNEL) {
            ant_channels[channel].lp_timeout = pBuffer[BUFFER_INDEX_MESG_DATA];
          } else {
            ret = CHANNEL_IN_WRONG_STATE;
          }
          break;
        case MESG_CHANNEL_RADIO_FREQ_ID:
          // TODO do something with this
          if (ant_channels[channel].status != STATUS_UNASSIGNED_CHANNEL) {
            ant_channels[channel].rf_freq = pBuffer[BUFFER_INDEX_MESG_DATA];
          } else {
            ret = CHANNEL_IN_WRONG_STATE;
          }
          break;
        case MESG_NETWORK_KEY_ID:
          // not so worried about this...just response no error
          // first byte is network number
          if (len >= 9 && channel < FAKE_NETWORKS) {
            memcpy(network_key[channel], &pBuffer[BUFFER_INDEX_MESG_DATA], 8);
          } else {
            ret = INVALID_MESSAGE;
          }
          break;
        // this is not channel specific (0x60 is for channel) but global - regardless, it doesn't really matter for this use case
        case MESG_RADIO_TX_POWER_ID:
          ant_device.tx_power = pBuffer[BUFFER_INDEX_MESG_DATA];
          break;
        case MESG_CHANNEL_RADIO_POWER:
          // TODO do something with this
          if (ant_channels[channel].status != STATUS_UNASSIGNED_CHANNEL) {
            ant_channels[channel].tx_power = pBuffer[BUFFER_INDEX_MESG_DATA];
          } else {
            ret = CHANNEL_IN_WRONG_STATE;
          }
          break;
        case MESG_OPEN_CHANNEL_ID:
          if (ant_channels[channel].status != STATUS_UNASSIGNED_CHANNEL) {
            // TODO start transmitting fake ANT master devices
            ant_channels[channel].open = true;
          } else {
            ret = CHANNEL_ID_NOT_SET;
          }
          break;
        case MESG_CLOSE_CHANNEL_ID:
          // TODO stop transmitting fake ANT master devices
          ant_channels[channel].open = false;
          if (channel == 0) ant_device.rx_scanning = false;
          break;
        case MESG_OPEN_RX_SCAN_MODE_ID:
          // TODO something with this see 9.5.4.5
          // open channel 0 and print return all device types in event Tx loop, assert flag for this and prevent other channels open as per specification
          if (ant_channels[channel].status != STATUS_UNASSIGNED_CHANNEL) {
            ant_channels[channel].open = true;
            ant_device.rx_scanning = true;
          } else {
            ret = CHANNEL_IN_WRONG_STATE;
          }
          break;
        case MESG_ENABLE_LED:
          HAL_GPIO_WritePin(ANT_CONTROL_LED_PORT, ANT_CONTROL_LED, !pBuffer[BUFFER_INDEX_MESG_DATA]);
          break;
        case MESG_ENABLE_EXT_MESSAGE:
          ant_device.ext_enabled = pBuffer[BUFFER_INDEX_MESG_DATA];
          break;
        case MESG_RADIO_CW_MODE_ID:
        case MESG_SEARCH_WAVEFORM_ID:
        default:
          break;
    }
  } else {
    ret = INVALID_MESSAGE;
  }

  return ret;
}


// the passed pMsg buffer must be large enough to the size and the ANT frame! pData should be 8 bytes
void ant_construct_data_message(uint8_t id, uint8_t size, uint8_t channel, uint16_t device_no, uint8_t device_type, uint8_t trans_type, uint8_t *pMsg, uint8_t *pData) {
  uint8_t extData[MESG_EXTENDED_SIZE];
  memcpy(extData, pData, MESG_MAX_DATA_BYTES);
  // bytes 8-> are extended
  extData[MESG_MAX_DATA_BYTES] = 0x80; // extended data flag
  extData[MESG_MAX_DATA_BYTES + 1] = LOW_BYTE(device_no);
  extData[MESG_MAX_DATA_BYTES + 2] = HIGH_BYTE(device_no);
  extData[MESG_MAX_DATA_BYTES + 3] = device_type;
  extData[MESG_MAX_DATA_BYTES + 4] = trans_type;
  ant_construct_message(id, size, channel, pMsg, extData);
}

void ant_construct_message(uint8_t id, uint8_t size, uint8_t channel, uint8_t *pMsg, uint8_t *pData) {
  pMsg[0] = MESG_TX_SYNC;
  pMsg[1] = size;
  pMsg[2] = id;
  pMsg[3] = channel; // channel
  memcpy(&pMsg[4], pData, size - 1); // -1 because size includes CRC but data does not
  pMsg[ANT_MESSAGE_SIZE(pMsg) - 1] = calculate_crc(pMsg, ANT_MESSAGE_SIZE(pMsg) - 1);
}

inline void channel_message_reply(uint8_t *pBuffer, uint8_t code, uint8_t *pReply) {
  pReply[1] = MESG_RESPONSE_EVENT_SIZE;
  pReply[2] = MESG_RESPONSE_EVENT_ID;
  pReply[3] = pBuffer[BUFFER_INDEX_CHANNEL_NUM]; // channel
  pReply[4] = pBuffer[BUFFER_INDEX_MESG_ID]; // initiating request
  pReply[5] = code;
}

// this function takes a virtual master device _wireless_ packet and sends it on the channel configured on the slave device (ours) by matching it to an assigned and open channel
// it's slightly backwards un-packing the data from ant_contruct_data_message but this seems like the most compatiable way of emulating the functionality of master/slave ANT device
void ant_process_tx_event(uint8_t *pMsg, size_t len) {
  if (len >= 8) {
    uint8_t *pBuffer = &pMsg[1];
    uint8_t *pData = &pBuffer[BUFFER_INDEX_MESG_DATA];
    uint8_t msg = pBuffer[BUFFER_INDEX_MESG_ID];

    uint16_t device_no = pData[MESG_MAX_DATA_BYTES+1] | (pData[MESG_MAX_DATA_BYTES+2] << 8);
    uint8_t device_type = pData[MESG_MAX_DATA_BYTES+3];
    uint8_t trans_type = pData[MESG_MAX_DATA_BYTES+4];

    for (int i = 0; i < FAKE_CHANNELS; i++) {
      // should the fake channel send the device?
      if (ant_channels[i].open &&
          // matching device or wildcard
          (ant_channels[i].device_no == device_no || ant_channels[i].device_no == 0) &&
          // matching type or wildcard
          (ant_channels[i].device_type == device_type || ant_channels[i].device_type == 0) &&
          // matching trans or pairing search
          (ant_channels[i].transmission_type == trans_type || ant_channels[i].transmission_type == 0))
      {
        // if extended option is not background scan, set channel id to match master
        if (!((ant_channels[i].ext_assign & 0x01) == 0x01)) {
          ant_channels[i].device_no = device_no;
          ant_channels[i].device_type = device_type;
          ant_channels[i].transmission_type = trans_type;
        }
        // set the packet channel is fake channel not fake device...getting confusing but...
        pBuffer[BUFFER_INDEX_CHANNEL_NUM] = i | (pBuffer[BUFFER_INDEX_CHANNEL_NUM] & SEQUENCE_NUMBER_MASK);
        if (ant_device.ext_enabled) {
          // re-calculate CRC due to potential channel change
          pMsg[ANT_MESSAGE_SIZE(pMsg) - 1] = calculate_crc(pMsg, ANT_MESSAGE_SIZE(pMsg) - 1);
        } else {
          // re-order hold message for just standard size
          ant_construct_message(msg, MESG_DATA_SIZE, i | (pBuffer[BUFFER_INDEX_CHANNEL_NUM] & SEQUENCE_NUMBER_MASK), pMsg, pData);
        }
        // TODO add this to a USBD Tx stream
        transmit_message(pMsg, ANT_MESSAGE_SIZE(pMsg), 20);
      }
    }
  }
}

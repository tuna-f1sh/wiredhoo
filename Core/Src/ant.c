// TODO
// * add channel search timeout using timer if period != 0xFF, send timeout event on timeout
// * burst and advanced burst
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "main.h"
#include "utilities.h"
#include "antmessage.h"
#include "antdefines.h"
#include "ant.h"
#include "trainer.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "timers.h"

static uint8_t network_key[FAKE_NETWORKS][8] = {0};
static const uint32_t device_serial = DEVICE_SERIAL;
static ANT_Channel_t ant_channels[FAKE_CHANNELS];
static ANT_Control_t ant_device;
static const char* ant_version = ANT_VERSION;

ANT_MessageStatus_t process_ant_request(uint8_t *pBuffer, size_t len, uint8_t *pReply);
ANT_MessageStatus_t process_ant_configuration(uint8_t config, uint8_t *pBuffer, size_t len);
inline void channel_message_reply(uint8_t *pBuffer, uint8_t code, bool rf_event, uint8_t *pReply);
ANT_MessageStatus_t process_ant_broadcast(uint8_t event, uint8_t *pBuffer, size_t len);

ANT_Device_t power_device = {
  .device_no = DEVICE_ID,
  .device_type = ANT_DEVTYPE_POWER,
  .transmission_type = ANT_TXTYPE_POWER,
  .period = ANT_PERIOD_POWER,
  .event_counter = 0,
  .page_counter = 0,
  .channel = 0,
  .tid = 0,
};

ANT_Device_t fec_device = {
  .device_no = DEVICE_ID,
  .device_type = ANT_DEVTYPE_FEC,
  .transmission_type = ANT_TXTYPE_FEC,
  .period = ANT_PERIOD_FEC,
  .event_counter = 0,
  .page_counter = 0,
  .channel = 0,
  .tid = 1,
};
extern Trainer_t trainer;

ANT_MessageStatus_t process_ant_message(uint8_t *pMessage, size_t len, uint8_t *pReply) {
  ANT_MessageStatus_t ret = MESG_OK;

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
            ant_stop_device(&power_device);
            ant_stop_device(&fec_device);
            HAL_GPIO_WritePin(ANT_CONTROL_LED_PORT, ANT_CONTROL_LED, 1);
            pReply[3] = 0x20; // watchdog reset
            break;

          case MESG_OPEN_CHANNEL_ID:
          case MESG_CLOSE_CHANNEL_ID:
          case MESG_OPEN_RX_SCAN_MODE_ID:
            channel_message_reply(pBuffer, process_ant_configuration(msg, pBuffer, len - 1), false, pReply);
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
            channel_message_reply(pBuffer, process_ant_configuration(msg, pBuffer, len - 1), false, pReply);
            break;

          // --- request / response ---
          case MESG_REQUEST_ID:
            ret = process_ant_request(pBuffer, len - 1, pReply);
            break;

          // --- channel data received ---
          // no action on this for emulated devices...
          case MESG_BROADCAST_DATA_ID:
            ret = process_ant_broadcast(msg, pBuffer, len);
            break;

          case MESG_ACKNOWLEDGED_DATA_ID:
            ret = process_ant_broadcast(msg, pBuffer, len);
            channel_message_reply(pBuffer, EVENT_TRANSFER_TX_COMPLETED, true, pReply);
            break;

          // TODO - support burst
          // --- channel events receieved ---
          // TODO - do something here
          case MESG_RESPONSE_EVENT_ID:
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

ANT_MessageStatus_t process_ant_request(uint8_t *pBuffer, size_t len, uint8_t *pReply) {
  ANT_MessageStatus_t ret = MESG_OK;
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

ANT_MessageStatus_t process_ant_configuration(uint8_t config, uint8_t *pBuffer, size_t len) {
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
          if (ant_channels[channel].device_type == ANT_DEVTYPE_POWER || ant_channels[channel].device_type == 0) {
            ant_start_device(&power_device, channel);
          }
          if (ant_channels[channel].device_type == ANT_DEVTYPE_FEC || ant_channels[channel].device_type == 0) {
            ant_start_device(&fec_device, channel);
          }
          break;
        case MESG_CLOSE_CHANNEL_ID:
          // TODO stop transmitting fake ANT master devices
          ant_channels[channel].open = false;
          if (channel == 0) ant_device.rx_scanning = false;
          if (!ant_any_channels_open()) {
            ant_stop_device(&power_device);
            ant_stop_device(&fec_device);
          }
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
          ant_start_device(&power_device, channel);
          ant_start_device(&fec_device, channel);
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

ANT_MessageStatus_t process_ant_broadcast(uint8_t event, uint8_t *pBuffer, size_t len) {
  uint8_t ret = MESG_OK;
  uint8_t channel = pBuffer[BUFFER_INDEX_CHANNEL_NUM];

  switch (event) {

    case MESG_BROADCAST_DATA_ID:
      ret = MESG_NO_REPLY;
      break;

    // ret MESG_OK will return ACK
    case MESG_ACKNOWLEDGED_DATA_ID:
      // check for channel assigned to fake device - maybe should check for extended channel id instead?
      if (ant_channels[channel].open &&
          // matching device or wildcard
          (ant_channels[channel].device_no == DEVICE_ID || ant_channels[channel].device_no == 0) &&
          // matching type or wildcard
          (ant_channels[channel].device_type == ANT_DEVTYPE_POWER || ant_channels[channel].device_type == 0) &&
          // matching trans or pairing search
          (ant_channels[channel].transmission_type == ANT_TXTYPE_POWER || ant_channels[channel].transmission_type == 0))
      {
        memcpy(&power_device.last_request, &pBuffer[BUFFER_INDEX_DATA0], 8);
      } else if (ant_channels[channel].open &&
          // matching device or wildcard
          (ant_channels[channel].device_no == DEVICE_ID || ant_channels[channel].device_no == 0) &&
          // matching type or wildcard
          (ant_channels[channel].device_type == ANT_DEVTYPE_FEC || ant_channels[channel].device_type == 0) &&
          // matching trans or pairing search
          (ant_channels[channel].transmission_type == ANT_TXTYPE_FEC || ant_channels[channel].transmission_type == 0))
      {
        memcpy(&fec_device.last_request, &pBuffer[BUFFER_INDEX_DATA0], 8);
      }
      break;

    default:
      ret = MESG_UNKNOWN_ID;
      break;
  }

  return ret;
}

// the passed pMsg buffer must be large enough to the size and the ANT frame! pData should be 8 bytes
void ant_construct_data_message(uint8_t id, uint8_t size, ANT_Device_t *dev, uint8_t *pMsg, uint8_t *pData) {
  uint8_t extData[MESG_EXTENDED_SIZE];
  memcpy(extData, pData, MESG_MAX_DATA_BYTES);
  // bytes 8-> are extended
  extData[MESG_MAX_DATA_BYTES] = 0x80; // extended data flag
  extData[MESG_MAX_DATA_BYTES + 1] = LOW_BYTE(dev->device_no);
  extData[MESG_MAX_DATA_BYTES + 2] = HIGH_BYTE(dev->device_no);
  extData[MESG_MAX_DATA_BYTES + 3] = dev->device_type;
  extData[MESG_MAX_DATA_BYTES + 4] = dev->transmission_type;
  ant_construct_message(id, size, dev->channel, pMsg, extData);
}

void ant_construct_message(uint8_t id, uint8_t size, uint8_t channel, uint8_t *pMsg, uint8_t *pData) {
  pMsg[0] = MESG_TX_SYNC;
  pMsg[1] = size;
  pMsg[2] = id;
  pMsg[3] = channel; // channel
  memcpy(&pMsg[4], pData, size - 1); // -1 because size includes CRC but data does not
  pMsg[ANT_MESSAGE_SIZE(pMsg) - 1] = calculate_crc(pMsg, ANT_MESSAGE_SIZE(pMsg) - 1);
}

inline void channel_message_reply(uint8_t *pBuffer, uint8_t code, bool rf_event, uint8_t *pReply) {
  pReply[1] = MESG_RESPONSE_EVENT_SIZE;
  pReply[2] = MESG_RESPONSE_EVENT_ID;
  pReply[3] = rf_event ? 0x01 : pBuffer[BUFFER_INDEX_CHANNEL_NUM]; // channel
  pReply[4] = pBuffer[BUFFER_INDEX_MESG_ID]; // initiating request
  pReply[5] = code;
}

// this function takes a virtual master device _wireless_ packet and sends it on the channel configured on the slave device (ours) by matching it to an assigned and open channel
// it's slightly backwards un-packing the data from ant_contruct_data_message but this seems like the most compatiable way of emulating the functionality of master/slave ANT device
uint8_t ant_process_tx_event(uint8_t *pMsg, size_t len) {
  if (len >= MESG_EXTENDED_SIZE) {
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

        if (DEBUG && DEBUG_CH_TX_EVENT) {
          printf("ANT+ Tx Event: ");
          print_hex((char*) pMsg, len);
        }

      }
    }

    return 0;
  } else {
    return 1;
  }
}

void ant_generate_common_page(uint8_t id, uint8_t *page) {
  switch (id) {

    case 0x50:
      page[0] = 0x50;
      page[1] = 0xff;
      page[2] = 0xff;
      page[3] = HW_VER; // hardware version
      page[4] = ANT_MANU_ID & 0xff; // manufacturer ID
      page[5] = ANT_MANU_ID >> 8;
      page[6] = ANT_MODEL_NO & 0xff; // model number
      page[7] = ANT_MODEL_NO >> 8;
      break;

    case 0x51:
      page[0] = 0x51;
      page[1] = 0xff;
      page[2] = 0xff;
      page[3] = SW_VER; // SW version
      page[4] = device_serial & 0xff; // serial#
      page[5] = (device_serial >> 8) & 0xff;
      page[6] = (device_serial >> 16) & 0xff;
      page[7] = (device_serial >> 24) & 0xff;
      break;

    default:
      break;
  }
}

void ant_generate_data_page(ANT_Device_t *dev, uint8_t *page) {
  uint8_t process_ret = 1;

  switch (dev->device_type) {

    case ANT_DEVTYPE_POWER:
      // set page counter to 0xD0 so that we send the calibration response for a number of cycles before rolling over
      if (dev->last_request[0] == ANT_POWER_CALIBRATION_PAGE) {
        dev->page_counter = 0xd0;
        dev->last_request[0] = 0;
      }
      // 0x50 common page 1
      if (dev->page_counter == 60) {
        ant_generate_common_page(ANT_COMMON_PAGE1, page);
      // 0x51 common page 2
      } else if (dev->page_counter == 121) {
        ant_generate_common_page(ANT_COMMON_PAGE2, page);
      // calibration response (hacky way to use same process to reply to calibration - set page_counter to 0xd0 on request)
      } else if (dev->page_counter >= 0xd0) {
        // Calibration response
        page[0] = ANT_POWER_CALIBRATION_PAGE; // calibration message
        page[1] = 0xaf; // always fail since should use spin-down
        page[2] = 0xff; // no auto-zero
        page[3] = 0xff;
        page[4] = 0xff;
        page[5] = 0xff;
        page[6] = 0;
        page[7] = 0;
      } else {
        // fake power for now
        /* if (trainer.power < 500) { */
        /*   trainer.power += 10; */
        /* } else { */
        /*   trainer.power = 0; */
        /* } */
        // accumulate power and counter
        trainer.accumulated_power += trainer.power;
        trainer.accumulated_power_counter++;

        page[0] = ANT_POWER_STANDARD_PAGE; // general data
        page[1] = trainer.accumulated_power_counter;
        page[2] = 0xFF; // pedal power not used
        page[3] = 0xFF; // cadence not used
        page[4] = LOW_BYTE(trainer.accumulated_power); // accumulated power lsb
        page[5] = HIGH_BYTE(trainer.accumulated_power); // accumulated power hsb
        page[6] = LOW_BYTE(trainer.power); // instananous power lsb
        page[7] = HIGH_BYTE(trainer.power); // instananous power hsb
      }

      dev->page_counter++;

      if ((dev->page_counter > 121) && (dev->page_counter < 0xd0)) {
        dev->page_counter = 0;
      }
      break;

    case ANT_DEVTYPE_FEC:
      if (fec_device.last_request[0] != 0) {
        process_ret = trainer_process_request(dev->last_request, page);
        dev->last_request[0] = 0;
      }

      // carry on filling page if process request didn't return a page
      if (process_ret != 0) {
        if (dev->page_counter >= 0xd0) {
          trainer_generate_page(ANT_FEC_CALIBRATION_REQ, page);
        } else if (trainer.fsm == SPIN_DOWN) {
          // interweave calibration progress with general state
          if (dev->page_counter % 2 == 0) {
            trainer_generate_page(ANT_FEC_CALIBRATION_PROG, page);
          } else {
            trainer_generate_page(ANT_FEC_GENERAL_PAGE, page);
          }
        // 0x50 common page 1
        } else if (dev->page_counter == 60) {
          ant_generate_common_page(ANT_COMMON_PAGE1, page);
        // 0x51 common page 2
        } else if (dev->page_counter == 121) {
          ant_generate_common_page(ANT_COMMON_PAGE2, page);
        // every 20 show settings page
        } else if (dev->page_counter % 20 == 0) {
          trainer_generate_page(ANT_FEC_GENERAL_SET_PAGE, page);
        // every 5 show general page
        } else if (dev->page_counter % 5 == 0) {
          trainer_generate_page(ANT_FEC_GENERAL_PAGE, page);
        /* } else if (dev->page_counter % 6 == 0) { */
        /*   trainer_generate_page(ANT_FEC_KICKR_SIG, page); */
        // otherwise main data
        } else {
          // accumulate power and counter if power meter profile not running, otherwise leave it to that
          if (ant_device_active(&power_device)) {
            trainer.accumulated_power += trainer.power;
            trainer.accumulated_power_counter++;
          }

          page[0] = ANT_FEC_TRAINER_DATA_PAGE;
          page[1] = trainer.accumulated_power_counter;
          page[2] = trainer.cadence;
          page[3] = LOW_BYTE(trainer.accumulated_power); // accumulated power lsb
          page[4] = HIGH_BYTE(trainer.accumulated_power); // accumulated power hsb
          page[5] = LOW_BYTE(trainer.power < 4094 ? trainer.power : 4094); // instananous power lsb
          page[6] = (HIGH_BYTE(trainer.power < 4094 ? trainer.power : 4094) & 0x0F) | (trainer.status.byte << 4); // instananous power hsb 0:3 status 4:7
          page[7] = (trainer.flag & 0x0F) | (trainer.state << 4);
        }
      }

      dev->page_counter++;

      if ((dev->page_counter > 121) && (dev->page_counter < 0xd0)) {
        dev->page_counter = 0;
      }
      break;

    default:
      break;
  }
}

uint8_t ant_start_device(ANT_Device_t *dev, uint8_t channel) {
  if( dev->timer == NULL ) {
    return 1;
  } else {
    // set to assigned channel
    dev->channel = channel;
    xTimerStart(*dev->timer, 0);
    return 0;
  }
}

uint8_t ant_stop_device(ANT_Device_t *dev) {
  if( dev->timer == NULL ) {
    return 1;
  } else {
    xTimerStop(*dev->timer, 0);
    return 0;
  }

  // clear the LED as it was blinky in timer
  HAL_GPIO_WritePin(GLED_GPIO_Port, GLED_Pin, 1);
}

bool ant_device_active(ANT_Device_t *dev) {
  if( dev->timer == NULL ) {
    return 0;
  } else {
    return xTimerIsTimerActive(*dev->timer);
  }
}

uint8_t ant_any_channels_open(void) {
  for (int i = 0; i < FAKE_CHANNELS; i++) {
    if (ant_channels[i].open) {
      return 1;
    }
  }

  return 0;
}

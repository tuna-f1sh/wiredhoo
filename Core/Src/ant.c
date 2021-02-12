#include <string.h>
#include "main.h"
#include "utilities.h"
#include "antmessage.h"
#include "antdefines.h"
#include "ant.h"

static const uint8_t sAntNetworkKey[] = {
  ANT_NETWORK_KEY_0, ANT_NETWORK_KEY_1,
  ANT_NETWORK_KEY_2, ANT_NETWORK_KEY_3,
  ANT_NETWORK_KEY_4, ANT_NETWORK_KEY_5,
  ANT_NETWORK_KEY_6, ANT_NETWORK_KEY_7,
};
static const uint32_t device_serial = DEVICE_SERIAL;

ANT_MessageStatus process_ant_request(uint8_t *pBuffer, size_t len, uint8_t *pReply);

ANT_MessageStatus process_ant_message(uint8_t *pMessage, size_t len, uint8_t *pReply) {
  ANT_MessageStatus ret = MESG_OK;

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
        case MESG_SYSTEM_RESET_ID:
          pReply[1] = MESG_SYSTEM_RESET_SIZE;
          pReply[2] = MESG_START_UP;
          pReply[3] = 0x20; // watchdog reset
          pReply[4] = calculate_crc(pReply, ANT_MESSAGE_SIZE(pReply) - 1);
          break;
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
      ret = MESG_CRC_ERR;
    }
  } else {
    ret = MESG_SIZE_ERR;
  }

  return ret;
}

ANT_MessageStatus process_ant_request(uint8_t *pBuffer, size_t len, uint8_t *pReply) {
  ANT_MessageStatus ret = MESG_OK;
  uint8_t channel = pBuffer[BUFFER_INDEX_CHANNEL_NUM];
  uint8_t request = pBuffer[BUFFER_INDEX_MESG_DATA];

  switch (request) {
    case MESG_CAPABILITIES_ID:
      pReply[1] = MESG_CAPABILITIES_SIZE;
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
      /* pReply[7] = // advanced options 2 */
      /*   (0 << 0) | // led */
      /*   (1 << 1) | // extended message */
      /*   (1 << 2) | // scan mode */
      /*   (0 << 3) | // reserved */
      /*   (1 << 4) | // prox search */
      /*   (1 << 5) | // ext assign */
      /*   (0 << 6) | // ant fs */
      /*   (0 << 7); // fit1 */
      /* pReply[8] = 0x00; // sensRcore channels */
      /* pReply[9] = 0xDF; // advanced 3 */
      /* pReply[9] = 0xDF; // advanced 3 */
      break;
      // TODO reply with channel status info
      pReply[1] = MESG_CHANNEL_STATUS_SIZE;
      pReply[2] = MESG_CHANNEL_STATUS_ID;
      if (channel == 0) {
        pReply[3] = channel; // channel number
        pReply[4] = LOW_BYTE(POWER_ID);
      } else if (channel == 1) {
        pReply[3] = channel; // channel number
        pReply[4] = 0x00;
      } else {
        ret = MESG_INVALID_REQ;
      }
    case MESG_CHANNEL_STATUS_ID:
    case MESG_CHANNEL_ID_ID:
      // TODO reply with channel setup info
      pReply[1] = MESG_CHANNEL_ID_SIZE;
      pReply[2] = MESG_CHANNEL_ID_ID;
      if (channel == 0) {
        pReply[3] = channel; // channel number
        pReply[4] = LOW_BYTE(POWER_ID);
        pReply[5] = HIGH_BYTE(POWER_ID);
      } else if (channel == 1) {
        pReply[3] = channel; // channel number
        pReply[4] = LOW_BYTE(POWER_FC_ID);
        pReply[5] = HIGH_BYTE(POWER_FC_ID);
      } else {
        ret = MESG_INVALID_REQ;
      }
    case 0x61: // device serial
      pReply[1] = 0x04;
      pReply[2] = 0x61;
      memcpy(&pReply[3], &device_serial, 4);
      break;
    default:
      ret = MESG_UNKNOWN_ID;
      break;
  }

  return ret;
}

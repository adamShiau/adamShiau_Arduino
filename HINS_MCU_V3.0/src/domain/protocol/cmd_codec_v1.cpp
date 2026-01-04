// #include "domain/protocol/cmd_codec_v1.h"
#include "cmd_codec_v1.h"
#include "../../utils/endian.h"



void decode_cmd_v1(uint8_t* data, cmd_ctrl_t* rx)
{
  if (!data || !rx) return;

  rx->hins_payload = nullptr;
  rx->hins_payload_len = 0;

  rx->condition = (rx_condition_t)data[0];

  if (rx->condition == RX_CONDITION_ABBA_5556) {
    // payload: [cmd][v3][v2][v1][v0][ch]  (6 bytes total)
    rx->complete = 1;
    rx->cmd      = data[1];
    rx->value    = be_bytes_to_i32(data[2], data[3], data[4], data[5]);
    rx->ch       = data[6];

    DEBUG_PRINT("condition: %d, ", RX_CONDITION_ABBA_5556);
    DEBUG_PRINT("cmd: %x, value: %d, ch: %d\n", rx->cmd, rx->value, rx->ch);

  }
  else if (rx->condition == RX_CONDITION_CDDC_5758) {
    // payload: [cmd][SN[0..11]] (12 bytes SN, not including NUL)
    rx->complete = 1;
    rx->cmd      = data[1];

    for (int i = 0; i < 12; i++) {
      rx->SN[i] = data[i + 2];
    }
    rx->SN[12] = '\0';

  }
  else if (rx->condition == RX_CONDITION_EFFE_5354) {
    // payload: [cmd][v3][v2][v1][v0][ch]
    rx->complete = 1;
    rx->cmd      = data[1];
    rx->value    = be_bytes_to_i32(data[2], data[3], data[4], data[5]);
    rx->ch       = data[6];

    DEBUG_PRINT("condition: %d, ", RX_CONDITION_EFFE_5354);
    DEBUG_PRINT("cmd: %x, value: %d, ch: %d\n", rx->cmd, rx->value, rx->ch);
  }
  else if (rx->condition == RX_CONDITION_BCCB_5152) {
    // payload (scheme 4B): [cmd][len][params...len bytes]
    rx->complete = 1;
    rx->cmd      = data[1];
    rx->hins_payload_len = data[2];
    rx->hins_payload     = &data[3];

    // DEBUG_PRINT("condition: %d, ", RX_CONDITION_BCCB_5152);
    // DEBUG_PRINT("cmd: %x, hins_len: %d\n", rx->cmd, rx->hins_payload_len);
    DEBUG_PRINT("[HINS_PING] condition=%d cmd=0x%02X len=%u\r\n",
              rx->condition, rx->cmd, (unsigned)rx->hins_payload_len);

  }
  else {
    // Unknown condition; leave rx->complete unchanged (typically 0)
  }
}

#include "dump_service.h"
#include <ctype.h>
#include <string.h>

#include "../app/app_state.h"
#include "../drivers/link/nios_link.h"
#include "../domain/protocol/ack_codec_v1.h"

// 你 common.h 裡的 header/trailer 全域常數（目前是 extern）
#include "../common.h"
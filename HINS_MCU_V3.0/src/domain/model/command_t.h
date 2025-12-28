#pragma once
#include <stdint.h>

/**
 * @brief Canonical command model used by usecases.
 *
 * Protocol decoders (V1/V2/...) should translate raw bytes into this model so
 * usecases do not depend on framing/header/version details.
 *
 * NOTE: This is a skeleton added early (Step A). You don't need to wire it into
 * the main loop yet.
 */
typedef struct {
    uint8_t  id;        ///< Command ID / opcode
    int32_t  value;     ///< Command value (signed)
    uint8_t  ch;        ///< Channel / index
    uint8_t  flags;     ///< Optional: condition/flags byte from protocol
} command_t;

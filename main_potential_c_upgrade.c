#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define SOF 0x55
#define STUFFED_BYTE_AA 0xAA
#define STUFFED_BYTE_55 0x05
#define STUFFED_BYTE_AA_ESCAPE 0x0A
#define MTU 255

// Packed structures to minimize memory padding
#pragma pack(push, 1)
typedef struct {
    uint8_t *data;
    uint16_t length;
} Frame;

typedef struct {
    Frame *frame;
    uint8_t phase : 3;
    uint8_t flags : 5;  // Combined boolean flags
    uint16_t index;
    uint8_t pendingByte;
    uint8_t crc;
} FrameSerializer;

typedef struct {
    uint8_t state : 3;
    uint8_t awaitingUnstuffedByte : 1;
    uint8_t padding : 4;
    uint16_t index;
    uint8_t crc;
    Frame frame;
} FrameDeserializer;
#pragma pack(pop)

// Enum with explicit size
typedef enum __attribute__((packed)) {
    PHASE_SOF,
    PHASE_LENGTH,
    PHASE_PAYLOAD,
    PHASE_CRC,
    PHASE_EOF,
    PHASE_END
} Phase;

// Static CRC table to prevent modifications
uint8_t CRC_TABLE[256] = {
        0x00, 0x1D, 0x3A, 0x27, 0x74, 0x69, 0x4E, 0x53,
        0xE8, 0xF5, 0xD2, 0xCF, 0x9C, 0x81, 0xA6, 0xBB,
        0xCD, 0xD0, 0xF7, 0xEA, 0xB9, 0xA4, 0x83, 0x9E,
        0x25, 0x38, 0x1F, 0x02, 0x51, 0x4C, 0x6B, 0x76,
        0x87, 0x9A, 0xBD, 0xA0, 0xF3, 0xEE, 0xC9, 0xD4,
        0x6F, 0x72, 0x55, 0x48, 0x1B, 0x06, 0x21, 0x3C,
        0x4A, 0x57, 0x70, 0x6D, 0x3E, 0x23, 0x04, 0x19,
        0xA2, 0xBF, 0x98, 0x85, 0xD6, 0xCB, 0xEC, 0xF1,
        0x13, 0x0E, 0x29, 0x34, 0x67, 0x7A, 0x5D, 0x40,
        0xFB, 0xE6, 0xC1, 0xDC, 0x8F, 0x92, 0xB5, 0xA8,
        0xDE, 0xC3, 0xE4, 0xF9, 0xAA, 0xB7, 0x90, 0x8D,
        0x36, 0x2B, 0x0C, 0x11, 0x42, 0x5F, 0x78, 0x65,
        0x94, 0x89, 0xAE, 0xB3, 0xE0, 0xFD, 0xDA, 0xC7,
        0x7C, 0x61, 0x46, 0x5B, 0x08, 0x15, 0x32, 0x2F,
        0x59, 0x44, 0x63, 0x7E, 0x2D, 0x30, 0x17, 0x0A,
        0xB1, 0xAC, 0x8B, 0x96, 0xC5, 0xD8, 0xFF, 0xE2,
        0x26, 0x3B, 0x1C, 0x01, 0x52, 0x4F, 0x68, 0x75,
        0xCE, 0xD3, 0xF4, 0xE9, 0xBA, 0xA7, 0x80, 0x9D,
        0xEB, 0xF6, 0xD1, 0xCC, 0x9F, 0x82, 0xA5, 0xB8,
        0x03, 0x1E, 0x39, 0x24, 0x77, 0x6A, 0x4D, 0x50,
        0xA1, 0xBC, 0x9B, 0x86, 0xD5, 0xC8, 0xEF, 0xF2,
        0x49, 0x54, 0x73, 0x6E, 0x3D, 0x20, 0x07, 0x1A,
        0x6C, 0x71, 0x56, 0x4B, 0x18, 0x05, 0x22, 0x3F,
        0x84, 0x99, 0xBE, 0xA3, 0xF0, 0xED, 0xCA, 0xD7,
        0x35, 0x28, 0x0F, 0x12, 0x41, 0x5C, 0x7B, 0x66,
        0xDD, 0xC0, 0xE7, 0xFA, 0xA9, 0xB4, 0x93, 0x8E,
        0xF8, 0xE5, 0xC2, 0xDF, 0x8C, 0x91, 0xB6, 0xAB,
        0x10, 0x0D, 0x2A, 0x37, 0x64, 0x79, 0x5E, 0x43,
        0xB2, 0xAF, 0x88, 0x95, 0xC6, 0xDB, 0xFC, 0xE1,
        0x5A, 0x47, 0x60, 0x7D, 0x2E, 0x33, 0x14, 0x09,
        0x7F, 0x62, 0x45, 0x58, 0x0B, 0x16, 0x31, 0x2C,
        0x97, 0x8A, 0xAD, 0xB0, 0xE3, 0xFE, 0xD9, 0xC4 };

// Inline critical functions
static inline void crc_reset(uint8_t *crc) {
    *crc = 0xFF;
}

static inline void crc_write(uint8_t *crc, uint8_t input) {
    *crc = CRC_TABLE[*crc ^ input];
}

// Safe memory allocation with error checking
static uint8_t* safe_malloc(size_t size) {
    uint8_t *ptr = malloc(size);
    if (!ptr) {
        fprintf(stderr, "Memory allocation failed\n");
        exit(EXIT_FAILURE);
    }
    return ptr;
}

Frame create_frame(const uint8_t *data, uint16_t length) {
    if (length > MTU) {
        fprintf(stderr, "Frame length exceeds MTU\n");
        exit(EXIT_FAILURE);
    }

    Frame frame;
    frame.data = safe_malloc(length);
    frame.length = length;
    memcpy(frame.data, data, length);
    return frame;
}

// Optimized byte stuffing without temporary array
static inline uint8_t stuff_byte(uint8_t input, uint8_t *extra, bool *has_extra) {
    if (input == 0x55) {
        *extra = STUFFED_BYTE_55;
        *has_extra = true;
        return STUFFED_BYTE_AA;
    } else if (input == 0xAA) {
        *extra = STUFFED_BYTE_AA_ESCAPE;
        *has_extra = true;
        return STUFFED_BYTE_AA;
    }
    return input;
}

FrameSerializer create_serializer(Frame *frame) {
    FrameSerializer serializer = {0};  // Zero-initialize
    serializer.frame = frame;
    serializer.phase = PHASE_SOF;
    crc_reset(&serializer.crc);
    return serializer;
}

bool serializer_has_next(const FrameSerializer *serializer) {
    return (serializer->flags & 0x01) || serializer->phase != PHASE_END;
}

uint8_t serializer_next(FrameSerializer *serializer) {
    if (serializer->flags & 0x01) {
        serializer->flags &= ~0x01;  // Clear pending byte flag
        return serializer->pendingByte;
    }

    uint8_t output;
    bool has_extra = false;

    switch (serializer->phase) {
        case PHASE_SOF:
            serializer->phase = PHASE_LENGTH;
            return SOF;

        case PHASE_LENGTH:
            serializer->phase = serializer->frame->length > 0 ? PHASE_PAYLOAD : PHASE_CRC;
            crc_write(&serializer->crc, serializer->frame->length);
            return serializer->frame->length;

        case PHASE_PAYLOAD:
            output = stuff_byte(serializer->frame->data[serializer->index++],
                                &serializer->pendingByte, &has_extra);
            if (serializer->index >= serializer->frame->length) {
                serializer->phase = PHASE_CRC;
            }
            crc_write(&serializer->crc, serializer->frame->data[serializer->index - 1]);
            break;

        case PHASE_CRC:
            output = stuff_byte(serializer->crc, &serializer->pendingByte, &has_extra);
            serializer->phase = PHASE_EOF;
            break;

        case PHASE_EOF:
            serializer->phase = PHASE_END;
            return SOF;

        default:
            return 0;
    }

    if (has_extra) {
        serializer->flags |= 0x01;  // Set pending byte flag
    }
    return output;
}

FrameDeserializer create_deserializer(void) {
    FrameDeserializer deserializer = {0};  // Zero-initialize
    crc_reset(&deserializer.crc);
    deserializer.frame.data = safe_malloc(MTU);
    return deserializer;
}

void free_frame(Frame *frame) {
    if (frame && frame->data) {
        free(frame->data);
        frame->data = NULL;
        frame->length = 0;
    }
}

void free_deserializer(FrameDeserializer *deserializer) {
    if (deserializer) {
        free_frame(&deserializer->frame);
    }
}

// Error handling return codes
typedef enum {
    DESERIALIZE_OK = 0,
    DESERIALIZE_CONTINUE = 1,
    DESERIALIZE_ERROR = -1
} DeserializeResult;

DeserializeResult deserialize(FrameDeserializer *deserializer, uint8_t input) {
    if (deserializer->awaitingUnstuffedByte) {
        deserializer->awaitingUnstuffedByte = 0;
        switch (input) {
            case STUFFED_BYTE_55:
                input = 0x55;
                break;
            case STUFFED_BYTE_AA_ESCAPE:
                input = 0xAA;
                break;
            default:
                return DESERIALIZE_ERROR;
        }
    } else if (input == STUFFED_BYTE_AA) {
        deserializer->awaitingUnstuffedByte = 1;
        return DESERIALIZE_CONTINUE;
    }

    switch (deserializer->state) {
        case PHASE_SOF:
            if (input == SOF) {
                deserializer->state = PHASE_LENGTH;
                deserializer->index = 0;
            }
            break;

        case PHASE_LENGTH:
            if (input > MTU) {
                return DESERIALIZE_ERROR;
            }
            deserializer->frame.length = input;
            crc_write(&deserializer->crc, input);
            deserializer->state = input > 0 ? PHASE_PAYLOAD : PHASE_CRC;
            break;

        case PHASE_PAYLOAD:
            if (deserializer->index >= MTU) {
                return DESERIALIZE_ERROR;
            }
            deserializer->frame.data[deserializer->index++] = input;
            crc_write(&deserializer->crc, input);
            if (deserializer->index >= deserializer->frame.length) {
                deserializer->state = PHASE_CRC;
            }
            break;

        case PHASE_CRC:
            if (deserializer->crc != input) {
                return DESERIALIZE_ERROR;
            }
            deserializer->state = PHASE_SOF;
            return DESERIALIZE_OK;
    }
    return DESERIALIZE_CONTINUE;
}

void print_frame(const Frame *frame) {
    printf("Frame[data=");
    for (uint16_t i = 0; i < frame->length; i++) {
        printf("%02X%s", frame->data[i], i < frame->length - 1 ? "," : "");
    }
    printf("]\n");
}

int main() {
    uint8_t data[] = {0x66, 0x55, 0x77, 0x88, 0x99, 0xAA};
    Frame frame = create_frame(data, sizeof(data));

    uint8_t serializedData[MTU * 2];
    uint16_t serializedLength = 0;

    FrameSerializer serializer = create_serializer(&frame);
    printf("Serialized data: ");

    while (serializer_has_next(&serializer) && serializedLength < sizeof(serializedData)) {
        serializedData[serializedLength] = serializer_next(&serializer);
        printf("%02X ", serializedData[serializedLength++]);
    }
    printf("\n");

    printf("Deserialized output: ");
    FrameDeserializer deserializer = create_deserializer();

    for (uint16_t i = 0; i < serializedLength; i++) {
        DeserializeResult result = deserialize(&deserializer, serializedData[i]);
        if (result == DESERIALIZE_ERROR) {
            fprintf(stderr, "Deserialization error at byte %d\n", i);
            break;
        }
        if (result == DESERIALIZE_OK) {
            print_frame(&deserializer.frame);
        }
    }

    free_frame(&frame);
    free_deserializer(&deserializer);
    return 0;
}
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

typedef struct {
    uint8_t *data;
    int length;
} Frame;

typedef struct {
    Frame *frame;
    int phase;
    int index;
    bool hasPendingByte;
    uint8_t pendingByte;
    uint8_t crc;
    bool sendEOF;
} FrameSerializer;

enum Phase { PHASE_SOF, PHASE_LENGTH, PHASE_PAYLOAD, PHASE_CRC, PHASE_EOF, PHASE_END };

typedef struct {
    int state;
    Frame frame;
    int index;
    uint8_t crc;
    bool awaitingUnstuffedByte;
} FrameDeserializer;

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

// CRC8 functions
void crc_reset(uint8_t *crc) {
    *crc = 0xFF;
}

void crc_write(uint8_t *crc, uint8_t input) {
    *crc = CRC_TABLE[*crc ^ input];
}

// Frame functions
Frame create_frame(uint8_t *data, int length) {
    Frame frame;
    frame.data = (uint8_t *)malloc(length);
    memcpy(frame.data, data, length);
    frame.length = length;
    return frame;
}

void free_frame(Frame *frame) {
    free(frame->data);
}

void print_frame(const Frame *frame) {
    printf("Frame[data=");
    for (int i = 0; i < frame->length; i++) {
        printf("%02X,", frame->data[i]);
    }
    printf("]\n");
}

// Byte stuffing
int8_t handleByteStuffing(uint8_t input, uint8_t *stuffedBytes) {
    if (input == 0x55) {
        stuffedBytes[0] = 0xAA;
        stuffedBytes[1] = 0x05;
        return 2;
    } else if (input == 0xAA) {
        stuffedBytes[0] = 0xAA;
        stuffedBytes[1] = 0x0A;
        return 2;
    } else {
        stuffedBytes[0] = input;
        return 1;
    }
}

// FrameSerializer functions
FrameSerializer create_serializer(Frame *frame, bool sendEOF) {
    FrameSerializer serializer;
    serializer.frame = frame;
    serializer.phase = PHASE_SOF;
    serializer.index = 0;
    serializer.hasPendingByte = false;
    serializer.pendingByte = 0;
    crc_reset(&serializer.crc);
    serializer.sendEOF = sendEOF;
    return serializer;
}

bool serializer_has_next(FrameSerializer *serializer) {
    return serializer->hasPendingByte || serializer->phase != PHASE_END;
}

uint8_t serializer_next(FrameSerializer *serializer) {
    uint8_t stuffedBytes[2];
    if (serializer->hasPendingByte) {
        serializer->hasPendingByte = false;
        return serializer->pendingByte;
    }
    switch (serializer->phase) {
        case PHASE_SOF:
            serializer->phase = PHASE_LENGTH;
            return SOF;
        case PHASE_LENGTH:
            serializer->phase = serializer->frame->length > 0 ? PHASE_PAYLOAD : PHASE_CRC;
            uint8_t lengthByte = serializer->frame->length;
            crc_write(&serializer->crc, lengthByte);
            return lengthByte;
        case PHASE_PAYLOAD:
            if (serializer->index >= serializer->frame->length - 1) serializer->phase = PHASE_CRC;
            uint8_t payloadByte = serializer->frame->data[serializer->index++];
            crc_write(&serializer->crc, payloadByte);
            int stuffedLength = handleByteStuffing(payloadByte, stuffedBytes);
            if (stuffedLength == 2) {
                serializer->hasPendingByte = true;
                serializer->pendingByte = stuffedBytes[1];
            }
            return stuffedBytes[0];
        case PHASE_CRC:
            serializer->phase = serializer->sendEOF ? PHASE_EOF : PHASE_END;
            int crcStuffedLength = handleByteStuffing(serializer->crc, stuffedBytes);
            if (crcStuffedLength == 2) {
                serializer->hasPendingByte = true;
                serializer->pendingByte = stuffedBytes[1];
            }
            return stuffedBytes[0];
        case PHASE_EOF:
            serializer->phase = PHASE_END;
            return SOF;
        default:
            return 0;
    }
}

// FrameDeserializer functions
FrameDeserializer create_deserializer() {
    FrameDeserializer deserializer;
    deserializer.state = PHASE_SOF;
    deserializer.index = 0;
    crc_reset(&deserializer.crc);
    deserializer.awaitingUnstuffedByte = false;
    deserializer.frame.data = (uint8_t *)malloc(MTU);
    return deserializer;
}

void deserialize(FrameDeserializer *deserializer, uint8_t input) {
    if (deserializer->awaitingUnstuffedByte) {
        deserializer->awaitingUnstuffedByte = false;
        if (input == STUFFED_BYTE_55) {
            input = 0x55;
        } else if (input == STUFFED_BYTE_AA_ESCAPE) {
            input = 0xAA;
        }
    } else if (input == STUFFED_BYTE_AA) {
        deserializer->awaitingUnstuffedByte = true;
        return;
    }

    switch (deserializer->state) {
        case PHASE_SOF:
            if (input == SOF) {
                deserializer->state = PHASE_LENGTH;
                deserializer->index = 0;
            }
            break;
        case PHASE_LENGTH:
            deserializer->frame.length = input;
            crc_write(&deserializer->crc, input);
            deserializer->state = deserializer->frame.length > 0 ? PHASE_PAYLOAD : PHASE_CRC;
            break;
        case PHASE_PAYLOAD:
            deserializer->frame.data[deserializer->index++] = input;
            crc_write(&deserializer->crc, input);
            if (deserializer->index >= deserializer->frame.length) deserializer->state = PHASE_CRC;
            break;
        case PHASE_CRC:
            if (deserializer->crc == input) {
                printf("Deserialized frame: ");
                print_frame(&deserializer->frame);
            } else {
                printf("CRC error in deserialized frame.\n");
            }
            deserializer->state = PHASE_SOF;
            break;
    }
}

// Main function to test serialization and deserialization
int main() {
    uint8_t data[] = {0x66, 0x55, 0x77, 0x88, 0x99, 0xAA};
    Frame frame = create_frame(data, sizeof(data) / sizeof(data[0]));

    // Serialize the frame
    FrameSerializer serializer = create_serializer(&frame, true);
    printf("Serialized data: ");
    uint8_t serializedData[MTU * 2];
    int serializedLength = 0;

    while (serializer_has_next(&serializer)) {
        uint8_t byte = serializer_next(&serializer);
        serializedData[serializedLength++] = byte;
        printf("%02X ", byte);
    }
    printf("\n");

    // Deserialize the serialized data
    printf("Deserialized output:\n");
    FrameDeserializer deserializer = create_deserializer();
    for (int i = 0; i < serializedLength; i++) {
        deserialize(&deserializer, serializedData[i]);
    }

    // Free resources
    free_frame(&frame);
    free_frame(&deserializer.frame);
    return 0;
}

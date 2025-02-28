#include <iostream>
#include "data_link.h"
#include "network_layer.h"

// make a main that connect the process between layers
// basically session draft of session manager ??

// logic
// things are waiting for bytes and either go up or down the pipeline

//top down
//application - takes an input makes ID + Payload (FRAME)
//network - takes a Frame makes chunks
//data link - takes chunks makes bytes
//uart - takes bytes makes electrical signals


//bottom up
//uart - takes signals makes byte
//data link - takes stream of bytes makes chunk
//network - takes chunk makes frame
//application - takes frame makes instructions






// this is the test runs we had in the individual classes that I moved here
// to be deleted or commented out
int main() {
// test frames given by josh
    uint8_t data[] = {0x55, 0x66, 0x77,0x88, 0x99, 0xAA};
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


    // Test Cases
    uint8_t testData1[] = {0x88, 0x66, 0x55, 0x77, 0x88, 0x99, 0xAA};
    test_serialization_and_deserialization(testData1, sizeof(testData1));

    uint8_t testData2[] = {}; // Empty data
    test_serialization_and_deserialization(testData2, sizeof(testData2));
    // here the data gets serialized to 55, check sum, check sum, 55 -> so when deserialize its just empty
    // question would be,  do we want it to do the serialising in the first place ?

    uint8_t testData3[] = {0x55, 0xAA, 0x55, 0xAA}; // All special characters
    test_serialization_and_deserialization(testData3, sizeof(testData3));

    uint8_t testData4[] = {0x01, 0x02, 0x03, 0x04, 0x05}; // Simple sequence
    test_serialization_and_deserialization(testData4, sizeof(testData4));

    uint8_t testData5[MTU]; // Maximum allowed data size
    for (uint16_t i = 0; i < MTU; ++i) {
        testData5[i] = static_cast<uint8_t>(i);
    }
    test_serialization_and_deserialization(testData5, sizeof(testData5));

    std::cout << "All tests completed.\n";


    // network layer part

    // Example 1: Create a local packet with GPS data
    Packet gps_packet;
    uint8_t gps_data[] = {0x12, 0x34, 0x56, 0x78}; // Example GPS coordinates
    if (packet_create(&gps_packet, true, PID_RAC_GPS, gps_data, sizeof(gps_data))) {
        print_packet(&gps_packet);
    }

    // Example 2: Create a broadcast packet with temperature data
    Packet temp_packet;
    uint8_t temp_data[] = {0x25}; // Example temperature value
    if (packet_create(&temp_packet, false, PID_BAC_TEMP, temp_data, sizeof(temp_data))) {
        print_packet(&temp_packet);
    }
    

    return 0;
}

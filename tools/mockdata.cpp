//mosquitto_pub -h localhost -t 'test/topic0'  -f enable_buttone_1.dat
#include <stdio.h>

#pragma pack(1)
unsigned char robot_packet_disable[] =
        {
                /* packet index */
                0x00, 0x01,0x01,
                /* Add control code, request flags and team station */
                0x02, 0x04,0x01,

                /*Encode date/time in datagram */
                0x0f,
                0x00, 0x04,0x01,
                0x06, 0x04,0x01,
                0x06, 0x04,0x01,
                0x06, 0x04,0x01,

                0x01, //joystick_size
                0x0c, //TagJoystick

                /* Add axis data */
                0x04, //JoystickNumAxes
                0x06, 0x04,0x01,0x30,

                /* Add button data */
                0x0c, //JoystickNumButtons
                0x00, 0x00,

                /* Add hat data */
                0x04, //JoystickNumHats
                0x00, 0x01,
                0x00, 0x01,
                0x00, 0x01,
                0x00, 0x01
        };


unsigned char robot_packet_enable_button_1[] =
        {
                /* packet index */
                0x00, 0x01,0x01,
                /* Add control code, request flags and team station */
                0x23, 0x04,0x01,

                /*Encode date/time in datagram */
                0x0f,
                0x00, 0x04,0x01,
                0x06, 0x04,0x01,
                0x06, 0x04,0x01,
                0x06, 0x04,0x01,

                0x01, //joystick_size
                0x0c, //TagJoystick

                /* Add axis data */
                0x04, //JoystickNumAxes
                0x06, 0x04,0x01,0x30,

                /* Add button data */
                0x0c, //JoystickNumButtons
                0x00, 0x01,

                /* Add hat data */
                0x04, //JoystickNumHats
                0x00, 0x01,
                0x00, 0x01,
                0x00, 0x01,
                0x00, 0x01
        };


unsigned char robot_packet_enable_button_2[] =
        {
                /* packet index */
                0x00, 0x01,0x01,
                /* Add control code, request flags and team station */
                0x23, 0x04,0x01,

                /*Encode date/time in datagram */
                0x0f,
                0x00, 0x04,0x01,
                0x06, 0x04,0x01,
                0x06, 0x04,0x01,
                0x06, 0x04,0x01,

                0x01, //joystick_size
                0x0c, //TagJoystick

                /* Add axis data */
                0x04, //JoystickNumAxes
                0x06, 0x04,0x01,0x30,

                /* Add button data */
                0x0c, //JoystickNumButtons
                0x00, 0x02,

                /* Add hat data */
                0x04, //JoystickNumHats
                0x00, 0x01,
                0x00, 0x01,
                0x00, 0x01,
                0x00, 0x01
        };

#pragma pack()

int main() {
    FILE *file_pointer;

    // Open the file in binary write mode ("wb")
    file_pointer = fopen("enable_buttone_1.dat", "wb");

    if (file_pointer == NULL) {
        perror("Error opening file");
        return 1;
    }

    size_t len = sizeof(robot_packet_enable_button_1)/sizeof(unsigned char);

    // Write the struct to the file
    size_t bytes_written = fwrite(robot_packet_enable_button_1, sizeof(unsigned char), sizeof(robot_packet_enable_button_1)/sizeof(unsigned char), file_pointer);

    if (bytes_written != len) {
        perror("Error writing to file");
        fclose(file_pointer);
        return 1;
    }
    // Close the file
    fclose(file_pointer);


    // Open the file in binary write mode ("wb")
    file_pointer = fopen("enable_buttone_2.dat", "wb");

    if (file_pointer == NULL) {
        perror("Error opening file");
        return 1;
    }

    // Write the struct to the file
    bytes_written = fwrite(robot_packet_enable_button_2,sizeof(unsigned char), sizeof(robot_packet_enable_button_2)/sizeof(unsigned char), file_pointer);

    if (bytes_written != len) {
        perror("Error writing to file");
        fclose(file_pointer);
        return 1;
    }

    // Close the file
    fclose(file_pointer);

    // Open the file in binary write mode ("wb")
    file_pointer = fopen("disable.dat", "wb");

    if (file_pointer == NULL) {
        perror("Error opening file");
        return 1;
    }

    // Write the struct to the file
    bytes_written = fwrite(robot_packet_disable, sizeof(unsigned char), sizeof(robot_packet_disable)/sizeof(unsigned char), file_pointer);

    if (bytes_written != len) {
        perror("Error writing to file");
        fclose(file_pointer);
        return 1;
    }
    // Close the file
    fclose(file_pointer);

    printf("Data written to binary file successfully.\n");
    return 0;
}

//
// Created by fulva1230 on 2020/8/25.
//

#ifndef ROSSERIAL_MBED_HARDWAREIMPL_H
#define ROSSERIAL_MBED_HARDWAREIMPL_H

#include <chrono>
#include <vector>

#include <USBSerial.h>
#include <mbed.h>

//TODO change the implementation to adapt some protocols
USBSerial bufferedSerial{true, 0x1f00, 0x2012, 0x0001};
constexpr int BUFFER_SIZE = 20;

class HardwareImpl {
public:
    explicit HardwareImpl() {

    }

    // any initialization code necessary to use the serial port
    void init() {
        bufferedSerial.init();
        bufferedSerial.set_blocking(false);
        timer.start();
    }

    // read a byte from the serial port. -1 = failure
    int read() {
        if (readindex >= bufferBoundary) {
            int avail = bufferedSerial.available();
            if (avail > BUFFER_SIZE) bufferBoundary = bufferedSerial.read(&buffer, BUFFER_SIZE);
            else bufferBoundary = bufferedSerial.read(&buffer, avail);
            if (bufferBoundary > 0) {
                readindex = 0;
            } else {
                return -1;
            }
        }
        uint8_t returnByte = buffer[readindex];
        ++readindex;
        return returnByte;
    }

    // write data to the connection to ROS
    void write(uint8_t *data, int length) {
        bufferedSerial.write(data, length);
    }

    // returns milliseconds since start of program
    unsigned long time() {
        return timer.read_ms();
    }
private:
    Timer timer;
    char buffer[BUFFER_SIZE];
    int bufferBoundary{0};
    int readindex{0};
};

#endif //ROSSERIAL_MBED_HARDWAREIMPL_H

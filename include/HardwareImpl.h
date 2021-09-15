//
// Created by fulva1230 on 2020/8/25.
//

#ifndef ROSSERIAL_MBED_HARDWAREIMPL_H
#define ROSSERIAL_MBED_HARDWAREIMPL_H

#include <chrono>
#include <vector>

#include <USBSerial.h>
#include <mbed.h>

template<class T, size_t N>
class CycleBuffer {
public:
    std::pair<T *, size_t> remaining_content() {
        if (cross_boundary) {
            int output_i = 0;
            for (int i = begin; i < N; ++i) {
                _outputBuffer[output_i] = _buffer[i];
                ++output_i;
            }
            for (int i = 0; i < end; ++i) {
                _outputBuffer[output_i] = _buffer[i];
                ++output_i;
            }
            return {_outputBuffer, output_i};
        } else {
            return {_buffer + begin, end - begin};
        }
    }

    void consume(int num) {
        begin = begin + num;
        if (begin >= N) {
            cross_boundary = false;
            begin = begin % N;
        }
    }

    void insert(T *buffer, int length) {
        for (int i = 0; i < length; ++i) {
            _buffer[end] = buffer[i];
            ++end;
            if (end == N) {
                cross_boundary = true;
                end = 0;
            }
        }
    }

private:
    size_t begin = 0;
    size_t end = 0;
    bool cross_boundary = false;
    T _buffer[N];
    T _outputBuffer[N];
};

USBSerial bufferedSerial{true, 0x1f00, 0x2012, 0x0001};
constexpr int BUFFER_SIZE = 200;

class HardwareImpl {
private:
    Timer timer;
    char buffer[BUFFER_SIZE];
    int bufferBoundary{0};
    int readindex{0};
    CycleBuffer<uint8_t, BUFFER_SIZE> write_buffer{};
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
        write_buffer.insert(data, length);
        auto cur_write_buffer = write_buffer.remaining_content();
        auto written_num = bufferedSerial.write(cur_write_buffer.first, cur_write_buffer.second);
        write_buffer.consume(written_num);
    }

    // returns milliseconds since start of program
    unsigned long time() {
        return timer.read_ms();
    }
};

#endif //ROSSERIAL_MBED_HARDWAREIMPL_H

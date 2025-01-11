#pragma once

#include <stdio.h>
#include <unistd.h>

#define VEX_USB_BAUD_RATE 115200

enum MessageDictionary {

};

namespace vpp {
    enum Message {

    };

    class Dashboard {
    private:
        FILE *file = nullptr;

    public:
        Tuner();
        ~Tuner();

        void open() {
            this->file = *fopen("/dev/serial1", "w");
        };

        void close() {
            fclose(this->file);
        };

        void send(Message id, char[] payload) {
            if (file == nullptr) {
                open();
            }
            FILE *fp = fopen("/dev/serial1", "w");
            fwrite(msg, 1, strlen(msg), fp);
            fclose(fp);
        };
    };
}  // namespace vpp
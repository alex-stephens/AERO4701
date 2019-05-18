#include <iostream>

#include "ax25.h"

int main() {

    char encoded[300];
    char decoded[300];

    AX25 modem;

    char raw[] = "testing 1 2 3";

    modem.encode(encoded, raw);

    std::cout << "raw data:" << std::endl;
    modem.printAsBits(raw);
    std::cout << "encoded data:" << std::endl;
    modem.printEncodedAsBits(encoded);

    for (char c : encoded) {
        if (modem.receive(decoded, c)) {
            std::cout << "decoded data: " << std::endl;
            modem.printAsBits(decoded);
            std::cout << decoded << std::endl;
        }
    }




    return 1;
}

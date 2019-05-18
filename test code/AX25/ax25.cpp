#include "ax25.h"

AX25::AX25() {
    std::cout << "constructed AX25 modem" << std::endl;
}

void AX25::encode(char* encoded, char* data) {
    int i = 0, j = 0, rep = 0;
    int ins = 0; // index to insert at
    int i2, j2;
    char mask;
    char c = data[0];

    std::cout << "data: " << data << std::endl;

    // start with 01111110
    insertToEncoded(encoded, ins++, 0);
    for (j = 0; j < 6; j++) {
        insertToEncoded(encoded, ins++, 1);
    }
    insertToEncoded(encoded, ins++, 0);

    while (c) {
        
        for (j = 0; j < 8; j++){
            mask = 0x01 << (7-j);

            // number of repeated 1s
            if (mask & c) {
                rep++;
            }
            else {
                rep = 0;
            }

            if (mask & c) {
                insertToEncoded(encoded, ins, 1);
            }
            ins++;

            // if this was the 5th 1, add a 0 to the encoding
            if (rep == 5) {
                ins++;
            }
        }

        c = data[++i];
    }

    // end with 01111110
    while (ins % 8 != 0) {
        ins++;
    }
    insertToEncoded(encoded, ins++, 0);
    for (j = 0; j < 6; j++) {
        insertToEncoded(encoded, ins++, 1);
    }
    insertToEncoded(encoded, ins++, 0);
}

void AX25::decode(char* decoded) {
    char current_char = 0x00;
    int ic = 0;
    char consecutive_ones = 0;
    int ins = 0;

    char c = buffer[0];
    char mask;
    int i = 0;

    while (c != '~') {

        for (int j = 0; j < 8; j++){
            mask = 0x01 << (7-j);

            // ignore the 0 after five 1s
            if (consecutive_ones == 5 && !(mask & c)) {
                continue;
            }

            // number of repeated 1s
            if (mask & c) {
                consecutive_ones++;
            }
            else {
                consecutive_ones = 0;
            }

            if (mask & c) {
                current_char |= (0x01 << (7-ic));
            }
            ic = (ic + 1) % 8;
            if (ic == 0) {
                decoded[ins++] = current_char;
                current_char = 0x00;
            }
        }

        c = buffer[++i];
    }



}

bool AX25::receive(char* decoded, char c) {
    if (c == '~') {
        // null terminate and decode the buffer
        buffer[buf_ind++] = '~';
        buffer[buf_ind++] = '\0';

        if (buf_ind <= 10){
            buf_ind = 0;
            return false;// only decode if buffer is long enough
        }

        decode(decoded);

        buf_ind = 0;
        return true;
    }

    else {
        buffer[buf_ind++] = c;

        if (buf_ind >= 300) {
            std::cout << "MAX BUFFER SIZE EXCEEDED" << std::endl;
        }
    }

    return false;
}


void AX25::insertToEncoded(char* encoded, int ins, char val) {

    if (val == 0) {
        return;
    }

    char i = (int) ins / 8;
    char j = ins % 8;

    if (val == 1) {
        encoded[i] |= 0x01 << (7-j);
    }
}

void AX25::printAsBits(char* data) {
    char c = data[0];
    char mask;
    int i = 0;
    int j;

    while (c) {

        for (j = 0; j < 8; j++){
            mask = 0x01 << (7-j);
            if (mask & c) {
                std::cout << 1;
            }
            else {
                std::cout << 0;
            }
        }
        std::cout << " ";

        c = data[++i];
    }
    std::cout << std::endl;
}

void AX25::printEncodedAsBits(char* data) {
    char c = data[0];
    char mask;
    int i = 0;
    int j;

    while (true) {

        for (j = 0; j < 8; j++){
            mask = 0x01 << (7-j);
            if (mask & c) {
                std::cout << 1;
            }
            else {
                std::cout << 0;
            }
        }
        std::cout << " ";

        if (c == '~' && i > 0) {
            break;
        }
        c = data[++i];
    }
    std::cout << std::endl;
}

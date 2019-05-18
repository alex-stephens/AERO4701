#ifndef AX25_H
#define AX25_H

#include <iostream>

class AX25 {
public:

    AX25();

    void encode(char* encoded, char* data);
    bool receive(char* decoded, char c);
    void decode(char* decoded);

    void insertToEncoded(char* encoded, int ins, char val);

    void printAsBits(char* data);
    void printEncodedAsBits(char* data);


private:

    char buffer[300];
    int buf_ind = 0;
};

#endif

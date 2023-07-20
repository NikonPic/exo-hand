// read_mux.h

#ifndef READ_MUX_H
#define READ_MUX_H

// input-pin multiplexer
    #define muxSIG 36
 // control-pins multiplexer
    #define muxS0 12
    #define muxS1 14
    #define muxS2 27
    #define muxS3 26

int read_mux(int channel);

#endif
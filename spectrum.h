#ifndef SPECTRUM_H
#define SPECTRUM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "fft.h"

enum {
    SP_CHAN_LEFT = 0,
    SP_CHAN_RIGHT,

    SP_CHAN_END
};

typedef enum {
    SP_MODE_STEREO = 0,
    SP_MODE_MIXED,
    SP_MODE_WATERFALL,

    SP_MODE_END
} SpMode;

#define SPECTRUM_SIZE   128
#define N_HANN          1024
#define N_DB            256

typedef struct {
    uint8_t raw[SPECTRUM_SIZE];
} SpChan;

typedef struct {
    SpChan chan[SP_CHAN_END];

    SpMode mode;
    int16_t wtfX;  // waterfall X position
    bool ready;
    bool redraw;
} Spectrum;

void spInit(void);
Spectrum *spGet(void);

void spGetADC(Spectrum *sp);

void spConvertADC(void);

#ifdef __cplusplus
}
#endif

#endif // SPECTRUM_H

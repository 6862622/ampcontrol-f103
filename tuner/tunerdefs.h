#ifndef TUNERDEFS_H
#define TUNERDEFS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define TUNER_DIR_UP        (1)
#define TUNER_DIR_DOWN      (-1)

#define TUNER_VOLUME_MIN    0
#define TUNER_VOLUME_MAX    15

typedef enum {
    TUNER_IC_NO = 0,
    TUNER_IC_RDA5807,
    TUNER_IC_SI4703,
    TUNER_IC_TEA5767,

    TUNER_IC_TEST,

    TUNER_IC_END
} TunerIC;

typedef uint16_t TunerFlag;
enum {
    TUNER_FLAG_INIT     = 0x0000,

    TUNER_FLAG_READY    = 0x0001, // Ready (seek/tune complete)
    TUNER_FLAG_STEREO   = 0x0002, // Stereo reception
    TUNER_FLAG_BANDLIM  = 0x0004, // Band limit reached

    TUNER_FLAG_SEEKUP   = 0x0010, // Seek up in progress
    TUNER_FLAG_SEEKDOWN = 0x0020, // Seek down in progress
};

typedef enum {
    TUNER_BAND_FM_US_EUROPE,  // 87..108 MHz
    TUNER_BAND_FM_JAPAN,      // 76..97 MHz
    TUNER_BAND_FM_WORLDWIDE,  // 79..108 MHz
    TUNER_BAND_FM_EASTEUROPE, // 65..76 MHz

    TUNER_BAND_END,
} TunerBand;

typedef enum {
    TUNER_STEP_50K,
    TUNER_STEP_100K,
    TUNER_STEP_200K,

    TUNER_STEP_END,
} TunerStep;

typedef enum {
    TUNER_DEEMPH_50u,
    TUNER_DEEMPH_75u,

    TUNER_DEEMPH_END,
} TunerDeemph;

typedef enum {
    TUNER_MODE_GRID,
    TUNER_MODE_STATIONS,
    TUNER_MODE_SCAN,

    TUNER_MODE_END,
} TunerMode;

typedef struct {
    TunerIC ic;
    TunerBand band;
    TunerStep step;
    TunerDeemph deemph;
    TunerMode mode;

    bool mute;
    bool bassBoost;
    bool forcedMono;
    bool rds;

    uint16_t freq;
    uint16_t fMin;
    uint16_t fMax;
    uint8_t fStep;
    int8_t volume;
} TunerParam;

typedef struct {
    TunerFlag flags;
    uint16_t freq;
    uint8_t rssi;
} TunerStatus;

#ifdef __cplusplus
}
#endif

#endif // TUNERDEFS_H

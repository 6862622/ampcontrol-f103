#include "input.h"

#include "display/dispdrv.h"
#include "settings.h"

static volatile int8_t encCnt;
static volatile CmdBtn cmdBuf;

static int8_t encRes = 0;

static uint16_t adcData[AIN_END];
static const uint32_t adcChan[AIN_END] = {
    LL_ADC_CHANNEL_6,
    LL_ADC_CHANNEL_5,
    LL_ADC_CHANNEL_7,
    LL_ADC_CHANNEL_4,
};

// Divider top resistor
#define TOP_R       10000
// Divider bottom resistors
#define BTM_R1      100
#define BTM_R2      3900
#define BTM_R3      15000

#define ADC_MAX     4095

#define ADC_VAL_1   (ADC_MAX * BTM_R1 / (TOP_R + BTM_R1))
#define ADC_VAL_2   (ADC_MAX * BTM_R2 / (TOP_R + BTM_R2))
#define ADC_VAL_3   (ADC_MAX * BTM_R3 / (TOP_R + BTM_R3))

#define ADC_VAL_4   (ADC_MAX)

#define UP_TH_1    ((ADC_VAL_1 + ADC_VAL_2) / 2)
#define UP_TH_2    ((ADC_VAL_2 + ADC_VAL_3) / 2)
#define UP_TH_3    ((ADC_VAL_3 + ADC_VAL_4) / 2)

static void inputInitAdc(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC2);

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ANALOG);
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_7, LL_GPIO_MODE_ANALOG);

    if (!LL_ADC_IsEnabled(ADC2)) {
        LL_ADC_SetDataAlignment(ADC2, LL_ADC_DATA_ALIGN_RIGHT);

        LL_ADC_SetSequencersScanMode(ADC2, LL_ADC_SEQ_SCAN_DISABLE);

        LL_ADC_REG_SetTriggerSource(ADC2, LL_ADC_REG_TRIG_SOFTWARE);

        LL_ADC_REG_SetContinuousMode(ADC2, LL_ADC_REG_CONV_SINGLE);

        LL_ADC_REG_SetSequencerLength(ADC2, LL_ADC_REG_SEQ_SCAN_DISABLE);

        LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_4, LL_ADC_SAMPLINGTIME_71CYCLES_5);
        LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_5, LL_ADC_SAMPLINGTIME_71CYCLES_5);
        LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_6, LL_ADC_SAMPLINGTIME_71CYCLES_5);
        LL_ADC_SetChannelSamplingTime(ADC2, LL_ADC_CHANNEL_7, LL_ADC_SAMPLINGTIME_71CYCLES_5);

        LL_ADC_Enable(ADC2);
        while (!LL_ADC_IsEnabled(ADC2));

        LL_ADC_StartCalibration(ADC2);
        while (LL_ADC_IsCalibrationOnGoing(ADC2) != 0);
    }
}

static uint16_t getAnalogButtons(void)
{
    uint16_t ret = BTN_NO;

    uint16_t btnRaw = adcData[AIN_BTN];

    if (btnRaw < UP_TH_1) {
        ret |= BTN_A0;
    } else if (btnRaw < UP_TH_2) {
        ret |= BTN_A1;
    } else if (btnRaw < UP_TH_3) {
        ret = BTN_A2;
    }

    return ret;
}

void inputInit(void)
{
    inputInitAdc();

    encRes = (int8_t)settingsRead(PARAM_SYSTEM_ENC_RES);
}

void inputSetEncRes(int8_t value)
{
    encRes = value;
}

int8_t inputGetEncRes(void)
{
    return encRes;
}

void inputPoll(void)
{
    // Antibounce counter
    static int16_t btnCnt = 0;

    // Previous state
    static uint16_t btnPrev = BTN_NO;
    static uint16_t encPrev = ENC_NO;

    // Current state
    uint8_t dispBus = ~dispdrvGetBus();

    uint16_t ain = getAnalogButtons();

    uint16_t btnNow = dispBus | ain;

    // If encoder event has happened, inc/dec encoder counter
    if (encRes) {
        uint16_t encNow = btnNow & ENC_AB;
        btnNow &= ~ENC_AB;

        if ((encPrev == ENC_NO && encNow == ENC_A) ||
            (encPrev == ENC_A && encNow == ENC_AB) ||
            (encPrev == ENC_AB && encNow == ENC_B) ||
            (encPrev == ENC_B && encNow == ENC_NO))
            encCnt++;
        if ((encPrev == ENC_NO && encNow == ENC_B) ||
            (encPrev == ENC_B && encNow == ENC_AB) ||
            (encPrev == ENC_AB && encNow == ENC_A) ||
            (encPrev == ENC_A && encNow == ENC_NO))
            encCnt--;
        encPrev = encNow;
    }

    // On button event place it to command buffer
    if (btnNow) {
        if (btnNow == btnPrev) {
            btnCnt++;
            if (btnCnt == LONG_PRESS) {
                cmdBuf.btn = btnPrev;
                cmdBuf.flags |= BTN_FLAG_LONG_PRESS;
                if (btnNow & (BTN_D3 | BTN_D4 | ENC_A | ENC_B)) {
                    btnCnt -= AUTOREPEAT;
                }
            }
        }
    } else {
        if ((btnCnt > SHORT_PRESS) && (btnCnt < LONG_PRESS - AUTOREPEAT)) {
            cmdBuf.btn = btnPrev;
        }
        btnCnt = 0;
    }
    btnPrev = btnNow;
}

int8_t getEncoder(void)
{
    int8_t ret = 0;

    if (encRes) {
        ret = encCnt / encRes;
        encCnt -= (ret * encRes);
    } else {
        ret = encCnt;
        encCnt = 0;
    }

    return ret;
}

CmdBtn getBtnCmd(void)
{
    CmdBtn ret = cmdBuf;

    cmdBuf.btn = BTN_NO;
    cmdBuf.flags = BTN_FLAG_NO;

    return ret;
}

void inputConvertADC(void)
{
    if (LL_ADC_IsEnabled(ADC2)) {
        static uint8_t chan = 0;

        adcData[chan] = LL_ADC_REG_ReadConversionData12(ADC2);

        LL_ADC_REG_SetSequencerRanks(ADC2, LL_ADC_REG_RANK_1, adcChan[chan]);
        LL_ADC_REG_StartConversionSWStart(ADC2);

        if (++chan >= ANALOG_INPUT_NUM) {
            chan = 0;
        }
    }
}

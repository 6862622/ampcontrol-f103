#include "control.h"

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "audio/audio.h"
#include "bt.h"
#include "debug.h"
#include "hwlibs.h"
#include "ringbuf.h"
#include "tr/labels.h"
#include "tuner/tuner.h"
#include "usart.h"
#include "utils.h"

#define CMDBUF_SIZE     128

#define REPORT_ENABLED

static RingBuf cmdRb;
static char cmdRbData[CMDBUF_SIZE];
static LineParse cmdLp;

static const char *BT201_MOUNT      = "MU";
static const char *BT201_ADD_USB    = "+01";
static const char *BT201_DEL_USB    = "+02";
static const char *BT201_ADD_SD     = "+03";
static const char *BT201_DEL_SD     = "+04";

static const char *BT201_SEL    = "QM";
static const char *BT201_NO     = "+00";
static const char *BT201_BT     = "+01";
static const char *BT201_USB    = "+02";
static const char *BT201_SD     = "+03";

static const char *CTRL_AMP     = "amp";

static const char *CTRL_STATUS  = ".status";

static const char *CTRL_AUDIO   = ".audio";
static const char *CTRL_INPUT   = ".input";
static const char *CTRL_VOLUME  = ".volume";
static const char *CTRL_BASS    = ".bass";
static const char *CTRL_MIDDLE  = ".middle";
static const char *CTRL_TREBLE  = ".treble";

static const char *CTRL_TUNER  = ".tuner";
static const char *CTRL_FREQ   = ".freq";

static const char *CTRL_ON      = ".on";
static const char *CTRL_OFF     = ".off";
static const char *CTRL_SWITCH  = ".switch";

static const char *CTRL_PREV    = ".prev";
static const char *CTRL_NEXT    = ".next";

static const char *CTRL_DEC     = ".dec";
static const char *CTRL_INC     = ".inc";

#define GENERATE_AUDIO_TUNE_TEXT(TUNE)  # TUNE,

static const char *const CTRL_AUDIO_TUNE[] = {
    FOREACH_AUDIO_TUNE(GENERATE_AUDIO_TUNE_TEXT)
};

static bool isEndOfLine(const char *line)
{
    return (*line < ' ');
}

static void controlParseBt201Mount(char *line)
{
    if (strstr(line, BT201_ADD_USB) == line) {
        btAddInput(BT_IN_USB);
    } else if (strstr(line, BT201_DEL_USB) == line) {
        btDelInput(BT_IN_USB);
    } else if (strstr(line, BT201_ADD_SD) == line) {
        btAddInput(BT_IN_SDCARD);
    } else if (strstr(line, BT201_DEL_SD) == line) {
        btDelInput(BT_IN_SDCARD);
    }
}

static void controlParseBt201Select(char *line)
{
    if (strstr(line, BT201_BT) == line) {
        btSetInput(BT_IN_BLUETOOTH);
    } else if (strstr(line, BT201_USB) == line) {
        btSetInput(BT_IN_USB);
    } else if (strstr(line, BT201_SD) == line) {
        btSetInput(BT_IN_SDCARD);
    } else if (strstr(line, BT201_NO) == line) {
        btDelInput(BT_IN_USB | BT_IN_SDCARD);
    }
}

static void controlParseAmpStatus(char *line)
{
    if (isEndOfLine(line)) {
        controlReportAmpStatus();
    }
}

static bool controlParseNumber(char **line, int16_t *number)
{
    char *end;
    int16_t value = (int16_t)strtol(*line, &end, 10);
    if (*line != end) {
        *number = value;
        *line = end;
        return true;
    }

    return false;
}

static void controlParseAmpAudio(char *line)
{
    if (isEndOfLine(line)) {
        controlReportAudioIc();
    }
}

static void controlParseAmpInput(char *line)
{
    if (isEndOfLine(line)) {
        controlReportAudioInput();
    } else if (strstr(line, CTRL_PREV) == line) {
        ampActionQueue(ACTION_AUDIO_INPUT_CHANGE, -1);
    } else if (strstr(line, CTRL_NEXT) == line) {
        ampActionQueue(ACTION_AUDIO_INPUT_CHANGE, +1);
    } else if (' ' == *line) {
        int16_t val;
        if (controlParseNumber(&line, &val)) {
            ampActionQueue(ACTION_AUDIO_INPUT_SET, val);
        }
    }
}

static void controlParseAmpTune(char *line, AudioTune tune)
{
    for (AudioTune t = AUDIO_TUNE_VOLUME; t < AUDIO_TUNE_END - 1; t++) {
        if (t == tune) {
            ampSelectTune(tune);
            if (isEndOfLine(line)) {
                controlReportAudioTune(tune);
            } else if (strstr(line, CTRL_DEC) == line) {
                ampActionQueue(ACTION_AUDIO_PARAM_CHANGE, -1);
            } else if (strstr(line, CTRL_INC) == line) {
                ampActionQueue(ACTION_AUDIO_PARAM_CHANGE, +1);
            } else if (' ' == *line) {
                int16_t val;
                if (controlParseNumber(&line, &val)) {
                    ampActionQueue(ACTION_AUDIO_PARAM_SET, val);
                }
            }
        }
    }

    if (strstr(line, ".volume") == line) {

    }
}

static void controlParseAmpTunerFreq(char *line)
{
    if (isEndOfLine(line)) {
        controlReportTunerFreq(true);
    } else if (' ' == *line) {
        int16_t val;
        if (controlParseNumber(&line, &val)) {
            ampActionQueue(ACTION_TUNER_SET_FREQ, val);
        }
    }
}

static void controlParseAmpTuner(char *line)
{
    if (isEndOfLine(line)) {
        controlReportTunerIc();
    } else if (strstr(line, CTRL_FREQ) == line) {
        controlParseAmpTunerFreq(line + strlen(CTRL_FREQ));
    }
}


static void controlParseAmp(char *line)
{
    AmpStatus ampStatus = ampGet()->status;

    if (ampStatus != AMP_STATUS_ACTIVE) {
        if (strstr(line, CTRL_SWITCH) == line) {
            ampActionQueue(ACTION_STANDBY, FLAG_SWITCH);
        } else if (strstr(line, CTRL_ON) == line) {
            ampActionQueue(ACTION_STANDBY, FLAG_EXIT);
        } else {
            controlReportAmpStatus();
            return;
        }
    }

    if (isEndOfLine(line)) {
        //
    } else if (strstr(line, CTRL_SWITCH) == line) {
        ampActionQueue(ACTION_STANDBY, FLAG_SWITCH);
    } else if (strstr(line, CTRL_ON) == line) {
        ampActionQueue(ACTION_STANDBY, FLAG_EXIT);
    } else if (strstr(line, CTRL_OFF) == line) {
        ampActionQueue(ACTION_STANDBY, FLAG_ENTER);
    } else if (strstr(line, CTRL_STATUS) == line) {
        controlParseAmpStatus(line + strlen(CTRL_STATUS));

    } else if (strstr(line, CTRL_AUDIO) == line) {
        controlParseAmpAudio(line + strlen(CTRL_AUDIO));
    } else if (strstr(line, CTRL_INPUT) == line) {
        controlParseAmpInput(line + strlen(CTRL_INPUT));
    } else if (strstr(line, CTRL_VOLUME) == line) {
        controlParseAmpTune(line + strlen(CTRL_VOLUME), AUDIO_TUNE_VOLUME);
    } else if (strstr(line, CTRL_BASS) == line) {
        controlParseAmpTune(line + strlen(CTRL_BASS), AUDIO_TUNE_BASS);
    } else if (strstr(line, CTRL_MIDDLE) == line) {
        controlParseAmpTune(line + strlen(CTRL_MIDDLE), AUDIO_TUNE_MIDDLE);
    } else if (strstr(line, CTRL_TREBLE) == line) {
        controlParseAmpTune(line + strlen(CTRL_TREBLE), AUDIO_TUNE_TREBLE);

    } else if (strstr(line, CTRL_TUNER) == line) {
        controlParseAmpTuner(line + strlen(CTRL_TUNER));
    }
}

static void controlParseLine(char *line)
{
    if (strstr(line, CTRL_AMP) == line) {
        controlParseAmp(line + strlen(CTRL_AMP));
    } else if (strstr(line, BT201_SEL) == line) {
        controlParseBt201Select(line + strlen(BT201_SEL));
    } else if (strstr(line, BT201_MOUNT) == line) {
        controlParseBt201Mount(line + strlen(BT201_MOUNT));
    }
}

static void sendReport(char *report)
{
#ifdef REPORT_ENABLED
    dbg(report);
#endif
}

void controlInit(void)
{
    ringBufInit(&cmdRb, cmdRbData, sizeof(cmdRbData));
}

void USART_DBG_HANDLER(void)
{
    // Check RXNE flag value in SR register
    if (LL_USART_IsActiveFlag_RXNE(USART_DBG) && LL_USART_IsEnabledIT_RXNE(USART_DBG)) {
        char ch = LL_USART_ReceiveData8(USART_DBG);
#ifdef _DEBUG_KARADIO
        usartSendChar(USART_KARADIO, ch);
#endif
        ringBufPushChar(&cmdRb, ch);
    } else {
        // Call Error function
    }
}

void controlGetData(void)
{
    while (ringBufGetSize(&cmdRb) > 0) {
        char ch = ringBufPopChar(&cmdRb);
        usartSendChar(USART_KARADIO, ch);
        if (!utilReadChar(&cmdLp, ch)) {
            if (cmdLp.line[0] != '\0') {
                controlParseLine(cmdLp.line);
            }
        }
    }
}

void controlReportAmpStatus(void)
{
    AmpStatus ampStatus = ampGet()->status;

    char *status = "UNKNOWN";

    switch (ampStatus) {
    case AMP_STATUS_STBY:
        status = "STANDBY";
        break;
    case AMP_STATUS_HW_READY:
    case AMP_STATUS_ACTIVE:
        status = "ACTIVE";
        break;
    }

    sendReport(utilMkStr("##AMP.STATUS#: %s", status));
}

void controlReportAudioIc(void)
{
    AudioParam *par = &audioGet()->par;

    sendReport(utilMkStr("##AMP.AUDIO.IC#: %s", labelsGetDefault(LABEL_AUDIO_IC + par->ic)));
}

void controlReportAudioInput(void)
{
    AudioParam *par = &audioGet()->par;

    sendReport(utilMkStr("##AMP.AUDIO.INPUT#: %u", par->input));
}

void controlReportAudioTune(AudioTune tune)
{
    AudioParam *par = &audioGet()->par;

    sendReport(utilMkStr("##AMP.AUDIO.%s#: %d", CTRL_AUDIO_TUNE[tune], par->tune[tune].value));
}

void controlReportAudioTunes(void)
{
    for (AudioTune tune = AUDIO_TUNE_VOLUME; tune < AUDIO_TUNE_END - 1; tune++) {
        controlReportAudioTune(tune);
    }
}

void controlReportTunerIc(void)
{
    TunerParam *par = &tunerGet()->par;

    sendReport(utilMkStr("##AMP.TUNER.IC#: %s", labelsGetDefault(LABEL_TUNER_IC + par->ic)));
}

void controlReportTunerFreq(bool force)
{
    TunerStatus *st = &tunerGet()->status;

    static TunerStatus stOld;

    if (force || stOld.freq != st->freq) {
        sendReport(utilMkStr("##AMP.TUNER.FREQ#: %d", st->freq));
    }

    memcpy(&stOld, st, sizeof (stOld));
}

void controlReportAll(void)
{
    controlReportAmpStatus();

    controlReportAudioIc();
    controlReportAudioInput();
    controlReportAudioTunes();

    controlReportTunerIc();
    controlReportTunerFreq(true);
}

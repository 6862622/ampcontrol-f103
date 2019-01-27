#include "amp.h"

#include "emuldisp.h"

#include <QDebug>

#include "../../../canvas/layout.h"
#include "../../../display/dispdefs.h"
#include "../../../input.h"

static EmulDisp *disp;
static Amp *amp;
static DispDriver drv;

extern "C" void emulDrawPixel(int16_t x, int16_t y, uint16_t color)
{
    disp->drawPixel(x, y, color);
}

extern "C" void emulDrawRectangle(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    disp->drawRectangle(x, y, w, h, color);
}

extern "C" void emulDrawImage(tImage *img, int16_t x, int16_t y, uint16_t color, uint16_t bgColor)
{
    disp->drawImage(img, x, y, color, bgColor);
}

extern "C" void dispdrvInit(DispDriver **driver)
{
    drv.width = EMUL_DISP_WIDTH;
    drv.height = EMUL_DISP_HEIGHT;
    drv.drawPixel = emulDrawPixel;
    drv.drawRectangle = emulDrawRectangle;
    drv.drawImage = emulDrawImage;

    *driver = &drv;
}

extern "C" uint8_t dispdrvGetBus(void)
{
    return amp->bus;
}

extern "C" void dispdrvSetBrightness(int8_t value)
{
    disp->setBrightness(value);
}

extern "C" const Layout *ltEmulGet(void)
{
    const Layout *lt;

#if EMUL_DISP_WIDTH == 160 && EMUL_DISP_HEIGHT == 128
    lt = lt160x128Get();
#elif EMUL_DISP_WIDTH == 176 && EMUL_DISP_HEIGHT == 132
    lt = lt176x132Get();
#elif EMUL_DISP_WIDTH == 220 && EMUL_DISP_HEIGHT == 176
    lt = lt220x176Get();
#elif EMUL_DISP_WIDTH == 320 && EMUL_DISP_HEIGHT == 240
    lt = lt320x240Get();
#elif EMUL_DISP_WIDTH == 400 && EMUL_DISP_HEIGHT == 240
    lt = lt400x240Get();
#elif EMUL_DISP_WIDTH == 480 && EMUL_DISP_HEIGHT == 320
    lt = lt480x320Get();
#else
#error "No such layout"
#endif

    return lt;
}

Amp::Amp(QWidget *parent) :
    QMainWindow(parent)
{
    setupUi(this);

    amp = this;
    disp = getEmulDisp();

    this->bus = BTN_NO;

    disp->setFixedSize((EMUL_DISP_WIDTH + 2 * EMUL_DISP_BORDER) * EMUL_DISP_SCALE,
                       (EMUL_DISP_HEIGHT + 2 * EMUL_DISP_BORDER) * EMUL_DISP_SCALE);
    setFixedSize(sizeHint());

    dialValue = dial->value();
    dialMax = dial->maximum();

    dialTimer = new QTimer(this);
    dialTimer->setSingleShot(true);
    connect(dialTimer, SIGNAL(timeout()), SLOT(dialTimerElapsed()));

    disp->init();
}

EmulDisp *Amp::getEmulDisp()
{
    return emulDisp;
}

void Amp::dialChanged(int value)
{
    int diff = (value + dialMax - dialValue) % dialMax;
    if (diff > dialMax / 2) {
        this->bus |= ENC_A;
    } else if (diff < dialMax / 2) {
        this->bus |= ENC_B;
    }
    dialValue = value;
    dialTimer->start(100);

    this->statusBar()->showMessage(QString("ENC value: %1").arg(value), 1000);
}

void Amp::on_btn0_pressed()
{
    this->bus |= BTN_D0;
    this->statusBar()->showMessage("BTN_D0 pressed", 1000);
}

void Amp::on_btn0_released()
{
    this->bus &= ~BTN_D0;
    this->statusBar()->showMessage("BTN_D0 released", 1000);
}

void Amp::on_btn1_pressed()
{
    this->bus |= BTN_D1;
    this->statusBar()->showMessage("BTN_D1 pressed", 1000);
}

void Amp::on_btn1_released()
{
    this->bus &= ~BTN_D1;
    this->statusBar()->showMessage("BTN_D1 released", 1000);
}

void Amp::on_btn2_pressed()
{
    this->bus |= BTN_D2;
    this->statusBar()->showMessage("BTN_D2 pressed", 1000);
}

void Amp::on_btn2_released()
{
    this->bus &= ~BTN_D2;
    this->statusBar()->showMessage("BTN_D2 released", 1000);
}

void Amp::on_btn3_pressed()
{
    this->bus |= BTN_D3;
    this->statusBar()->showMessage("BTN_D3 pressed", 1000);
}

void Amp::on_btn3_released()
{
    this->bus &= ~BTN_D3;
    this->statusBar()->showMessage("BTN_D3 released", 1000);
}

void Amp::on_btn4_pressed()
{
    this->bus |= BTN_D4;
    this->statusBar()->showMessage("BTN_D4 pressed", 1000);
}

void Amp::on_btn4_released()
{
    this->bus &= ~BTN_D4;
    this->statusBar()->showMessage("BTN_D4 released", 1000);
}

void Amp::on_btn5_pressed()
{
    this->bus |= BTN_D5;
    this->statusBar()->showMessage("BTN_D5 pressed", 1000);
}

void Amp::on_btn5_released()
{
    this->bus &= ~BTN_D5;
    this->statusBar()->showMessage("BTN_D5 released", 1000);
}

void Amp::on_dial_valueChanged(int value)
{
    dialChanged(value);
}

void Amp::on_dial_sliderMoved(int position)
{
    dialChanged(position);
}

void Amp::dialTimerElapsed()
{
    this->bus &= ~(ENC_A | ENC_B);
}

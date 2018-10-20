#include "ili9163.h"

#include <stm32f1xx_ll_utils.h>
#include "../../pins.h"

#define ILI9163_WIDTH           128
#define ILI9163_HEIGHT          160
#define ILI9163_PIXELS          (ILI9163_WIDTH * ILI9163_HEIGHT)

static DispDriver drv = {
    .width = ILI9163_HEIGHT,
    .height = ILI9163_WIDTH,
    .drawPixel = ili9163DrawPixel,
    .drawRectangle = ili9163DrawRectangle,
    .drawImage = ili9163DrawImage,
};

static inline void ili9163SelectReg(uint8_t reg) __attribute__((always_inline));
static inline void ili9163SelectReg(uint8_t reg)
{
    CLR(DISP_8BIT_RS);
    dispdrvSendData8(reg);
    SET(DISP_8BIT_RS);
}

static inline void ili9163InitSeq(void)
{
    // Wait for reset
    LL_mDelay(50);

    CLR(DISP_8BIT_CS);

    // Initial Sequence
    //************* Start Initial Sequence **********//
    ili9163SelectReg(0x11); //Exit Sleep
    LL_mDelay(20);

    ili9163SelectReg(0x26); //Set Default Gamma
    dispdrvSendData8(0x04);

    ili9163SelectReg(0xB1);
    dispdrvSendData8(0x0C);
    dispdrvSendData8(0x14);

    ili9163SelectReg(0xC0); //Set VRH1[4:0] & VC[2:0] for VCI1 & GVDD
    dispdrvSendData8(0x0C);
    dispdrvSendData8(0x05);

    ili9163SelectReg(0xC1); //Set BT[2:0] for AVDD & VCL & VGH & VGL
    dispdrvSendData8(0x02);

    ili9163SelectReg(0xC5); //Set VMH[6:0] & VML[6:0] for VOMH & VCOML
    dispdrvSendData8(0x29);
    dispdrvSendData8(0x43);

    ili9163SelectReg(0xC7);
    dispdrvSendData8(0x40);

    ili9163SelectReg(0x3a); //Set Color Format
    dispdrvSendData8(0x05);

    ili9163SelectReg(0x2A); //Set Column Address
    dispdrvSendData8(0x00);
    dispdrvSendData8(0x00);
    dispdrvSendData8(0x00);
    dispdrvSendData8(0x7F);

    ili9163SelectReg(0x2B); //Set Page Address
    dispdrvSendData8(0x00);
    dispdrvSendData8(0x00);
    dispdrvSendData8(0x00);
    dispdrvSendData8(0x9F);

    ili9163SelectReg(0x36); //Set Scanning Direction
    dispdrvSendData8(0x88);

    ili9163SelectReg(0xB7); //Set Source Output Direction
    dispdrvSendData8(0x00);

    ili9163SelectReg(0xf2); //Enable Gamma bit
    dispdrvSendData8(0x01);

    ili9163SelectReg(0xE0);
    dispdrvSendData8(0x36);//p1
    dispdrvSendData8(0x29);//p2
    dispdrvSendData8(0x12);//p3
    dispdrvSendData8(0x22);//p4
    dispdrvSendData8(0x1C);//p5
    dispdrvSendData8(0x15);//p6
    dispdrvSendData8(0x42);//p7
    dispdrvSendData8(0xB7);//p8
    dispdrvSendData8(0x2F);//p9
    dispdrvSendData8(0x13);//p10
    dispdrvSendData8(0x12);//p11
    dispdrvSendData8(0x0A);//p12
    dispdrvSendData8(0x11);//p13
    dispdrvSendData8(0x0B);//p14
    dispdrvSendData8(0x06);//p15

    ili9163SelectReg(0xE1);
    dispdrvSendData8(0x09);//p1
    dispdrvSendData8(0x16);//p2
    dispdrvSendData8(0x2D);//p3
    dispdrvSendData8(0x0D);//p4
    dispdrvSendData8(0x13);//p5
    dispdrvSendData8(0x15);//p6
    dispdrvSendData8(0x40);//p7
    dispdrvSendData8(0x48);//p8
    dispdrvSendData8(0x53);//p9
    dispdrvSendData8(0x0C);//p10
    dispdrvSendData8(0x1D);//p11
    dispdrvSendData8(0x25);//p12
    dispdrvSendData8(0x2E);//p13
    dispdrvSendData8(0x34);//p14
    dispdrvSendData8(0x39);//p15

    ili9163SelectReg(0x29); // Display On

    SET(DISP_8BIT_CS);
}

static inline void ili9163SetWindow(uint16_t x, uint16_t y, uint16_t w,
                                    uint16_t h) __attribute__((always_inline));
static inline void ili9163SetWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h)
{
    ili9163SelectReg(0x2A);
    dispdrvSendData16(y);
    dispdrvSendData16(y + h - 1);

    ili9163SelectReg(0x2B);
    dispdrvSendData16(x);
    dispdrvSendData16(x + w - 1);

    ili9163SelectReg(0x2C);
}

void ili9163Init(DispDriver **driver)
{
    *driver = &drv;

    SET(DISP_8BIT_LED);
    SET(DISP_8BIT_RD);
    SET(DISP_8BIT_WR);
    SET(DISP_8BIT_RS);
    SET(DISP_8BIT_CS);

    CLR(DISP_8BIT_RST);
    LL_mDelay(1);
    SET(DISP_8BIT_RST);

    ili9163InitSeq();
}

void ili9163Sleep(void)
{
    CLR(DISP_8BIT_CS);

    ili9163SelectReg(0x28);    // Display OFF
    LL_mDelay(100);
    ili9163SelectReg(0x10);

    SET(DISP_8BIT_CS);
}

void ili9163Wakeup(void)
{
    CLR(DISP_8BIT_CS);

    ili9163SelectReg(0x11);    // Display OFF
    LL_mDelay(100);
    ili9163SelectReg(0x29);

    SET(DISP_8BIT_CS);
}

void ili9163DrawPixel(int16_t x, int16_t y, uint16_t color)
{
    CLR(DISP_8BIT_CS);

    ili9163SetWindow(x, y, 1, 1);
    dispdrvSendData16(color);

    SET(DISP_8BIT_CS);
}

void ili9163DrawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
    CLR(DISP_8BIT_CS);

    ili9163SetWindow(x, y, w, h);
    dispdrvSendFill(w * h, color);

    SET(DISP_8BIT_CS);
}

void ili9163DrawImage(tImage *img, int16_t x, int16_t y, uint16_t color, uint16_t bgColor)
{
    uint16_t w = img->width;
    uint16_t h = img->height;

    CLR(DISP_8BIT_CS);

    ili9163SetWindow(x, y, w, h);
    dispdrvSendImage(img, color, bgColor);

    SET(DISP_8BIT_CS);
}

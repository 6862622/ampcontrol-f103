#ifndef EMULDISP_H
#define EMULDISP_H

#include <QWidget>
#include <QPainter>
#include <QMenu>
#include <QContextMenuEvent>
#include <QTimer>

#include "../../../actions.h"
#include "../../../display/fonts.h"

namespace Ui {
class EmulDisp;
}

class EmulDisp : public QWidget
{
    Q_OBJECT

public:
    explicit EmulDisp(QWidget *parent = nullptr);
    ~EmulDisp() override {}

    void init();
    void drawPixel(int16_t x, int16_t y, uint16_t color);
    void drawRectangle(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
    void drawVertGrad(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *gradient);
    void drawImage(tImage *img, int16_t x, int16_t y, uint16_t color, uint16_t bgColor);

    void setSize(uint16_t w, uint16_t h);

    void setBrightness(const int8_t &value);

private slots:
    void systick();
    void drawscreen();

private:
    Ui::EmulDisp *ui;
    QImage image;

    QTimer *updateTimer;
    QTimer *mSecTimer;

    int8_t brightness;

    void resizeImage(QImage *image, const QSize &newSize);

protected:
    virtual void paintEvent(QPaintEvent *pe) override;
    void resizeEvent(QResizeEvent *event) override;
};

#endif // EMULDISP_H

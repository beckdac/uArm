#include <7segment.h>
#include <font5x7.h>
#include <font8x16.h>
#include <fontlargenumber.h>
#include <MicroView.h>
#include <space01.h>
#include <space02.h>
#include <space03.h>

#include <VarSpeedServo.h>

int SCREEN_WIDTH = uView.getLCDWidth();
int SCREEN_HEIGHT = uView.getLCDHeight();
MicroViewWidget *widgetL, *widgetR, *widgetRot, *widgetHRot, *widgetH;

const int servoPinL = 2;
const int servoPinR = 3;
const int servoPinRot = 5;
const int servoPinHRot = 6;
const int servoPinH = A0;
const int slowSweepSpeed = 20;
VarSpeedServo servoL, servoR, servoRot, servoHRot, servoH;

void setup() {
  uView.begin();
  uView.clear(PAGE);
  uView.display();

  uView.begin();
  widgetL = new MicroViewSlider(0, 0, 0, 180);
  widgetR = new MicroViewSlider(0, 10, 0, 180);
  widgetRot = new MicroViewSlider(0, 20, 0, 180);
  widgetHRot = new MicroViewSlider(0, 30, 0, 180);
  widgetH = new MicroViewSlider(0, 40, 0, 180);
  
  widgetL->setValue(90);
  widgetR->setValue(90);
  widgetRot->setValue(90);
  widgetHRot->setValue(90);
  widgetH->setValue(90);

  servoL.attach(servoPinL);
  servoR.attach(servoPinR);
  servoRot.attach(servoPinRot);
  servoHRot.attach(servoPinHRot);
  servoH.attach(servoPinH);

  servoL.write(90, slowSweepSpeed, false);
  servoR.write(90, slowSweepSpeed, false);
  servoRot.write(90, slowSweepSpeed, false);
  servoHRot.write(90, slowSweepSpeed, false);
  servoH.write(90, slowSweepSpeed, true);

  uView.display();
}

uint8_t state = 180;

void loop()
{
  if (state == 180) {
    state = 0;
  } else {
    state = 180;
  }

  uView.begin();  
  servoL.write(state,slowSweepSpeed,false);
  widgetL->setValue(state);
  servoR.write(state,slowSweepSpeed,false);
  widgetR->setValue(state);
  servoRot.write(state,slowSweepSpeed,false);
  widgetRot->setValue(state);
  servoHRot.write(state,slowSweepSpeed,false);
  widgetHRot->setValue(state);
  widgetH->setValue(state);
  uView.display();
  servoH.write(state,slowSweepSpeed,true);
}



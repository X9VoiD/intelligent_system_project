/*

Buttons:
TestNotebook
BallPen
Pencil
Eraser

LEDs:
TestNotebook
BallPen
Pencil
Eraser

Coinslot 1P
Coinslot 5P

*/

#include "vending_masheesh.h"

// COMPONENTS

Item testBooklet;
Item ballPen;
Item pencil;
Item eraser;

Item* items[] = {
  &testBooklet,
  &ballPen,
  &pencil,
  &eraser
};

INIT_COINSLOT(coinSlotOne, coinSlotOneHandler);
INIT_COINSLOT(coinSlotFive, coinSlotFiveHandler);

// CODE

void setup()
{
  waitForSerial();
  SERIAL_PRINTLN();

  // Initialize components
  testBooklet = Item("Test Booklet", 3, testBookletButton, testBookletLed, new ContinuousServoDispenser(testBookletServo, 100, 1000));
  // ballPen = Item("Ballpen", 10, ballPenButton, ballPenLed, new ServoDispenser(ballPenServo, 0, 180, 1000));
  pencil = Item("Pencil", 10, pencilButton, pencilLed, new ServoDispenser(pencilServo, 0, 180, 1000, 500, 500));
  eraser = Item("Eraser", 8, eraserButton, eraserLed, new ServoDispenser(eraserServo, 0, 120, 1000, 10, 500));

  coinSlotOne = CoinSlot(coinSlotOnePin, 1);
  attachInterrupt(coinSlotOne.getInterrupt(), coinSlotOneHandler, FALLING);

  coinSlotFive = CoinSlot(coinSlotFivePin, 5);
  attachInterrupt(coinSlotFive.getInterrupt(), coinSlotFiveHandler, FALLING);

  lcd.init();
  lcd.backlight();

  pinMode(buzzerPin, OUTPUT);

  delay(1000);
  coinManager.initialize();
}

void loop()
{
  coinManager.processLoop();
  for (int i = 0; i < sizeof_arr(items); i++)
  {
    if (items[i]->isUsable()){
      items[i]->updateStatus();
    }
  }
  delay(10);
}

void waitForSerial()
{
#ifdef BWAIN_SERIAL_DEBUG
  Serial.begin(115200);
  while (!Serial.availableForWrite())
  {
    delay(10);
  }

  Serial.println();
#endif
}

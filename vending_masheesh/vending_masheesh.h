#ifndef MASHEESH_H
#define MASHEESH_H

#include <LiquidCrystal_I2C.h>
#include <Servo.h>

#define sizeof_arr(arr) sizeof(arr) / sizeof(arr[0])

#define SDA 20
#define SCL 21

#define G5_TONE 784
#define C6_TONE 1047
#define A5_TONE 880

// uncomment below to enable infinite coins
// #define BWAIN_INF_COIN

// uncomment below to enable serial debug
#define BWAIN_SERIAL_DEBUG

#ifdef BWAIN_SERIAL_DEBUG
#define SERIAL_PRINT(...) Serial.print(__VA_ARGS__)
#define SERIAL_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define SERIAL_PRINT(...)
#define SERIAL_PRINTLN(...)
#endif

class CoinManager;
class Dispenser;
class ContinuousServoDispenser;
class ServoDispenser;
class Item;
class CoinSlot;
#define INIT_COINSLOT(name, interruptHandlerName) \
    CoinSlot name;                                \
    void interruptHandlerName()                   \
    {                                             \
        name.handleInterrupt();                   \
    }

LiquidCrystal_I2C lcd(0x27, 16, 2);

const unsigned long debounceMs = 400;
const unsigned long milliSecondsToDecay = 20 * 1000;

// PINS

const int testBookletButton = 22;
const int ballPenButton = 23;
const int pencilButton = 24;
const int eraserButton = 25;

const int testBookletLed = 40;
const int ballPenLed = 41;
const int pencilLed = 42;
const int eraserLed = 43;

const int testBookletServo = 4;
const int pencilServo = 5;
const int ballPenServo = 6;
const int eraserServo = 7;

const int coinSlotOnePin = 2;
const int coinSlotFivePin = 3;

const int buzzerPin = 12;
const int testBookletServoFeedback = 31;

int updateButtonStatus(int buttonPin, int *buttonState)
{
  int newState = digitalRead(buttonPin);
  if (*buttonState != newState)
  {
    *buttonState = newState;
    if (newState == HIGH)
    {
      return RISING;
    }
    else
    {
      return FALLING;
    }
  }
  else
  {
    return newState;
  }
}

class CoinManager
{
private:
  volatile int coins;
  volatile bool changed;
  volatile unsigned long sinceLastChange;

public:
  void initialize() {
#ifdef BWAIN_INF_COIN
    this->coins = 999;
#endif
    printCoins();
  }

  void processLoop() {
    resetOnExpiry();
    updateLcd();
  }

  void increment(int increment, unsigned long now)
  {
#ifndef BWAIN_INF_COIN
    this->coins += increment;
#endif
    this->changed = true;
    this->armExpiry(now);
    SERIAL_PRINT("Coins: ");
    SERIAL_PRINTLN(this->coins);
  }

  void decrement(int decrement)
  {
#ifndef BWAIN_INF_COIN
    this->coins -= decrement;
#endif
    this->changed = true;
    this->armExpiry();
    SERIAL_PRINT("Coins: ");
    SERIAL_PRINTLN(this->coins);
  }

  int getCoins()
  {
    return this->coins;
  }

  void armExpiry(unsigned long now)
  {
    this->sinceLastChange = now;
  }

  void armExpiry()
  {
    armExpiry(millis());
  }

private:
  void updateLcd()
  {
    if (this->changed)
    {
      SERIAL_PRINTLN("Updating LCD...");
      printCoins();
      this->changed = false;
    }
  }

  void resetOnExpiry()
  {
    unsigned long now = millis();
    bool isExpired = this->sinceLastChange != UINT32_MAX && now - this->sinceLastChange >= milliSecondsToDecay;
    if (isExpired)
    {
      SERIAL_PRINTLN("Active session expired");
#ifndef BWAIN_INF_COIN
      this->coins = 0;
#endif
      this->sinceLastChange = UINT32_MAX; // push reset to the future until next input
      lcd.clear();
    }
  }

  void printCoins()
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Coins: ");
    lcd.print(this->coins);
  }
};

CoinManager coinManager;

class Dispenser
{
public:
  int pin;

  virtual void dispense() = 0;
};

class ContinuousServoDispenser : public Dispenser
{
private:
  Servo servo;
  int rotationSpeed;
  int rotationDurationMs;

public:
  ContinuousServoDispenser(int servoPin, int rotationSpeed, int rotationDurationMs)
  {
    this->servo.attach(servoPin);
    this->rotationSpeed = rotationSpeed;
    this->rotationDurationMs = rotationDurationMs;

    servo.write(90);

    SERIAL_PRINT("Initialized Continuous Servo (Pin ");
    SERIAL_PRINT(servoPin);
    SERIAL_PRINT(", Speed: ");
    SERIAL_PRINT(rotationSpeed);
    SERIAL_PRINTLN(")");
  }

  void dispense()
  {
    servo.write(rotationSpeed);
    delay(rotationDurationMs);
    servo.write(90); // pause
  }
};

class FeedbackControlledServoDispenser : public Dispenser
{
private:
  Servo servo;
  int rotationSpeed;
  int feedbackPin;
  int feedbackPinState;

public:
  FeedbackControlledServoDispenser(int servoPin, int rotationSpeed, int feedbackPin)
  {
    this->servo.attach(servoPin);
    this->rotationSpeed = rotationSpeed;
    this->feedbackPin = feedbackPin;

    servo.write(90);
    pinMode(feedbackPin, INPUT_PULLUP);
    for (int i = 0; i < 5; i++) {
      this->feedbackPinState = digitalRead(feedbackPin); // warm up
    }

    SERIAL_PRINT("Initialized Feedback-Controlled Servo (Pin ");
    SERIAL_PRINT(servoPin);
    SERIAL_PRINT(", Speed: ");
    SERIAL_PRINT(rotationSpeed);
    SERIAL_PRINTLN(")");
  }

  void dispense()
  {
    servo.write(rotationSpeed);
    unsigned long now = millis();
    while (updateButtonStatus(feedbackPin, &feedbackPinState) != RISING && millis() - now < 6000) {}

    delay(50);
    servo.write(90); // pause
  }
};

class ServoDispenser : public Dispenser
{
private:
  Servo servo;
  int angle0;
  int angle1;
  int resetDelay; // in ms
  int goSpeed; // in ms
  int backSpeed; // in ms

public:
  ServoDispenser(int servoPin, int angle0, int angle1, int resetDelay, int goSpeed, int backSpeed)
  {
    this->servo.attach(servoPin);
    this->angle0 = angle0;
    this->angle1 = angle1;
    this->resetDelay = resetDelay;
    this->goSpeed = goSpeed;
    this->backSpeed = backSpeed;

    servo.write(angle0);

    SERIAL_PRINT("Initialized Servo (Pin ");
    SERIAL_PRINT(servoPin);
    SERIAL_PRINT(", Angle0: ");
    SERIAL_PRINT(angle0);
    SERIAL_PRINT(", Angle1: ");
    SERIAL_PRINT(angle1);
    SERIAL_PRINT(", Delay: ");
    SERIAL_PRINT(resetDelay);
    SERIAL_PRINT(", GoSpeed: ");
    SERIAL_PRINT(goSpeed);
    SERIAL_PRINT(", BackSpeed: ");
    SERIAL_PRINT(backSpeed);
    SERIAL_PRINTLN(")");
  }

  void dispense()
  {
    // split transition into 1 degree steps that will approximately take goSpeed
    // and backSpeed ms
    move(angle0, angle1, goSpeed);
    delay(resetDelay);
    move(angle1, angle0, backSpeed);
  }
private:
  void move(int from, int to, int speedMs) {
    int delayMs = speedMs / abs(to - from);
    if (delayMs < 1) {
      servo.write(to);
    }
    else {
      int increment = from < to ? 1 : -1;
      for (int i = from; i != to; i += increment) {
        servo.write(i);
        delay(delayMs);
      }
      servo.write(to);
    }
  }
};

class Item
{
private:
  Dispenser *dispenser;
  int buttonState;
  const char *name;
  int cost;
  int button;
  int led;
  bool usable;

public:
  Item() {
    this->usable = false;
  }

  Item(const char *name, int cost, int button, int led, Dispenser *dispenser)
  {
    this->name = name;
    this->cost = cost;
    this->button = button;
    this->led = led;
    this->dispenser = dispenser;
    this->usable = true;

    pinMode(button, INPUT_PULLUP);
    pinMode(led, OUTPUT);

    for (int i = 0; i < 5; i++) {
      this->buttonState = digitalRead(button); // warm up
    }

    SERIAL_PRINT("Initialized ");
    SERIAL_PRINTLN(name);
  }

  const char *getName()
  {
    return this->name;
  }

  void updateStatus()
  {
    bool canBeBought = this->canBeBought();
    updateLed(canBeBought);
    if (this->updateButtonStatus() == RISING)
    {
      SERIAL_PRINT(this->name);
      SERIAL_PRINT(" button pressed");
      if (canBeBought)
      {
        SERIAL_PRINTLN(" and can be bought");

        tone(buzzerPin, G5_TONE, 200);
        delay(200);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Dispensing ");
        lcd.setCursor(0, 1);
        lcd.print(name);

        coinManager.decrement(this->cost);
        this->updateLed(this->canBeBought());
        this->dispenser->dispense();

        lcd.clear();
        tone(buzzerPin, C6_TONE, 200);
        delay(200);
        tone(buzzerPin, A5_TONE, 150);
      }
      else
      {
        SERIAL_PRINTLN(" but cannot be bought");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("KULANG PA");
        tone(buzzerPin, 262, 200);
        delay(400);
        tone(buzzerPin, 262, 600);
        coinManager.armExpiry();
      }
    }
  }

  bool isUsable()
  {
    return this->usable;
  }

private:
  bool canBeBought()
  {
    return coinManager.getCoins() >= this->cost;
  }

  void updateLed(bool canBeBought)
  {
    digitalWrite(this->led, canBeBought ? HIGH : LOW);
  }

  int updateButtonStatus()
  {
    return ::updateButtonStatus(this->button, &this->buttonState);
  }
};

class CoinSlot
{
private:
  volatile unsigned long sinceLastInsertedCoin = 0; // time in milliseconds
  int pin;
  int increment;

public:
  CoinSlot() {}
  CoinSlot(int pin, int increment)
  {
    pinMode(pin, INPUT_PULLUP);
    this->pin = pin;
    this->increment = increment;

    SERIAL_PRINT("Initialized CoinSlot (Pin ");
    SERIAL_PRINT(pin);
    SERIAL_PRINT(", Increment: ");
    SERIAL_PRINT(increment);
    SERIAL_PRINTLN(")");
  }

  void handleInterrupt()
  {
    unsigned long now = millis();
    if (debounce(now))
    {
      this->sinceLastInsertedCoin = now;
      coinManager.increment(this->increment, now);
    }
  }

  int getInterrupt()
  {
    return digitalPinToInterrupt(this->pin);
  }

private:
  bool debounce(unsigned long now)
  {
    bool r = now - sinceLastInsertedCoin > debounceMs;
    sinceLastInsertedCoin = now;
    return r;
  }
};

#endif
#include "LiquidCrystal.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include "Wire.h"
#include "math.h"
#include "SPI.h"
#include "SD.h"

const int PIN_CHIP_SELECT = 4;


LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
int button;  //вводим числовые значения для кнопок
const int BUTTON_NONE   = 0; //присваиваем постоянное значение для BUTTON_NONE
const int BUTTON_RIGHT  = 1; //присваиваем постоянное значение для BUTTON_RIGHT
const int BUTTON_UP     = 2; //присваиваем постоянное значение для BUTTON_UP
const int BUTTON_DOWN   = 3; //присваиваем постоянное значение для BUTTON_DOWN
const int BUTTON_LEFT   = 4; //присваиваем постоянное значение для BUTTON_LEFT
const int BUTTON_SELECT = 5; //присваиваем постоянное значение для BUTTON_SELECT

MPU6050 mpu;

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
bool blinkState = false;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];

uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

float angleY1 = 0, angleY2 = 0;

const float toDeg = 180.0 / M_PI;

double b = 1000, H, a, o1, v1, h1, o2, v2, h2 ;

int n;


int getPressedButton() //инициализация переменной
{
  int buttonValue = analogRead(0); // считываем значения с аналогового входа
  if (buttonValue < 100) { //если при нажатии кнопки значение меньше 100
    return BUTTON_RIGHT;   // выводим значение BUTTON_RIGHT
  }
  else if (buttonValue < 200) { //если при нажатии кнопки значение меньше 200
    return BUTTON_UP; // выводим значение BUTTON_UP
  }
  else if (buttonValue < 400) { //если при нажатии кнопки значение меньше 400
    return BUTTON_DOWN; // выводим значение BUTTON_DOWN
  }
  else if (buttonValue < 600) { //если при нажатии кнопки значение меньше 600
    return BUTTON_LEFT; // выводим значение BUTTON_LEFT
  }
  else if (buttonValue < 800) { //если при нажатии кнопки значение меньше 800
    return BUTTON_SELECT; // выводим значение BUTTON_SELECT
  }
  return BUTTON_NONE; //иначе, выводим значение BUTTON_NONE
}

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {

  //настройка LCD Shield
  Serial.begin(115200);
  lcd.begin(16, 2);
  lcd.print("Visotomer");
  delay(500);

  //настройка SD
  Serial.println("Инициализация SD card...");
  if (!SD.begin(PIN_CHIP_SELECT)) {
    Serial.println("SD card ошибка или отсутствует");
    return;
  }
  Serial.println("SD card инициализирована");

  //настройка гироскопа
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  while (!Serial);

  Serial.println(F("Инициализация устройств I2C..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Проверка соединений устройств..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 соединение успешно") : F("Ошибка подключения MPU6050"));

  Serial.println(F("Инициализация DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    Serial.println(F("Включение DMP..."));
    mpu.setDMPEnabled(true);

    Serial.print(F("Включение обнаружения прерываний (Arduino external interrupt)"));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP готов! Ожидание первого прерывания..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    Serial.print(F("Ошибка инициализации DMP (код)"));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  pinMode(10, OUTPUT);
  pinMode(13, OUTPUT);
}


void loop()
{
  button = getPressedButton();
  //Присваиваем значение переменной getPressedButton к переменной button
  switch (button) //перебираем значения в цикле
  {
    case BUTTON_UP:
      if (angleY1 > 0)
      {
        getAngles1();
        lcd.begin(16, 2); //очистить экран
        lcd.setCursor(0, 1);
        lcd.print("Ygol 1 = ");
        lcd.print(angleY1);
        delay(200);
        lcd.begin(16, 2); //очистить экран
      }
      else
      {
        getAngles1();
        lcd.begin(16, 2); //очистить экран
        lcd.setCursor(0, 1);
        lcd.print("Ygol vverh");
        delay(200);
        lcd.begin(16, 2); //очистить экран
      }
      break; //переходим к следующему значению цикла

    case BUTTON_DOWN:
      if (angleY2 > 0)
      {
        getAngles2();
        lcd.begin(16, 2); //очистить экран
        lcd.setCursor(0, 1);
        lcd.print("Ygol 2 = ");
        lcd.print(angleY2);
        delay(200);
        lcd.begin(16, 2); //очистить экран
      }
      else
      {
        getAngles2();
        lcd.begin(16, 2); //очистить экран
        lcd.setCursor(0, 1);
        lcd.print("Ygol vniz");
        delay(200);
        lcd.begin(16, 2); //очистить экран
      }
      break; //переходим к следующему значению цикла

    case BUTTON_RIGHT:
      do {
        a = a + b;
        lcd.begin(16, 2); //очистить экран
        lcd.setCursor(0, 2);
        lcd.print("Baza = ");
        lcd.print(a);
        delay(1000);
        lcd.begin(16, 2); //очистить экран
      }
      while (getPressedButton());
      break; //переходим к следующему значению цикла

    case BUTTON_LEFT:
      do {
        if (a > 0)
        {
          a = a - b;
          lcd.begin(16, 2); //очистить экран
          lcd.setCursor(0, 2);
          lcd.print("Baza = ");
          lcd.print(a);
          delay(1000);
          lcd.begin(16, 2); //очистить экран}
        }
        else
        {
          lcd.begin(16, 2); //очистить экран
          lcd.setCursor(0, 2);
          lcd.print("Baza > 0 ");

          //настройка гироскопа
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
          Wire.begin();
          Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
          Fastwire::setup(400, true);
#endif
          while (!Serial);

          Serial.println(F("Инициализация устройств I2C..."));
          mpu.initialize();
          pinMode(INTERRUPT_PIN, INPUT);

          Serial.println(F("Проверка соединений устройств..."));
          Serial.println(mpu.testConnection() ? F("MPU6050 соединение успешно") : F("Ошибка подключения MPU6050"));

          Serial.println(F("Инициализация DMP..."));
          devStatus = mpu.dmpInitialize();

          mpu.setXGyroOffset(220);
          mpu.setYGyroOffset(76);
          mpu.setZGyroOffset(-85);
          mpu.setZAccelOffset(1788);

          if (devStatus == 0) {
            mpu.CalibrateAccel(6);
            mpu.CalibrateGyro(6);
            mpu.PrintActiveOffsets();
            Serial.println(F("Включение DMP..."));
            mpu.setDMPEnabled(true);

            Serial.print(F("Включение обнаружения прерываний (Arduino external interrupt)"));
            Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
            Serial.println(F(")..."));
            attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
            mpuIntStatus = mpu.getIntStatus();

            Serial.println(F("DMP готов! Ожидание первого прерывания..."));
            dmpReady = true;

            packetSize = mpu.dmpGetFIFOPacketSize();
          }
          else {
            Serial.print(F("Ошибка инициализации DMP (код)"));
            Serial.print(devStatus);
            Serial.println(F(")"));
          }

        }
      }
      while (getPressedButton());
      break; //переходим к следующему значению цикла

    case BUTTON_SELECT: // при нажатии кнопки со значением BUTTON_SELECT
      lcd.begin(16, 2); //очистить экран

      double o1 = DEG_TO_RAD * angleY1;
      double v1 = tan(o1);
      double h1 = a * v1;

      double o2 = DEG_TO_RAD * angleY2;
      double v2 = tan(o2);
      double h2 = a * v2;

      double H = h1 + h2;
      lcd.print("Visota = "); //выводим
      lcd.print(H);
      delay(5000);
      lcd.begin(16, 2); //очистить экран
      // Строка с данными, которые мы поместим в файл:
      int logStringData0 = n;
      String logStringData1 = ".";
      String logStringData2 = "Visota = ";
      double logStringData3 = H;

      // Открываем файл, но помним, что одновременно можно работать только с одним файлом.
      // Если файла с таким именем не будет, ардуино создаст его.
      File dataFile = SD.open("datalog.csv", FILE_WRITE);

      // Если все хорошо, то записываем строку:
      if (dataFile) {
        n = n + 1;
        dataFile.print(logStringData0);
        dataFile.print(logStringData1);
        dataFile.print(logStringData2);
        dataFile.println(logStringData3);
        dataFile.close();

        // Публикуем в мониторе порта для отладки
        Serial.print(logStringData0);
        Serial.print(logStringData1);
        Serial.print(logStringData2);
        Serial.println(logStringData3);
      }
      else {
        // Сообщаем об ошибке, если все плохо
        Serial.println("error opening datalog.csv");
      }
      break; //переходим к следующему значению цикла
  }
}

// получение углов в angleY
void getAngles1() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angleY1 = ypr[0] * toDeg;
  }
}
void getAngles2() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angleY2 = -1 * (ypr[0] * toDeg);
  }
}

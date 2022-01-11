#include <PPMReader.h>
#include <SoftwareSerial.h>
#include <Servo.h>

/*

Functionality <- Remote Channel:
  Throttle <- 2 (Stick Up - Down Right)
  Steer <- 1 (Stick Right - Left Right)
  Trubine <- 5 (Switch Left)
  Arm Servo 1 <- 3 (Stick Left)
  Arm Servo 2 <- 4 (Stick Left)
  
Pins:
  PPM Remote: D3 - has to be an interrupt pin
  Arm1 Servo: D5
  Arm1 Servo: D6
  Turbine PWM: D11
  Hoverboard Controller RX: 8
  Hoverboard Controller TX: 9

Hoverboard:
  - speed and steer are values beween -1000 and 1000.
  - crc is for error checking

PPM Remote:
  - has quite a few error packets 
    - differenz to last received value on these faulty values is big -> basic filtering based on the last received value

*/

PPMReader ppm(3, 6); //Digital pin number of interrupt, Channel amount

#define LOOP_DELAY 50
unsigned long nextLoop = 0;

SoftwareSerial hoverSerial(8, 9); // RX, TX

typedef struct
{
  int16_t steer;
  int16_t speed;
  uint32_t crc;
} Serialcommand;
Serialcommand oCmd;

typedef struct
{
  int16_t iSpeedL; // 100* km/h
  int16_t iSpeedR; // 100* km/h
  uint16_t iHallSkippedL;
  uint16_t iHallSkippedR;
  uint16_t iTemp; // °C
  uint16_t iVolt; // 100* V
  int16_t iAmpL;  // 100* A
  int16_t iAmpR;  // 100* A
  uint32_t crc;
} SerialFeedback;
SerialFeedback oFeedback;

enum CHANNEL
{
  THROTTLE = 2,
  STEER = 1,
  SWITCH_LEFT = 5,
  SWITCH_RIGHT = 6,
  ARM_1 = 3,
  ARM_2 = 4,
};

Servo armServo1;
Servo armServo2;
Servo turbineController;

void setup()
{
  Serial.begin(9600);
  hoverSerial.begin(9600);

  armServo1.attach(5);
  armServo2.attach(6);
  turbineController.attach(11);

  pinMode(LED_BUILTIN, OUTPUT);

  turbineController.write(0);
  delay(750);
  turbineController.write(90);
  delay(750);
  turbineController.write(0);
}

uint32_t crc32_for_byte(uint32_t r)
{
  for (int j = 0; j < 8; ++j)
  {
    r = (r & 1 ? 0 : (uint32_t)0xEDB88320L) ^ r >> 1;
  }
  return r ^ (uint32_t)0xFF000000L;
}

void crc32(const void *data, size_t n_bytes, uint32_t *crc)
{
  static uint32_t table[0x100];
  if (!*table)
  {
    for (size_t i = 0; i < 0x100; ++i)
    {
      table[i] = crc32_for_byte(i);
    }
  }
  for (size_t i = 0; i < n_bytes; ++i)
  {
    *crc = table[(uint8_t)*crc ^ ((uint8_t *)data)[i]] ^ *crc >> 8;
  }
}

void hoverSend(int16_t iSpeed, int16_t iSteer)
{
  oCmd.steer = iSteer;
  oCmd.speed = iSpeed;

  uint32_t crc = 0;
  crc32((const void *)&oCmd, sizeof(Serialcommand) - 4, &crc);
  oCmd.crc = crc;

  hoverSerial.write((uint8_t *)&oCmd, sizeof(oCmd));
}

int iFailedRec = 0;
boolean hoverReceive()
{
  if (hoverSerial.available() < sizeof(SerialFeedback))
  {
    return false;
  }

  SerialFeedback oNew;
  byte *p = (byte *)&oNew;
  for (unsigned int i = 0; i < sizeof(SerialFeedback); i++)
  {
    *p++ = hoverSerial.read();
  }

  uint32_t crc = 0;
  crc32((const void *)&oNew, sizeof(SerialFeedback) - 4, &crc);

  if (oNew.crc == crc)
  {
    memcpy(&oFeedback, &oNew, sizeof(SerialFeedback));
    return true;
  }

  return false;
}

int16_t lastSpeed = 0;
int16_t lastSteer = 0;

void loop(void)
{
  unsigned long iNow = millis();
  if (nextLoop > iNow)
  {
    return;
  }
  nextLoop = iNow + LOOP_DELAY;

  int16_t speed = map(ppm.latestValidChannelValue(CHANNEL::THROTTLE, 1500), 1000, 2000, -1000, 1000);
  int16_t steer = map(ppm.latestValidChannelValue(CHANNEL::STEER, 1500), 1000, 2000, -1000, 1000);
  int16_t arm1 = map(ppm.latestValidChannelValue(CHANNEL::ARM_1, 1000), 1000, 2000, 500, 2500);
  int16_t arm2 = map(ppm.latestValidChannelValue(CHANNEL::ARM_2, 1000), 1000, 2000, 500, 2500);
  int16_t turbine = ppm.latestValidChannelValue(CHANNEL::SWITCH_LEFT, 1000);

  armServo1.writeMicroseconds(arm1);
  armServo2.writeMicroseconds(arm2);
  turbineController.writeMicroseconds(turbine);

  if (abs(speed) < 60)
  {
    speed = 0;
  }
  if (abs(steer) < 60)
  {
    steer = 0;
  }

  if (abs(speed - lastSpeed) < 400 && abs(steer - lastSteer) < 400)
  {
    hoverSend(speed, steer);
  }

  lastSpeed = speed;
  lastSteer = steer;

  digitalWrite(LED_BUILTIN, (iNow % 2000) < 1000);
}

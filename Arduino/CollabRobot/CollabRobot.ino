#include <FastLED.h>
#include <Ticker.h>

#define LED_PIN 2
#define NUM_LEDS 9

CRGB leds[NUM_LEDS];

String serialString = "";  // string to hold incoming data
bool serialComplete = false;
bool streamData = false;
void sendData();
void readSerial();
void parseSerial(String &str);

void setAllLED(uint8_t g, uint8_t r, uint8_t b);
void flashLED();
void setLEDColor(char color);

void sendForceData();
Ticker sendForceDataTimer(sendForceData, 30);  // every 30  ms

void setup() {
  serialString.reserve(20);
  Serial.begin(230400);

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);

  flashLED();
  sendForceDataTimer.start();
}

void loop() {
  readSerial();
  if (serialComplete) {
    parseSerial(serialString);
    // clear the string
    serialString = "";
    serialComplete = false;
  }
  sendForceDataTimer.update();
}


void setAllLED(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(r, g, b);
  }
  FastLED.show();
}

void flashLED() {
  // turn all white
  setAllLED(255, 255, 255);
  delay(500);

  // turn all off
  setAllLED(0, 0, 0);
}

void readSerial() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n')
      serialComplete = true;
    else
      serialString += inChar;
  }
}

void parseSerial(String &str) {
  char identifier = str.charAt(0);
  int val;
  char c;
  // Serial.println(str);
  switch (identifier) {
    case 'c':  // set color
      c = str.charAt(1);
      setLEDColor(c);
      break;

    default:
      break;
  }
}

void setLEDColor(char color) {
  switch (color) {
    case 'k':  // black
      setAllLED(0, 0, 0);
      break;

    case 'w':  // white
      setAllLED(255, 255, 255);
      break;

    case 'r':  // red
      setAllLED(255, 0, 0);
      break;

    case 'g':  // green
      setAllLED(0, 255, 0);
      break;

    case 'b':  // blue
      setAllLED(0, 0, 255);
      break;

    case 'c':  // right blue
      setAllLED(0, 255, 255);
      break;
    case 'd':  // right green
      setAllLED(204, 255, 204);
      break;
    case 'p':  // purple
      setAllLED(255, 0, 255);
      break;

    case 'y':  // yellow
      setAllLED(255, 255, 0);
      break;

    case 'o':  // orange
      setAllLED(255, 160, 16);
      break;

    case 'n':  // pink
      setAllLED(255, 96, 208);
      break;
    default:
      break;
  }
}

void sendForceData() {
  int MxValue = analogRead(A0);
  int MyValue = analogRead(A1);
  int MzValue = analogRead(A2);

  // Serial.print(millis());
  // Serial.print(',');
  Serial.print(MxValue);
  Serial.print(',');
  Serial.print(MyValue);
  Serial.print(',');
  Serial.println(MzValue);
}

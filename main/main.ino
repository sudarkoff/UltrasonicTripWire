#include <EEPROM.h>

const int pingPin = 13;             // The pin PING))) sensor is connected to

const int savePin = 34;             // The pin for the save safe distance button
unsigned long lastDebounceTime = 0; // Last time the save button was toggled
unsigned long debounceDelay = 50;   // Debounce time; increase if the output flickers
int buttonState;                    // Current value of the save button
int lastButtonState = LOW;          // Previous value of the save button

const int distanceLimitAddr = 0;    // EEPROM address of the distance threshold parameter
const long distanceMax = 300;       // 300 cm is the maximum distance PING))) can detect
long distanceLimit=300;             // Local instance of the distance threshold parameter (cm)

/**
 * Flash LED
 */
void flashLed(int pin, int times, int wait) {
  for (int i = 0; i < times; i++) {
    digitalWrite(pin, HIGH);
    delay(wait);
    digitalWrite(pin, LOW);

    if (i + 1 < times) {
      delay(wait);
    }
  }
}

/**
 * Write a value from EEPROM
 */
template <typename T>
int EEPROM_write(int addr, T const& value)
{
    const byte* p = reinterpret_cast<byte const* const>(reinterpret_cast<void const* const>(&value));
    int i;
    for (i = 0; i < sizeof(value); i++) {
        EEPROM.write(addr++, *p++);
    }
    return i;
}

template <typename T>
int EEPROM_read(int addr, T& value)
{
    byte* p = reinterpret_cast<byte*>(reinterpret_cast<void*>(&value));
    int i;
    for (i = 0; i < sizeof(value); i++) {
        *p++ = EEPROM.read(addr++);
    }
    return i;
}

/**
 * Read a value to EEPROM, if it exists at the addr.
 * Validate the range, return default value if not in the range.
 */
template <typename T>
float EEPROM_read(int addr, T& value, T const& minVal, T const& maxVal, T const& defaultVal)
{
    EEPROM_read(addr, value);
    // if the value is out of range ...
    if ((value < minVal) || (value > maxVal)) {
        // ... replace it with default value
        value = defaultVal;
        EEPROM_write(addr, value);
    }
    return value;
}

/**
 * Read PING))) sensor
 *
 * Result:
 *   time it took the sound to travel to the target and back in microseconts
 */
long readPing()
{
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(pingPin, INPUT);
  long duration = pulseIn(pingPin, HIGH);

  return duration;
}

/**
 * Convert time delay in microseconds it took the sound to travel to the target and back to centimeters.
 */
long usec2cm(long usec)
{
  // Speed of sound is 340 m/s or 29 usec/cm.
  // The sound travels out and back, so devide the distance by 2.
  return usec / 29 / 2;
}

void setup() {
  // initialize digital pin LED_BUILTIN as an output for BT Connected status.
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialize Save button pin
  pinMode(savePin, INPUT);

  // Read the distance threshold value from EEPROM (default is 300 cm)
  EEPROM_read(distanceLimitAddr, distanceLimit,
    static_cast<long>(3), static_cast<long>(distanceMax), static_cast<long>(distanceMax));
}

void loop() {
  // Read the sensor
  long duration = readPing();

  // Convert duration to distance
  long cm = usec2cm(duration);

  if (cm < distanceLimit) {
    flashLed(LED_BUILTIN, 3, 50);
  }

  // Read the Save button
  int save = digitalRead(savePin);

  // If the button state changed
  if (save != lastButtonState) {
    // reset the timer
    lastDebounceTime = millis();
  }

  // if enough time passed
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // and the button state changed
    if (save != buttonState) {
      buttonState = save;

      // and the button state is HIGH
      if (buttonState == HIGH) {
        // save the new distance threshold
        EEPROM_write(distanceLimitAddr, cm);
        distanceLimit = cm;

        // and slowly flash LED twice
        flashLed(LED_BUILTIN, 2, 200);
      }
    }
  }

  lastButtonState = save;

  delay(50);
}

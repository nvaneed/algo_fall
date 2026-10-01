#include <Arduino.h>
#include <ld2410.h>
#include <Adafruit_NeoPixel.h>

// ======================================================
// ESP32-S3 + LD2410C
// Presence + 10-second human height estimation
// ======================================================

// ---------------- RADAR PINS ----------------
#define RADAR_RX_PIN 18
#define RADAR_TX_PIN 17

// ---------------- RGB LED ----------------
#define RGB_LED_PIN 48
#define NUM_LEDS 1

// ---------------- ROOM SETTINGS ----------------
// Measure the actual sensor-to-floor distance.
// Example: sensor is 3.00 meters above floor.
#define CEILING_HEIGHT_CM 300.0

// Ignore detections too close to sensor
// and too far away (helps reject edge/wall detections).
#define MIN_TARGET_DISTANCE_CM 80
#define MAX_TARGET_DISTANCE_CM 190

// Minimum radar energy accepted as a valid measurement.
// Tune this if necessary.
#define MIN_ENERGY 20

// How long to measure each person
#define HEIGHT_MEASURE_TIME_MS 10000UL

// How often to take a sample
#define SAMPLE_INTERVAL_MS 100UL

// If presence disappears for this long,
// the current person/session is considered gone.
#define PERSON_GONE_TIME_MS 1500UL

// Optional correction.
// Keep 0 initially.
// Later you can tune this using a known person's height.
#define HEIGHT_CORRECTION_CM 0.0

// ======================================================

ld2410 radar;

Adafruit_NeoPixel rgb(
  NUM_LEDS,
  RGB_LED_PIN,
  NEO_GRB + NEO_KHZ800
);

bool radarConnected = false;

// Height measurement state
bool measuringHeight = false;
bool measurementComplete = false;

unsigned long measurementStart = 0;
unsigned long lastSampleTime = 0;
unsigned long lastPresenceTime = 0;

// Measurement data
float distanceSum = 0;
uint32_t sampleCount = 0;

// Stored result
float storedHeight = 0;
bool storedHeightValid = false;

// ======================================================
// RGB FUNCTIONS
// ======================================================

void setRed()
{
  rgb.setPixelColor(0, rgb.Color(255, 0, 0));
  rgb.show();
}

void setGreen()
{
  rgb.setPixelColor(0, rgb.Color(0, 255, 0));
  rgb.show();
}

// ======================================================
// GET BEST TARGET DISTANCE
// ======================================================
//
// If moving and stationary targets are both available,
// use the closest valid one.
//
// This helps when the radar switches between target states.
// ======================================================

bool getBestTargetDistance(uint16_t &distance, uint8_t &energy)
{
  bool found = false;

  uint16_t bestDistance = 65535;
  uint8_t bestEnergy = 0;

  // Moving target
  if (radar.movingTargetDetected())
  {
    uint16_t d = radar.movingTargetDistance();
    uint8_t e = radar.movingTargetEnergy();

    if (d > 0 &&
        e >= MIN_ENERGY &&
        d >= MIN_TARGET_DISTANCE_CM &&
        d <= MAX_TARGET_DISTANCE_CM)
    {
      bestDistance = d;
      bestEnergy = e;
      found = true;
    }
  }

  // Stationary target
  if (radar.stationaryTargetDetected())
  {
    uint16_t d = radar.stationaryTargetDistance();
    uint8_t e = radar.stationaryTargetEnergy();

    if (d > 0 &&
        e >= MIN_ENERGY &&
        d >= MIN_TARGET_DISTANCE_CM &&
        d <= MAX_TARGET_DISTANCE_CM)
    {
      if (!found || d < bestDistance)
      {
        bestDistance = d;
        bestEnergy = e;
        found = true;
      }
    }
  }

  if (found)
  {
    distance = bestDistance;
    energy = bestEnergy;
    return true;
  }

  return false;
}

// ======================================================
// START HEIGHT MEASUREMENT
// ======================================================

void startHeightMeasurement()
{
  measuringHeight = true;
  measurementComplete = false;

  measurementStart = millis();
  lastSampleTime = 0;

  distanceSum = 0;
  sampleCount = 0;

  Serial.println();
  Serial.println("================================");
  Serial.println("HUMAN DETECTED");
  Serial.println("Starting 10-second height scan...");
  Serial.println("Edge readings are being rejected.");
  Serial.println("================================");
}

// ======================================================
// FINISH HEIGHT MEASUREMENT
// ======================================================

void finishHeightMeasurement()
{
  measuringHeight = false;
  measurementComplete = true;

  if (sampleCount < 10)
  {
    Serial.println();
    Serial.println("HEIGHT MEASUREMENT FAILED");
    Serial.println("Not enough valid radar samples.");
    Serial.println();

    storedHeightValid = false;
    return;
  }

  // Average target distance
  float averageDistance =
      distanceSum / (float)sampleCount;

  // Estimate person's height
  float estimatedHeight =
      CEILING_HEIGHT_CM
      - averageDistance
      + HEIGHT_CORRECTION_CM;

  // Basic sanity limit
  if (estimatedHeight < 50)
    estimatedHeight = 50;

  if (estimatedHeight > 250)
    estimatedHeight = 250;

  storedHeight = estimatedHeight;
  storedHeightValid = true;

  Serial.println();
  Serial.println("================================");
  Serial.println("HEIGHT MEASUREMENT COMPLETE");
  Serial.println("================================");

  Serial.print("Valid samples: ");
  Serial.println(sampleCount);

  Serial.print("Average target distance: ");
  Serial.print(averageDistance, 1);
  Serial.println(" cm");

  Serial.print("Estimated human height: ");
  Serial.print(storedHeight, 1);
  Serial.println(" cm");

  Serial.print("Estimated human height: ");
  Serial.print(storedHeight / 100.0, 2);
  Serial.println(" m");

  Serial.println("Height stored.");
  Serial.println("================================");
  Serial.println();
}

// ======================================================
// SETUP
// ======================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // RGB
  rgb.begin();
  rgb.setBrightness(50);

  setRed();

  Serial.println();
  Serial.println("================================");
  Serial.println("ESP32-S3 + LD2410C");
  Serial.println("Presence + Human Height");
  Serial.println("================================");

  Serial.print("Configured ceiling height: ");
  Serial.print(CEILING_HEIGHT_CM);
  Serial.println(" cm");

  Serial.print("Accepted target range: ");
  Serial.print(MIN_TARGET_DISTANCE_CM);
  Serial.print(" - ");
  Serial.print(MAX_TARGET_DISTANCE_CM);
  Serial.println(" cm");

  // Radar UART
  Serial1.begin(
    256000,
    SERIAL_8N1,
    RADAR_RX_PIN,
    RADAR_TX_PIN
  );

  delay(500);

  Serial.print("Connecting to LD2410C... ");

  if (radar.begin(Serial1))
  {
    radarConnected = true;

    Serial.println("OK");

    Serial.print("Firmware: ");
    Serial.print(radar.firmware_major_version);
    Serial.print(".");
    Serial.print(radar.firmware_minor_version);
    Serial.print(".");
    Serial.println(
      radar.firmware_bugfix_version,
      HEX
    );
  }
  else
  {
    radarConnected = false;

    Serial.println("FAILED");

    setRed();
  }
}

// ======================================================
// LOOP
// ======================================================

void loop()
{
  radar.read();

  // ----------------------------------------------------
  // RADAR CONNECTION CHECK
  // ----------------------------------------------------

  if (!radar.isConnected())
  {
    radarConnected = false;

    setRed();

    static unsigned long lastError = 0;

    if (millis() - lastError > 2000)
    {
      lastError = millis();

      Serial.println(
        "LD2410C not connected"
      );
    }

    return;
  }

  radarConnected = true;

  // ----------------------------------------------------
  // PRESENCE DETECTION
  // ----------------------------------------------------

  if (radar.presenceDetected())
  {
    setGreen();

    lastPresenceTime = millis();

    // ----------------------------------------------
    // START NEW PERSON MEASUREMENT
    // ----------------------------------------------

    if (!measuringHeight &&
        !measurementComplete)
    {
      startHeightMeasurement();
    }

    // ----------------------------------------------
    // 10-SECOND MEASUREMENT
    // ----------------------------------------------

    if (measuringHeight)
    {
      unsigned long now = millis();

      // Take samples every 100 ms
      if (now - lastSampleTime >= SAMPLE_INTERVAL_MS)
      {
        lastSampleTime = now;

        uint16_t distance;
        uint8_t energy;

        if (getBestTargetDistance(distance, energy))
        {
          distanceSum += distance;
          sampleCount++;

          Serial.print("Valid sample: ");
          Serial.print(distance);
          Serial.print(" cm | Energy: ");
          Serial.println(energy);
        }
        else
        {
          Serial.println(
            "Rejected sample: outside central/valid range"
          );
        }
      }

      // --------------------------------------------
      // FINISH AFTER 10 SECONDS
      // --------------------------------------------

      if (now - measurementStart >=
          HEIGHT_MEASURE_TIME_MS)
      {
        finishHeightMeasurement();
      }
    }

    // ----------------------------------------------
    // PRINT STORED HEIGHT
    // ----------------------------------------------

    static unsigned long lastHeightPrint = 0;

    if (storedHeightValid &&
        millis() - lastHeightPrint > 3000)
    {
      lastHeightPrint = millis();

      Serial.print("Stored human height: ");
      Serial.print(storedHeight, 1);
      Serial.println(" cm");
    }
  }

  // ----------------------------------------------------
  // NO PRESENCE
  // ----------------------------------------------------

  else
  {
    setRed();

    // Allow a short dropout without resetting the person.
    if (millis() - lastPresenceTime >
        PERSON_GONE_TIME_MS)
    {
      // Person has actually left.
      measuringHeight = false;
      measurementComplete = false;

      distanceSum = 0;
      sampleCount = 0;

      static unsigned long lastNoTarget = 0;

      if (millis() - lastNoTarget > 1000)
      {
        lastNoTarget = millis();

        Serial.println("NO PRESENCE");
      }
    }
  }

  delay(5);
}

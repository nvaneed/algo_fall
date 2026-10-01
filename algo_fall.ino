#include <Arduino.h>
#include <ld2410.h>
#include <Adafruit_NeoPixel.h>

// ===============================
// ==============================

// LD2410C UART pins
#define RADAR_RX_PIN 18   // ESP32 RX <- LD2410 TX
#define RADAR_TX_PIN 17   // ESP32 TX -> LD2410 RX

// Built-in RGB LED on many ESP32-S3 DevKit boards
#define RGB_LED_PIN 48
#define NUM_LEDS 1

// Create radar object
ld2410 radar;

// Create RGB LED object
Adafruit_NeoPixel rgb(NUM_LEDS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

bool radarConnected = false;

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

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // RGB LED setup
  rgb.begin();
  rgb.setBrightness(50);
  setRed();

  Serial.println();
  Serial.println("================================");
  Serial.println("ESP32-S3 + LD2410C");
  Serial.println("Basic Presence Detection");
  Serial.println("================================");

  // Start LD2410C UART
  Serial1.begin(
    256000,
    SERIAL_8N1,
    RADAR_RX_PIN,
    RADAR_TX_PIN
  );

  delay(500);

  // Start radar
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
    Serial.println(radar.firmware_bugfix_version, HEX);
  }
  else
  {
    radarConnected = false;
    Serial.println("FAILED");
    setRed();
  }
}

void loop()
{
  // Read radar continuously
  radar.read();

  // If radar is not connected
  if (!radar.isConnected())
  {
    radarConnected = false;

    // Red = no usable radar connection
    setRed();

    static unsigned long lastError = 0;

    if (millis() - lastError > 2000)
    {
      lastError = millis();
      Serial.println("LD2410C not connected");
    }

    return;
  }

  radarConnected = true;

  // ===============================
  // BASIC PRESENCE LOGIC
  // ===============================

  if (radar.presenceDetected())
  {
    // Moving OR stationary target detected
    setGreen();

    static unsigned long lastPrint = 0;

    if (millis() - lastPrint > 1000)
    {
      lastPrint = millis();

      Serial.println("HUMAN/PRESENCE DETECTED");

      if (radar.movingTargetDetected())
      {
        Serial.print("Moving target: ");
        Serial.print(radar.movingTargetDistance());
        Serial.print(" cm | Energy: ");
        Serial.println(radar.movingTargetEnergy());
      }

      if (radar.stationaryTargetDetected())
      {
        Serial.print("Stationary target: ");
        Serial.print(radar.stationaryTargetDistance());
        Serial.print(" cm | Energy: ");
        Serial.println(radar.stationaryTargetEnergy());
      }
    }
  }
  else
  {
    // No target
    setRed();

    static unsigned long lastNoTarget = 0;

    if (millis() - lastNoTarget > 1000)
    {
      lastNoTarget = millis();
      Serial.println("NO PRESENCE");
    }
  }

  delay(10);
}

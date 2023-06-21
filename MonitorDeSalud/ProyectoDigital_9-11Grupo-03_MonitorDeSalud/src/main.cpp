#include <Wire.h>

#include <Adafruit_GFX.h>       // OLED libraries
#include <Adafruit_SSD1306.h>

#include "MAX30105.h"           // MAX3010x library
#include "heartRate.h"          // Heart rate calculating algorithm
#include "spo2_algorithm.h"     // Saturation percentage oxygen algorithm

MAX30105 particleSensor;

#define BUFFER_LENGHT 100 // Buffer lenght
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels
#define OLED_RESET    -1  // Reset pin # (or -1 if sharing Arduino reset pin)

const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0;        // Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

uint32_t irBuffer[BUFFER_LENGHT];  // IR LED sensor data
uint32_t redBuffer[BUFFER_LENGHT]; // Red LED sensor data
int32_t spo2;            // Calculated SpO2 value
int8_t validSPO2;        // 1 if the SpO2 calculation is valid
int32_t heartRate;       // Calculated heart rate value
int8_t validHeartRate;   // 1 if the heart rate calculation is valid

// Declaring the display name (display)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Logo 2 (Little heart)
static const unsigned char PROGMEM logo2_bmp[] =
    { 0x03, 0xC0, 0xF0, 0x06, 0x71, 0x8C, 0x0C, 0x1B, 0x06, 0x18, 0x0E, 0x02, 0x10, 0x0C, 0x03, 0x10,
      0x04, 0x01, 0x10, 0x04, 0x01, 0x10, 0x40, 0x01, 0x10, 0x40, 0x01, 0x10, 0xC0, 0x03, 0x08, 0x88,
      0x02, 0x08, 0xB8, 0x04, 0xFF, 0x37, 0x08, 0x01, 0x30, 0x18, 0x01, 0x90, 0x30, 0x00, 0xC0, 0x60,
      0x00, 0x60, 0xC0, 0x00, 0x31, 0x80, 0x00, 0x1B, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x04, 0x00,  };

// Logo 3 (Big heart)
static const unsigned char PROGMEM logo3_bmp[] =
    {0x01, 0xF0, 0x0F, 0x80, 0x06, 0x1C, 0x38, 0x60, 0x18, 0x06, 0x60, 0x18, 0x10, 0x01, 0x80, 0x08,
     0x20, 0x01, 0x80, 0x04, 0x40, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x02, 0xC0, 0x00, 0x08, 0x03,
     0x80, 0x00, 0x08, 0x01, 0x80, 0x00, 0x18, 0x01, 0x80, 0x00, 0x1C, 0x01, 0x80, 0x00, 0x14, 0x00,
     0x80, 0x00, 0x14, 0x00, 0x80, 0x00, 0x14, 0x00, 0x40, 0x10, 0x12, 0x00, 0x40, 0x10, 0x12, 0x00,
     0x7E, 0x1F, 0x23, 0xFE, 0x03, 0x31, 0xA0, 0x04, 0x01, 0xA0, 0xA0, 0x0C, 0x00, 0xA0, 0xA0, 0x08,
     0x00, 0x60, 0xE0, 0x10, 0x00, 0x20, 0x60, 0x20, 0x06, 0x00, 0x40, 0x60, 0x03, 0x00, 0x40, 0xC0,
     0x01, 0x80, 0x01, 0x80, 0x00, 0xC0, 0x03, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x30, 0x0C, 0x00,
     0x00, 0x08, 0x10, 0x00, 0x00, 0x06, 0x60, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x01, 0x80, 0x00};

void setup()
{
  //Start the OLED display
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(3000);

  // Initialize sensor
  particleSensor.begin(Wire, I2C_SPEED_FAST);   //Use default I2C port, 400kHz speed
  particleSensor.setup();                       //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A);    //Turn Red LED to low to indicate sensor is running

  for (byte i = 0; i < BUFFER_LENGHT; i++)
  {
    redBuffer[i] = 0;
    irBuffer[i] = 0;
  }
}

void loop()
{
  long delta, irValue;
  byte samplesCount;

  while (particleSensor.available() == false) // do we have new data?
    particleSensor.check();                   // Check the sensor for new data
  irValue = particleSensor.getIR();

  // If no finger is detected it inform the user and put the average BPM to 0 or it will be stored for the next measure
  if (irValue < 7000)
  {
    beatAvg = 0;
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(30, 5);
    display.println("Please Place ");
    display.setCursor(30, 15);
    display.println("your finger ");
    display.display();
    noTone(3);
  }

  samplesCount = 0;

  // If finger is detected
  if (irValue > 7000)
  {
    // take 25 sets of samples before calculating the heart rate.
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) // do we have new data?
        particleSensor.check();                   // Check the sensor for new data

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();

      // DISPLAY when beat is not detected (with logo 2)
      display.clearDisplay();                              // Clear the display
      display.drawBitmap(5, 5, logo2_bmp, 24, 21, WHITE);  // Draw the first bmp picture (little heart)
      display.setTextSize(1);                              // Near it display the average BPM you can display the BPM if you want
      display.setTextColor(WHITE);
      display.setCursor(45, 0);
      display.println("BPM");
      display.setCursor(45, 18);
      display.println(beatAvg);
      display.setCursor(95, 0);
      display.println("SpO2");
      display.setCursor(95, 18);
      display.println(spo2);
      display.display();

      // If a heart beat is detected
      if (checkForBeat(irBuffer[i]) == true)
      {
        // Measure duration between two beats
        delta = millis() - lastBeat;
        lastBeat = millis();

        // Calculating the BPM
        beatsPerMinute = 60 / (delta / 1000.0);

        // Calculating the AvgBeat
        // To calculate it we store some values (4 in this case) and then do some math to calculate the average
        if (beatsPerMinute < 255 && beatsPerMinute > 20)
        {
          rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
          rateSpot %= RATE_SIZE;                    // Wrap variable

          // Take average of readings
          beatAvg = 0;
          for (byte x = 0; x < RATE_SIZE; x++)
          {
            beatAvg += rates[x];
          }
          beatAvg /= RATE_SIZE;
        }
        // DISPLAY when beat is detected, shows animation (logo 3)
        display.clearDisplay();                               // Clear the display
        display.drawBitmap(0, 0, logo3_bmp, 32, 32, WHITE);   // Draw the second picture (bigger heart)
        display.setTextSize(1);                               // And still displays the average BPM
        display.setTextColor(WHITE);
        display.setCursor(45, 0);
        display.println("BPM");
        display.setCursor(45, 18);
        display.println(beatAvg);
        display.setCursor(95, 0);
        display.println("SpO2");
        display.setCursor(95, 18);
        display.println(spo2);
        display.display();
        tone(3, 1000);  // Tone the buzzer for 80ms
        delay(80);
        noTone(3);      // Deactivate the buzzer to have a "bip" effect
      }
      
      // particleSensor.nextSample(); // We're finished with this sample so move to next sample

      // If the finger is not detected any more break out of the for loop
      if (irBuffer[i] < 7000)
        break;

      samplesCount++;
      
    }

    // If the 25 new samples were taken obtain the new oxygen value
    if (samplesCount >= 25)
    {
      maxim_heart_rate_and_oxygen_saturation(irBuffer, BUFFER_LENGHT, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

      // If new value is not valid --> spo2 = 0
      if(validSPO2 != 1)
        spo2 = 0;

      // Shifting buffer for the 25 new samples of the next loop
      for (byte i = 25; i < 100; i++)
      {
        redBuffer[i - 25] = redBuffer[i];
        irBuffer[i - 25] = irBuffer[i];
      }
    }
  }
}


#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define LOGO_HEIGHT 16
#define LOGO_WIDTH 16

// Numer pinu do którego podłaczasz czujnik
#define ONEWIRE_PIN 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;

OneWire onewire(ONEWIRE_PIN);
DallasTemperature sensor(&onewire);

#define BUTTON_PIN 9
#define LED_BLUE_PIN 11
#define LED_GREEN_PIN 12
#define LED_RED_PIN 13

boolean showTemperature = true;

unsigned long diffTime = 0;
unsigned long savedTime = 0;
unsigned long actualTime = 0;

int buttonState = HIGH;
int lastButtonState = HIGH;         // the previous reading from the input pin
unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
const int DEBOUNCE_DELAY = 50;

void initPins();
void initTemperatureSond();
void initBmeSensor();
void initDisplay();
void blinkRedLed(int count);

void setup()
{
  Serial.begin(9600);
  initPins();

  digitalWrite(LED_BLUE_PIN, LOW);
  blinkRedLed(1);

  initTemperatureSond();
  initBmeSensor();
  initDisplay();

  digitalWrite(LED_BLUE_PIN, HIGH);
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, LOW);
}

void processButtonState();
void displayTemperatureScreen();
void displayPressureAndHumidityScreen();

void loop()
{
  processButtonState();

  actualTime = millis();
  diffTime = actualTime - savedTime;

  if (showTemperature)
  {
    if (diffTime >= 500UL)
    {
      digitalWrite(LED_BLUE_PIN, LOW);
      digitalWrite(LED_GREEN_PIN, HIGH);

      displayTemperatureScreen();

      savedTime = actualTime;
      digitalWrite(LED_BLUE_PIN, HIGH);
      digitalWrite(LED_GREEN_PIN, LOW);
    }
  }
  else
  {
    if (diffTime >= 500UL)
    {
      digitalWrite(LED_BLUE_PIN, LOW);
      digitalWrite(LED_GREEN_PIN, HIGH);

      displayPressureAndHumidityScreen();

      savedTime = actualTime;
      digitalWrite(LED_BLUE_PIN, HIGH);
      digitalWrite(LED_GREEN_PIN, LOW);
    }
  }
}

void displayPressureAndHumidityScreen()
{

  display.setCursor(0, 0); // Start at top-left corner
  display.clearDisplay();

  display.println("Pressure:");
  float pressure = bme.readPressure();
  float pressureInHpa = pressure / 100; // preasure in hectopascals

  display.print(pressureInHpa, 1);
  display.println(" hPa");

  display.println("Humidity:");
  float humidity = bme.readHumidity();

  display.print(humidity);
  display.println(" %");
  display.display();
}

void displayTemperatureScreen()
{
  display.setCursor(0, 0); // Start at top-left corner
  display.clearDisplay();

  sensor.requestTemperaturesByIndex(0); // to trwa wieki
  display.println(F("Sonda:"));
  display.print(sensor.getTempCByIndex(0), 1);
  display.println(" *C");

  display.println(F("Temp:"));
  display.print(bme.readTemperature(), 1);
  display.println(" *C");
  display.display();
}

void processButtonState()
{
  int currentButtonState = digitalRead(BUTTON_PIN);

  if (currentButtonState != lastButtonState)
  {
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, HIGH);
    lastDebounceTime = millis();
    lastButtonState = currentButtonState;
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY)
  {
    if (currentButtonState == LOW && buttonState == HIGH)
    {
      showTemperature = !showTemperature;
    }

    buttonState = currentButtonState;
    digitalWrite(LED_RED_PIN, HIGH);
    digitalWrite(LED_GREEN_PIN, LOW);
  }
}

void initPins()
{
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_BLUE_PIN, OUTPUT);
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
}

void initTemperatureSond()
{
  sensor.begin(); //inicjalizacja czujnika DS18B20
  sensor.setWaitForConversion(false);
}

void blinkRedLed(int count)
{
  int i = 0;
  while (i < count)
  {
    digitalWrite(LED_RED_PIN, LOW);
    delay(1000);
    digitalWrite(LED_RED_PIN, HIGH);
    delay(1000);
    ++i;
  }
}

void initBmeSensor()
{
  if (!bme.begin(0x76))
  {
    //Serial.println("Could not find a valid BME280 sensor, check wiring!");
    blinkRedLed(2);
  }
}

void initDisplay()
{
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3D for 128x64, 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
    blinkRedLed(3);
  }

  display.display();
  display.setTextSize(2);              // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text

  // Clear the buffer
  display.clearDisplay();
}
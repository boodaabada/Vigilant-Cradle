#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ESP32Servo.h>
#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"

// WiFi credentials
const char* ssid = "LIU";
const char* password = "liu@std23";

// Telegram bot credentials
const char* telegramToken = "7045321554:AAFZP29ZOlu8EyFpDCWV0jXuU1tLtkqY1wQ";
const char* chatId = "6981784578"; // Replace with your chat ID

// Define pin connections for ESP32
#define RXD2 4  // Use GPIO 4 for RX (D4)
#define TXD2 2  // Use GPIO 2 for TX (D2)
#define SoundSensor 33 // Use GPIO 33 for the sound sensor
#define servoPin 13 // Use GPIO 13 for the servo motor
#define buttonPin 23 // Use GPIO 23 for the button

HardwareSerial FPSerial(2);  // Use UART2 on the ESP32

DFRobotDFPlayerMini myDFPlayer;
WiFiClientSecure client;
UniversalTelegramBot bot(telegramToken, client);

int micvalue = 0;
Servo servoMotor; // Define servo object
unsigned long cryStartTime = 0; // Variable to store the start time of crying
bool crying = false; // Flag to indicate if baby is crying
bool playFile1 = true; // Flag to alternate between file 1 and file 2

bool systemOn = false; // System state flag
bool lastButtonState = LOW; // Previous state of the button

void printDetail(uint8_t type, int value);

void setup() {
  Serial.begin(115200);

  pinMode(SoundSensor, INPUT);
  pinMode(buttonPin, INPUT_PULLUP); // Initialize button pin
  servoMotor.attach(servoPin); // Attaching servo to pin

  FPSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);

  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(FPSerial, /*isACK = */true, /*doReset = */true)) {  // Use serial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1. Please recheck the connection!"));
    Serial.println(F("2. Please insert the SD card!"));
    while (true);
  }
  Serial.println(F("DFPlayer Mini online."));

  myDFPlayer.setTimeOut(500); // Set serial communication timeout 500ms

  // Set volume
  myDFPlayer.volume(25);  // Set volume value (0~30).

  // Set EQ
  myDFPlayer.EQ(DFPLAYER_EQ_NORMAL);

  // Set device to use SD as default
  myDFPlayer.outputDevice(DFPLAYER_DEVICE_SD);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi!");

  client.setInsecure();  // This is for skipping certificate validation. Not secure for production!
}

void loop() {
  bool buttonState = digitalRead(buttonPin);

  // Check if button is pressed
  if (buttonState == LOW && lastButtonState == HIGH) {
    systemOn = !systemOn; // Toggle system state
    if (systemOn) {
      Serial.println("System turned ON");
      sendTelegramMessage("System turned ON");
    } else {
      Serial.println("System turned OFF");
      sendTelegramMessage("System turned OFF");
    }
  }

  lastButtonState = buttonState;

  if (systemOn) {
    micvalue = analogRead(SoundSensor);
    Serial.println(micvalue);

    // Check if baby is crying
    if (micvalue > 4000) {
      if (!crying) { // Start the timer if crying has just started
        cryStartTime = millis();
        crying = true;

        // Alternate between file 1 and file 2
        if (playFile1) {
          myDFPlayer.play(1);  // Play the first mp3 (001.mp3)
          Serial.println("Baby is crying. Playing track 1");
        } else {
          myDFPlayer.play(2);  // Play the second mp3 (002.mp3)
          Serial.println("Baby is crying. Playing track 2");
        }

        playFile1 = !playFile1; // Toggle the flag for the next cry
        sendTelegramMessage("Baby is crying");
      }

      // Control servo motor for 30 seconds
      if (millis() - cryStartTime <= 30000) {
        // Move servo from left to right twice
        for (int i = 0; i < 6; i++) {
          // Move servo from left to right
          for (int angle = 0; angle <= 120; angle++) {
            servoMotor.write(angle);
            delay(15);
          }
          // Move servo from right to left
          for (int angle = 120; angle >= 0; angle--) {
            servoMotor.write(angle);
            delay(15);
          }
        }
      } else {
        myDFPlayer.stop();  // Stop the audio playback after 30 seconds
        crying = false; // Reset the crying flag after 30 seconds
      }
    } else if ((micvalue >= 3310) && (micvalue <= 3500)) {
      Serial.println("Baby is laughing");
      sendTelegramMessage("Baby is laughing");
    }

    if (myDFPlayer.available()) {
      printDetail(myDFPlayer.readType(), myDFPlayer.read()); // Print the detail message from DFPlayer to handle different errors and states.
    }
  }

  delay(50);
}

void sendTelegramMessage(const char* message) {
  if (WiFi.status() == WL_CONNECTED) { // Check Wi-Fi connection
    int httpResponseCode = bot.sendMessage(chatId, message, "");
    if (httpResponseCode == 200) {
      Serial.println("Message sent successfully.");
    } else {
      Serial.print("Error sending message. HTTP response code: ");
      Serial.println(httpResponseCode);
    }
  } else {
    Serial.println("WiFi connection lost. Telegram message not sent.");
  }
}

void printDetail(uint8_t type, int value) {
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerUSBInserted:
      Serial.println("USB Inserted!");
      break;
    case DFPlayerUSBRemoved:
      Serial.println("USB Removed!");
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}

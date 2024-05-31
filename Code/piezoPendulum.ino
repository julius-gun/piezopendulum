/*!
 * @file main.ino
 * @brief This script is the main script to use the Piezo pendulum. The Raspberry Pi Pico RP2040 controls the display, reads the encoder and button inputs and controls the piezostack. 
 * @n This demo supports at least Raspberry Pi Pico. Other boards are not tested. 
 * @author 
 * @author [JuliusGun]
 * @version V1.0 
 * @date 2024-05-31
 * @thanks to Jan Rodewald, who developed the first version V0.1, and for help along the way
 */

//  * @version V0.2 → platformIO refactoring, Raspberry Pico rp2040

// #include <Arduino.h>
// ------------------------------------------------------------------------------------------------------------------------------
// --- Read the libaries --------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------
#include "pico/stdlib.h"
#include "hardware/irq.h" // might be needed to access lower-level interrupt functions
#include "DFRobot_GDL.h" // official Display Library
#include <DAC7571.h> // digital to analog converter library from https://github.com/jrj12591/DAC7571/
// #include "src/DAC7571/DAC7571.h"
#include <SD.h>
#include <Wire.h>
#include <SPI.h>
// ------------------------------------------------------------------------------------------------------------------------------
#define SYSTICK_FREQUENCY 133000000UL // 133MHz for RP2040
// ------------------------------------------------------------------------------------------------------------------------------
// --- Define the debug level ---------------------------------------------------------------------------------------------------
// RPI_PICO_Timer Timer; // Instantiate a timer object

// --- Initialize DAC5571 with default I2C address 0x4C → A0 connected to GND-------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------
DAC5571 dac;
#define I2C_SDA 20
#define I2C_SCL 21
#define I2C_PORT i2c0
const float VDD = 3.3; // Set the voltage that the DAC will run at
// Voltage regulation
float current_voltage;
// ------------------------------------------------------------------------------------------------------------------------------
// --- Define fixed variables ---------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------
// Display
#define TFT_DC 6
#define TFT_RST 7
#define TFT_MISO 16
#define TFT_CS 17
#define TFT_SCLK 18
#define TFT_MOSI 19
// Analog read Voltage
#define V_MEASURE 27
// Input pins definitions
#define ENCODER_A 1
#define ENCODER_B 0
#define PUSH_BUTTON 3
#define HIT_IMPULSE 15 // Impulse detection
// Output pins definitions
#define LED_IMPULSE 4
#define LED_POWER 2
#define GATE_DRIVE_HIN 13
#define GATE_DRIVE_LIN 14
// Colors in RGB565 format
#define COLOR_BLACK 0x0000
#define COLOR_WHITE 0xFFFF
#define COLOR_TUM_BLUE 0x3396 // TUM Blue
#define COLOR_GREY 0x6CAF
#define COLOR_MIMED_YELLOW 0xFE60           // MiMed Yellow
#define FONT_COLOR_HEADLINES COLOR_TUM_BLUE // White
#define FONT_COLOR_READY_GREEN 0x07E0       // GREEN
#define FONT_COLOR_READY_RED 0xF800         // RED
#define COLOR_FILL_RECT COLOR_WHITE
// define positions
#define SCREEN_WIDTH 240  // 240
#define SCREEN_HEIGHT 320 // 320
#define TEXT_SIZE 3

// Text positions and dimensions (based on the selected font's characteristics)
#define CHARACTER_WIDTH (6 * TEXT_SIZE)
#define CHARACTER_HEIGHT (8 * TEXT_SIZE)

// Dynamic starting positions for text based on screen sections (as a fraction of SCREEN_HEIGHT)
#define STARTING_Y_READY (SCREEN_HEIGHT * .05)    // down from the top
#define STARTING_Y_HEAD (SCREEN_HEIGHT * 0.12)    // down from the top of the screen
#define STARTING_Y_MODES (SCREEN_HEIGHT * 0.25)   // down from the top
#define STARTING_Y_VARIABLE (SCREEN_HEIGHT * 0.8) // down from the top

#define STARTING_X 0
#define SCREEN_CLC_MODES_HEIGHT 160
#define SCREEN_CLC_READY_WIDTH 82
#define SCREEN_CLC_READY_HEIGHT 18

// define delays
#define TIME_DELAY_MAIN_LOOP 1
#define TIME_ENCODER_DELAY 700
#define TIME_BUTTON_DELAY 400
#define TIME_READY_DELAY 3500
#define TIME_DELAY_INIT 500

// Initialize the display using hardware SPI communication
DFRobot_ST7789_240x320_HW_SPI screen(/*dc=*/TFT_DC, /*cs=*/TFT_CS, /*rst=*/TFT_RST);

// Miscellaneous variables
const int serialDebug = 1; // CHANGE TO 0 in normal mode. Do you want to see the debug messages? 1 = yes, 0 = no
int menuItemIndex = 0;     // Use this variable to store the current index of the menu
int currentStateClock;     // Store the status of the clock pin (HIGH or LOW)
int lastEncoderAClock;     // Store the PREVIOUS status of the clock pin (HIGH or LOW)
int currentENCODER_B;
int lastEncoderB;
int buttonState = HIGH;            // Current state of the button
unsigned long lastButtonState = 0; // Use this to store if the push button was pressed or not
unsigned long lastEncoderUpdateTime = 0;
unsigned long lastPushDebounceTime = 0;       // Last time the input pin was toggled
const unsigned long PUSH_DEBOUNCE_DELAY = 50; // Debounce delay in milliseconds

// ------------------------------------------------------------------------------------------------------------------------------
// --- Define menu items --------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------
// max Values
#define MAX_MENU_ITEMS 7 // Number of menu items
String menuOptions[MAX_MENU_ITEMS] = {
    "All functions",
    "Push",
    "Catch",
    "Catch after 2",
    "Catch after 3",
    "Resonance",
    "Resonance        after 2"};
// Resonance and catch after third

// for 64V
//  Timer delay for each menu item in microseconds
volatile unsigned int timerDelays[MAX_MENU_ITEMS] = {
    0, // Delay for "All functions"
    0, // Delay for "Push"
    8, // Delay for "Catch"
    5, // Delay for "Catch 2"
    1, // Delay for "Catch 3"
    0, // Delay for "Resonance"
    0  // Delay for "Resonance 2"
};
// int catchDelay1 = 8;
// int catchDelay2 = 5;
// int catchDelay3 = 1;

String headline = "Select action:";

String ready_string[2] = {
    "execute",
    "ready"};
int readyState = 1;
int iLast_ButtonPushed = 0;

// Piezo variables
volatile bool ball_detected = false;
volatile int resonance_number = 0;

// serial communication
volatile bool newDataAvailable = false;
String inputString;          // a String to hold incoming ENCODER_B
bool stringComplete = false; // whether the string is complete

// LED impulse variables
volatile bool led_on = false;
volatile uint16_t led_on_time = 0;

// ------------------------------------------------------------------------------------------------------------------------------
// --- Define functions ---------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------

// forward declarations for platformIO
// input
bool checkButtonPressed();
int checkEncoderChanged();
void inputMethods();
int changeMenuItem(int iChange);
void displayClearSection(int x, int y, int width, int height); // Display functions
void displayMenuText();
void displayReadyString();
void displayVoltage();
void chargePiezo(); // Piezofunctions
void waitForBallImpacts(int number_of_repetitions);
void interruptBallDetected();
void ledImpulse();
void performPiezoAction(int piezo_select);
void executePushAction();
void executePushAndCatchAction(int bounceCount);
void executeResonanceAction(int bounceCount);
void printNumberBeingPlayed(int repetitionIndex);
void handleSerialInput();
void dischargePiezo(uint32_t time_to_stop_us);
float checkVoltagePiezo(); // voltage
void setVoltagePiezo(float V_want_piezo);
void setupI2CDAC();// Setup helper functions
void setupGatedriver();
void setupEncoder();
void setupInterrupts();
void setupScreen();

// ------------------------------------------------------------------------------------------------------------------------------

/**
 * Checks if the button is pressed and returns a value indicating the button state, with debouncing.
 *
 * @return true if the button is pressed, false otherwise.
 */
bool checkButtonPressed()
{
  // Read the state of the switch/button into a local variable:
  int reading = digitalRead(PUSH_BUTTON);

  // Check if the button state has changed
  if (reading != lastButtonState)
  {
    // reset the debouncing timer
    lastPushDebounceTime = millis();
  }

  if ((millis() - lastPushDebounceTime) > PUSH_DEBOUNCE_DELAY)
  {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState)
    {
      buttonState = reading;

      // only toggle if the new button state is LOW (button pressed)
      if (buttonState == LOW)
      {
        // Button has been pressed
        return true;
      }
    }
  }

  // Save the reading. Next time through the loop, it'll be the lastButtonState
  lastButtonState = reading;
  return false;
}
// variables for the encoder
const int ENCODER_NO_CHANGE = -1;
const int ENCODER_COUNTER_CLOCKWISE = -1;
const int ENCODER_CLOCKWISE = 1;

// Function to check if encoder has changed
int checkEncoderChanged()
{
  int changeValue = ENCODER_NO_CHANGE; // Default to no change
  unsigned long currentEncoderTime = millis();

  // Read the current state of the encoder pins
  int currentEncoderAClock = digitalRead(ENCODER_A);
  int currentEncoderB = digitalRead(ENCODER_B);

  // Check if there has been any change in the state of the encoder pins
  if (lastEncoderAClock != currentEncoderAClock || lastEncoderB != currentEncoderB)
  {
    if (currentEncoderTime > (lastEncoderUpdateTime + TIME_ENCODER_DELAY))
    {
      // Check the transition of state to determine direction
      if (((currentEncoderAClock == 1 && currentEncoderB == 0) && (lastEncoderAClock == 0 && lastEncoderB == 0)) || ((currentEncoderAClock == 1 && currentEncoderB == 1) && (lastEncoderAClock == 1 && lastEncoderB == 0)) || ((currentEncoderAClock == 0 && currentEncoderB == 1) && (lastEncoderAClock == 1 && lastEncoderB == 1)) || ((currentEncoderAClock == 0 && currentEncoderB == 0) && (lastEncoderAClock == 0 && lastEncoderB == 1)))
      {
        changeValue = changeMenuItem(ENCODER_COUNTER_CLOCKWISE); // Counter clockwise rotation
      }
      else if (((currentEncoderAClock == 0 && currentEncoderB == 1) && (lastEncoderAClock == 0 && lastEncoderB == 0)) || ((currentEncoderAClock == 1 && currentEncoderB == 1) && (lastEncoderAClock == 0 && lastEncoderB == 1)) || ((currentEncoderAClock == 1 && currentEncoderB == 0) && (lastEncoderAClock == 1 && lastEncoderB == 1)) || ((currentEncoderAClock == 0 && currentEncoderB == 0) && (lastEncoderAClock == 1 && lastEncoderB == 0)))
      {
        changeValue = changeMenuItem(ENCODER_CLOCKWISE); // Clockwise rotation
      }

      // Update the last state

      // If a valid item change has occurred, update the time
      if (changeValue != ENCODER_NO_CHANGE)
      {
        lastEncoderUpdateTime = millis();
      }
    }
    lastEncoderAClock = currentEncoderAClock;
    lastEncoderB = currentEncoderB;
  }

  return changeValue; // Return the change in menu item (-1, 0, 1)
}

int changeMenuItem(int step)
{
  menuItemIndex = menuItemIndex + step;
  if (menuItemIndex >= MAX_MENU_ITEMS)
  {
    menuItemIndex = MAX_MENU_ITEMS - 1; // 0;
    return -1;
  }
  if (menuItemIndex < 0)
  {
    menuItemIndex = 0; // MAX_PIEZO_MODES-1;
    return -1;
  }
  return menuItemIndex;
}

void inputMethods()
{
  int iNew_ButtonPushed = millis();
  if (iNew_ButtonPushed > iLast_ButtonPushed + TIME_READY_DELAY && readyState != 1)
  {
    readyState = 1;
    displayReadyString();
  }
  else if (checkButtonPressed() == 1)
  {
    if (iNew_ButtonPushed > iLast_ButtonPushed + TIME_READY_DELAY)
    {
      iLast_ButtonPushed = iNew_ButtonPushed;
      readyState = 0;
      displayReadyString();
    }
  }
  if (checkEncoderChanged() != -1)
  {
    displayMenuText();
  }

  // Execute the selected action
  if (readyState == 0) // Condition met when execute button clicked
  {
    menuSelectAction();
  }
}

void menuSelectAction()
{
  int commandToExecute = menuItemIndex + 1; // Menu item index matches performPiezoAction case
  if (serialDebug)
  {
    Serial.print("Piezo command selected: ");
    Serial.println(menuOptions[menuItemIndex]); // Print the string
  }
  delay(500);                           // Delay so that the push button does not cause the ball to not be in the right position
  ball_detected = false;                // if the push button caused the interrupt to be triggered, falsify it again
  performPiezoAction(commandToExecute); // menuItemIndex starts at 0, and case 1 is "Push"
  readyState = 1;                       // Reset the ready flag to prevent repeated command execution without button presses
  displayReadyString();                 // Update the display to show the change in state
  displayVoltage();
}

// Modular helper function implementations
void displayClearSection(int x, int y, int width, int height)
{
  screen.fillRect(x, y, width, height, COLOR_FILL_RECT); // Clear the designated screen section
}

void displayMenuText()
{
  // Calculate the number of items to display before and after the selected item.
  const int itemsBeforeAfter = 2;

  // Clear the menu area before displaying the new text
  displayClearSection(STARTING_X, STARTING_Y_MODES, screen.width(), SCREEN_CLC_MODES_HEIGHT);

  // Display 5 menu options at a time: 2 before, 1 current, and 2 after
  for (int offset = -itemsBeforeAfter; offset <= itemsBeforeAfter; ++offset)
  {
    int menuIndex = menuItemIndex + offset;

    // Skip if the menuIndex is out of the array bounds
    if (menuIndex < 0 || menuIndex >= MAX_MENU_ITEMS)
    {
      continue;
    }

    // Calculate the Y position for each item; adjust it based on your screen's layout
    int yPos = STARTING_Y_MODES + (CHARACTER_HEIGHT + 2) * (offset + itemsBeforeAfter);

    // Set cursor position
    screen.setCursor(STARTING_X, yPos);

    // Set the text color: highlight the current menu item, otherwise use the default color
    uint16_t color = (offset == 0) ? COLOR_MIMED_YELLOW : FONT_COLOR_HEADLINES;
    screen.setTextColor(color);

    // Set the text size for uniformity
    screen.setTextSize(TEXT_SIZE);

    // Print the menu index and option
    screen.println(String(menuIndex + 1) + " - " + menuOptions[menuIndex]);
  }

  // Print the header at the top
  screen.setCursor(STARTING_X, STARTING_Y_HEAD);
  screen.setTextColor(COLOR_GREY);
  screen.setTextSize(TEXT_SIZE);
  screen.println(headline);
  ledImpulse();
}

void displayReadyString()
{
  ledImpulse();
  // Calculate the dimensions needed for the text based on text length and size
  int text_length = ready_string[readyState].length();
  int text_width = CHARACTER_WIDTH * text_length * 1.4; // Width of entire text string
  int text_height = CHARACTER_HEIGHT;                   // Height of text string

  // Calculate starting X so the text is horizontally centered
  int text_x = STARTING_X;
  int text_y = STARTING_Y_READY - (text_height / 2); // Adjust Y so the text is vertically centered

  // Clear the rectangle area where the ready string will be displayed
  displayClearSection(text_x, text_y, text_width, text_height);

  // Set the cursor to the calculated starting position for the ready string
  screen.setCursor(text_x, text_y);
  // Set the text color depending on the state of readyState
  if (readyState == 0)
  {
    screen.setTextColor(FONT_COLOR_READY_RED);
  }
  else
  {
    screen.setTextColor(FONT_COLOR_READY_GREEN);
  }
  // Set the text size and print the string
  screen.setTextSize(TEXT_SIZE);
  screen.println(ready_string[readyState]);
  if (readyState == 0)
  {
    if (serialDebug)
    {
      displayTimeMicroseconds();
    }
  }
  else
  {
    displayVoltage();
  }
}

void displayVoltage()
{
  current_voltage = (int)checkVoltagePiezo();
  int text_width = 300; // Width of entire text string
  int text_x = 245;
  int text_y = STARTING_Y_READY - (CHARACTER_HEIGHT / 2);
  screen.setCursor(text_x, STARTING_Y_READY - (CHARACTER_HEIGHT / 2));
  screen.setTextColor(COLOR_TUM_BLUE);
  screen.setTextSize(TEXT_SIZE);
  // Clear the rectangle area where the ready string will be displayed
  displayClearSection(140, text_y, text_width, CHARACTER_HEIGHT);
  screen.print(current_voltage, 0);
  screen.print("V");
}

void displayTimeMicroseconds()
{
  int text_x = 145;
  int text_y = STARTING_Y_READY - (CHARACTER_HEIGHT / 2);
  screen.setCursor(text_x, STARTING_Y_READY - (CHARACTER_HEIGHT / 2));
  screen.setTextColor(COLOR_TUM_BLUE);
  screen.setTextSize(TEXT_SIZE);
  // Clear the rectangle area where the ready string will be displayed
  displayClearSection(text_x, text_y, 200, CHARACTER_HEIGHT);
  int timeDelay = timerDelays[menuItemIndex];
  screen.print("delay:");
  screen.print(timeDelay);
  screen.print("us");
}

/**
 * @brief Turns the piezo actuator instantly on, using the gatedriver.
 */
void chargePiezo()
{
  digitalWrite(GATE_DRIVE_LIN, HIGH); // charge bootstrap capacitor
  delayMicroseconds(30);              // todo: optimize time
  digitalWrite(GATE_DRIVE_LIN, LOW);
  delayMicroseconds(10);
  digitalWrite(GATE_DRIVE_HIN, HIGH); // SHOOT: Turn on higher side MOSFET (Q1)
  delayMicroseconds(100);             // todo: optimize time
  digitalWrite(GATE_DRIVE_HIN, LOW);  // Turn off higher side MOSFET (Q1)
  delay(25);                          // todo: how long does it take to charge the capacitor? + 10%
  // ball_detected = false;              // Reset ball_detected for the next detection, so that it does not automatically jumps a step in the while loop
}

int MAXIMUM_TIME_WAITING = 5000000; // s
int piezo_on_time;
// volatile uint32_t time_to_release_nanosec = 5385; // time in ns This will hold the time to stop piezo, adjustable via encoder
volatile int number_of_repetitions = 1;

void waitForBallImpacts(int number_of_repetitions)
{
  unsigned int startTime = micros();
  int numberBouncesDetected = 0;
  while (numberBouncesDetected <= number_of_repetitions)
  {
    // Timeout check
    if (micros() - startTime > MAXIMUM_TIME_WAITING)
    {
      break; // Exit the loop
    }
    if (ball_detected)
    {
      numberBouncesDetected++;
      ball_detected = false; // Reset ball_detected for the next detection
    }

    // Non-blocking delay check
    // if (micros() - previousTime >= 10)
    // {                          // Check if 10 microseconds have passed
    //   previousTime = micros(); // Reset the timer
    //                            // ledImpulse(); // Uncomment this if you wish to trigger an LED impulse
    // }
  }
}

unsigned long lastDetectionTime = 0;      // Tracks the time since the last detection
const unsigned long debounceDelay = 3000; // Debounce delay in microseconds

void resonancePiezo(int number_of_repititions)
{
  int number_detected = 0;
  unsigned int startTime = micros();
  unsigned long previousTime = micros();

  // The variable 'previousTime' is used for non-blocking delay
  while (number_detected <= number_of_repititions)
  {
    if (micros() - startTime > MAXIMUM_TIME_WAITING)
    {
      break; // Exit the loop
    }
    if (ball_detected && (micros() - lastDetectionTime >= debounceDelay))
    {
      number_detected++;
      ball_detected = false;        // Reset ball_detected for the next detection
      lastDetectionTime = micros(); // Update the last detection time
    }
  }
}

void dischargePiezo(uint32_t time_to_stop_us)
{
  delayMicroseconds(time_to_stop_us);
  digitalWrite(GATE_DRIVE_LIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(GATE_DRIVE_LIN, LOW);
}

volatile unsigned long lastImpulseDebounceTime = 0; // Timestamp of the last time the ISR was triggered
const long impulseDebounceDelay = 8;                // Debounce time in milliseconds
volatile bool resonanceMode = false;                // Flag to indicate if the resonance mode is active

// Interrupt Service Routine (ISR) for the ball detected pin
void interruptBallDetected()
{
  // // Check if enough time has passed since the last detected interrupt (debouncing)
  if ((millis() - lastImpulseDebounceTime) > impulseDebounceDelay)
  {
    ball_detected = true;
    // led_on = true;
    // digitalWrite(LED_IMPULSE, HIGH); // Turn on the LED
    // led_on_time = millis();

    // if (serialDebug)
    // {
    //   Serial.println("Ball detected!");
    // }

    // Update the lastImpulseDebounceTime to the current time
    lastImpulseDebounceTime = millis();
  }
  if (resonanceMode)
  {
    ball_detected = true;
    // digitalWrite(LED_IMPULSE, HIGH); // Turn on the built-in LED
    led_on = true;
    led_on_time = millis(); // perhaps to slow
  }
}

// Function to handle the LED impulse
void ledImpulse()
{
  if (led_on)
  {
    // Turn on the LED only if it is not already on
    if (!digitalRead(LED_IMPULSE))
    {
      digitalWrite(LED_IMPULSE, HIGH); // Turn on the LED
    }

    // Check if 600ms passed since the LED was turned on
    if (millis() - led_on_time > 600)
    {
      digitalWrite(LED_IMPULSE, LOW); // Turn off the LED
      led_on = false;                 // Clear the led_on flag
      ball_detected = false;          // Reset the ball_detected flag
    }
  }
}
void executePushAction()
{
  for (int repetitionIndex = 0; repetitionIndex < number_of_repetitions; repetitionIndex++)
  {
    chargePiezo();
    delay(100);
    dischargePiezo(0);
    delay(1200);
    printNumberBeingPlayed(repetitionIndex);
  }
}

void executeAllPiezoActions()
{
  // Perform all actions in sequence
  readyState = 0;       // Reset the ready flag to prevent repeated command execution without button presses
  displayReadyString(); // Update the display to show the change in state
  for (int currentMenuIndex = 2; currentMenuIndex < MAX_MENU_ITEMS + 1; currentMenuIndex++)
  {
    // performPiezoAction(currentMenuIndex);
    // delay(500);
    menuItemIndex = currentMenuIndex - 1;
    displayMenuText();                    // Update the display to show the change in state
    performPiezoAction(currentMenuIndex); // menuItemIndex starts at 0, and case 1 is "Push"
    delay(800);
  }
  menuItemIndex = 0;
  readyState = 1;
  displayReadyString(); // Update the display to show the "ready" state
  displayMenuText();    // Update the display to show the first menu item
}

void executePushAndCatchAction(int bounceCount)
{
  Serial.print("Bounce count in executePushAndCatchAction: "); // Print the text
  Serial.println(bounceCount);                                 // Print the variable
  Serial.print("timerDelays[bounceCount+1]: ");                // Print the text
  Serial.println(timerDelays[bounceCount + 1]);                // Print the variable

  for (int repetitionIndex = 0; repetitionIndex < number_of_repetitions; repetitionIndex++)
  {
    chargePiezo();
    waitForBallImpacts(bounceCount);
    dischargePiezo(timerDelays[bounceCount + 1]);
    delay(1600);
    printNumberBeingPlayed(repetitionIndex);
    ledImpulse();
  }
}

void executeResonanceAction(int bounceCount)
{
  for (int repetitionIndex = 0; repetitionIndex < 7; repetitionIndex++)
  {
    chargePiezo();
    // waitForBallImpacts(bounceCount);
    resonancePiezo(bounceCount);
    dischargePiezo(0);

    // printNumberBeingPlayed(repetitionIndex);
  }
}

void performPiezoAction(int piezo_select)
{
  // Log the selected command if serial debug is enabled
  delay(500);
  // Execute action based on the selected command
  switch (piezo_select)
  {
  case 1: // "All functions" - Perform all actions in sequence
    executeAllPiezoActions();
    break;

  case 2: // "Push"
    executePushAction();
    break;

  case 3:                         // "Push & Catch"
    executePushAndCatchAction(1); // Catch after 1 pulse
    break;

  case 4:                         // "Catch after 2nd pulse"
    executePushAndCatchAction(2); // Catch after 2 pulses
    break;

  case 5:                         // "Catch after 3rd pulse"
    executePushAndCatchAction(3); // "Catch after 3rd pulse"
    break;

  case 6:                      // "Resonance (Impulse on detection)"
    executeResonanceAction(1); // Resonance after 1 pulse
    break;

  case 7:                      // "Resonance after 2nd pulse"
    executeResonanceAction(2); // Resonance after 2 pulses
    break;

  default:
    // Optional default case to handle undefined actions
    break;
  }
}

void printNumberBeingPlayed(int repetitionIndex)
{
  if (serialDebug)
  {
    // Serial.print("Number being played: "); // Print the text
    // Serial.println(repetitionIndex + 1);   // Print the variable
  }
}

/**
 * @brief Checks the voltage of the piezo sensor.
 *
 * This function reads the analog value from the V_MEASURE pin and converts it to voltage.
 * The voltage is calculated using the formula: current_voltage = analogRead(V_MEASURE) / 4095 * V_DD * 46.454.
 *
 * @return The voltage of the piezo sensor.
 */

float checkVoltagePiezo()
{
  int current_voltage = analogRead(V_MEASURE);
  delay(10);
  // current_voltage = analogRead(V_MEASURE) / 4095 * VDD * 46.454;
  current_voltage *= 0.0374; // Voltage divider equals to: analogRead(V_MEASURE) * (2200 + 100000) / 2200
  return current_voltage;
}

/**
 * Sets the voltage for the piezo.
 *
 * @param V_want_piezo The desired voltage for the piezo.
 */
void setVoltagePiezo(float V_want_piezo)
{
  if (V_want_piezo > 100) // max voltage
  {
    V_want_piezo = 100;
  }
  else if (V_want_piezo < 50) // min voltage
  {
    V_want_piezo = 50;
  }
  float slope = 21.25; // 85/4 = 21.25; 85 V output per 4 V input
  float V_send_i2c = (V_want_piezo - 50) / slope;
  V_send_i2c = V_send_i2c / .71304; // Voltage divider equals to 82e3 / (82e3 + 33e3)
  // Serial.println(V_send_i2c);
  // Set the voltage to the piezo V_P
  dac.writedac(V_send_i2c, VDD);
  delay(20); // Small delay to allow DAC to settle
}

void handleSerialInput()
{
  if (Serial.available())
  {
    // Get the new byte:
    char inChar = (char)Serial.read();
    // Add it to the inputString:
    inputString += inChar;
    // If the incoming character is a newline, set a flag so the main loop can process it
    if (inChar == '\n')
    {
      stringComplete = true;
    }
  }

  if (stringComplete)
  {
    // Trim the input string to remove whitespace and the line ending
    inputString.trim();

    // Convert the string to a float
    float newValue = inputString.toInt();

    // Check if the float conversion has returned a non-zero value or the input string is "0"
    if (newValue != 0 || inputString == "0")
    {
      // Assign the new value to time_to_release_nanosec
      timerDelays[menuItemIndex] = newValue;
      Serial.println("New " + menuOptions[menuItemIndex] + " value set: " + inputString);
    }
    else
    {
      Serial.println("Invalid input");
    }

    // Clear the string for new input and reset the complete flag
    inputString = "";
    stringComplete = false;
  }
}

void setupPins()
{
  // Initialize LED's as output
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(LED_IMPULSE, OUTPUT);
  pinMode(LED_POWER, OUTPUT);
  pinMode(PUSH_BUTTON, INPUT_PULLUP);
  pinMode(HIT_IMPULSE, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HIT_IMPULSE), interruptBallDetected, RISING);
  pinMode(V_MEASURE, INPUT); // Initialize voltage measurement pin
  analogReadResolution(12);  // 4096 possible values
}

void setupGatedriver()
{
  pinMode(GATE_DRIVE_HIN, OUTPUT);
  pinMode(GATE_DRIVE_LIN, OUTPUT);
  digitalWrite(GATE_DRIVE_LIN, HIGH); // Charge the bootstrap capacitor by driving the low side first high
  delay(50);
  digitalWrite(GATE_DRIVE_LIN, LOW);
}

void setupi2C_DAC()
{
  // i2c_init(I2C_PORT, 100 * 1000); // Initialize I2C with a clock speed of 100kHz
  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
  // Wire.begin(); // Initialize the I2C bus
  dac.begin(); // Initialize the DAC
  setVoltagePiezo(100);
}

void setupEncoder()
{
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(PUSH_BUTTON, INPUT_PULLUP);
  currentStateClock = digitalRead(ENCODER_A);
  lastEncoderAClock = currentStateClock;
  currentENCODER_B = digitalRead(ENCODER_B);
  lastEncoderB = currentENCODER_B;
}

void setupScreen()
{
  screen.begin();
  screen.setRotation(1);
  screen.fillScreen(COLOR_FILL_RECT);
  displayMenuText();    // show mode text
  displayReadyString(); // show ready string
}

void setupSerial()
{
  Serial.print(F("CPU Frequency = "));
  Serial.print(F_CPU / 1000000);
  Serial.println(F(" MHz"));
  // inputString.reserve(200); // Reserve 200 bytes for the inputString to reduce dynamic allocation

  Serial.begin(115200);
  // while (!Serial) // start serial comms, microcontroller won't start until communication is established
  ;
  delay(50);
}

// void testTimes()
// {
//   handleSerialInput();
//   executePushAndCatchAction(1);
//   timerDelays[menuItemIndex]++;
//   Serial.println(menuOptions[menuItemIndex]);
//   Serial.println(timerDelays[menuItemIndex]);
// }

void setup()
{
  setupEncoder();
  setupGatedriver();
  // setupi2C_DAC();
  setupPins();
  setupSerial();
  // executePushAction();
  setupScreen();
  delay(TIME_DELAY_INIT);
}

void loop()
{
  inputMethods();

  ledImpulse();
  // testTimes();
}

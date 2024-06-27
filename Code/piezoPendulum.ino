/*!
 * @file piezoPendulum.ino
 * @brief This script is the main script to use the Piezo pendulum. The Raspberry Pi Pico RP2040 controls the display, reads the encoder and button inputs and controls the piezostack.
 * @n This demo supports at least Raspberry Pi Pico. Other boards are not tested.
 * @authors [Julius Gun, Jan Rodewald] //Jan Rodewald developed the first version V0.1
 * @version V1.2
 * @date 2024-06-14
 * https://github.com/julius-gun/piezopendulum
 */

// #include <Arduino.h>
// ------------------------------------------------------------------------------------------------------------------------------
// --- Read the libaries --------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------
#include "pico/stdlib.h"
#include "hardware/irq.h" // might be needed to access lower-level interrupt functions
#include "DFRobot_GDL.h"  // official Display Library
// #include <DAC7571.h>      // digital to analog converter library from https://github.com/jrj12591/DAC7571/
#include "src/DAC7571.h"
#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <ButtonControl.h>
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
float currentVoltage;
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
ButtonControl PUSH_BUTTON(3);
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
#define TIME_ENCODER_DELAY 500
#define TIME_READY_DELAY 1500
#define TIME_DELAY_INIT 500

// Initialize the display using hardware SPI communication
DFRobot_ST7789_240x320_HW_SPI screen(/*dc=*/TFT_DC, /*cs=*/TFT_CS, /*rst=*/TFT_RST);

// Miscellaneous variables
bool SERIAL_DEBUG = 0;  // CHANGE TO 0 in normal mode. Do you want to see the debug messages? 1 = yes, 0 = no
bool EXPERT_MODE = 0;   // CHANGE TO 0 in normal mode. Without it the delay finder function is disabled. 1 = yes, 0 = no
int menuItemIndex = 0;  // Use this variable to store the current index of the menu
bool currentStateClock; // Store the status of the clock pin (HIGH or LOW)
bool lastEncoderA;      // Store the PREVIOUS status of the clock pin (HIGH or LOW)
bool currentENCODER_B;
bool lastEncoderB;
bool buttonState = HIGH;  // Current state of the button
bool lastButtonState = 0; // Use this to store if the push button was pressed or not
unsigned int lastEncoderUpdateTime = 0;
unsigned int lastPushDebounceTime = 0;       // Last time the input pin was toggled
const unsigned int PUSH_DEBOUNCE_DELAY = 50; // Debounce delay in milliseconds

// ------------------------------------------------------------------------------------------------------------------------------
// --- Define menu items --------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------
// max Values
#define MAX_MENU_ITEMS 7 // Number of menu items
String menuOptionsShort[MAX_MENU_ITEMS] = {
    "All functions",
    "Push",
    "Catch",
    "Catch after 2",
    "Catch after 3",
    "Resonance",
    "Resonance        after 2"};

// once a function is selected, the long description is displayed instead of the short one, filling the lower side of the screen
String menuOptionsLong[MAX_MENU_ITEMS] = {
    "Cycle Through all Functions   Sequentially",
    "Push the     Pendulum Ball",
    "Push and     Catch after First Impulse",
    "Push and     Catch after      Second Impulse",
    "Push and     Catch after      Third Impulse",
    "Resonance:   Propels the Ball upon Immediate   Impulse Detection",
    "Resonance 2: Propels the Ball upon Second      Impulse Detection"};
// Resonance and catch after third

// for 64V
//  Timer delay for each menu item in microseconds
volatile unsigned int timerDelays[MAX_MENU_ITEMS] = {
    0, // Delay for "All functions"
    0, // Delay for "Push"
    14, // Delay for "Catch"
    5, // Delay for "Catch 2"
    5, // Delay for "Catch 3"
    0, // Delay for "Resonance"
    0  // Delay for "Resonance 2"
};

String headline = "Select action:";

String readyString[2] = {
    "execute",
    "ready"};
unsigned int readyState = 1;
unsigned int lastButtonPushed = 0;

// Piezo variables
volatile bool ballDetected = false;

// LED impulse variables
long ledOnTime = 0;

// ------------------------------------------------------------------------------------------------------------------------------
// --- Define functions ---------------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------------------------------------

// forward declarations for platformIO
bool checkButtonPressed(); // Button handling
int getEncoderPosition();
void inputMethods();
int changeMenuItem(int iChange);
void displayClearSection(int x, int y, int width, int height); // Display functions
void displayMenuText();
void displayReadyString();
void displayVoltage();
void chargePiezo(); // Piezofunctions
void waitForBallImpacts(int numberOfDetections);
void interruptBallDetected();
void ledImpulse();
void performPiezoAction(int piezoSelect);
void executePushAction();
void executePushAndCatchAction(int bounceCount);
void executeResonanceAction(int bounceCount);
void handleSerialInput();
void dischargePiezo(uint32_t timeToDischargePiezoMicroseconds);
void chargeBootstrapCapacitor(uint32_t timeToChargeCapacitorMicroseconds);
float readVoltagePiezo(); // voltage
void setPiezoActuatorVoltage(float voltage);
void setupI2CDAC(); // Setup helper functions
void setupGatedriver();
void setupEncoder();
void setupInterrupts();
void setupScreen();

// ------------------------------------------------------------------------------------------------------------------------------
// ----Functions------------------------
void enablePiezoInterrupt() // Enable the interrupts for the ball detection pin

{
  attachInterrupt(digitalPinToInterrupt(HIT_IMPULSE), interruptBallDetected, RISING);
}

// void disablePiezoInterrupt() // Disable the interrupts for the ball detection pin → rather not use this function because it takes too long to reenable the interrupt
// {
//   // Disable the interrupts for the ball detection pin
//   detachInterrupt(digitalPinToInterrupt(HIT_IMPULSE));
// }

// unsigned long lastDetectionTime = 0;                // Tracks the time since the last detection
volatile unsigned int lastImpulseDebounceTime = 0; // Timestamp of the last time the ISR was triggered

// Interrupt Service Routine (ISR) for the ball detected pin
const uint16_t IMPULSE_DEBOUNCE_DELAY_CATCH = 4; // Debounce delay for resonance in milliseconds

void interruptBallDetected()
{
  // Check if enough time has passed since the last detected interrupt (debouncing)
  if ((millis() - lastImpulseDebounceTime) > IMPULSE_DEBOUNCE_DELAY_CATCH)
  {
    ballDetected = true;
    digitalWrite(LED_IMPULSE, HIGH); // Turn on the LED
    ledOnTime = millis();            // Set the time the LED was turned on
    // Update the lastImpulseDebounceTime to the current time
    lastImpulseDebounceTime = millis();
  }
}

void delayAndDoStuff(int timeDelayMilliseconds)
{
  unsigned int currentTime = millis();
  while (millis() - currentTime < timeDelayMilliseconds)
  {
    ledImpulse();
  }
}

bool longPressActive = false;
/**
 * Checks if the button is pressed and returns a value indicating the button state, with debouncing.
 *
 * @return true if the button is pressed, false otherwise.
 */
int LONG_PRESS_DELAY = 1000;
bool checkButtonPressed()
{
  if (PUSH_BUTTON.longClick())
  {
    int commandToExecute = menuItemIndex + 1;
    if (commandToExecute > 2 && commandToExecute < 6)
    {
      if (PUSH_BUTTON.getLongClickTime() > LONG_PRESS_DELAY) // press button for 1s to start delay function
      {
        longPressActive = true;
        return false;
      }
      else
      {
        longPressActive = false;
        return true;
      }
    }
    else
    {
      longPressActive = false;
      return true;
    }
  }
  else
  {
    return false;
  }
}

// variables for the encoder
const int ENCODER_COUNTER_CLOCKWISE = -1; // ↺ Counter clockwise rotation
const int ENCODER_NO_CHANGE = 0;
const int ENCODER_CLOCKWISE = 1; // ↻ Clockwise rotation

// Function to check if encoder has changed
int getEncoderPosition()
{
  int changeValue = ENCODER_NO_CHANGE; // Default to no change
  unsigned long currentEncoderTime = millis();

  // Read the current state of the encoder pins
  int currentEncoderA = digitalRead(ENCODER_A);
  int currentEncoderB = digitalRead(ENCODER_B);

  // Check if there has been any change in the state of the encoder pins
  if (lastEncoderA != currentEncoderA || lastEncoderB != currentEncoderB)
  {
    if (currentEncoderTime > (lastEncoderUpdateTime + TIME_ENCODER_DELAY))
    {
      // Check the transition of state to determine direction
      if (((currentEncoderA == 1 && currentEncoderB == 0) && (lastEncoderA == 0 && lastEncoderB == 0)) || ((currentEncoderA == 1 && currentEncoderB == 1) && (lastEncoderA == 1 && lastEncoderB == 0)) || ((currentEncoderA == 0 && currentEncoderB == 1) && (lastEncoderA == 1 && lastEncoderB == 1)) || ((currentEncoderA == 0 && currentEncoderB == 0) && (lastEncoderA == 0 && lastEncoderB == 1)))
      {
        changeValue = ENCODER_COUNTER_CLOCKWISE; // ↺ Counter clockwise rotation
      }
      else if (((currentEncoderA == 0 && currentEncoderB == 1) && (lastEncoderA == 0 && lastEncoderB == 0)) || ((currentEncoderA == 1 && currentEncoderB == 1) && (lastEncoderA == 0 && lastEncoderB == 1)) || ((currentEncoderA == 1 && currentEncoderB == 0) && (lastEncoderA == 1 && lastEncoderB == 1)) || ((currentEncoderA == 0 && currentEncoderB == 0) && (lastEncoderA == 1 && lastEncoderB == 0)))
      {
        changeValue = ENCODER_CLOCKWISE; // ↻ Clockwise rotation
      }

      // If a valid item change has occurred, update the time
      lastEncoderUpdateTime = millis();
    }
    lastEncoderA = currentEncoderA;
    lastEncoderB = currentEncoderB;
  }

  return changeValue; // Return the change in menu item (-1, 0, 1)
}

int changeMenuItem(int step)
{
  menuItemIndex += step;
  if (menuItemIndex >= MAX_MENU_ITEMS)
  {
    menuItemIndex = MAX_MENU_ITEMS - 1;
  }
  else if (menuItemIndex < 0)
  {
    menuItemIndex = 0;
  }
  return menuItemIndex; // Return the updated menu item index.
}

void inputMethods()
{
  // Check for encoder rotation
  int encoderChange = getEncoderPosition();
  if (encoderChange != ENCODER_NO_CHANGE)
  {
    changeMenuItem(encoderChange);
    displayMenuText();
  }

  // Check for button press
  int newButtonPushed = millis();
  if (newButtonPushed > lastButtonPushed + TIME_READY_DELAY && readyState != 1)
  {
    readyState = 1;
    displayReadyString();
  }
  else if (checkButtonPressed() == 1)
  {
    if (newButtonPushed > lastButtonPushed + TIME_READY_DELAY)
    {
      lastButtonPushed = newButtonPushed;
      readyState = 0;
      displayReadyString();
    }
  }
  // Execute the selected action
  if (readyState == 0) // Condition met when execute button clicked
  {
    menuSelectAction();
  }
  if (longPressActive && EXPERT_MODE)
  {
    delayTimeHelper();
  }
}

void menuSelectAction()
{
  int commandToExecute = menuItemIndex + 1; // Menu item index matches performPiezoAction case
  if (SERIAL_DEBUG)
  {
    Serial.print("Piezo command selected: ");
    Serial.println(menuOptionsShort[menuItemIndex]); // Print the string
  }
  delayAndDoStuff(500);                 // Delay so that the push button does not cause the ball to not be in the right position
  ballDetected = false;                 // if the push button caused the interrupt to be triggered, falsify it again
  performPiezoAction(commandToExecute); // menuItemIndex starts at 0, and case 1 is "Push"
  readyState = 1;                       // Reset the ready flag to prevent repeated command execution without button presses
  displayMenuText();                    // Update the display to show the Menu text
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
    screen.println(String(menuIndex + 1) + " - " + menuOptionsShort[menuIndex]);
  }

  ledImpulse();
}

// instead of displayMenuText the full function name is displayed for comprehension
void displayCurrentFunctionDescription()
{
  displayClearSection(STARTING_X, STARTING_Y_MODES, screen.width(), SCREEN_CLC_MODES_HEIGHT);
  screen.setCursor(STARTING_X, STARTING_Y_MODES);
  screen.setTextColor(COLOR_MIMED_YELLOW);
  screen.setTextSize(TEXT_SIZE);
  screen.println(String(menuItemIndex + 1) + " - " + menuOptionsLong[menuItemIndex]);
  ledImpulse();
}

// instead of displayMenuText the full function name is displayed for comprehension
void displayDelayHelperText()
{
  displayClearSection(STARTING_X, 200, screen.width(), 40);
  screen.setCursor(STARTING_X, 220);
  screen.setTextColor(COLOR_MIMED_YELLOW);
  screen.setTextSize(TEXT_SIZE);
  screen.println("Set delay time");
  ledImpulse();
}

void displayReadyString()
{
  ledImpulse();
  // Calculate the dimensions needed for the text based on text length and size
  int textLength = readyString[readyState].length();
  int textWidth = CHARACTER_WIDTH * textLength * 1.4; // Width of entire text string
  int textHeight = CHARACTER_HEIGHT;                  // Height of text string

  // Calculate starting X so the text is horizontally centered
  int textX = STARTING_X;
  int textY = STARTING_Y_READY - (textHeight / 2); // Adjust Y so the text is vertically centered

  // Clear the rectangle area where the ready string will be displayed
  displayClearSection(textX, textY, textWidth, textHeight);

  // Set the cursor to the calculated starting position for the ready string
  screen.setCursor(textX, textY);
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
  screen.println(readyString[readyState]);
  if (readyState == 0)
  {
    if (SERIAL_DEBUG)
    {
      displayTimeMicroseconds();
    }
  }
  else
  {
    displayVoltage();
  }
}

void displayHeader(String headerText)
{
  displayClearSection(STARTING_X, STARTING_Y_HEAD, screen.width(), CHARACTER_HEIGHT);
  screen.setCursor(STARTING_X, STARTING_Y_HEAD);
  screen.setTextColor(COLOR_BLACK);
  screen.setTextSize(TEXT_SIZE);
  screen.println(headerText);
}

void displayVoltage()
{
  currentVoltage = (int)readVoltagePiezo();
  int textWidth = 300; // Width of entire text string
  int textX = 245;
  int textY = STARTING_Y_READY - (CHARACTER_HEIGHT / 2);
  screen.setCursor(textX, STARTING_Y_READY - (CHARACTER_HEIGHT / 2));
  screen.setTextColor(COLOR_TUM_BLUE);
  screen.setTextSize(TEXT_SIZE);
  // Clear the rectangle area where the ready string will be displayed
  displayClearSection(140, textY, textWidth, CHARACTER_HEIGHT);
  screen.print(currentVoltage, 0);
  screen.print("V");
}

void displayTimeMicroseconds()
{
  int textX = 140;
  int textY = STARTING_Y_READY - (CHARACTER_HEIGHT / 2);
  displayClearSection(textX, textY, 200, CHARACTER_HEIGHT);
  int charHeight = CHARACTER_HEIGHT;
  screen.setTextSize(TEXT_SIZE - 1);
  screen.setCursor(textX, STARTING_Y_READY - (charHeight / 2));
  screen.setTextColor(COLOR_TUM_BLUE);
  int timeDelay = timerDelays[menuItemIndex];
  screen.print("delay: ");
  screen.print(timeDelay);
  screen.print("us");
}

// --- Piezo functions ------------------------------------------------------------------------------------------

/*in a bootstrap configuration, the low side MOSFET (Q2) is turned on to discharge
the piezo and charge the bootstrap capacitor, so that the piezo can be charged
again. The bootstrap capacitor looses its charge over time, so it needs to be
recharged before switching the high side MOSFET (Q1)*/

void chargeBootstrapCapacitorFast()
{
  delayMicroseconds(1); // → never comment this, this is shoot through delay time
  digitalWrite(GATE_DRIVE_LIN, HIGH);
  digitalWrite(GATE_DRIVE_LIN, LOW);
}
void chargeBootstrapCapacitor(uint32_t timeToChargeCapacitorMicroseconds)
{
  delayMicroseconds(1); // → never comment this, this is shoot through delay time
  digitalWrite(GATE_DRIVE_LIN, HIGH);
  // delayMicroseconds(timeToChargeCapacitorMicroseconds);
  delayMicroseconds(30);
  digitalWrite(GATE_DRIVE_LIN, LOW);
  delayMicroseconds(10);
}

/**
 * @brief Turns the piezo actuator instantly on, using the gatedriver.
 */
void chargePiezo()
{
  digitalWrite(GATE_DRIVE_HIN, HIGH); // SHOOT: Turn on higher side MOSFET (Q1)
  delayMicroseconds(100);             // todo: optimize time
  digitalWrite(GATE_DRIVE_HIN, LOW);  // Turn off higher side MOSFET (Q1)
  delay(25);                          // now: 50ms todo: how long does it take to charge the capacitor? + 10%
}

unsigned int DISCHARGE_PIEZO_TIME = 30; // this time needs to be adjusted according to the piezo
void dischargePiezo(uint32_t timeToDischargePiezoMicroseconds)
{
  delayMicroseconds(1); // → never comment this, this is shoot through delay time
  delayMicroseconds(timeToDischargePiezoMicroseconds);
  digitalWrite(GATE_DRIVE_LIN, HIGH);
  delayMicroseconds(DISCHARGE_PIEZO_TIME);
  digitalWrite(GATE_DRIVE_LIN, LOW);
}

volatile int NUMBER_OF_PIEZO_REPETITIONS = 1; // Number of times a selected function in the menu is repeated
int MAXIMUM_TIME_WAITING_FOR_BALL = 1500000;  // microseconds → 1.5 seconds
int piezoOnTime;

void waitForBallImpacts(int numberOfDetections)
{
  unsigned int startTime = micros();
  ballDetected = false;         // Reset the ballDetected flag
  int numberBouncesDetected = 0;
  while (numberBouncesDetected < numberOfDetections)
  {
    // Timeout check
    if (micros() - startTime > MAXIMUM_TIME_WAITING_FOR_BALL)
    {
      break; // Exit the loop
    }
    if (ballDetected)
    {
      numberBouncesDetected++;
      ballDetected = false; // Reset ballDetected for the next detection
    }
  }
}

int LED_ON_TIME = 150;
// Function to handle the LED impulse
void ledImpulse()
{
  if (digitalRead(LED_IMPULSE))
  {
    // Check if time passed since the LED was turned on
    if (millis() - ledOnTime > LED_ON_TIME)
    {
      digitalWrite(LED_IMPULSE, LOW); // Turn off the LED
    }
  }
}

unsigned int CHARGE_BOOTSTRAP_CAP_TIME_MICROSECONDS = 300;
void executePushAction()
{
  for (int repetitionIndex = 0; repetitionIndex < NUMBER_OF_PIEZO_REPETITIONS; repetitionIndex++)
  {

    chargeBootstrapCapacitor(CHARGE_BOOTSTRAP_CAP_TIME_MICROSECONDS);
    chargePiezo();
    delayAndDoStuff(100);
    dischargePiezo(0);
    ledImpulse();
    delayAndDoStuff(500);
    ledImpulse();
  }
}

void executePushAndCatchAction(int bounceCount)
{
  int timeToDischargePiezoMicroseconds = timerDelays[bounceCount + 1];
  for (int repetitionIndex = 0; repetitionIndex < NUMBER_OF_PIEZO_REPETITIONS; repetitionIndex++)
  {
    delayAndDoStuff(500);
    chargeBootstrapCapacitor(CHARGE_BOOTSTRAP_CAP_TIME_MICROSECONDS);
    chargePiezo(); // Shoot the ball away
    waitForBallImpacts(bounceCount);
    dischargePiezo(timeToDischargePiezoMicroseconds); // Catch the ball bounce-free
    delayAndDoStuff(800);
  }
}

int NUMBER_OF_BALL_REPETITIONS_RESONANCE = 8;
int chargeBootstrapCapacitorTime = 12;
void executeResonanceAction(int bounceCount)
{
  delayAndDoStuff(500);
  for (int repetitionIndex = 0; repetitionIndex < NUMBER_OF_BALL_REPETITIONS_RESONANCE; repetitionIndex++)
  {
    chargeBootstrapCapacitor(chargeBootstrapCapacitorTime);
    chargePiezo(); // Shoot the ball away
    waitForBallImpacts(bounceCount);
    dischargePiezo(0); // Shoot the ball again, this time higher then before
    chargeBootstrapCapacitorTime += 1; // Increase the charge time for the bootstrap capacitor so that the ball flies higher each following bounce
  }
}

void executeAllPiezoActions()
{
  // Perform all actions in sequence
  readyState = 0;       // Reset the ready flag to prevent repeated command execution without button presses
  displayReadyString(); // Update the display to show the change in state
  for (int currentMenuIndex = 2; currentMenuIndex < MAX_MENU_ITEMS + 1; currentMenuIndex++)
  {
    menuItemIndex = currentMenuIndex - 1;
    if (SERIAL_DEBUG)
    {
      displayTimeMicroseconds();
    }
    performPiezoAction(currentMenuIndex); // menuItemIndex starts at 0, and case 1 is "Push"
    delayAndDoStuff(800);
  }
  menuItemIndex = 0;
  readyState = 1;
  displayReadyString(); // Update the display to show the "ready" state
  displayMenuText();    // Update the display to show the first menu item
}

void performPiezoAction(int piezoSelect)
{
  displayCurrentFunctionDescription(); // Update the display to show the long description of the selected function
  ledImpulse();
  // Log the selected command if serial debug is enabled
  delayAndDoStuff(700); // Delay to show description
  // Execute action based on the selected command
  switch (piezoSelect)
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

/**
 * @brief Checks the voltage of the piezo sensor.
 *
 * This function reads the analog value from the V_MEASURE pin and converts it to voltage.
 * The voltage is calculated using the formula: current_voltage = analogRead(V_MEASURE) / 4095 * V_DD * 46.454.
 *
 * @return The voltage of the piezo sensor.
 */

float readVoltagePiezo()
{
  int current_voltage = analogRead(V_MEASURE);
  delayAndDoStuff(10);
  current_voltage *= 0.0374; // Voltage divider equals to: analogRead(V_MEASURE) * (2200 + 100000) / 2200
  return current_voltage;
}

/**
 * Sets the voltage for the piezo.
 *
 * @param voltage The desired voltage for the piezo.
 */
void setPiezoActuatorVoltage(float voltage)
{
  if (voltage > 100) // max voltage
  {
    voltage = 100;
  }
  else if (voltage < 50) // min voltage
  {
    voltage = 50;
  }
  float slope = 21.25; // 85/4 = 21.25; 85 V output per 4 V input
  float V_send_i2c = (voltage - 50) / slope;
  V_send_i2c = V_send_i2c / .71304; // Voltage divider equals to 82e3 / (82e3 + 33e3)
  // Serial.println(V_send_i2c);
  // Set the voltage to the piezo V_P
  dac.writedac(V_send_i2c, VDD);
  delay(20); // Small delay to allow DAC to settle
}

unsigned long DELAY_HELPER_INTERVAL_TIMER = 1000; // Desired interval in which a new value can be set
// Function to help to find the optimal delay time for executePushAndCatchAction()
void delayTimeHelper()
{
  readyState = 1;        // Reset the ready flag
  displayReadyString();  // Update the display to show state change
  int encoderChange = 0; // Variable to store the encoder change value
  String delayHelpHeadline = "Turn to set delay";
  String delayIncreasingHeadline = "Delay Up";
  String delayDecreasingHeadline = "Delay Down";
  displayHeader(delayHelpHeadline);
  int commandToExecute = menuItemIndex + 1; // Menu item index matches performPiezoAction case
  int lastEncoderPosition = 0;
  int currentEncoderPosition; // Function to get current encoder position
  delayAndDoStuff(500);       // Delay to prevent accidental rotation of encoder
  while (longPressActive)
  {
    readyState = 0;
    displayReadyString(); // Update the display to show the change in state
    performPiezoAction(commandToExecute);
    readyState = 1;       // Reset the ready flag to prevent repeated command execution without button presses
    displayReadyString(); // Update the display to show the change in state
    displayTimeMicroseconds();
    // Use a flag to determine if the interval has finished
    bool intervalHasFinished = false;
    unsigned long previousTimer = millis(); // Start time for the interval
    int lastTimerDelay;
    int currentTimerDelay;
    lastTimerDelay = timerDelays[menuItemIndex];
    // Loop until the encoder is changed or the interval has elapsed, or the button is pressed
    while (!intervalHasFinished)
    {

      unsigned long currentTimer = millis();
      // Check for encoder changes
      currentEncoderPosition = getEncoderPosition();
      if (currentEncoderPosition != ENCODER_NO_CHANGE && timerDelays[menuItemIndex] >= 0)
      {
        timerDelays[menuItemIndex] += currentEncoderPosition; // Adjust the delay time
        displayTimeMicroseconds();                            // Update the display to show the new delay time
      }
      if (lastEncoderPosition != currentEncoderPosition && currentEncoderPosition != ENCODER_NO_CHANGE) // If change happened
      {
        lastEncoderPosition = currentEncoderPosition; // Update the last accounted position
      }
      else if (currentEncoderPosition == ENCODER_NO_CHANGE)
      {
        currentEncoderPosition = lastEncoderPosition; // If no change, set the current position to the last position
      }

      // Exit the loop
      if (PUSH_BUTTON.click() == 1)
      {
        longPressActive = false;
        break;
      }
      if (currentTimer - previousTimer > DELAY_HELPER_INTERVAL_TIMER)
      {
        intervalHasFinished = true;
      }
    } // End of inner while loop for adjusting delay time
      // lastTimerDelay != timerDelays[menuItemIndex]
    if (lastTimerDelay != timerDelays[menuItemIndex] + 1 || lastTimerDelay != timerDelays[menuItemIndex] - 1)
    {
      if (currentEncoderPosition > 0) // Increase the value
      {
        displayHeader(delayIncreasingHeadline);
        timerDelays[menuItemIndex] += 1;
      }
      else // Decrease the value
      {
        displayHeader(delayDecreasingHeadline);
        if (timerDelays[menuItemIndex] >= 1)
        {
          timerDelays[menuItemIndex] -= 1;
        }
        else
        {
          timerDelays[menuItemIndex] = 0;
          longPressActive = false;
          break;
        }
      }
    }
    else
    {
      displayHeader(delayHelpHeadline); // Update the display to show the help text
    }
  } // End of the main while loop that continually allows for delay time adjustments
  readyState = 1;          // "ready" state
  displayReadyString();    // Update the display to show "ready" state
  displayMenuText();       // Update the display for the menu
  displayHeader(headline); // Print the header at the top
}

void setupPins()
{
  PUSH_BUTTON.setMinLongClickTime(100); // Set the minimum time for a long click in milliseconds
  pinMode(LED_BUILTIN, OUTPUT);         // Initialize LED's as output
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(LED_IMPULSE, OUTPUT);
  pinMode(LED_POWER, OUTPUT);
  digitalWrite(LED_POWER, HIGH);
  pinMode(HIT_IMPULSE, INPUT_PULLUP);
  enablePiezoInterrupt();    // Enable interrupts for the ball detection pin
  pinMode(V_MEASURE, INPUT); // Initialize voltage measurement pin
  analogReadResolution(12);  // 12 BITS → 4096 possible values
}

void setupGatedriver()
{
  pinMode(GATE_DRIVE_HIN, OUTPUT);
  pinMode(GATE_DRIVE_LIN, OUTPUT);
}

void setupi2C_DAC()
{
  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
  dac.begin(); // Initialize the DAC
  setPiezoActuatorVoltage(100);
}

void setupEncoder()
{
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  currentStateClock = digitalRead(ENCODER_A);
  lastEncoderA = currentStateClock;
  currentENCODER_B = digitalRead(ENCODER_B);
  lastEncoderB = currentENCODER_B;
}

void setupScreen()
{
  while (1) // Wait until the external 12V supply is connected, so as to initialize the display. The display is not powered from the microcontroller in this version.
  {
    if (readVoltagePiezo() > 10)
    {
      break;
    }
  }
  digitalWrite(TFT_RST, LOW); // Reset the display
  screen.begin();
  screen.setRotation(1);
  screen.fillScreen(COLOR_FILL_RECT);
  displayMenuText();       // show mode text
  displayReadyString();    // show ready string
  displayHeader(headline); // Print the header at the top
}

void setupSerial()
{
  Serial.print(F("CPU Frequency = "));
  Serial.print(F_CPU / 1000000);
  Serial.println(F(" MHz"));
  // inputString.reserve(200); // Reserve 200 bytes for the inputString to reduce dynamic allocation
  int startTimerSerial = millis();
  Serial.print(startTimerSerial);
  Serial.begin(1000000);
  delay(300);
}

void setup()
{
  setupEncoder();
  setupGatedriver();
  // setupi2C_DAC(); // doesn't work in PCB version 1.0
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

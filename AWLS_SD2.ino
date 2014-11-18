// Platform: Arduino Mega 2560
// Authors: Garrett Doorenbos and Chris Childers
// Date: Fall 2014

#include <TimerOne.h>
#include <LiquidCrystal.h>
#include <inttypes.h>

// constants
const float volt0 = 0.1;
const float volt45 = 2.1;
const float volt135 = 3.25;
const float volt225 = 3.65;
const float volt315 = 3.85;
const float volt405 = 3.95;

// General IO pins
const int navBtn = 7;
const int selBtn = 6;
const int buzzer = 44;

// Motor Control
const int MCout = 41;
const int MCin = 39;
const int MCFullSpeed = 37;
const int MCVarSpeed = 35;
const int MCVS1 = 33;
const int MCVS2 = 31;
const int MCVS3 = 29;
const int MCVS4 = 27;
const int MCVS5 = 25;
const int MCVS6 = 23;

// Pressure Sensor
const int pressureSensor = 0;
int weight = 0;

// foot pedal
const int footPedal = 51;
const int footPedalConn = 53;

// Wheel Encoder variables
const int WEA = 2;  // int0
const int WEB = 3;  // int1
volatile int WECounts = 0;
volatile int WEError = 0;
volatile byte last_aVal, last_bVal;
int helpLevel = 0;
int reps = 0;

// Timer vals
volatile int lastCount = 0;
volatile boolean upwards = false;
volatile boolean downwards = false;
volatile int stall = 0;
volatile int countDiff = 0;
const int speedThreshold = -25;
volatile int downwardsCnt = 0;
volatile int LockCnt = 0;
volatile boolean lockoutFlag = false;
volatile int minutes = 0;

// initialize LiquidCrystal library with the pins to be used
// Arduino Pin | LCD Pin
// ------------+-----------
//          13 | rs
//          12 | enable
//          11 | d4
//          10 | d5
//           9 | d6
//           8 | d7
LiquidCrystal lcd(13,12,11,10,9,8);

// Define Main State Machine
const byte waitForInput = 1;  // Wait on user input
const byte navBtnPressed = 2;   // Navigation button pressed
const byte selBtnPressed = 3;   // Select button pressed
boolean returnToMainMenu = false;

// defines for lift subroutine
const byte barInRack = 1;
const byte downwardState = 2;
const byte upwardState = 3;
const byte exitLift = 4;

// defines for calibrate subroutine --TODO: change this before final!
int topVal = 75;
int bottomVal = -58;
int topThreshold = 0.75*(topVal-bottomVal) + bottomVal;
int bottomThreshold = 0.25*(topVal-bottomVal) + bottomVal;
const int rackPosition = 0;

// flags
boolean calibrateFlag = true;  // TODO: change this before final!
boolean liftingFlag = false;

void waitForButtonPress()
{
  while((digitalRead(navBtn) == HIGH) && (digitalRead(selBtn) == HIGH))
  {
    delay(50);  // debounce
  }
}

void splashScreen()
{
  lcd.setCursor(0,0);
  lcd.print("     Automatic      ");
  lcd.setCursor(0,1);
  lcd.print("   Weightlifting    ");
  lcd.setCursor(0,2);
  lcd.print("      Spotter       ");
  lcd.setCursor(0,3);
  lcd.print("                    ");
  delay(3000);
}

void setupMenu()
{
  lcd.setCursor(0,0);
  lcd.print("Select an Option:   ");
  lcd.setCursor(0,1);
  lcd.print(">Calibrate          ");
  lcd.setCursor(0,2);
  lcd.print(" Lift               ");
  lcd.setCursor(0,3);
  lcd.print(" Statistics         ");
}

void newMenu()
{
  lcd.setCursor(0,0);
  lcd.print(">Calibrate          ");
  lcd.setCursor(0,1);
  lcd.print(" Lift               ");
  lcd.setCursor(0,2);
  lcd.print(" Statistics         ");
  lcd.setCursor(0,3);
  lcd.print(" Spool-Out Line     ");
}

void liftingScreen()
{
  lcd.setCursor(0,0);
  lcd.print("Lifting State Chosen");
  lcd.setCursor(0,1);
  lcd.print("Ready to Spot!      ");
  lcd.setCursor(0,2);
  lcd.print("                    ");
  lcd.setCursor(0,3);
  lcd.print("                    ");  
}

void clearScreen()
{
  lcd.setCursor(0,0);
  lcd.print("                    "); 
  lcd.setCursor(0,1);
  lcd.print("                    "); 
  lcd.setCursor(0,2);
  lcd.print("                    ");
  lcd.setCursor(0,3);
  lcd.print("                    "); 
}

void comingSoon()
{
  lcd.setCursor(0,0);
  lcd.print("                    ");
  lcd.setCursor(0,1);
  lcd.print("    Coming Soon!    ");
  lcd.setCursor(0,2);
  lcd.print("                    ");
  lcd.setCursor(0,3);
  lcd.print("                    ");
}

void configPins()
{
  // foot pedal
  pinMode(footPedal, INPUT_PULLUP);
  pinMode(footPedalConn, INPUT_PULLUP);
  
  // buzzer
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);
  
  // Motor Control
  pinMode(MCout, OUTPUT);
  digitalWrite(MCout, LOW);
  pinMode(MCin, OUTPUT);
  digitalWrite(MCin, LOW);
  pinMode(MCFullSpeed, OUTPUT);
  digitalWrite(MCFullSpeed, LOW);
  pinMode(MCVarSpeed, OUTPUT);
  digitalWrite(MCVarSpeed, LOW);
  pinMode(MCVS1, OUTPUT);
  digitalWrite(MCVS1, LOW);
  pinMode(MCVS2, OUTPUT);
  digitalWrite(MCVS2, LOW);
  pinMode(MCVS3, OUTPUT);
  digitalWrite(MCVS3, LOW);
  pinMode(MCVS4, OUTPUT);
  digitalWrite(MCVS4, LOW);
  pinMode(MCVS5, OUTPUT);
  digitalWrite(MCVS5, LOW);
  pinMode(MCVS6, OUTPUT);
  digitalWrite(MCVS6, LOW);
    
  // setup buttons
  pinMode(navBtn, INPUT_PULLUP);
  pinMode(selBtn, INPUT_PULLUP);
  
  // Wheel Encoder 
  noInterrupts();
  attachInterrupt(0, WEHandler, CHANGE);
  attachInterrupt(1, WEHandler, CHANGE);
  last_aVal = digitalRead(WEA);
  last_bVal = digitalRead(WEB);
  interrupts();
}

void WEHandler()
{
  // Read current values of the outputs
  byte aVal = digitalRead(WEA);
  byte bVal = digitalRead(WEB);
  
  // Compare to last values (Quadrature Algorithm found in PololuWheelEncoders.cpp)  
  // Update counter appropriately
  if (aVal ^ last_bVal) WECounts++;
  if (bVal ^ last_aVal) WECounts--;
  
  // if both outputs change at same time, that's an error
  if (aVal != last_aVal && bVal != last_bVal)
  {
    WEError ++;
  }
    
  // Update "last" values
  last_aVal = aVal;
  last_bVal = bVal;
}

void timerISR()
{
  countDiff = WECounts - lastCount;
  
  if (countDiff > 0)
  {
    upwards = true;
    downwards = false;
    downwardsCnt = 0;
//    stall = 0;
  }
  else if (countDiff < 0)
  {
    upwards = false;
    downwards = true;
    downwardsCnt++;
//    stall = 0;
  }
  else
  {
    upwards = downwards = false;
//    stall++;
  }
  
  if (countDiff >= -1 && countDiff <= 1) stall++;
  else stall = 0;
  
  lastCount = WECounts;
}

void configTimer()
{
  Timer1.initialize(100000);  // initialize timer for 100 ms (don't do math in function call)
  Timer1.attachInterrupt(timerISR);  // attach interrupt to function
}

void Lockout()
{
  lockoutFlag = true;
  LockCnt = 0;
  minutes = 0;
  Timer1.initialize(1000000);  // 1s
  Timer1.attachInterrupt(LockoutISR);
}

void LockoutISR()
{
  LockCnt++;
  if (minutes < 30)
  {
    if (LockCnt >= 60)
    {
      LockCnt = 0;
      minutes++;
    }
  }
  else
  {
    // Lockout timer expired - reset everything
    lockoutFlag = false;
    configTimer();
  }
}

void lockoutScreen()
{
  // calculate remaining time
  int tmp = minutes;
  int remaining = 30 - tmp;
  
  // notify user of lockout
  lcd.setCursor(0,0);
  lcd.print("User locked out     ");
  lcd.setCursor(0,1);
  lcd.print("30min for motor     ");
  lcd.setCursor(0,2);
  lcd.print("cool-down.          ");
  lcd.setCursor(0,3);
  lcd.print("Remaining: ");
  lcd.print(remaining);
  lcd.print("       ");
  
  waitForButtonPress();
}

void readWeight()
{
  int i, cnt, sum = 0;
  for (i=0; i<20; i++)
  {
    sum += analogRead(pressureSensor);
  }
  float avg = (float)sum / 20.0;
  float analogVal = (float) avg / 1023.0 * 5.0;
  
  if (analogVal < ((volt45-volt0)/2 + volt0)) 
  {
    weight = 0;
  }
  else if (analogVal < ((volt135-volt45)/2 + volt45)) 
  {
    weight = 45;
  }
  else if (analogVal < ((volt225-volt135)/2 + volt135))
  {
    weight = 135;
  }
  else if (analogVal < ((volt315-volt225)/2 + volt225))
  {
    weight = 225;
  }
  else if (analogVal < ((volt405-volt315)/2 + volt315))
  {
    weight = 315;
  }
  else
  {
    weight = 405;
  }
}

void MCSpoolOut()
{
  digitalWrite(MCout, HIGH);
  digitalWrite(MCin, LOW);
}

void MCReelIn()
{
  digitalWrite(MCout, LOW);
  digitalWrite(MCin, HIGH);
}

void MCShutOff()
{
  digitalWrite(MCout, LOW);
  digitalWrite(MCin, LOW);
  digitalWrite(MCFullSpeed, LOW);
  digitalWrite(MCVarSpeed, LOW);
  digitalWrite(MCVS1, LOW);
  digitalWrite(MCVS2, LOW);
  digitalWrite(MCVS3, LOW);
  digitalWrite(MCVS4, LOW);
  digitalWrite(MCVS5, LOW);
  digitalWrite(MCVS6, LOW);
}

void emergencyLift()
{
  // Reel in Rope until bar reaches rack
  while(WECounts < topThreshold)
  {
    MCReelIn();
    digitalWrite(MCFullSpeed, HIGH);
  }
  
  // Bar at rack level, shutoff motor
  MCShutOff();
  
  // Exit lifting loop
  liftingFlag = false;
  
  // Indicate help level for statistics screen
  helpLevel = 3;
  Lockout();
}

void freeFallLift()
{
  // Reel in rope at full speed for x amount of milliseconds
  MCReelIn();
  digitalWrite(MCFullSpeed, HIGH);
  delay(3000);
  
  // Shutoff motor
  MCShutOff();
  
  // Exit lifting loop
  liftingFlag = false;
  
  // Indicate help level for statistics screen
  helpLevel = 3;
  Lockout();
}

void assist(int level)
{
  if (WECounts < topThreshold)
  {
    if ((level > 0) && (level <= 20))
    {
      MCReelIn();
      digitalWrite(MCVarSpeed, HIGH);
      digitalWrite(MCVS5, HIGH);
      digitalWrite(MCVS6, LOW);
      helpLevel = 1;
    }
    else if (level <= 40)
    {
      MCReelIn();
      digitalWrite(MCVarSpeed, HIGH);
      digitalWrite(MCVS5, LOW);
      digitalWrite(MCVS6, HIGH);
      helpLevel = 2;
    }
    else
    {
      digitalWrite(MCVarSpeed, LOW);
      digitalWrite(MCVS5, LOW);
      digitalWrite(MCVS6, LOW);
      emergencyLift();
    }
  }
  else 
  {
    MCShutOff();
    liftingFlag = false;  // exit lifting loop
    Lockout();
  }
}

void calibrate()
{
  readWeight();
  if (weight == 45)
  {
    // print to LCD display
    lcd.setCursor(0,0);
    lcd.print("Calibration Started ");
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(0,3);
    lcd.print("                    ");
    
    // Wait for bar to come off rack
    while(analogRead(pressureSensor) > 204)
    {
      delay(50); // debounce
    }
    delay(100);  // give user a chance to lift bar up to top of lift. Without this, the arduino might stall out before the weight gets out of the rack very well
    
    // Wait for stall at the top of lift
    stall = 0;
    while(stall < 5);
    topVal = WECounts;
    
    // print to screen
    lcd.setCursor(0,1);
    lcd.print("TOL Recorded        ");
    
    // sound buzzer to confirm top of reach recorded
    digitalWrite(buzzer, HIGH);
    delay(500);
    digitalWrite(buzzer, LOW);
    
    // let user spool rope out
    while (digitalRead(footPedal) == HIGH)  // wait for user to press foot pedal
    {
      delay(50);  // debounce
    }
    delay(1000); // wait a bit
    if (digitalRead(footPedal) == LOW)  // if user still has pedal pressed, spool out rope
    {
      MCSpoolOut();
      digitalWrite(MCVarSpeed, HIGH);
      digitalWrite(MCVS3, HIGH);
    }
    while (digitalRead(footPedal) == LOW)  // Quit spooling out rope when user releases foot pedal
    {
      delay(50);  // debounce
    }
    bottomVal = WECounts;
    MCShutOff();
      
    // print to screen
    lcd.setCursor(0,2);
    lcd.print("BOL Recorded");
    
    digitalWrite(buzzer, HIGH);
    delay(500);
    digitalWrite(buzzer, LOW);
    
    // Error Checking
    int errorCheck = topVal - bottomVal;
    if (errorCheck <= 0)
    {
      lcd.setCursor(0,0);
      lcd.print("Error:              ");
      lcd.setCursor(0,1);
      lcd.print("Calibration Failed. ");
      lcd.setCursor(0,2);
      lcd.print("Please try again.   ");
      lcd.setCursor(0,3);
      lcd.print("                    ");
      calibrateFlag = false;
      
      // wait for bar to be placed in rack
      while(analogRead(pressureSensor) < 204)
      {
        delay(50);  // de bounce
      }
      
      // wait for user to exit
      while(digitalRead(navBtn) == HIGH && digitalRead(selBtn) == HIGH)
      {
        delay(50);  // delay
      }
      delay(100);
    
      return;  // exit calibrate function
    }
    
    // print values to screen
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Top of Reach: ");
    lcd.print(topVal);
    lcd.setCursor(0,1);
    lcd.print("Chest Val: ");
    lcd.print(bottomVal);
    lcd.setCursor(0,2);
    lcd.print("Correct?");
    
    // wait for bar to be placed in rack
    while(analogRead(pressureSensor) < 204)
    {
      delay(50);
    }
    
    while(true)
    {
      if (digitalRead(selBtn) == LOW)
      {
        topThreshold = 0.75*(topVal-bottomVal) + bottomVal;
        bottomThreshold = 0.25*(topVal-bottomVal) + bottomVal;
        calibrateFlag = true;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Calibration Accepted");
        
        // wait for user to exit
        while (digitalRead(navBtn) && digitalRead(selBtn))
        {
          delay(50);
        }
        delay(100);
        break;
      }
      if (digitalRead(navBtn) == LOW)
      {
        calibrateFlag = false;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Calibration Rejected");
        
        // wait for user to exit
        while (digitalRead(navBtn) && digitalRead(selBtn))
        {
          delay(50);
        }
        delay(100);
        break;
      }
      delay(50);  // debounce
    }
  }
  else
  {
    // Print error message
    lcd.setCursor(0,0);
    lcd.print("Error:              ");
    lcd.setCursor(0,1);
    lcd.print("Calibration Failed. ");
    lcd.setCursor(0,2);
    lcd.print("Unload bar          "); 
    lcd.setCursor(0,3);
    lcd.print("Weight: ");
    lcd.print(weight);
    lcd.print("     ");
    digitalWrite(buzzer, HIGH);
    delay(500);
    digitalWrite(buzzer, LOW);
    
    // wait for user to exit
    while(digitalRead(navBtn) == HIGH && digitalRead(selBtn) == HIGH)
    {
      delay(50);
    }
    delay(100);
  }
}

void lift()
{
  byte liftState = barInRack;
  boolean startFlag = false;
  int tmp;
  
  // Only proceed if the spotter is calibrated
  if (calibrateFlag)
  {
    // Wait for user to press foot pedal
    // debounce foot pedal
    while (true)
    {
      if (digitalRead(footPedal) == LOW)
      {
        delay(500);
        if (digitalRead(footPedal) == LOW) break;
      }
    }
    liftingFlag = true;
    
    // enter lifting loop
    while (liftingFlag)  
    {
      // Emergency lift if foot pedal is released
      if ((digitalRead(footPedal) == HIGH) && (analogRead(pressureSensor) < 204))  // 1 volt
      {
        // debounce foot pedal
        delay(500);
        if (digitalRead(footPedal) == HIGH)
        {
//          Serial.println("Foot pedal released while lifting");
          lcd.setCursor(0,3);
          lcd.print("FPRWL          ");
          emergencyLift();
          liftingFlag = false;
          return;
        }
      }
      
      // Exit lifting routine when bar is placed back in rack
      if (startFlag && (analogRead(pressureSensor) > 204))  // 1 volt
      {
        // debounce pressure sensor
        delay(500);
        if (analogRead(pressureSensor) > 204)  // 1 volt
        {
//          Serial.println("Bar placed back in rack");
          lcd.setCursor(0,3);
          lcd.print("BPBIR          ");
          liftingFlag = false;
          liftState = exitLift;
          return;
        }
      }
      
      // Main lifting routine
      switch (liftState)
      {
        // Wait for bar to be lifted out of rack
        case barInRack:
//          Serial.println("State: barInRack");
          lcd.setCursor(0,2);
          lcd.print("SBIR");
          if (analogRead(pressureSensor) < 204)  // 1 volt
          {
            startFlag = true;
            liftState = upwardState;
            delay(500);
            stall = 0;  // ignore first stall count
          }
          break;
          
        // Bar is moving downwards
        case downwardState:
//          Serial.println("State: downwardState");
          lcd.setCursor(0,2);
          lcd.print("SDWS ");
          lcd.print(WECounts);
          lcd.print("     ");
          // Go to upwards state if upwards motion detected
          tmp = WECounts;
          if (upwards && tmp < bottomThreshold)
          {
//            WECounts = 0;  // reset counter to maintain accuracy
            stall = 0;
            reps++;
            liftState = upwardState;
            break;
          }
          
          // Guard against free fall
          if (countDiff < speedThreshold)  // Threshold and countdiff are both negative
          {
//            Serial.println("Freefall detected");
            lcd.setCursor(0,3);
            lcd.print("FFD          ");
//            emergencyLift();
            emergencyLift();
            liftingFlag = false;
            return;
          }
          
          // Redundant??
//          if (WECounts <= bottomVal)
//          {
//            liftState = upwardState;
//            break;
//          }
          
          break;
          
        case upwardState:
//          Serial.println("State: upward state");
          lcd.setCursor(0,2);
          lcd.print("SUWS ");
//          if (upwards) lcd.print("U ");
//          else if (downwards) lcd.print("D ");
//          else lcd.print("N ");
          lcd.print(WECounts);
//          lcd.print(stall);
          lcd.print("     ");
          
          // Clear stalls if they are irrelevant
          // redundant???
//          if (WECounts <= 15 || WECounts >= 0.85*topVal) stall = 0;
          
          // help user if stall detected
          if ((stall > 0) && (WECounts < topThreshold))
          {
            //Serial.println("Stall detected");
            lcd.setCursor(0,3);
            lcd.print("STALL          ");
            assist(stall);
            break;
          }
          
//          // emergency lift if downwards motion detected before user gets to topVal
//          // commented out due to bugginess
//          if (downwardsCnt >= 2 && WECounts < (0.85*topVal))
//          {
//            //Serial.println("downward motion detected before top of lift");
//            lcd.setCursor(0,3);
//            lcd.print("DMD            ");
//            emergencyLift();
//            liftingFlag = false;
//            return;
//          }
          
          // If downward motion is detected at top of lift, go to downward state
          if (downwards && WECounts >= topThreshold)
          {
            liftState = downwardState;
            stall = 0;  
          }
          break;
          
        default:
          // exit lifting loop
//          Serial.println("Exit lift");
          lcd.setCursor(0,3);
          lcd.print("EXIT_SUCCESS     ");
          liftingFlag = false;
          MCShutOff();
          return;
      }
      
      delay(100);
    }
  }
  else
  {
    lcd.setCursor(0,0);
    lcd.print("       Error:       ");
    lcd.setCursor(0,1);
    lcd.print("  Please Calibrate  ");
    lcd.setCursor(0,2);
    lcd.print("        First       ");
    lcd.setCursor(0,3);
    lcd.print("                    ");
    returnToMainMenu = true;
  }
}

void statistics()
{
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Statistics:         ");
  lcd.setCursor(0,1);
  lcd.print("Weight: ");
  lcd.print(weight);
  lcd.setCursor(0,2);
  lcd.print("Help: ");
  lcd.print(helpLevel);
  lcd.setCursor(0,3);
  lcd.print("Reps: ");
  lcd.print(reps);
  
  while((digitalRead(navBtn) == HIGH) && (digitalRead(selBtn) == HIGH));
  delay(100);
}

bool barIsInRack()
{
  if(analogRead(pressureSensor) > 204) // 1 volt
  {
    return true;
  }
  return false;
}

void spoolOutLine()
{
  // only proceed if bar is not in rack
  if (!barIsInRack())  
  {
    // only proceed if bar is higher than bottom value
    if (WECounts > bottomVal)
    {
      // spool out until bar reaches bottom value
      digitalWrite(MCVarSpeed, HIGH);
      digitalWrite(MCVS4, HIGH);
      while(WECounts > bottomVal);
      MCShutOff();
      
      // Print confirmation screen
      clearScreen();
      lcd.setCursor(0,0);
      lcd.print("Done spooling out");
    }
    else
    {
      clearScreen();
      lcd.setCursor(0,0);
      lcd.print("Already enough slack");
      lcd.print("      in line       ");
    }
  }
  else
  {
    // bar needs to be removed from rack
    WECounts = rackPosition;  // bar is in rack, so make sure counts is right
    clearScreen();
    lcd.setCursor(0,0);
    lcd.print("Remove bar from rack");
    lcd.print("and try again");
  }
  
  waitForButtonPress();
}

void setup()
{
  configPins();
  configTimer();
    
  // set up the LCD's number of columns and rows:
  lcd.begin(20,4);
  
  // Print a message to the LCD.
  splashScreen();
  newMenu();
}

void loop()
{
  static uint8_t menuRow = 1;
  static byte myState = waitForInput;
  
  switch (myState)
  {
    case waitForInput:
      if (digitalRead(navBtn) == LOW) myState = navBtnPressed;
      else if (digitalRead(selBtn) == LOW) myState = selBtnPressed;
      break;
      
    case navBtnPressed:
      lcd.setCursor(0,menuRow);
      lcd.print(" ");
      menuRow++;
      if (menuRow > 4) menuRow = 1;
      lcd.setCursor(0,menuRow);
      lcd.print(">");
      myState = waitForInput;
      break;
      
    case selBtnPressed:
      if (menuRow == 1)
      {
        // make sure user isn't locked out
        if (lockoutFlag)
        {
          lockoutScreen();
        }
        else
        {
          WECounts = 0;
          calibrate();
        }
      }
      else if (menuRow == 2)
      {
        // make sure user isn't locked out
        if (lockoutFlag)
        {
          lockoutScreen();
        }
        else
        {
          // Setup and enter lifting routine
          WECounts = 0;
          reps = 0;
          readWeight();
          liftingScreen();
          lift();
          waitForButtonPress();
        }
      }
      else if (menuRow == 3)
      {
        statistics();
      }
      else if (menuRow == 4)
      {
        spoolOutLine();
      }
      
      menuRow = 1;
      setupMenu();
      myState = waitForInput;
      

      break;
      
    default:
      // do nothing
      break;
  }
  
//  noInterrupts();
//  int tmpCnt = WECounts;
//  int tmpErr = WEError;
//  interrupts();
  
//  Serial.print("WE Counts: ");
//  Serial.print(tmpCnt);
//  Serial.println();
  
//  Serial.print("WE Error: ");
//  Serial.print(tmpErr);
//  Serial.println();
//  Serial.println();  
  
  delay(100);
}

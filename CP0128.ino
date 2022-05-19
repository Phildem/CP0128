
/*
CP0128  Copyright Cyrob 2022
Cyrob Smart Servo tester by Philippe Demerliac
See my presentation video in French : https://youtu.be/Up00w3Z8Sso
=====================================================================================
==========================   OPEN SOURCE LICENCE   ==================================
=====================================================================================
Copyright 2021 Philippe Demerliac Cyrob.org
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
................................................................................................................
Release history
................................................................................................................
Version Date        Author    Comment
1.0     14/05/2022 Phildem   First blood

*/


//____________________________________________________________________________________________________________
// Includes __________________________________________________________________________________

#include <Servo.h> 

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>


// CP0128 is base on C.A.T.H task supervisor see https://github.com/Phildem/Cath
//____________________________________________________________________________________________________________
// Start of Cath definition__________________________________________________________________________________
 
#define kMaxCathTask    6         // Max Number of task instances. MUST BE >= to tasks instancied

#define __CathOpt_SmallCounter__  // Comment this line to allow 32 bit delay. If not, max period is 65536 ms

#ifdef __CathOpt_SmallCounter__
typedef uint16_t CathCnt;
#else
typedef uint32_t CathCnt;
#endif


class Cath{

  public:

// Derived class MUST implement these 2 methods
  virtual void          SetUp() =0;                 // Called at setup
  virtual void          Loop()  =0;                 // Called periodically

  CathCnt               m_CurCounter;               // Curent number of ms before next Loop call
  CathCnt               m_LoopDelay;                // Default period of Loop call (in ms)

  static uint8_t        S_NbTask;                   // Actual number of task instances
  static Cath*          S_CathTasks[kMaxCathTask];  // Array of task object pointers
  static uint8_t        S_LastMilli;                // Used to call every ms (a byte is enought to detect change)

  //..............................................................
  // Must be called in task constructors to register in the task list
  // WARNING : think to set kMaxCathTask as needed
  // Task   : Pointer to the derivated task to register
  // Period : Loop call Period (in ms). WARNING do not pass 0!
  // Offset : Delay of the first call in ms (1 def). WARNING do not pass 0!
  static void S_Register(Cath* Task,CathCnt Period,CathCnt Offset=1){
    Task->m_LoopDelay=Period;
    Task->m_CurCounter= Offset;
    Cath::S_CathTasks[Cath::S_NbTask++]=Task;
  }

  //..............................................................
  // Must be called once in Arduino setup to call all the task setups
  static void S_SetUp(){
    for(int T=0;T<S_NbTask;T++)
      Cath::S_CathTasks[T]->SetUp();
  }

   //..............................................................
  // Must be called once in Arduino Loop to call all the task loop if needed
  static void S_Loop(){
    uint8_t CurMilli=millis();
    if (CurMilli!=S_LastMilli) {
      S_LastMilli=CurMilli;
      for(int T=0;T<S_NbTask;T++) 
        if ( Cath::S_CathTasks[T]->m_CurCounter--==0) {
          Cath::S_CathTasks[T]->m_CurCounter=Cath::S_CathTasks[T]->m_LoopDelay;
          Cath::S_CathTasks[T]->Loop();
        }
     }
  }

};

//Cath static variables definitions 
//(Note set to 0 for code clarity but done by default anyway because they are static)
uint8_t       Cath::S_NbTask=0;
Cath*         Cath::S_CathTasks[kMaxCathTask];
uint8_t       Cath::S_LastMilli=0;

// End of Cath definition ___________________________________________________________________________________
//___________________________________________________________________________________________________________




//****************************************************************************************************************
// I/O Abstraction
#define kOutPinServo1     2       // Servo PWM output pins
#define kOutPinServo2     3
#define kOutPinServo3     4
#define kOutPinServo4     5

#define kOutPinFine1      6       // Fine leds output pins lite HIGH
#define kOutPinFine2      7
#define kOutPinFine3      8
#define kOutPinFine4      9

#define kOutPinMove       13      // Move mode led lite HIGH (LED_BUILTIN on nano)

#define kOutPinDisSCL     A5      // Oled Display cnx    
#define kOutPinDisSDA     A4

#define kInPinMin         10     // Switches Activ low
#define kInPinMax         11
#define kInPinMove        12

#define kInPinPot1        A3     // Servo potentiometers ANALOG
#define kInPinPot2        A2
#define kInPinPot3        A1
#define kInPinPot4        A0

#define kInPinFine        A7     // Fine Switch resistor chain ANALOG


//****************************************************************************************************************
// Misc Constants

// Global ---------------------------
#define kTaskShift          10      // Task phase shifter

// Servo ---------------------------
#define kServoTaskPeriod    20      // servo task period in ms

#define kServoMin           400L    // Minimum possible min Servo pulse duration in µs
#define kServoMax           2600L   // Maximum possible min Servo pulse duration in µs
#define kServoDefMin        800L    // Default min Servo pulse duration in µs
#define kServoDefMax        2200L   // Default max Servo pulse duration in µs

#define kServoFineMin       -1000L  // Fine adjust limits In a 10000 range 
#define kServoFineMax       1000L

#define kServoSpeedRate     25L     // Speed rate set between 5 and 100 vs wanted speed (Lower is faster)

#define OptRevDir                   // if defined pulse grow when pot turn ccw (To be coherent with servo dir) 
                                    // Comment to grow cw                   


// Display ---------------------------
#define kDisplayTaskPeriod  250      // Display task period in ms

#define SCREEN_WIDTH        128     // OLED display width, in pixels
#define SCREEN_HEIGHT       64      // OLED display height, in pixels
#define OLED_RESET          -1      // Reset pin # (or -1 if sharing Arduino reset pin) NOT USED!!!
#define SCREEN_ADDRESS      0x3C    //< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

// IO ---------------------------
#define kIoTaskPeriod       50      // IO task period in ms

//****************************************************************************************************************
// Globals

Adafruit_SSD1306 gDisplay(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);    // Global Display

uint8_t           gMinEdit    = 0;      // Current min edit 1->4 (o if not)
uint8_t           gMaxEdit    = 0;      // Current min edit 1->4 (o if not)
bool              gMinMaxEdit =false;   // True if min/max edit mode set.

uint8_t           gFineSelect = 0;      // Current fine toggle (must be acknoledged by reset) (o if not)

uint8_t           gMoveEdit   = 0;      // Current move edit 1->4 (o if not)

//Tasks___________________________________________________________________________________________________________


//ServoCtl ..........................................................................................
class ServoCtl: public Cath{

  public:

  //..............................................................
  ServoCtl(uint8_t Id,uint8_t PotPin,int8_t ServoPin,uint8_t LedFinePin) {

    m_Id=Id;
    m_PotPin=PotPin;
    m_ServoPin=ServoPin;
    m_LedFinePin=LedFinePin;

    m_Pos=1500;
    m_Min=kServoDefMin;
    m_Max=kServoDefMax;
  
    m_isFine=false;

    m_Speed=-1L;
    m_MoveDown=false;

    Cath::S_Register(this,kServoTaskPeriod,kTaskShift*Cath::S_NbTask);
  }

  //..............................................................
  void SetUp() {

    pinMode(m_LedFinePin,OUTPUT);
    digitalWrite(m_LedFinePin,m_isFine);

    m_Servo.attach(m_ServoPin);     // Init Servo
  }

  //..............................................................
  void Loop() {

    long Fine;

    if (m_Speed>=0) {   // ** Auto Move mode *******************


      m_Speed=map(analogRead(m_PotPin),0,1023,0,100);   // Get speed %

      long Inc=map(m_Speed,0,100,0,(m_Max-m_Min)/kServoSpeedRate);    // Calc lopp inc in µs

      if (m_MoveDown) {
        m_Pos-=Inc;
        if (m_Pos<m_Min) {      // Lower than Min, limit and invert direction
          m_Pos=m_Min;
          m_MoveDown=false;
        }
      }
      else {
        m_Pos+=Inc;
        if (m_Pos>m_Max) {      // Greater than Max, limit and invert direction
          m_Pos=m_Max;
          m_MoveDown=true;
        }
      }

    }
    else {            // ** Normal mode ***********************

      if (m_isFine)
        Fine=analogRead(m_PotPin);
      else
      {
        m_Coarse=analogRead(m_PotPin);
        Fine=512;
      }

      #ifdef OptRevDir
        Fine=map(Fine,0L,1023L,kServoFineMax,kServoFineMin);
        long Coarse=map(m_Coarse,0L,1023L,10000L,0L)+Fine;
      #else
        Fine=map(Fine,0L,1023L,kServoFineMin,kServoFineMax);
        long Coarse=map(m_Coarse,0L,1023L,0L,10000L)+Fine;
      #endif

      if (gMinMaxEdit && (gMinEdit==m_Id || gMaxEdit==m_Id))
        m_Pos=map(Coarse,0L,10000L,kServoMin,kServoMax);
      else
        m_Pos=map(Coarse,0L,10000L,m_Min,m_Max);
    }

    m_Servo.writeMicroseconds(m_Pos);   // Set servo position

    // Handle fine mode
    if (gFineSelect==m_Id) {
      gFineSelect=0;
      m_isFine=!m_isFine;
      digitalWrite(m_LedFinePin,m_isFine);  // Lit Fine led as needed
    }

  }

  //..............................................................
  void SetMin()
  {
    m_Min=m_Pos;
    if (m_Max<m_Min)
      m_Max=m_Min;
    m_isFine=false;
    digitalWrite(m_LedFinePin,m_isFine);
  }

  //..............................................................
  void SetMax()
  {
    m_Max=m_Pos;
    if (m_Min>m_Max)
      m_Min=m_Max;
    m_isFine=false;
    digitalWrite(m_LedFinePin,m_isFine);
  }

    //..............................................................
  void ToggleSpeed()
  {
    if (m_Speed<0)
      m_Speed=1; // Temp value will be overide by pot
    else
      m_Speed=-1;
  }

  uint8_t     m_Id;           // Servo Id
  uint8_t     m_PotPin;       // Analog input of potentometer
  uint8_t     m_ServoPin;     // Digital output of Fine Led
  uint8_t     m_LedFinePin;   // Digital output of Fine Led
  Servo       m_Servo;        // Servo Object

  long        m_Coarse;       // Pot value in Coarse mode

  long        m_Pos;          // Pos in µs
  long        m_Min;          // Min in µs
  long        m_Max;          // Max in µs

  bool        m_isFine;       // True if fine mode

  long        m_Speed;       // Speed % (-1 if not)
  bool        m_MoveDown;    // True if move down
};

//Display ..........................................................................................
class DisplayCtl: public Cath{

  public:

  //..............................................................
  DisplayCtl() {

    Cath::S_Register(this,kDisplayTaskPeriod,kTaskShift*Cath::S_NbTask);
  }

  //..............................................................
  void SetUp() {
    pinMode(LED_BUILTIN, OUTPUT);  // Built in led is an indicator

    // Display Init
    if(!gDisplay.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
      for(;;){                            // Init Error indicator
        digitalWrite(LED_BUILTIN, HIGH);
        delay(250);
        digitalWrite(LED_BUILTIN, LOW);
        delay(250);
        digitalWrite(LED_BUILTIN, HIGH);
        delay(250);
        digitalWrite(LED_BUILTIN, LOW);
        delay(2000);
        }
      }
 
    // Test display
    gDisplay.display();          // Display SplashScreen
    delay(2000);                // Pause for 2 seconds
    gDisplay.clearDisplay();     // Clear the buffer
  }

  //..............................................................
  void Loop(){
    // Clear the buffer
    gDisplay.clearDisplay();

    gDisplay.setTextSize(1);                 // Normal 1:1 pixel scale
    gDisplay.setTextColor(SSD1306_WHITE);    // Draw white text
    gDisplay.setCursor(0,0);                 // Start at top-left corner
    
    for (int D=1;D<=4;D++){

      ServoCtl* SERVO =  (ServoCtl*)Cath::S_CathTasks[D];

      if (D==gMoveEdit) {
        gDisplay.setTextColor(BLACK, WHITE);
      }

      gDisplay.print(D);
      if (SERVO->m_Speed>=0) {
        gDisplay.print(F(": Speed "));
        gDisplay.print(SERVO->m_Speed);
        gDisplay.println(F("%"));
      }
      else {
        gDisplay.print(F(": "));
        gDisplay.print(SERVO->m_Pos);
        gDisplay.println(F("us"));
      }

      gDisplay.setTextColor(WHITE); 

      gDisplay.print(F("   "));
      if (D==gMinEdit){
        gDisplay.setTextColor(BLACK, WHITE);
      }
      gDisplay.print(F("Min:"));
      gDisplay.setTextColor(WHITE); 

      gDisplay.print(SERVO->m_Min);

      gDisplay.print(F("  "));
      if (D==gMaxEdit){
        gDisplay.setTextColor(BLACK, WHITE);
      }
      gDisplay.print(F("Max:"));
      gDisplay.setTextColor(WHITE); 

      gDisplay.println(SERVO->m_Max);
    }

    gDisplay.display();
  }


};


//IoCtl ..........................................................................................
class IoCtl: public Cath {

  public:

  //..............................................................
  IoCtl() {

    Cath::S_Register(this,kIoTaskPeriod,kTaskShift*Cath::S_NbTask);
  }
  //..............................................................
  void SetUp() {
    pinMode(kOutPinMove,OUTPUT);

    pinMode(kInPinMin,INPUT_PULLUP);
    pinMode(kInPinMax,INPUT_PULLUP);
    pinMode(kInPinMove,INPUT_PULLUP);

    m_LastMin=true;
    m_LastMax=true;
    m_LastMove=true;
  }

  //..............................................................
  void Loop(){

    // Handle Min/MoveSel Bt
    if (digitalRead(kInPinMin)!=m_LastMin){
      m_LastMin=!m_LastMin;
      if (!m_LastMin) { // count on push down only

        if (gMoveEdit==0) {        // Normal mode
          gMinMaxEdit=false;
          gMaxEdit=0;
          gMinEdit++;
          if (gMinEdit>4)
            gMinEdit=0;
        }
        else {                    // Move mode select
          gMoveEdit++;
          if (gMoveEdit>4)
            gMoveEdit=1;
        }

      }
    }

    // Handle Max/MoveToggle Bt
    if (digitalRead(kInPinMax)!=m_LastMax) {
      m_LastMax=!m_LastMax;
      if (!m_LastMax) { // count on push down only

        if (gMoveEdit==0){
          gMinMaxEdit=false;
          gMinEdit=0;
          gMaxEdit++;
          if (gMaxEdit>4)
            gMaxEdit=0;
        }
        else {
          ((ServoCtl*)(Cath::S_CathTasks[gMoveEdit]))->ToggleSpeed();
        }

      }
    }

    // Handle Move Bt
    if (digitalRead(kInPinMove)!=m_LastMove){
      m_LastMove=!m_LastMove;
      if (!m_LastMove) { // count on push down only
        if (gMinEdit>0 || gMaxEdit>0) {

          if (gMinMaxEdit) {
            if (gMinEdit) {
              ((ServoCtl*)(Cath::S_CathTasks[gMinEdit]))->SetMin();
              gMinEdit=0;
            }
            else if (gMaxEdit>0) {
              ((ServoCtl*)(Cath::S_CathTasks[gMaxEdit]))->SetMax();
              gMaxEdit=0;
            }
            gMinMaxEdit=false;
          }
          else
            gMinMaxEdit=true;
        }
        else {    // Toggle move mode
          if (gMoveEdit==0)
            gMoveEdit=1;
          else
          {
            gMoveEdit=0;
            for (int S=1;S<=4;S++)
              ((ServoCtl*)(Cath::S_CathTasks[S]))->m_Speed=-1L;
          }
        }
      }   
    } 

    // Move blink
    if (gMinMaxEdit)
      digitalWrite(kOutPinMove,!digitalRead(kOutPinMove));
    else
      digitalWrite(kOutPinMove,gMoveEdit!=0);
     
    
    // Handle fine
    uint8_t FineBt=map(analogRead(kInPinFine),0,1000,0,5);    // Transform analog to button push

    if (FineBt!=m_LastFine) {
      if (FineBt>m_LastFine)
          gFineSelect=FineBt;
      m_LastFine=FineBt;
    }

  }


  bool        m_LastMin;    // Last known value of Min bt
  bool        m_LastMax;    // Last known value of Max bt
  bool        m_LastMove;   // Last known value of Move bt

  uint8_t     m_LastFine;   // Last known value of Fine bt

};

//****************************************************************************************************************
// Global tasks instanciation
// **** WARNING TASK ORDER MUST BE RESPECTED ****

// 1 instance of DisplayCtl to control Oled display
DisplayCtl  gDisplayCtl;

// 4 Instances of the ServoCtl task to control Oled servo
ServoCtl    gServoCtl1(1,kInPinPot1,kOutPinServo1,kOutPinFine1);
ServoCtl    gServoCtl2(2,kInPinPot2,kOutPinServo2,kOutPinFine2);
ServoCtl    gServoCtl3(3,kInPinPot3,kOutPinServo3,kOutPinFine3);
ServoCtl    gServoCtl4(4,kInPinPot4,kOutPinServo4,kOutPinFine4);

// 1 instance of ioctl to control IO
IoCtl       gIoCtl;

//-----------------------------------------------------------------------------------------------------------------
void setup() {
  Cath::S_SetUp();    // Just ask Cath to call the task's setup
}

//-----------------------------------------------------------------------------------------------------------------
void loop() {
  Cath::S_Loop();    // Just ask Cath to call the task's loop
}

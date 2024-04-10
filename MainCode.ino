////Define IO Pins////
  //Aiming Subsystem
    const int vert1A1Pin = 2;
    const int vert1A2Pin = 3;
    const int vert1B1Pin = 4;
    const int vert1B2Pin = 5;
    const int vert1PWMABPin = 39; //Alway ON
    const int vert1VccPin = 40; //Always ON
    const int vert2A1Pin = 6;
    const int vert2A2Pin = 7;
    const int vert2B1Pin = 8;
    const int vert2B2Pin = 9;
    const int vert2PWMABPin = 41; //Alway ON
    const int vert2VccPin = 42; //Always ON
    const int rotA1Pin = 10;
    const int rotA2Pin = 11;
    const int rotB1Pin = 12;
    const int rotB2Pin = 13;
    const int rotPWMABPin = 52; //Alway ON
    const int rotVccPin = 53; //Always ON

  //Firing Subsystem
    const int fly1IN1Pin = 44;
    const int fly1IN2Pin = 47; // ALWAYS OFF
    const int fly2IN1Pin = 45;
    const int fly2IN2Pin = 48; // ALWAYS OFF
    const int loadIN1Pin = 46;
    const int loadIN2Pin = 49; // ALWAYS OFF

  //User Interface Subsystem
    const int disp1Sel1Pin = 16;
    const int disp1Sel2Pin = 17;
    const int disp1Sel3Pin = 15;
    const int disp1Val1Pin = 36;
    const int disp1Val2Pin = 20;
    const int disp1Val3Pin = 21;
    const int disp1Val4Pin = 22;
    const int disp1Val5Pin = 23;
    const int disp1Val6Pin = 24;
    const int disp1Val7Pin = 25;
    const int disp2Sel1Pin = 27;
    const int disp2Sel2Pin = 28;
    const int disp2Sel3Pin = 29;
    const int disp2Val1Pin = 30;
    const int disp2Val2Pin = 31;
    const int disp2Val3Pin = 32;
    const int disp2Val4Pin = 33;
    const int disp2Val5Pin = 34;
    const int disp2Val6Pin = 35;
    const int disp2Val7Pin = 19;
    const int fireSWPin = 38;
    const int xdirectionPin = 3;
    const int ydirectionPin = 4;
    const int firingSpeedPin = 5;
    const int aimingSensitivityPin = 6;

  //Projectile Subsystem
      const int shotCounterPin = 18; //18 is an interrupt driving pin
      const int laserONPin = 51;

  //Power Subsystem
      const int batteryVoltagePin = 12;
  
////Define Functions////
  //Aiming Subsystem
    //rotational control
    int rControl(int currentStep, int direction, bool holding, int mPinA1, int mPinA2, int mPinB1, int mPinB2) {

      if (direction == 1){

        switch (currentStep){

          case 1:
            //1 to 2
            digitalWrite(mPinA1, 0);
            digitalWrite(mPinA2, 1);
            currentStep = 2;
            break;

          case 2:
            //2 to 3
            digitalWrite(mPinB1, 0);
            digitalWrite(mPinB2, 1);
            currentStep = 3;
            break;

          case 3:
            //3 to 4
            digitalWrite(mPinA2, 0);
            digitalWrite(mPinA1, 1);
            currentStep = 4;
            break;

          case 4:
            //4 to 1
            digitalWrite(mPinB2, 0);
            digitalWrite(mPinB1, 1);
            currentStep = 1;
            break;
        }
      }
      if (direction == 2){
        switch (currentStep){

          case 1:
            //1 to 4
            digitalWrite(mPinB1, 0);
            digitalWrite(mPinB2, 1);
            currentStep = 4;
            break;

          case 2:
            //2 to 1
            digitalWrite(mPinA2, 0);
            digitalWrite(mPinA1, 1);
            currentStep = 1;
            break;

          case 3:
            //3 to 2
            digitalWrite(mPinB2, 0);
            digitalWrite(mPinB1, 1);
            currentStep = 2;
            break;

          case 4:
            //4 to 3
            digitalWrite(mPinA1, 0);
            digitalWrite(mPinA2, 1);
            currentStep = 3;
            break;
        }
      }
      if (direction == 0){
        if(holding){
          digitalWrite(mPinA1, 0);
          digitalWrite(mPinA2, 0);
          digitalWrite(mPinB1, 0);
          digitalWrite(mPinB2, 0);
        }
        else{
          switch (currentStep){
            case 1:
              digitalWrite(mPinA1, 1);
              digitalWrite(mPinB1, 1);
              break;

            case 2:
              digitalWrite(mPinB1, 1);
              digitalWrite(mPinA2, 1);
              break;

            case 3:
              digitalWrite(mPinA2, 1);
              digitalWrite(mPinB2, 1);
              break;

            case 4:
              digitalWrite(mPinB2, 1);
              digitalWrite(mPinA1, 1);
              break;
          }
        }
      }
      return currentStep;
    }
    //vertical control
    int vControl(int currentStep, int direction, bool holding, int m1PinA1, int m1PinA2, int m1PinB1, int m1PinB2, int m2PinA1, int m2PinA2, int m2PinB1, int m2PinB2) {

      if (direction == 1){

        switch (currentStep){

          case 1:
            //1 to 2
            digitalWrite(m1PinA1, 0);
            digitalWrite(m1PinA2, 1);
            digitalWrite(m2PinA1, 0);
            digitalWrite(m2PinA2, 1);
            currentStep = 2;
            break;

          case 2:
            //2 to 3
            digitalWrite(m1PinB1, 0);
            digitalWrite(m1PinB2, 1);
            digitalWrite(m2PinB1, 0);
            digitalWrite(m2PinB2, 1);
            currentStep = 3;
            break;

          case 3:
            //3 to 4
            digitalWrite(m1PinA2, 0);
            digitalWrite(m1PinA1, 1);
            digitalWrite(m2PinA2, 0);
            digitalWrite(m2PinA1, 1);
            currentStep = 4;
            break;

          case 4:
            //4 to 1
            digitalWrite(m1PinB2, 0);
            digitalWrite(m1PinB1, 1);
            digitalWrite(m2PinB2, 0);
            digitalWrite(m2PinB1, 1);
            currentStep = 1;
            break;

        }
      }
      if (direction == 2){
        switch (currentStep){

          case 1:
            //1 to 4
            digitalWrite(m1PinB1, 0);
            digitalWrite(m1PinB2, 1);
            digitalWrite(m2PinB1, 0);
            digitalWrite(m2PinB2, 1);
            currentStep = 4;
            break;

          case 2:
            //2 to 1
            digitalWrite(m1PinA2, 0);
            digitalWrite(m1PinA1, 1);
            digitalWrite(m2PinA2, 0);
            digitalWrite(m2PinA1, 1);
            currentStep = 1;
            break;

          case 3:
            //3 to 2
            digitalWrite(m1PinB2, 0);
            digitalWrite(m1PinB1, 1);
            digitalWrite(m2PinB2, 0);
            digitalWrite(m2PinB1, 1);
            currentStep = 2;
            break;

          case 4:
            //4 to 3
            digitalWrite(m1PinA1, 0);
            digitalWrite(m1PinA2, 1);
            digitalWrite(m2PinA1, 0);
            digitalWrite(m2PinA2, 1);
            currentStep = 3;
            break;
        }
      }
      if (direction == 0){
        if(holding){
          digitalWrite(m1PinA1, 0);
          digitalWrite(m1PinA2, 0);
          digitalWrite(m1PinB1, 0);
          digitalWrite(m1PinB2, 0);
          digitalWrite(m2PinA1, 0);
          digitalWrite(m2PinA2, 0);
          digitalWrite(m2PinB1, 0);
          digitalWrite(m2PinB2, 0);
        }
        else{
          switch (currentStep){
            case 1:
              digitalWrite(m1PinA1, 1);
              digitalWrite(m1PinB1, 1);
              digitalWrite(m2PinA1, 1);
              digitalWrite(m2PinB1, 1);
              break;

            case 2:
              digitalWrite(m1PinB1, 1);
              digitalWrite(m1PinA2, 1);
              digitalWrite(m2PinB1, 1);
              digitalWrite(m2PinA2, 1);
              
              break;

            case 3:
              digitalWrite(m1PinA2, 1);
              digitalWrite(m1PinB2, 1);
              digitalWrite(m2PinA2, 1);
              digitalWrite(m2PinB2, 1);
              break;

            case 4:
              digitalWrite(m1PinB2, 1);
              digitalWrite(m1PinA1, 1);
              digitalWrite(m2PinB2, 1);
              digitalWrite(m2PinA1, 1);
              break;
          }
        }
      }
      return currentStep;
    }

  //Firing Subsystem
    //Turn on flywheels
    void flyON(int scaling, int mot1IN1, int mot2IN1){
      int speed = scaling * 2.55;
      if(speed>255){
        speed = 255;
      }
      analogWrite(mot1IN1, speed);
      analogWrite(mot2IN1, speed);
    }
    //Turn off flywheels
    void flyOFF(int mot1IN1, int mot2IN1){
      digitalWrite(mot1IN1, 0);
      digitalWrite(mot2IN1, 0);
    }
    //Turn loading motor on
    void loadON(int scaling, int loadIN1){
      int speed = scaling * 2.55;
      if(speed>255){
        speed = 255;
      }
      analogWrite(loadIN1, speed);
    }
    //Turn loading motor off
    void loadOFF(int loadIN1){
      digitalWrite(loadIN1, 0);
    }

  //User Interface Subsystem
    //analog input with scaling and offset
    float analogInput(int pin, float scaling, float offset) {
      return analogRead(pin) * scaling - offset;
    }
    //digital input
    bool digitalInput(int pin){
        return digitalRead(pin);
    }
    //7-segment display drivers
    int display(int value, int activeDisp, int d2, int d3, int d4, int a, int b, int c, int d, int e, int f, int g){
      int dispVal;
      switch(activeDisp){
        default:
          dispVal = value%10;
          digitalWrite(d2, 0); //ones
          digitalWrite(d3, 1); //tens
          digitalWrite(d4, 1); //hundreds
          break;
        case 1:
          dispVal = (value%1000)/100;
          digitalWrite(d2, 1);
          digitalWrite(d3, 1);
          digitalWrite(d4, 1);
          break;
        case 2:
          dispVal = (value%100)/10;
          digitalWrite(d2, 1);
          digitalWrite(d3, 0);
          digitalWrite(d4, 1);
          break;
        case 3:
          dispVal = value%10;
          digitalWrite(d2, 0);
          digitalWrite(d3, 1);
          digitalWrite(d4, 1);
          break;
      }

      //Increment activeDisp for next call
      if (activeDisp >= 3){
        activeDisp = 1;
      }
      else{
        activeDisp = activeDisp + 1;
      }

      switch(dispVal){
        default:
          digitalWrite(a, 0);
          digitalWrite(b, 0); //Good
          digitalWrite(c, 0); //Good
          digitalWrite(d, 0); //Good
          digitalWrite(e, 0); //Good
          digitalWrite(f, 0);
          digitalWrite(g, 0); //Good
          break;
        
        case 0:
          digitalWrite(a, 1);
          digitalWrite(b, 1);
          digitalWrite(c, 1);
          digitalWrite(d, 1);
          digitalWrite(e, 1);
          digitalWrite(f, 1);
          digitalWrite(g, 0);
        break;
        
        case 1:
          digitalWrite(a, 0);
          digitalWrite(b, 1);
          digitalWrite(c, 1);
          digitalWrite(d, 0);
          digitalWrite(e, 0);
          digitalWrite(f, 0);
          digitalWrite(g, 0);
          break;

        case 2:
          digitalWrite(a, 1);
          digitalWrite(b, 1);
          digitalWrite(c, 0);
          digitalWrite(d, 1);
          digitalWrite(e, 1);
          digitalWrite(f, 0);
          digitalWrite(g, 1);
          break;

        case 3:
          digitalWrite(a, 1);
          digitalWrite(b, 1);
          digitalWrite(c, 1);
          digitalWrite(d, 1);
          digitalWrite(e, 0);
          digitalWrite(f, 0);
          digitalWrite(g, 1);
          break;

        case 4:
          digitalWrite(a, 0);
          digitalWrite(b, 1);
          digitalWrite(c, 1);
          digitalWrite(d, 0);
          digitalWrite(e, 0);
          digitalWrite(f, 1);
          digitalWrite(g, 1);
          break;

        case 5:
          digitalWrite(a, 1);
          digitalWrite(b, 0);
          digitalWrite(c, 1);
          digitalWrite(d, 1);
          digitalWrite(e, 0);
          digitalWrite(f, 1);
          digitalWrite(g, 1);
          break;

        case 6:
          digitalWrite(a, 1);
          digitalWrite(b, 0);
          digitalWrite(c, 1);
          digitalWrite(d, 1);
          digitalWrite(e, 1);
          digitalWrite(f, 1);
          digitalWrite(g, 1);
          break;

        case 7:
          digitalWrite(a, 1);
          digitalWrite(b, 1);
          digitalWrite(c, 1);
          digitalWrite(d, 0);
          digitalWrite(e, 0);
          digitalWrite(f, 0);
          digitalWrite(g, 0);
          break;

        case 8:
          digitalWrite(a, 1);
          digitalWrite(b, 1);
          digitalWrite(c, 1);
          digitalWrite(d, 1);
          digitalWrite(e, 1);
          digitalWrite(f, 1);
          digitalWrite(g, 1);
          break;

        case 9:
          digitalWrite(a, 1);
          digitalWrite(b, 1);
          digitalWrite(c, 1);
          digitalWrite(d, 1);
          digitalWrite(e, 0);
          digitalWrite(f, 1);
          digitalWrite(g, 1);
          break;
      }
      return activeDisp;
    }
    //digital output control for aiming laser
    void aimingLaser(int pin, bool ON){
      digitalWrite(pin, ON);
    }

////Define Dynamic Variables////
  //System
    unsigned long currentTime;

  //User Interface Subsystem
    float aimVert;
    float aimRot;
    float aimScaling;
    float firingScaling;
    bool triggerReq;
    int activeDisp1;
    int activeDisp2;

  //Aiming Subsystem
    int rDirection;
    int rStep = 1;
    int rDelay = 10000; //Number of 4us counts
    int vDirection;
    int vStep = 1;
    int vDelay = 10000; //Number of 4us counts
    unsigned long sleepDelay = 10000; //ms
    unsigned long sleepStart;
    bool sleepMode;
    bool sleepTimeSet;

  //Firing Subsystem
    bool triggerCurrent;
    bool triggerPrevious;
    unsigned long flywheelOnStart;
    unsigned long flywheelOnDelay = 1000; //ms
    unsigned long loadOffStart;
    unsigned long loadOffDelay = 2000; //ms
    unsigned long singleStart;
    unsigned long singleTimeout = 5000; //ms
    int firingStateC = 0;

  //Projectile Subsystem
    bool shotFired = 0;
    int remainingAmmo = 25;

  //Power Subsystem
    int batteryPercent;

////SYSTEM INTERRUPTS////
  //Aiming Subsystem
    //Interrupt for rotational motion
    ISR(TIMER4_COMPA_vect)
    {
      OCR4A = OCR4A + rDelay; // Advance The COMPA Register by rDelay (max speed at 900, min speed at 20000)
      // Handle The Timer Interrupt
      rStep = rControl(rStep, rDirection, sleepMode, rotA1Pin, rotA2Pin, rotB1Pin, rotB2Pin);
    }

    //Interrupt for vertical motion
    ISR(TIMER4_COMPB_vect)
    {
      OCR4B = OCR4B + vDelay; // Advance The COMPB Register by vdelay (max speed at 2000, min speed at 20000)
      // Handle The Timer Interrupt
      vStep = vControl(vStep, vDirection, sleepMode, vert1A1Pin, vert1A2Pin, vert1B1Pin, vert1B2Pin, vert2A1Pin, vert2A2Pin, vert2B1Pin, vert2B2Pin);
    }

  //Firing Subsystem
    //Interrupt for break beam shot counter
    void shotCounter(){
      shotFired = 1;
      remainingAmmo = remainingAmmo - 1;
    }

void setup() {
  ////Serial Setup for Debugging////
    Serial.begin(9600);

  ////Timer4 Interrupt Setup (used for Aiming Subsystem Interrupts)////
    TCCR4A = 0;           // Init Timer1A
    TCCR4B = 0;           // Init Timer1B
    TCCR4B |= B00000011;  // Prescaler = 64 
    TIMSK4 |= B00000110;  // Enable COMPA and COMPB Interrupts
    OCR4A = 32768; //Default for rot, Every increment is 4us
    OCR4B = 16384; //Default for vert, Every increment is 4us

  ////Instantiate IO Pins////
    //Aiming Subsystem
      pinMode(vert1A1Pin, OUTPUT);
      pinMode(vert1A2Pin, OUTPUT);
      pinMode(vert1B1Pin, OUTPUT);
      pinMode(vert1B2Pin, OUTPUT);
      pinMode(vert1PWMABPin, OUTPUT);
      pinMode(vert1VccPin, OUTPUT);
      pinMode(vert2A1Pin, OUTPUT);
      pinMode(vert2A2Pin, OUTPUT);
      pinMode(vert2B1Pin, OUTPUT);
      pinMode(vert2B2Pin, OUTPUT);
      pinMode(vert2PWMABPin, OUTPUT);
      pinMode(vert2VccPin, OUTPUT);
      pinMode(rotA1Pin, OUTPUT);
      pinMode(rotA2Pin, OUTPUT);
      pinMode(rotB1Pin, OUTPUT);
      pinMode(rotB2Pin, OUTPUT);
      pinMode(rotPWMABPin, OUTPUT);
      pinMode(rotVccPin, OUTPUT);

    //Firing Subsystem
      pinMode(fly1IN1Pin, OUTPUT);
      pinMode(fly1IN2Pin, OUTPUT); // ALWAYS OFF
      pinMode(fly1IN1Pin, OUTPUT);
      pinMode(fly1IN2Pin, OUTPUT); // ALWAYS OFF
      pinMode(fly1IN1Pin, OUTPUT);
      pinMode(fly1IN2Pin, OUTPUT); // ALWAYS OFF

    //User Interface Subsystem
      pinMode(disp1Sel1Pin, OUTPUT);
      pinMode(disp1Sel2Pin, OUTPUT);
      pinMode(disp1Sel3Pin, OUTPUT);
      pinMode(disp1Val1Pin, OUTPUT);
      pinMode(disp1Val2Pin, OUTPUT);
      pinMode(disp1Val3Pin, OUTPUT);
      pinMode(disp1Val4Pin, OUTPUT);
      pinMode(disp1Val5Pin, OUTPUT);
      pinMode(disp1Val6Pin, OUTPUT);
      pinMode(disp1Val7Pin, OUTPUT);
      pinMode(disp2Sel1Pin, OUTPUT);
      pinMode(disp2Sel2Pin, OUTPUT);
      pinMode(disp2Sel3Pin, OUTPUT);
      pinMode(disp2Val1Pin, OUTPUT);
      pinMode(disp2Val2Pin, OUTPUT);
      pinMode(disp2Val3Pin, OUTPUT);
      pinMode(disp2Val4Pin, OUTPUT);
      pinMode(disp2Val5Pin, OUTPUT);
      pinMode(disp2Val6Pin, OUTPUT);
      pinMode(disp2Val7Pin, OUTPUT);
      pinMode(fireSWPin, INPUT_PULLUP); //Sourcing Input

    //Projectile Subsystem
      pinMode(shotCounterPin, INPUT_PULLUP);
      attachInterrupt(digitalPinToInterrupt(shotCounterPin), shotCounter, FALLING);//Pin driven interrupt
      pinMode(laserONPin, OUTPUT);

  ////Initialize Constant Outputs////
    //Aiming Subsystem
      digitalWrite(vert1PWMABPin, 1); //ALWAYS ON
      digitalWrite(vert1VccPin, 1); //ALWAYS ON
      digitalWrite(vert2PWMABPin, 1); //ALWAYS ON
      digitalWrite(vert2VccPin, 1); //ALWAYS ON
      digitalWrite(rotPWMABPin, 1); //ALWAYS ON
      digitalWrite(rotVccPin, 1); //ALWAYS ON

    //Firing Subsystem
      digitalWrite(fly1IN2Pin, 0); //ALWAYS OFF
      digitalWrite(fly2IN2Pin, 0); //ALWAYS OFF
      digitalWrite(loadIN2Pin, 0); //ALWAYS OFF

}

void loop() {
  ////Update currentTime////

    currentTime = millis();

  ////User Interface Subsystem////
    //Analog Inputs
      aimRot = analogInput(xdirectionPin, 0.09765, 0); //0.09765 = 100/1024; 100 = right, 0 = left
      aimVert = analogInput(ydirectionPin, 0.09765, 0); //0.09765 = 100/1024; 100 = up, 0 = down
      aimScaling = analogInput(aimingSensitivityPin, 0.09765, 0); //0.09765 = 100/1024; 0 to 100%
      firingScaling = analogInput(firingSpeedPin, 0.09765, 0); //0.09765 = 100/1024; 0 to 100%

    //Digital Inputs 
      triggerReq = !digitalInput(fireSWPin); //Inverted logic so signal is negated.

    //Displays
      activeDisp1 = display(batteryPercent, activeDisp1, disp1Sel3Pin, disp1Sel2Pin, disp1Sel1Pin, disp1Val1Pin, disp1Val2Pin, disp1Val3Pin, disp1Val4Pin, disp1Val5Pin, disp1Val6Pin, disp1Val7Pin);
      activeDisp2 = display(remainingAmmo, activeDisp2, disp2Sel3Pin, disp2Sel2Pin, disp2Sel1Pin, disp2Val1Pin, disp2Val2Pin, disp2Val3Pin, disp2Val4Pin, disp2Val5Pin, disp2Val6Pin, disp2Val7Pin);   

  ////Projectile Subsystem////
    //Projectile Management (INTERRUPT DRIVEN, void shotCount())

    //Aiming Laser
      aimingLaser(laserONPin, 1); //ALWAYS ON WHEN POWERED ON
    
  ////Aiming Subsystem////
    //Rotation speed and direction (actual motion is interrupt driven)
      rDirection = 0;

      if (aimRot > 55){
        rDirection = 1;
        rDelay = 39100 - 382 * aimRot * aimScaling / 100;
      }

      if(aimRot < 45){
        rDirection = 2;
        rDelay = 39100 - 382 * (100 - aimRot) * aimScaling / 100;
      }

    //Vertical speed and direction (actual motion is interrupt driven)
      vDirection = 0;

      if (aimVert > 55){
        vDirection = 2;
        vDelay = 38000 - 360 * aimVert * aimScaling / 100;
      }

      if(aimVert < 45){
        vDirection = 1;
        vDelay = 38000 - 360 * (100 - aimVert) * aimScaling / 100;
      }

    //Sleep mode (disable steppers after a certain amount of time to conserve battery)
      if (rDirection == 0 && vDirection == 0){
        if(sleepTimeSet == 0){
          sleepStart = currentTime;
          sleepTimeSet = 1;
        }

        if(currentTime > (sleepStart + sleepDelay)){
          sleepMode = 1;

        }
      }
      else {
        sleepMode = 0;
        sleepTimeSet = 0;
      }

  ////Firing Subsystem////

    //Continuous Shot Mode (all the way to the left)
    //Single Shot Mode (all the way to the right)
    //Controlled with FSM

    switch(firingStateC){
      case 0: //State 0: Flywheels and loading motors off
        flyOFF(fly1IN1Pin, fly2IN1Pin);
        if(!triggerReq){
          shotFired = 0;
        }
        if ( (triggerReq&&(firingScaling > 30)) || (triggerReq&&!shotFired&&(firingScaling < 30)) ){
          firingStateC = 1;
          flywheelOnStart = currentTime;
        }
        break;

      case 1: //State 1: Flywheel on and wait for delay
        flyON(255, fly1IN1Pin, fly2IN1Pin);
        if (currentTime>(flywheelOnStart + flywheelOnDelay)){
          firingStateC = 2;
          singleStart = currentTime;
        }
        break;

      case 2: //State 2: Loading motor (firing) and wait for no tirgger or shot through barrel
        if(firingScaling > 30){
          loadON(firingScaling, loadIN1Pin); 
        }
        else{
          loadON(40, loadIN1Pin); 
        }
        if ( (!triggerReq&&(firingScaling > 30)) || ( (shotFired||(currentTime > singleStart + singleTimeout) ) && (firingScaling < 30)) ){
          firingStateC = 3;
          loadOffStart = currentTime;
        }
        break;

      case 3: //State 3: Loading motor off and wait for delay or new trigger
        loadOFF(loadIN1Pin);
        if(!triggerReq){
          shotFired = 0;
        } 
        
        if (currentTime>(loadOffStart + loadOffDelay)){
          firingStateC = 0;
        }

        if ( (triggerReq&&(firingScaling > 30)) || (triggerReq&&!shotFired&&(firingScaling < 30)) ){
          firingStateC = 2;
        }
        break;
    }

  //// Power Subsystem////

    batteryPercent = analogInput(batteryVoltagePin,0.097656,0); //0.097656 = 100%/1024


  delay(4); //Slow down scan time to ensure displays do not flicker too fast
}

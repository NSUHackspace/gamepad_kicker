// For RAMPS 1.4
#define X_DIR_PIN          5
#define X_STEP_PIN         2

#define Y_DIR_PIN          6
#define Y_STEP_PIN         3

#define Z_DIR_PIN          7
#define Z_STEP_PIN         4

#define A_DIR_PIN          13
#define A_STEP_PIN         12


#define X_STEP_HIGH             PORTE |=  0b00010000;
#define X_STEP_LOW              PORTE &= ~0b00010000;

#define Y_STEP_HIGH             PORTE |=  0b00100000;
#define Y_STEP_LOW              PORTE &= ~0b00100000;

#define Z_STEP_HIGH             PORTG |=  0b00100000;
#define Z_STEP_LOW              PORTG &= ~0b00100000;

#define A_STEP_HIGH             PORTB |=  0b01000000;
#define A_STEP_LOW              PORTB &= ~0b01000000;


#define TIMER1_INTERRUPTS_ON    TIMSK1 |=  (1 << OCIE1A);
#define TIMER1_INTERRUPTS_OFF   TIMSK1 &= ~(1 << OCIE1A);

struct stepperInfo {
  // externally defined parameters
  float acceleration;
  volatile unsigned int minStepInterval;   // ie. max speed, smaller is faster
  void (*dirFunc)(int);
  void (*stepFunc)();
  long destination;

  // derived parameters
  unsigned int c0;                // step interval for first step, determines acceleration
  long stepPosition;              // current position of stepper (total of all movements taken so far)

  // per movement variables (only changed once per movement)
  volatile int dir;                        // current direction of movement, used to keep track of position
  // volatile unsigned int totalSteps;        // number of steps requested for current movement
  volatile bool movementDone = false;      // true if the current movement has been completed (used by main program to wait for completion)
  // volatile unsigned int rampUpStepCount;   // number of steps taken to reach either max speed, or half-way to the goal (will be zero until this number is known)

  // per iteration variables (potentially changed every interrupt)
  volatile unsigned int n;                 // index in acceleration curve, used to calculate next interval
  volatile float d;                        // current interval length
  volatile unsigned long di;               // above variable truncated
  // volatile unsigned int stepCount;         // number of steps completed in current movement
};

void xStep() {
  X_STEP_HIGH
  X_STEP_LOW
}
void xDir(int dir) {
  digitalWrite(X_DIR_PIN, dir);
}

void yStep() {
  Y_STEP_HIGH
  Y_STEP_LOW
}
void yDir(int dir) {
  digitalWrite(Y_DIR_PIN, dir);
}

void zStep() {
  Z_STEP_HIGH
  Z_STEP_LOW
}
void zDir(int dir) {
  digitalWrite(Z_DIR_PIN, dir);
}

void aStep() {
  A_STEP_HIGH
  A_STEP_LOW
}
void aDir(int dir) {
  digitalWrite(A_DIR_PIN, dir);
}

// void resetStepperInfo(volatile stepperInfo& si ) {
//   si.n = 0;
//   si.d = 0;
//   si.di = 0;
//   // si.stepCount = 0;
//   // si.rampUpStepCount = 0;
//   // si.totalSteps = 0;
//   si.stepPosition = 0;
//   si.movementDone = true;
  
// }

#define NUM_STEPPERS 4

#define CMD_MOVE_TO 0
#define CMD_SET_SPEED 1
#define CMD_SET_ACC 2
#define CMD_RESET_POS 3

volatile stepperInfo steppers[NUM_STEPPERS];
volatile long destUpdates[NUM_STEPPERS];
volatile bool hasDestUpdates[NUM_STEPPERS];

typedef struct __attribute__ ((packed)) {
    byte command;
    byte motor;
    union
    {
        float f;
        long l;
    };
    
} serial_msg_t;

void setup() {

  pinMode(X_STEP_PIN,   OUTPUT);
  pinMode(X_DIR_PIN,    OUTPUT);

  pinMode(Y_STEP_PIN,   OUTPUT);
  pinMode(Y_DIR_PIN,    OUTPUT);

  pinMode(Z_STEP_PIN,   OUTPUT);
  pinMode(Z_DIR_PIN,    OUTPUT);

  pinMode(A_STEP_PIN,   OUTPUT);
  pinMode(A_DIR_PIN,    OUTPUT);


  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 1000;                             // compare value
  TCCR1B |= (1 << WGM12);                   // CTC mode
  TCCR1B |= ((1 << CS11) | (1 << CS10));    // 64 prescaler
  interrupts();


  unsigned int buttonMinInterval = 75;
  float buttonAcc = 1000;


  unsigned int axisMinInterval = 300;
  float axisAcc = 3000;

  // button
  steppers[0].dirFunc = xDir;
  steppers[0].stepFunc = xStep;
  steppers[0].acceleration = buttonAcc;
  steppers[0].minStepInterval = buttonMinInterval;

  // axis
  steppers[1].dirFunc = yDir;
  steppers[1].stepFunc = yStep;
  steppers[1].acceleration = axisAcc;
  steppers[1].minStepInterval = axisMinInterval;

  // button
  steppers[2].dirFunc = zDir;
  steppers[2].stepFunc = zStep;
  steppers[2].acceleration = buttonAcc;
  steppers[2].minStepInterval = buttonMinInterval;

  // axis
  steppers[3].dirFunc = aDir;
  steppers[3].stepFunc = aStep;
  steppers[3].acceleration = axisAcc;
  steppers[3].minStepInterval = axisMinInterval;

  for(int i = 0; i < NUM_STEPPERS; i++){
    resetStepper(steppers[i]);
    hasDestUpdates[i] = false;
  }
  Serial.begin(115200);
  Serial.println("\nREADY!");
  Serial.println(sizeof(serial_msg_t));

  TIMER1_INTERRUPTS_ON
}

void resetStepper(volatile stepperInfo& si) {
  si.c0 = si.acceleration;
  si.d = si.c0;
  si.di = si.d;
  // si.stepCount = 0;
  si.n = 0;
  // si.rampUpStepCount = 0;
  si.movementDone = true;
  si.destination = 0;
  si.stepPosition = 0;
}

volatile byte remainingSteppersFlag = 0;

void prepareMovement(int whichMotor, long dest) {
  while (hasDestUpdates[whichMotor]);
  destUpdates[whichMotor] = dest;
  hasDestUpdates[whichMotor] = true;

  // volatile stepperInfo& si = steppers[whichMotor];
  // si.dirFunc( steps < 0 ? HIGH : LOW );
  // si.dir = steps > 0 ? 1 : -1;
  // si.totalSteps = abs(steps);
  // resetStepper(si);
  // remainingSteppersFlag |= (1 << whichMotor);
}

volatile byte nextStepperFlag = 0;

volatile int ind = 0;
volatile unsigned int intervals[100];

void setNextInterruptInterval() {

  bool movementComplete = true;

  unsigned int mind = 999999;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di < mind ) {
      mind = steppers[i].di;
    }
  }

  nextStepperFlag = 0;
  for (int i = 0; i < NUM_STEPPERS; i++) {
    if ( ! steppers[i].movementDone )
      movementComplete = false;

    if ( ((1 << i) & remainingSteppersFlag) && steppers[i].di == mind )
      nextStepperFlag |= (1 << i);
  }

  if ( remainingSteppersFlag == 0 ) {
    OCR1A = 65500;
  }

  OCR1A = mind;
}

ISR(TIMER1_COMPA_vect)
{
  unsigned int tmpCtr = OCR1A;

  OCR1A = 65500;

  for (int i = 0; i < NUM_STEPPERS; i++) {
    volatile stepperInfo& s = steppers[i];

    if(hasDestUpdates[i]){
      s.destination = destUpdates[i];
      hasDestUpdates[i] = false;
      if(s.movementDone && s.destination != s.stepPosition){
        remainingSteppersFlag |= (1 << i);
        nextStepperFlag |= (1 << i);
        s.movementDone = false;
        long distanceToGo = s.destination - s.stepPosition;
        s.dirFunc( distanceToGo < 0 ? HIGH : LOW );
        s.dir = distanceToGo > 0 ? 1 : -1;
        s.c0 = s.acceleration;
        s.d = s.c0;
        s.di = s.d;
      }

    }

    if ( ! ((1 << i) & remainingSteppersFlag) )
      continue;

    if ( ! (nextStepperFlag & (1 << i)) ) {
      steppers[i].di -= tmpCtr;
      continue;
    }

  
    if ( s.stepPosition != s.destination || s.n > 1 ) {
      s.stepFunc();
      s.stepPosition += s.dir;
      if ( s.stepPosition == s.destination && s.n <= 1 ) {
        s.movementDone = true;
        remainingSteppersFlag &= ~(1 << i);
      }
    }

    long distanceToGo = s.destination - s.stepPosition;
    // stepsToStop is n
    bool doAcc = false;
    bool doDec = false;

    if(distanceToGo > 0){
      if((s.n >= distanceToGo) || s.dir == -1)doDec = true;
      else doAcc = true;
    }
    if(distanceToGo < 0 ) {
      if((s.n >= -distanceToGo) || s.dir == 1)doDec = true;
      else doAcc = true;
    }

    if(doAcc){
      // try to accelerate, we are going the correct way
      if(s.d > s.minStepInterval){
        s.n++;
        s.d = s.d - (2 * s.d) / (4 * s.n + 1);
      }
    }else if (doDec && s.n != 0){
      // decelerate
      s.d = (s.d * (4 * s.n + 1)) / (4 * s.n + 1 - 2);
      s.n--;
    }else{
      // n == 0, we need to choose direction
      s.dirFunc( distanceToGo < 0 ? HIGH : LOW );
      s.dir = distanceToGo > 0 ? 1 : -1;
      s.c0 = s.acceleration;
      s.d = s.c0;
      s.di = s.d;
    }


    s.di = s.d; // integer

    // if ( s.stepCount < s.totalSteps ) {
    //   s.stepFunc();
    //   s.stepCount++;
    //   s.stepPosition += s.dir;
    //   if ( s.stepCount >= s.totalSteps ) {
    //     s.movementDone = true;
    //     remainingSteppersFlag &= ~(1 << i);
    //   }
    // }

    // if ( s.rampUpStepCount == 0 ) {
    //   s.n++;
    //   s.d = s.d - (2 * s.d) / (4 * s.n + 1);
    //   if ( s.d <= s.minStepInterval ) {
    //     s.d = s.minStepInterval;
    //     s.rampUpStepCount = s.stepCount;
    //   }
    //   if ( s.stepCount >= s.totalSteps / 2 ) {
    //     s.rampUpStepCount = s.stepCount;
    //   }
    // }
    // else if ( s.stepCount >= s.totalSteps - s.rampUpStepCount ) {
    //   s.d = (s.d * (4 * s.n + 1)) / (4 * s.n + 1 - 2);
    //   s.n--;
    // }

    // s.di = s.d; // integer
    // Serial.print(s.n);
    // Serial.print(' ');
  }
  // Serial.println();

  setNextInterruptInterval();

  TCNT1  = 0;
}

// void runAndWait() {
//   setNextInterruptInterval();
//   while ( remainingSteppersFlag );
// }

void parseSerialMessage(){
    serial_msg_t msg;
    Serial.readBytes((char*)&msg, sizeof(msg));
    Serial.println(String("Got cmd") + ' ' + msg.command +' ' + msg.motor + ' ' + msg.f + ' ' + msg.l);
    switch(msg.command)
    {
    case CMD_MOVE_TO:
        prepareMovement(msg.motor, msg.l);
        break;
    // case CMD_SET_SPEED:
    //     steppers[msg.motor].setSpeed(msg.f);
    //     break;
    // case CMD_SET_ACC:
    //     steppers[msg.motor].setAcceleration(msg.f);
    //     break;
    // case CMD_RESET_POS:
    //     steppers[msg.motor].setCurrentPosition(0);
    //     break;
    default:
        Serial.println("UNKNOWN COMMAND");
        break;
    }
    
}

void loop() {
  if(Serial.available() >= sizeof(serial_msg_t)){
      parseSerialMessage();
  }
  

  // for (int i = 0; i < 4; i++) {
  //   for (int k = 0; k < NUM_STEPPERS; k++) {
  //     prepareMovement( k,  200 );
  //     runAndWait();
  //   }
  // }
  // for (int i = 0; i < 4; i++) {
  //   for (int k = 0; k < NUM_STEPPERS; k++) {
  //     prepareMovement( k,  200 );
  //   }
  //   runAndWait();
  // }
  
  // for (int i = 0; i < NUM_STEPPERS; i++)
  //   prepareMovement( i, 400 );
  // runAndWait();
  // for (int i = 0; i < NUM_STEPPERS; i++)
  //   prepareMovement( i, -400 );
  // runAndWait();
  // for (int i = 0; i < NUM_STEPPERS; i++)
  //   prepareMovement( i, 200 );
  // runAndWait();
  // for (int i = 0; i < NUM_STEPPERS; i++)
  //   prepareMovement( i, -200 );
  // runAndWait();
  // for (int i = 0; i < NUM_STEPPERS; i++)
  //   prepareMovement( i, 600 );
  // runAndWait();
  // for (int i = 0; i < NUM_STEPPERS; i++)
  //   prepareMovement( i, -600 );
  // runAndWait();

  // while (true);

}









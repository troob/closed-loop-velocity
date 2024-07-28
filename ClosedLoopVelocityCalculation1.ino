/*
 * Closed Loop Velocity Calculation
 * Run motors at adjusted voltage,
 * output computed velocity from encoders,
 * and manually time rotations to check computation.
 * Measure in rotations per second, 
 * b/c that's visually easiest to confirm
 */
//======Advisor======
enum state { moving, resting };

state curState; // track current state

boolean motorStart = false;

//======Interface======
String inputString = "";

boolean stringComplete = false;  // whether the string is complete

//======Encoder======
const byte esPin = 3;

int pulsesPerRev = 20;

double minAngularRes,
  pubRotVel = 0.0,
  threshRotVel = 1.0, // [deg/s]
  prevRotPos, // [deg]
  setRotVel; // [deg/s]
  
// values change in callback methods:
volatile int velPulseCount,
  velocity;
  
volatile long pulseCount,
  prevPulseCount;

volatile double latestRotPos, // [deg]
  prevPubRotVel; // [deg/s], PUBLISHER

//======Motor Driver======
const byte mEnablePin = 6,
  mSigPin = 7;

//======Mobile Platform======
double wheelDiam = 6.35,
  wheelBase = 18.0;

//======Circle======
double piApprox = 3.14159,
  rads = 57.2958; // radians to deg conversion

//======Controller======
boolean inAuto = false;

int outMin,
  outMax,
  pubVelTmrCtr,
  maxOutVal; // max. output value

float pubVelRate = 20.0; // [Hz]

const int rollingPts = 2,
  manualControl = 0,
  autoControl = 1,
  directControl = 0,
  reverseControl = 1;

static unsigned long prevTime,
  prevPidTime;

double kp, ki, kd,
  threshIntegral;

double prevPubRotVels[rollingPts] = {};

volatile int pubMtrCmd, // PUBLISHER
  pwmPulse,
  controllerDirection,
  sign,
  motorOutAccum,
  prevVel,
  P, I, D; 

volatile double errRotVel,
  prevErrRotVel,
  sumErrRotVel,
  prevP, prevI, prevD, 
  dP, dI, dD,
  setVel;

volatile boolean forward = true;

void setup() 
{
  initNode("ClosedLoopVelocityCalculation1");

  initVars();

  setParams();

  initSubscribers();

  initPublishers();

  /* "spin()" */
  curState = resting;
  
  prevRotPos = latestRotPos;

  prevTime = millis();
}

void initNode(String id)
{
  Serial.begin(9600);

  while(!Serial);
  
  Serial.print("Starting ");
  Serial.print(id);
  Serial.println(".ino\n");
}

void initVars()
{
  setRotVel = 60.0; // [deg/s]

  pubMtrCmd = 0;

  pubRotVel = 0.0;

  prevPubRotVel = 0.0;

  prevVel = 0; // [pulses/(1/pubVelRate)s]

  errRotVel = 0.0;

  prevErrRotVel = 0.0;

  prevRotPos = 0.0;

  latestRotPos = 0.0;

  prevTime = millis();

  prevPulseCount = 0;
}

void setParams()
{  
  setPIDGains( 1.0, 0.0, 1.0 );

  maxOutVal = 100 * 256; // max. output value in fixed point integer

  setOutputLimits( 50, 255 );

  setMode( autoControl );

  //setControllerDirection( directControl );

  pubVelRate = 10; // [Hz]

  computeMinAngularRes();

  threshRotVel = 3.0;

  threshIntegral = 6.0; // approx from observing that pubMtrCmd b/t ~0-255 below this threshold

  for(int i=0; i < rollingPts; i++)
    prevPubRotVels[i] = 0.0;

  latestRotPos = 0.0; // confusing b/c already set in initVars()

  prevPidTime = millis();

  Serial.print("Controller got kp:");
  Serial.print(kp);
  Serial.print(", ki:");
  Serial.print(ki);
  Serial.print(", kd:");
  Serial.println(kd, 3);
  Serial.println();
}

void setPIDGains(double pg, double ig, double dg)
{
  if(pg < 0 || ig < 0 || dg < 0) return;
  
  kp = pg;

  ki = ig;

  kd = dg;

  if(controllerDirection == reverseControl)
  {
    kp = - kp;

    ki = - ki;

    kd = - kd;
  }
}

void setOutputLimits(int minO, int maxO)
{
  if(minO > maxO) return;

  outMin = minO;
  outMax = maxO;

  if(abs(pubMtrCmd) > outMax) 
  {
    if(pubMtrCmd < 0) pubMtrCmd = - outMax;
    else pubMtrCmd = outMax;
  }
  else if(abs(pubMtrCmd) < outMin) 
  {
    if(pubMtrCmd < 0) pubMtrCmd = - outMin;
    else pubMtrCmd = outMin;
  }

  if(abs(I) > (double) outMax) 
  {
    if(I < 0.0) I = (double) - outMax;
    else I = (double) outMax;
  }
  else if(abs(I) < (double) outMin) 
  {
    if(I < 0.0) I = (double) - outMin;
    else I = (double) outMin;
  }
}

void setMode(int mode)
{
  bool newAuto = (mode == autoControl);

  if(newAuto && !inAuto) // we just went from manual to auto
    initialize();

  inAuto = newAuto;
}

void initialize()
{
  prevPubRotVel = pubRotVel;

  I = pubMtrCmd;

  if(I > outMax) I = outMax;
  else if(I < outMin) I = outMin;
}

void computeMinAngularRes()
{
  minAngularRes = 360.0 / pulsesPerRev;
  
  //pulsesPerDeg = pulsesPerRev / 360.0;
  
  //pulsesPerMeter = (int) ( (double) pulsesPerRev 
    // (piApprox * wheelDiam / 100.0 ) ); // [pulses/m] = (x [pulses/rev])/(\pi*d [m])

  Serial.print("Min. Ang. Res. (deg): ");
  Serial.println(minAngularRes);
  Serial.println();
}

void initSubscribers()
{
  // pulse count
  attachInterrupt(digitalPinToInterrupt(esPin), encoderCallback, CHANGE);
}

void initPublishers()
{
  // motor command
  pinMode(mEnablePin, OUTPUT);
  pinMode(mSigPin, OUTPUT);

  initPubVelTimer(); // ADD: publish rotational (and later translational) velocity
}

void initPubVelTimer()
{
  noInterrupts();           // disable all interrupts
  
  TCCR1A = 0;
  TCCR1B = 0;
  pubVelTmrCtr = 59286;   // preload timer 65536-16MHz/256/2Hz (34286 for 0.5sec) (59286 for 0.1sec)

  
  TCNT1 = pubVelTmrCtr;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  
  interrupts();             // enable all interrupts
}

void stopMoving()
{
  digitalWrite(mEnablePin, LOW);
}

/* "spinOnce()" */
void loop() 
{
  checkUserInput();

  if(Serial.available())
    serialEvent();
}

void checkUserInput()
{
  if(stringComplete)
  {
    Serial.print("inputString: '");

    // receive command from user
    if(inputString.substring(0,5) == "start")
    {
      Serial.print("start");
      
      digitalWrite(mSigPin, HIGH); // LOW is CCW looking at motor face
  
      forward = true;
  
      motorStart = true;
    }
    else if(inputString.substring(0,4) == "stop")
    {
      Serial.print("stop");
      
      stopMoving();
  
      motorStart = false;
    }
    else if(inputString.substring(0,10) == "setrotvel ")
      setRotVel = inputString.substring(10, inputString.length()).toFloat(); // get string after 'setrotvel '
    else if(inputString.substring(0,3) == "kp ")
      kp = inputString.substring(3, inputString.length()).toFloat(); // get string after 'kp '
    else if(inputString.substring(0,3) == "ki ")
      ki = inputString.substring(3, inputString.length()).toFloat(); // get string after 'ki '
    else if(inputString.substring(0,3) == "kd ")
      kd = inputString.substring(3, inputString.length()).toFloat(); // get string after 'kd '

    Serial.println("'");
  
    Serial.print("motorStart: ");
    Serial.println(motorStart);

    // clear string:
    inputString = ""; //note: in code below, inputString will not become blank, inputString is blank until '\n' is received

    stringComplete = false;
  }
}

void calcRotVel()
{
  long dt; // [ms]
  
  double curRotVel;

  dt = millis() - prevTime; // [ms]

  double dtSec = dt / 1000.0;
  
  Serial.print("calcRotVel: dt = ");
  Serial.print(dtSec, 3);
  Serial.print(" s, latestRotPos = ");
  Serial.print(latestRotPos);
  Serial.print(" deg (");
  Serial.print(convertDegToRev(latestRotPos));
  Serial.print(" rev), prevRotPos = ");
  Serial.print(prevRotPos);
  Serial.println(" deg");

  if(dt > 0)
  {
    // we haven't received an updated wheel lately
    if(latestRotPos == prevRotPos)
    {
      curRotVel = 1000.0 * minAngularRes / dt; // if we got a pulse right now, this would be the rot. velocity
  
      // if the velocity is < threshold, consider our velocity 0
      if(abs(curRotVel) < threshRotVel)
      {
        Serial.print("Below threshold: curRotVel=");
        Serial.print(curRotVel);
        Serial.println(" deg/s, pubRotVel = 0.0");
  
        calcRollingVel(0);
      }
      else
      {
        Serial.print("Above threshold: curRotVel = ");
        Serial.print(curRotVel);
        Serial.println(" deg/s");
  
        if(abs(curRotVel) < pubRotVel) // we know we're slower than what we're currently publishing as a velocity
        {
          Serial.println("curRotVel < pubRotVel");
          
          calcRollingVel(curRotVel);
        }
      }
    }
    else // we received a new pulse value
    {
      curRotVel = 1000.0 * ( latestRotPos - prevRotPos ) / dt;
  
//      Serial.print("Received New Pulse Value: curRotVel = ");
//      Serial.print(curRotVel);
//      Serial.println(" deg/s");
        
      calcRollingVel(curRotVel);
      
      Serial.print("*** Pulse Count Updated: pubRotVel = ");
      Serial.print(pubRotVel);
      Serial.println(" deg/s ***");
    
      prevRotPos = latestRotPos;
  
      prevTime = millis(); // [ms]
    }
  }
}

double convertDegToRev(double deg)
{
  return deg / 360.0;
}

void calcRollingVel(double val)
{
//  for(int i=0; i < rollingPts; i++)
//  {
//    Serial.print("prevPubRotVels[");
//    Serial.print(i);
//    Serial.print("]: ");
//    Serial.println(prevPubRotVels[i]);
//  }
  
  for(int i=0; i < rollingPts - 1; i++)
  {
    prevPubRotVels[i] = prevPubRotVels[i + 1];

//    Serial.print("Rolling Vel ");
//    Serial.print(i);
//    Serial.print(" (deg/s): ");
//    Serial.println(prevPubRotVels[i]);
  }

  prevPubRotVels[rollingPts - 1] = val;

//  Serial.print("Rolling Vel ");
//  Serial.print(rollingPts - 1);
//  Serial.print(" (deg/s): ");
//  Serial.println(val);

  pubRotVel = getMean(prevPubRotVels);
}

double getMean(double vals[])
{
  double sum;
  
  for(int i=0; i < rollingPts; i++)
    sum += vals[i];

  return sum / (double) rollingPts;
}

void serialEvent()
{
  while (Serial.available()) 
  {
    // get the new byte:
    char inChar = (char) Serial.read();
    
    // add it to the inputString:
    inputString += inChar;
    
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n')
      stringComplete = true;
  }
}

//======Interrupt Service Routines======
void encoderCallback()
{
  determinePulseCount(); // increment if forward, decrement if backward // OLD: pulseCount++;
    
  latestRotPos = (double) pulseCount * minAngularRes;
    
  prevPulseCount = pulseCount;
}

/* The variable velPulseCount is read and zeroed by the
 * "ISR(TIMER1_OVF_vect)" routine, 
 * running 10 (change to maybe 20) times a second, which is
 * the sensor loop and update rate for this robot.
 * This routine copies the accumulated counts from
 * velPulseCount to the variable "velocity" and
 * then resets velPulseCount to zero.
 */
void determinePulseCount()
{
  if(forward)
  {
    pulseCount++;
    velPulseCount++;
  }
  else
  {
    pulseCount--;
    velPulseCount--;
  }

  Serial.print("\n\nPulse Count: ");
  Serial.print(pulseCount);
  Serial.print(", Vel. Pulse Count: ");
  Serial.print(velPulseCount);
  Serial.println("\n");
}

/* Run at x (maybe 10-20) Hz in sensor loop, 
 * interrupt service routine - tick every 0.1 s (10 Hz)
 * Read and zero velPulseCount.
 * Copy and accumulate counts from velPulseCount
 * to the variable "velocity" and
 * then reset velPulseCount to zero.
 */
ISR(TIMER1_OVF_vect) // speedometer(?)
{
  TCNT1 = pubVelTmrCtr; // set timer

  // Read analog input:
  velocity = velPulseCount * sign; // copy and accumulate counts from velPulseCount to velocity
  velPulseCount = 0; // reset velPulseCount to zero

  // Compute control signal and set analog output:
  controlVel();

  // Update controller variables (happens async?)
}

void controlVel()
{
  // Compute control signal:
  setVel = setRotVel / ( minAngularRes * pubVelRate ); // convert [deg/s] to [pulses/(1/pubVelRate)s]
  computeControlSignal(setVel); 

  // Set analog output:
  modulatePulseWidth((int) round( motorOutAccum / 256 ) ); // Note: divide by 256 and earlier multiply by 256 b/c earlier operation in fixed point integer
}

/* Basic behavior: generate error signal 
 * based on difference b/t measured velocity and
 * requested velocity for the wheel.
 * Use of error signal: Increase or decrease speed of motor
 * to force measured to equal requested velocity
 * Input to PID controller: Requested velocity "vel,"
 * which is input velocity expressed as encoder pulses
 * per 1/pubVelRate second.
 */
void computeControlSignal(double setPoint)
{
  int err;
  
  double b = 1.0; // set point weight
  
  Serial.print("Controller got kp:");
  Serial.print(kp);
  Serial.print(", ki:");
  Serial.print(ki);
  Serial.print(", kd:");
  Serial.println(kd, 3);
  Serial.println();

  if(motorStart)
  {
    Serial.print("setPoint: ");
    Serial.print(setPoint);
    Serial.print(", velocity: ");
    Serial.print(velocity);
    
    err = (int) round( ( b * setPoint - (double) velocity ) * 256 ); // [pulses/(1/pubVelRate)s]*256, generate error signal based on difference b/t measured velocity and requested velocity for the wheel. Note: multiply by 256 and later divide by 256 b/c operation in fixed point integer
    Serial.print(", err (pulses per 25.6 sec): ");
    Serial.println(err);
    
    P = (int) round( err / kp ); // P(t_k) = K(by_{sp}(t_k) — y(t_k))
    Serial.print("P: ");
    Serial.println(P);

    D = (int) round( ( ( velocity - prevVel ) * 256 ) / kd ); // large when amount of change requested by PID controller is large, and small as signal approaches zero
    Serial.print("D: ");
    Serial.println(D);

    motorOutAccum += P + D; // Increase or decrease speed of motor to force measured to equal requested velocity

    prevVel = velocity; // maintain history of previous measured velocity

    clipAccumulator(motorOutAccum, maxOutVal); // accumulator
    
    //dP = P - prevP;
    //prevP = P;

    //sumErrRotVel += errRotVel; // sum of error
    
//    if(abs(errRotVel) < threshIntegral)
//      I += ( ki / pubVelRate * errRotVel );
//    else
//      I = 0.0;
//
//    if(I < 0.0)
//      setControllerDirection(reverseControl);
//    else
//      setControllerDirection(directControl);
//      
//    if(abs(I) > (double) outMax) 
//    {
//      if(I < 0.0) I = (double) - outMax;
//      else I = (double) outMax;
//    }
//    else if(abs(I) < (double) outMin) 
//    {
//      if(I < 0.0) I = (double) - outMin;
//      else I = (double) outMin;
//    }
//
//    dI = I - prevI;
//    prevI = I;
//    
    
//    dD = D - prevD;
//    prevD = D;
    
    //pwmPulse += dP + dI + dD;

  
    
    //prevErrRotVel = errRotVel; // save last (previous) error
  
    // update new velocity
//    if(abs(pwmPulse) > outMax) 
//    {
//      if(pwmPulse < 0) pwmPulse = - outMax;
//      else pwmPulse = outMax;
//    }
//    else if(abs(pwmPulse) < outMin) 
//    {
//      if(pwmPulse < 0) pwmPulse = - outMin;
//      else pwmPulse = outMin;
//    }
  }
  else
  {
    err = 0;

//    prevErrRotVel = 0.0;
//
//    sumErrRotVel = 0.0;

    motorOutAccum = 0;

    prevVel = 0; // maintain history of previous measured velocity
  }

  if(setRotVel == 0.0)
    motorOutAccum = 0;  
}

/* The accumulator motorOutAccum must be clipped 
 * at some positive and negative value
 * to keep from overflowing the fixed point arithmetic.
 */
void clipAccumulator(int a, int maximum)
{
  Serial.print("Computed motorOutAccum: ");
  Serial.print(a);
    
  if(a > maximum) 
    a = maximum;
  else if(a < -maximum) 
    a = -maximum;

  Serial.print(", Clipped motorOutAccum: ");
  Serial.println(a);
}

/* The PWM code drives the hardware H-bridge, 
 * which actually control the motor.
 * This routine takes a signed value, 
 * -100 < signedVal < 100,
 * sets the sign variable used by the speedometer code,
 * sets the forward/backward (i.e. direct/reverse) bits 
 * on the H-bridge, and
 * uses abs(signedVal) as an index into a 100 entry table 
 * of linear PWM values.
 * This function uses a timer interrupt to generate 
 * a x Hz (maybe 120 Hz) variable pulse-width output.
 */
void modulatePulseWidth(int signedVal) // take signed value, b/t -100 and 100
{
  setSpeedometerSign(signedVal); // set sign variable used by speedometer code

  setHBridgeDirectionBit(signedVal);

  setPWMValueFromEntryTable(abs(signedVal)); // use abs(signedVal) as an index into a 100 entry table of linear PWM values

  analogWrite(mEnablePin, pubMtrCmd); // generate variable pulse-width output
}

/* The sign variable represents the direction of rotation
 * of the motor (1 for forward and -1 for backward).
 * With more expensive quadrature encoders this info is
 * read directly from the encoders.
 * In this implementation I have only simple encoders so
 * the direction of rotation is taken from the sign of the
 * most recent command issued by the PID to the PWM code.
 */
void setSpeedometerSign(int signedVal) // signedVal should be most recent cmd issued by PID to PWM code
{
  if(signedVal < 0) // {motor direction of rotation} = backward
    sign = -1;
  else if(signedVal >= 0)
    sign = 1; // {motor direction of rotation} = {forward | resting}
  else
    Serial.println("Invalid command issued by PID to PWM code!\n");
}

void setHBridgeDirectionBit(int signedVal)
{
  if(signedVal < 0) // {motor direction of rotation} = backward
    digitalWrite(mSigPin, LOW);
  else if(signedVal >= 0)
    digitalWrite(mSigPin, HIGH); // {motor direction of rotation} = {forward | resting}
  else
    Serial.println("Invalid command issued by PID to PWM code!\n");
}

// use magnitude as an index into a 100 entry table of linear PWM values
void setPWMValueFromEntryTable(int magnitude)
{
  pubMtrCmd = (int) round( magnitude * 255 / 100 );
}

//void calcMtrCmd()
//{  
//  if(!inAuto) return;
//  
//  Serial.print("calcMtrCmd: pubRotVel = ");
//  Serial.print(pubRotVel);
//  Serial.print(" deg/s, setRotVel = ");
//  Serial.print(setRotVel);
//  Serial.println(" deg/s");
//
//  pubMtrCmd = getControlVar();
//
//  Serial.print("pubMtrCmd (0-255, F|B): ");
//  Serial.print(pubMtrCmd);
//  if(!forward)
//    Serial.print(" backward");
//  Serial.println("\n");
//}
//
//int getControlVar()
//{
//  int b = 1; // set point weight
//  
//  Serial.print("Controller got kp:");
//  Serial.print(kp);
//  Serial.print(", ki:");
//  Serial.print(ki);
//  Serial.print(", kd:");
//  Serial.println(kd, 3);
//  Serial.println();
//
//  if(motorStart)
//  {
//    errRotVel = b * setRotVel - pubRotVel; // [deg/s], 
//    Serial.print("errRotVel (deg/s): ");
//    Serial.println(errRotVel);
//    P = kp * errRotVel; // P(t_k) = K(by_{sp}(t_k) — y(t_k))
//    dP = P - prevP;
//    prevP = P;
//
//    //sumErrRotVel += errRotVel; // sum of error
//    
//    if(abs(errRotVel) < threshIntegral)
//      I += ( ki / pubVelRate * errRotVel );
//    else
//      I = 0.0;
//
//    if(I < 0.0)
//      setControllerDirection(reverseControl);
//    else
//      setControllerDirection(directControl);
//      
//    if(abs(I) > (double) outMax) 
//    {
//      if(I < 0.0) I = (double) - outMax;
//      else I = (double) outMax;
//    }
//    else if(abs(I) < (double) outMin) 
//    {
//      if(I < 0.0) I = (double) - outMin;
//      else I = (double) outMin;
//    }
//
//    dI = I - prevI;
//    prevI = I;
//    
//    D = kd * pubVelRate * ( pubRotVel - prevPubRotVel );
//    dD = D - prevD;
//    prevD = D;
//    
//    pwmPulse += dP + dI + dD;
//
//    prevPubRotVel = pubRotVel;
//    
//    prevErrRotVel = errRotVel; // save last (previous) error
//  
//    // update new velocity
//    if(abs(pwmPulse) > outMax) 
//    {
//      if(pwmPulse < 0) pwmPulse = - outMax;
//      else pwmPulse = outMax;
//    }
//    else if(abs(pwmPulse) < outMin) 
//    {
//      if(pwmPulse < 0) pwmPulse = - outMin;
//      else pwmPulse = outMin;
//    }
//  }
//  else
//  {
//    errRotVel = 0.0;
//
//    prevErrRotVel = 0.0;
//
//    sumErrRotVel = 0.0;
//
//    pwmPulse = 0;
//  }
//
//  if(setRotVel == 0.0)
//    pwmPulse = 0;
//
//  //setSign(pwmPulse); // note: fcn probably better placed elsewhere; represent direction of rotation of motor
//  
//  return abs(pwmPulse);
//}
//
//void setControllerDirection(int d)
//{
//  if(d == directControl)
//  {
//    digitalWrite(mSigPin, HIGH);
//
//    forward = true;
//
//    
//  }
//  else
//  {
//    digitalWrite(mSigPin, LOW);
//
//    forward = false;
//  }
//  
//  controllerDirection = d;
//
//  setPIDGains(kp, ki, kd);
//}

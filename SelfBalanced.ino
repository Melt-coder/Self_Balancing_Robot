#include <PinChangeInterrupt.h>
#include "Motor.h"
#include "Encoder.h"
#include "Sensors.h"
#include "PID.h"
#include "LQR.h"
#include "RobotState.h"
#include <MPU6050_tockn.h>
#include <Wire.h>

Sensors sensorObj;
PID control;
LQR control_lqr;

// To determine which controller will run: 1=LQR, 0=PID
#define USE_LQR 1
#define TEST_C_MODE 1 // 1 =test c, 0 normal(test a/b mode)

//Encoder tick to metre conversion constant(motor datasheet: 26 pulses/revolution *30 gear ratio *2 edges = 1560 tick/revolution; wheel circumference 2*pi*r /1560 tick)
#define TICK_TO_METRE 1.3104e-4
#define TORQUE_to_PWM_ratio 400.6 // 255/0.6374, LQR/PID the same constant in its class (control_torque)

float current_angle   = 0.0;
float target_angle    = 0.0; 

// Encoder
long  LastEncoderCount = 0;
long  targetPosition   = 0.0;  // The position reference point is set during setup 
float robotVelocity    = 0.0;
float velocityLowPass  = 0.0;

// Timing
unsigned long LastTime          = 0;   // Timer
unsigned long previous_time     = 0;   
unsigned long balanceStartTime  = 0;   // The moment the balancing actually begins

// Safety limit
// When the angle deviates by this many degrees from the calibrated target_angle, the motor stops
// This means the robot has already tripped over; trying to rescue it is pointless and risky
const float FALL_LIMIT = 25.0;
bool robotFallen = false;

// Experimental protocol constants (For TEST A/B/D)
const unsigned long T_MAX_MS =60000; // Experimental upper limit 60 sec
bool experimentEnded = false; // When it reaches 60 sec, it prints "SUCCESS" once

// Voltaj Measurement (Elegoo's Method)
// There is a voltage divider connected to pin A2 (R1=10 kiloohm, R2=1.5 kiloohm), and the internal 1.1 V reference is used
// V_battery = V_A2 * (10 + 1.5) / 1.5 = V_A2 * 7.667
#define VOLTAGE_PIN      A2
#define V_BATT_HI        7.8   // It operates nomarlly above this value
#define V_BATT_LO        6.7   // Below this value is critical; it shuts down the motor
#define ADC_VREF         1.1   // Internal reference
#define V_DIVIDER_RATIO  ((10.0 + 1.5) / 1.5)  // = 7.667

float readBatteryVoltage() {
  const int SAMPLES = 8;      // Take the average of 8 samples instead of a single reading (to eliminate the transient noise from the motor PWM switching)
  long sum = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sum += analogRead(VOLTAGE_PIN);
  }
  float raw = sum / (float)SAMPLES;
  float v_pin = (raw / 1024.0) * ADC_VREF;
  float voltage = v_pin * V_DIVIDER_RATIO;

  // Plausibility check: A 2S Li-ion typically ranges from about 6.0V (discharged) to about 8.4V (fully charged)
  // A reading that falls outside this range indicates a measurement error. Instead of using such a value, it is rejected, and the last valid value is preserved
  static float lastGoodVoltage = V_BATT_HI;
  if (voltage < 5.5 || voltage > 9.0) {
    return lastGoodVoltage;
  }
  lastGoodVoltage = voltage;
  return voltage;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  analogReference(INTERNAL);  
  pinMode(VOLTAGE_PIN, INPUT);
  sensorObj.Init_MPU(); // calcGyroOffsets(true) already performance calibration
  PinEnable();
  Encoder_Pin_Enable();
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_LEFT),  ISR_LEFT,  CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(ENCODER_PIN_RIGHT), ISR_RIGHT, CHANGE);

  // Auto-Calibration
  Serial.println(("Hold the robot UPRIGHT! Calibration will begin in 1 second...")); // It provides the avrrage true heading over a 1-second interval
  delay(500); 
  float angleSum, gyroSum, angleMin, angleMax, calibSpread, gyroBias;
  const int MAX_CALIB_ATTEMPTS = 5; // If the calibration is poor, it provides the average of 5 measurements
  int attempt = 0;
  do {                      // do while loop
    attempt++;
    angleSum = 0.0;
    gyroSum  = 0.0;
    angleMin = 999.0;
    angleMax = -999.0;
    for (int i = 0; i < 100; i++) {
      float a = sensorObj.Angle_MPU();
      gyroSum += sensorObj.Gyro_MPU();
      angleSum += a;
      if (a < angleMin) angleMin = a;
      if (a > angleMax) angleMax = a;
      delay(10);
    }
    calibSpread = angleMax - angleMin; 
    gyroBias    = gyroSum / 100.0; // The average of 100 gyroscope readings
    if (calibSpread > 3.0 && attempt < MAX_CALIB_ATTEMPTS) { // If the difference betwen the largest and smallest angles in 100 measurements is greater than 3 degrees & teh current number of trials is less than 5
      Serial.print("The calibration error is too high! Hold the robot more steadily!") ; 
      Serial.print("Deviation: ");
      Serial.print(calibSpread,2); // The number 2 indicates how many digits there will be after the decimal point
      Serial.print("Trial: ");
      Serial.println(attempt); 
      Serial.println("Trying again");
      delay(500);
    }
  } while (calibSpread > 1.5 && attempt < MAX_CALIB_ATTEMPTS); // If the difference betwen the largest and smallest angles in 100 measurements is greater than 1.5 degrees & teh current number of trials is less than 5
  target_angle = angleSum / 100.0; // The average of 100 angle readings
  Serial.print("Gyroscope drift(degrees/second; should be close to zero): ");
  Serial.println(gyroBias, 3); // The number 3 indicates how many digits there will be after the decimal point
  if (abs(gyroBias) > 0.5) {  // gyroBias is an absolute value(whether it is positive or negative does not matter)
    Serial.println(("WARNING:The gyroscope's zero point may have drifted! The theta_dotterm may be constantly pushing in one direction"));
  }
  Serial.print("Calibration deviation (min-max): ");
  Serial.println(calibSpread, 2);
  if (calibSpread > 1.5) { // If the deviation had fallen below 1.5 degrees, it would not have entered this cycle.
    Serial.println(("WARNING:The robot was not held steady enough during calibration!"));
    Serial.println("Suggestion: Lean the robot against a flat wall or book and try again!");
  }

  // Check the battery voltage
  float vBat = readBatteryVoltage();
  Serial.print("Battery Voltage: ");
  Serial.print(vBat, 2); // The number 2 indicates how many digits there will be after the decimal point
  Serial.println("Volt");
  if (vBat < V_BATT_LO && vBat > 0.5) { 
    Serial.println("WARNING:Battery is critically low! Please charge it!");
  }
  Serial.print(">>> target_angle = ");
  Serial.println(target_angle, 4); // The number 4 indicates how many digits there will be after the decimal point
  Serial.println("Calibration complete!");

  
  const float TARGET_TRIGGER_DEG= 9.6; // Initial slope value(degrees) for the experiment. It is set manually, taking into account the target angle and the controller.
  // For this structure,if TEST_C_MODE 1(TEST C), the section up to #else is compiled.If it is 0 (TEST A/B), the section between #else and #endif is compiled.
  #if TEST_C_MODE
   Serial.println("TEST C MODE: Wait until the robot has stabilized.");
   Serial.println(("While the robot is balanced,give it a gentle push whenever you want."));
   Serial.println(("When promted, type 'n' in Serial and press Enter."));  
   // If there is no calm/trigger cycle, the system proceeds directly to the TRIAL START
  #else // In TESTA/B modes, the robot is expected to remain within a +/- 2.5-degree range for 500 ms.The user tilts the robot,and the test begins once the TARGET_TRIGGER_DEG threshold is crossed.In TEST C mode, these wait loops are skipped.
   const float TRIGGER_ANGLE_DEG = TARGET_TRIGGER_DEG;
   const float SETTLE_BAND_DEG= 2.5; //Before it was triggered,the robot was within this range; When I held it in my hand, 1 degree felt too tight,so I adjusted it to 2.5
   const unsigned long SETTLE_MS= 500; // The robot must remain calm for at least this long.
   const unsigned long SETTLE_TIMEOUT_MS= 15000; //If it does not calm down 15 sec,do not force it.
   Serial.println("STAND THE ROBOT UPRIGHT AND HOLD IT STEADY");
   Serial.println("Slowly tilt the robot until it reaches the approximate target angle.");

   float triggerTheta; // Angle measured in the slope cycle
   unsigned long settleStart=0; // the moment the settling period begins
   unsigned long settleWaitStart=millis(); // The 15-second timeout begins the moment it enters loop
   bool settled=false; // Flag signaling the end of the calm cycle
   unsigned long lastPrint=0;
   while(!settled){ 
     float a = sensorObj.Angle_MPU()-target_angle; // the difference between the filtered angle and target_angle
     if (abs(a)<SETTLE_BAND_DEG){ // The timers starts
       if (settleStart==0)
         settleStart=millis();
       if (millis()-settleStart>=SETTLE_MS) // If it remains within the band for 500 ms
         settled=true;
     }
     else{
       settleStart=0;
     }
     if (millis() - lastPrint >= 1000) { // About once per second, the current deviation value is displayed on the screen so we can see how stable the angle is
       lastPrint = millis();
       Serial.print("Current deviation(deg): "));
       Serial.println(a, 2);
     }
     //If they can't calm down in 15 seconds,continue without forcing them(this prevents an endless)
     if (millis() - settleWaitStart >= SETTLE_TIMEOUT_MS) { // If 15 seconds have passed and person still has not calmed down, proceed by force
       Serial.println(("The statute of limitations is being extended,the requirement for calm is being relaxed,and proceedings are continuing."));
       settled = true;
     }
     delay(5); 
   }
   Serial.println("The wait is over. Tilt the robot; trigger treshold(degrees): ");
   Serial.println(TRIGGER_ANGLE_DEG,1);
  do {
     triggerTheta = sensorObj.Angle_MPU() - target_angle;
     delay(5);
  } while (abs(triggerTheta) < TRIGGER_ANGLE_DEG); // If the slope exceeds the target slope, the loop ends and the experiment begins
  #endif

  // Trial protocol: Trial start band
  Serial.println("Trial start");
  Serial.print("Controller:"); 
  if(USE_LQR){
    Serial.println("LQR: ");
  }else{
    Serial.println("PID: ");
  } 

  Serial.print("target_angle (deg): "); 
  Serial.println(target_angle,4);
  Serial.println("Note: initial_angle,trial_id,experiment_type are provided externally");
  Serial.println("(Trials)");
  Serial.println("time,theta_deg,theta_dot_deg_s,x_m,xdot_m_s,control_pwm,control_torque_Nm,battery_voltage,fallen");


  // Use the starting position as a reference
  targetPosition = Encoder_Count_Left + Encoder_Count_Right;

  // Reset the timers after setup is complete
  unsigned long now = millis();
  LastTime      = now;
  previous_time = now;
  balanceStartTime = now;
}



void loop() {
  if (Serial.available()){ // Serial.available() returns the number of charcters that have been received from the computer but not yet been read
    char c= Serial.read();
    if (c == 'n' || c == 'N') {
      Serial.print(">>> NUDGE_MARK t=");
      Serial.println((millis()-balanceStartTime)/1000.0, 3);
    }
  }

  unsigned long CurrentTime = millis();  

  if (CurrentTime - LastTime >= 10) { // control loop at approximately 100 Hz

    float dt= (CurrentTime- previous_time)/1000.0;// dt: time eclapsed since the last call
    float Motor_Output = 0.0; // externally, so it can access the debug/log line
    RobotState state = {0,0,0,0}; //{x_m,xdo_ms,theta_rad,thetadot_rads} // Create a real box named "state" according to the RobotState definition
    

    // Encoder Speed
    long CurrentEncoderCount = Encoder_Count_Left + Encoder_Count_Right;

    unsigned long nowMicros = micros();
    unsigned long intervalLeft, intervalRight, lastPulseLeft, lastPulseRight;
   
    noInterrupts();
    intervalLeft   = Encoder_PulseInterval_Left;
    intervalRight  = Encoder_PulseInterval_Right;
    lastPulseLeft  = Encoder_LastPulseTime_Left;
    lastPulseRight = Encoder_LastPulseTime_Right;
    interrupts();

    
    const unsigned long STALE_US = 200000; // If no pulse is received for 200000 microseconds=200ms, the wheel is condidered to have truly stopped
    const unsigned long MIN_PLAUSIBLE_US = 500; // Lower limit for bounce noise
    if (intervalLeft  < MIN_PLAUSIBLE_US) intervalLeft  = MIN_PLAUSIBLE_US;
    if (intervalRight < MIN_PLAUSIBLE_US) intervalRight = MIN_PLAUSIBLE_US;

    float speedLeft = 0.0, speedRight = 0.0;
    if (nowMicros - lastPulseLeft < STALE_US) {
      speedLeft = 1000000.0 / (float)intervalLeft;  // pulse/sec
      if (digitalRead(DIR_PIN_LEFT) == HIGH) speedLeft = -speedLeft;
    }
    if (nowMicros - lastPulseRight < STALE_US) {
      speedRight = 1000000.0 / (float)intervalRight;
      if (digitalRead(DIR_PIN_RIGHT) == HIGH) speedRight = -speedRight;
    }

    //Average Speed
    robotVelocity = ((speedLeft + speedRight)/2.0) * 0.01; //Average speed, (pulse/sec to pulse/10ms)
    LastEncoderCount = CurrentEncoderCount; // A reference for the position, speed no longer uses this
   
    velocityLowPass = (velocityLowPass * 0.90) + (robotVelocity * 0.10); // Low-pass filter(noise suppression)
    
    //Angle measurement
    current_angle = sensorObj.Angle_MPU(); // complementary filter
    float gyroRate = sensorObj.Gyro_MPU(); // for thetadot raw gyroscope

   //  Battery Voltage(every 50 cycles-500 ms)
    static uint8_t voltageCounter = 0; //A counter ranging from 0 to 50; 1 byte is sufficient(uint8_t)(an int would take up 2 bytes)
    static float   batteryVoltage = V_BATT_HI; // static: The last measured voltage is stored for 50 cycles( 500 ms), then updated
    if (++voltageCounter >= 50) {
      voltageCounter  = 0;
      batteryVoltage  = readBatteryVoltage();
      if (batteryVoltage < V_BATT_LO && batteryVoltage > 0.5) {
        Serial.println("WARNING: Battery is low!Please charge it!");
      }
    }

    //Average position error
    float positionError = constrain((float)(CurrentEncoderCount - targetPosition)/ 2.0, -8000.0, 8000.0); // Average encoder position error

    
    // Security control: Wheter the robot fallen or not
    float angleFromVertical = current_angle - target_angle;
    if (abs(angleFromVertical) > FALL_LIMIT) {
      if (!robotFallen) {
        unsigned long fallTime = millis()-balanceStartTime;
        Serial.print("RESULT: FAIL fall_time_ms="); Serial.println(fallTime);
      }
      robotFallen = true;
    }
    // If the robot is returned to an upright position, it will resume operation automatically
    if (robotFallen && abs(angleFromVertical) < (FALL_LIMIT * 0.5)) {
      robotFallen = false;
    }

    //Experimental duration: Whether 60 secs have elapsed or not
    if (!robotFallen && !experimentEnded && (millis()-balanceStartTime)>= T_MAX_MS){
        experimentEnded =true;
        Serial.print ("RESULT:Success duration_ms= 60000");
    }

    if (robotFallen) {
      Stop();
      control.ResetState();  
    } else {
      
      state.theta_rad = (current_angle-target_angle)*PI /180.0; // The "target_angle" value obtained through calibration is used to determine the raw angle. // Place this calculated value in the "theta_rad" field in the "state" box
      state.thetadot_rads = gyroRate*PI /180.0;
      state.x_m = positionError* TICK_TO_METRE;
      state.xdot_ms = velocityLowPass* 100* TICK_TO_METRE;

      #if USE_LQR
         Motor_Output = control_lqr.LQR_Compute(state); // It is shipped in a single box; the contents are already labeled
      #else
         Motor_Output = control.PID_Compute(state,dt);

      #endif
        Motor_Control(Motor_Output);
        }


    // Debug — CSV data log (20Hz)
    static uint8_t debugCounter = 0;
    if (++debugCounter >= 5) { // 100Hz/5=20Hz logging rate
      debugCounter = 0;
      float t_sec = (CurrentTime-balanceStartTime)/1000.0;
      Serial.print(t_sec, 3); 
      Serial.print(",");
      Serial.print(state.theta_rad*180.0/PI,4); 
      Serial.print(",");
      Serial.print(state.thetadot_rads*180.0/PI,3); 
      Serial.print(",");
      Serial.print(state.x_m,5); 
      Serial.print(",");
      Serial.print(state.xdot_ms,4); 
      Serial.print(",");
      Serial.print(Motor_Output,1); 
      Serial.print(",");
      Serial.print(Motor_Output/TORQUE_to_PWM_ratio,4); 
      Serial.print(",");
      Serial.print(batteryVoltage,2);
      Serial.print(",");
      if (robotFallen){
        Serial.println(1); 
      }else{
        Serial.println(0); 
      }
    }

    previous_time = CurrentTime;  
    LastTime      = CurrentTime;
  }
}
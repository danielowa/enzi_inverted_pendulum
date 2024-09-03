//***********
//* MPU6050 *
//***********
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

//********
//* PID *
//********
#include <AutoPID.h>

double current_angle;
double filtered_angle = 0.0;  // Initialize filtered_angle to a reasonable starting value

float alpha = 0.98;  // Filter coefficient, controls the amount of filtering

double angle, target_angle, wheelspeed;
#define WHEELSPEED_MIN -30000
#define WHEELSPEED_MAX  30000

#define ANGLE_KP 300              
#define ANGLE_KI 0
#define ANGLE_KD 0

AutoPID angle_pid(&angle, &target_angle, &wheelspeed, WHEELSPEED_MIN, WHEELSPEED_MAX, ANGLE_KP, ANGLE_KI, ANGLE_KD);

//**************************
//* TMC2208 (Motor Driver) *
//**************************
#include <SoftwareSerial.h>
SoftwareSerial xSerial(2,5);
SoftwareSerial ySerial(3,6);

#include <TMC2208Stepper.h>							
TMC2208Stepper x_driver = TMC2208Stepper(&xSerial);	
TMC2208Stepper y_driver = TMC2208Stepper(&ySerial);	

#define EN_PIN    8
#define RED_BUTTON 7

unsigned long last_control_time = 0;
bool enable = LOW; 
int prev_red_button_val = HIGH;

void setup() {
  //**************
  //* Red Button *
  //**************
  pinMode(RED_BUTTON, INPUT_PULLUP);

  //***********
  //* MPU6050 *
  //***********
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  while (!Serial);
  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  angle_pid.setTimeStep(10);

  //**************************
  //* TMC2208 (Motor Driver) *
  //**************************
  xSerial.begin(115200);
  while(!xSerial);
	ySerial.begin(115200);
  while(!ySerial);

	pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);

	x_driver.pdn_disable(1);
	x_driver.I_scale_analog(0);
	x_driver.rms_current(900);
  x_driver.en_spreadCycle(0);

	y_driver.pdn_disable(1);
	y_driver.I_scale_analog(0);
	y_driver.rms_current(900);
  y_driver.en_spreadCycle(0);

  x_driver.VACTUAL(0.0);
  y_driver.VACTUAL(0.0);

	digitalWrite(EN_PIN, !enable);

  last_control_time = millis();
}

void loop() {
  int red_button_val = digitalRead(RED_BUTTON);
  if(prev_red_button_val == HIGH && red_button_val == LOW){
    digitalWrite(EN_PIN, enable);
    enable = !enable;
  }
  prev_red_button_val = red_button_val;

  if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    
    angle = ypr[1] * 180 / M_PI;

    current_angle = ypr[1] * 180 / M_PI;

    // Apply the low-pass filter
    filtered_angle = alpha * filtered_angle + (1 - alpha) * current_angle;

    // Use filtered_angle for further processing
    angle = filtered_angle;
  }

  // Calculate control step
  unsigned long now = millis();
  if (now - last_control_time >= 10) { // 100 Hz control loop
    last_control_time = now;
    
    // Adjust target_angle based on cart position
    target_angle = 0.0 + 0.02 * wheelspeed; // Modify the null angle slightly based on speed
    
    // Run PID to adjust wheelspeed
    angle_pid.run();
    
    x_driver.VACTUAL(-wheelspeed);
    y_driver.VACTUAL(+wheelspeed);

    Serial.print("angle:");
    Serial.print(angle);
    Serial.print(",");
    Serial.print("target_angle:");
    Serial.print(target_angle);
    Serial.print(",");
    Serial.print("wheelspeed:");
    Serial.println(wheelspeed);
  }
}
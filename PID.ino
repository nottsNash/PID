//class includes
#include "Calibrate.h"
#include "motors.h"
#include "linesensor.h"
#include "encoders.h"
#include "kinematics.h"
#include "pid.h"

//loop timer controls
# define LINE_SENSOR_UPDATE 50
# define MOTOR_UPDATE       200
# define ENCODER_UPDATE     200
# define PID_UPDATE         20

//motors
# define L_PWM_PIN 10
# define L_DIR_PIN 16
# define R_PWM_PIN 9
# define R_DIR_PIN 15

// declaration on class instances
Motors_c  leftMotor;
Motors_c  rightMotor;
Motors_c MotorsGeneral;
LineSensor_c  LineSensors;
Calibrate_c CalibrationClass;
PID_c  pidClass;

//change
//------------Global variables--------------//
unsigned long current_ts = 0;
unsigned long encoder_ts = 0;
unsigned long motor_ts = 0;
unsigned long PID_ts = 0;

int leftPWM = 0; //PWM values used to control the power going to the motors
int rightPWM = 0; //values sent to motors must be int's but calculations done with floats
int leftPWMfloat = 0;
int rightPWMfloat = 0;
int maxPWM = 100;
float LeftVelocity = 0;
float RightVelocity = 0;

float RotationsRight = 0; //number of rotations measured by the encoders on the right wheel
float RotationsLeft = 0;
float DistanceRight = 0; //distance travelled by the right wheel
float DistanceLeft = 0;
float WheelCircumference = 100.530964; //cicumference of wheels in mm
float WheelLocation = 47; //distance from wheel centre to robot centre in mm

float XrContribution = 0;
float YrContribution = 0;
float AngleContribution = 0;
float Xg = 0;
float Yg = 0;
float localAngle = 0;
float GlobalAngle = 0;

float FeedbackSignal = 0;
float TargetVelocity = 1;

// ----------------------------put your setup code here, to run once:
void setup() {

  Serial.begin(9600);
  delay(4000);
  Serial.println("***RESET***");

  setupEncoder0();
  setupEncoder1();
  
  leftMotor.initialise(L_PWM_PIN,L_DIR_PIN); //inititalise motors with correct pin layout
  rightMotor.initialise(R_PWM_PIN,R_DIR_PIN);
  
  pidClass.initialisePIDs();
  
  }

//-------------------------------------------------------------loop-----------------------------------------------------------------------------------  
void loop() {
 
  unsigned long elapsed_ts;
  current_ts = millis();
  
 
  elapsed_ts = current_ts - PID_ts;
  if( elapsed_ts > PID_UPDATE ){
    FeedbackSignal = updatePID(TargetVelocity, LeftVelocity);
  }




  elapsed_ts = current_ts - encoder_ts;
  if( elapsed_ts > ENCODER_UPDATE ){
  RotationsRight = (count_encoder_right/358.3)*-1;
  RotationsLeft = (count_encoder_left/358.3)*-1;
  DistanceRight = RotationsRight * WheelCircumference; //in mm
  DistanceLeft = RotationsLeft * WheelCircumference; //in mm
  
  XrContribution = (DistanceLeft/2) + (DistanceRight/2);
  AngleContribution = (DistanceLeft/(2*WheelLocation)) - (DistanceRight/(2*WheelLocation));
  Xg = Xg +(XrContribution*cos(GlobalAngle));
  Yg = Yg + (XrContribution*sin(GlobalAngle));
  GlobalAngle = GlobalAngle + AngleContribution;

  LeftVelocity = (DistanceLeft/(float)ENCODER_UPDATE*1000)*1; // mm/second
  RightVelocity = (DistanceRight/(float)ENCODER_UPDATE*1000)*1; // mm/second
  Serial.print("LeftVelocity: ");
  Serial.print(LeftVelocity);
  Serial.print("\n");

  count_encoder_right = 0;
  count_encoder_left = 0;

  encoder_ts = millis();
  }
  
  elapsed_ts = current_ts - motor_ts;
  if( elapsed_ts > MOTOR_UPDATE ) {
     //proportional controller used for line following
     leftPWM = FeedbackSignal;
     rightPWM = 0;
     leftPWM = leftMotor.rescaler(leftPWM, maxPWM); //scales values greater than PWM value 50 to 50 for the left motor only 
     rightPWM = rightMotor.rescaler(rightPWM, maxPWM);
 
     rightMotor.Direction(rightPWM, R_DIR_PIN, R_PWM_PIN);
     leftMotor.Direction(leftPWM, L_DIR_PIN, L_PWM_PIN); //reverses the motor direction for negative values of PWM
     motor_ts = millis();
  }

  
  
}

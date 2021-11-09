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


// declaration on class instances
Motors_c  leftMotor;
Motors_c  rightMotor;
Motors_c MotorsGeneral;
LineSensor_c  LineSensors;
Calibrate_c CalibrationClass;


//------------Global variables--------------//
unsigned long current_ts = 0;
unsigned long encoder_ts = 0;

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


// ----------------------------put your setup code here, to run once:
void setup() {

  Serial.begin(9600);
  delay(4000);
  Serial.println("***RESET***");

  setupEncoder0();
  setupEncoder1();
  
  }

//-------------------------------------------------------------loop-----------------------------------------------------------------------------------  
void loop() {
 
  unsigned long elapsed_t;
  current_ts = millis();
  elapsed_t = current_ts - encoder_ts;
  if( elapsed_t > ENCODER_UPDATE ){
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
  
}

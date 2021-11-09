// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _MOTORS_H
#define _MOTORS_H



// Class to operate the motor(s).
class Motors_c {
  public:
  #define P5 5
    // Constructor, must exist.
    Motors_c() {

    } 

    // Use this function to 
    // initialise the pins and 
    // state of your motor(s).
    void initialise(int PWMpin, int DIRpin) {
      pinMode(PWMpin, OUTPUT);
      pinMode(DIRpin, OUTPUT);
    }

    int rescaler(int PWMvalue, int maxPWM) {
        if (PWMvalue > maxPWM) {
            PWMvalue = maxPWM;
            Serial.print("overspeeding error left +60");
            Serial.print("\n");
        } else if (PWMvalue < -maxPWM){
            PWMvalue = -maxPWM;
            Serial.print("overspeeding error left -60");
            Serial.print("\n");
         }

         return PWMvalue;
    }

    void Direction(int PWMvalue, int DIR_PIN, int PWM_PIN){
        if (PWMvalue > 0) {
            digitalWrite(DIR_PIN, LOW);
            analogWrite(PWM_PIN, abs(PWMvalue));
        } else if (PWMvalue < 0) {
            digitalWrite(DIR_PIN, HIGH);
            analogWrite(PWM_PIN, abs(PWMvalue));
        } else if (PWMvalue == 0) {
            analogWrite(PWM_PIN, abs(PWMvalue));
        }
    }

    void CalibrationMovement(int R_DIR_PIN, int L_DIR_PIN, int R_PWM_PIN, int L_PWM_PIN ){
        int rightPWM = 15;
        int leftPWM = 15;
        Direction(rightPWM, R_DIR_PIN, R_PWM_PIN);
        Direction(leftPWM, L_DIR_PIN, L_PWM_PIN);
        delay(800);
        rightPWM = 0;
        leftPWM = 0;
        Direction(rightPWM, R_DIR_PIN, R_PWM_PIN);
        Direction(leftPWM, L_DIR_PIN, L_PWM_PIN);
        Serial.println("Motors moved");
    }
};



#endif

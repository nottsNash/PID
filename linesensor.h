// this #ifndef stops this file
// from being included more than
// once by the compiler. 
#ifndef _LINESENSOR_H
#define _LINESENSOR_H

// Class to operate the linesensor(s).
class LineSensor_c {
  public:
  
    // Constructor, must exist.
    LineSensor_c() {

    } 
//--------------------------------------------------------------------------------------------------------------------------------------------------
  void function(int Line_Sensor_midLeft, int Line_Sensor_centre, int Line_Sensor_midRight, int TimeOut, int NumberOfSensorPins,unsigned long *midLeftSensorValue,unsigned long *centreSensorValue,unsigned long *midRightSensorValue){
    
    int ls_pin[NumberOfSensorPins] = {Line_Sensor_midLeft, Line_Sensor_centre, Line_Sensor_midRight}; //array created to simulatnoesuly access pins
    int whichSensor = 0; //used to index through sensors
    
    unsigned long start_time;
    unsigned long end_time[NumberOfSensorPins];
    unsigned long elapsed_Time[NumberOfSensorPins] = {0, 0, 0};
    
    //set the capacitors for the sensors to be able to charge and start charging
    for( whichSensor = 0; whichSensor < NumberOfSensorPins; whichSensor++ ) {
       pinMode( ls_pin[whichSensor], OUTPUT );
       digitalWrite( ls_pin[whichSensor], HIGH );
     }
  
    // Tiny delay for capacitor to charge.
    delayMicroseconds(10);
    bool done = false; //flag to show when process is done, set as false initially
    
    //start emptying the capacitors
    for( whichSensor = 0; whichSensor < NumberOfSensorPins; whichSensor++ ) {
       pinMode(ls_pin[whichSensor], INPUT);
    }
     
    start_time = micros(); //start time to record emptying 

    int leftChange = 0;
    int centreChange = 0;
    int rightChange = 0;
    
    while( done == false ) {
      // Read all three sensors.  Happens very quickly. The while() above will repeat this process, until all the sensors have been read or timed out. 
    
      //3 seperate if statments to stop the end_time being written over once the initial end time has been recorded as low
      
      if( digitalRead(ls_pin[0]) == LOW && leftChange == 0) {
        end_time[0] = micros();
        leftChange = 1;
      }

      if( digitalRead(ls_pin[1]) == LOW && centreChange == 0) {
        end_time[1] = micros();
        centreChange = 1;
      }

      if( digitalRead(ls_pin[2]) == LOW && rightChange == 0) {
        end_time[2] = micros();
        rightChange = 1;
      }  
        
        if( (leftChange == 1) &&  (centreChange == 1) && (rightChange == 1)) {
            // Setting DONE to true will cause the
            // while loop to break (end)
            done = true;
        }
      }

    elapsed_Time[0] = end_time[0] - start_time;
    elapsed_Time[1] = end_time[1] - start_time;
    elapsed_Time[2] = end_time[2] - start_time;  

    *midLeftSensorValue = elapsed_Time[0];
    *centreSensorValue = elapsed_Time[1];
    *midRightSensorValue = elapsed_Time[2];
  }
//--------------------------------------------------------------------------------------------------------------------------------------------------
  void initialise(int EMIT, int Line_Sensor_midLeft, int Line_Sensor_centre, int Line_Sensor_midRight){
    pinMode(EMIT, OUTPUT);
    pinMode(Line_Sensor_midLeft, INPUT);
    pinMode(Line_Sensor_centre, INPUT);
    pinMode(Line_Sensor_midRight, INPUT);
  }

//--------------------------------------------------------------------------------------------------------------------------------------------------
  float CalibrateWhiteLeft(int Line_Sensor_midLeft, int Line_Sensor_centre, int Line_Sensor_midRight, int TimeOut, int NumberOfSensorPins){
      //Serial.println("Entered left calibration routine");
      int SensorReadings = 199;
      unsigned long midLeftSensorArray[200];
      unsigned long mlSV = 0;
      unsigned long cSV = 0;
      unsigned long mrSV = 0;
      float sumMidLeft = 0;
      float BlackSensorLeft = 0;
      float ScalingFactorLeft = 0;
      float offsetBiasLeft = 0;
      
      for (int i = 0; i <= SensorReadings; i++){
          //loop to record the current line sensor value at a given interval
          function(Line_Sensor_midLeft, Line_Sensor_centre, Line_Sensor_midRight, TimeOut, NumberOfSensorPins, &mlSV, &cSV, &mrSV);
          midLeftSensorArray[i] = mlSV;
          delay(10);
      }
        
      for (int i = 0; i <= SensorReadings; i++){
          sumMidLeft = sumMidLeft + (float)midLeftSensorArray[i];
      }
         
      offsetBiasLeft = sumMidLeft/SensorReadings;
      return offsetBiasLeft;
    }

//--------------------------------------------------------------------------------------------------------------------------------------------------
    float CalibrateWhiteMiddle(int Line_Sensor_midLeft, int Line_Sensor_centre, int Line_Sensor_midRight, int TimeOut, int NumberOfSensorPins){
      //Serial.println("Entered middle calibration routine");
      int SensorReadings = 199;
      unsigned long MiddleSensorArray[200];
      unsigned long mlSV = 0;
      unsigned long cSV = 0;
      unsigned long mrSV = 0;
      float sumMiddle = 0;
      float BlackSensorMiddle = 0;
      float ScalingFactorMiddle = 0;
      float offsetBiasMiddle = 0;
      
      //--------------DECLERATIONS-----------------------
      //loop for recording white
      for (int i = 0; i <= SensorReadings; i++){
          //loop to record the current line sensor value at a given interval
          function(Line_Sensor_midLeft, Line_Sensor_centre, Line_Sensor_midRight, TimeOut, NumberOfSensorPins, &mlSV, &cSV, &mrSV);
          MiddleSensorArray[i] = cSV;
          delay(10);
        }
        
        for (int i = 0; i <= SensorReadings; i++){
          sumMiddle = sumMiddle + (float)MiddleSensorArray[i];
        }
         
        offsetBiasMiddle = sumMiddle/SensorReadings;
        return offsetBiasMiddle;
    }

//--------------------------------------------------------------------------------------------------------------------------------------------------
  float CalibrateWhiteRight(int Line_Sensor_midLeft, int Line_Sensor_centre, int Line_Sensor_midRight, int TimeOut, int NumberOfSensorPins){
      //Serial.println("Entered right calibration routine");
      int SensorReadings = 199;
      unsigned long midRightSensorArray[200];
      unsigned long mlSV = 0;
      unsigned long cSV = 0;
      unsigned long mrSV = 0;
      float sumMidRight = 0;
      float BlackSensorRight = 0;
      float ScalingFactorRight = 0;
      float offsetBiasRight = 0;
      
      //loop for recording white
      for (int i = 0; i <= SensorReadings; i++){
          //loop to record the current line sensor value at a given interval
          function(Line_Sensor_midLeft, Line_Sensor_centre, Line_Sensor_midRight, TimeOut, NumberOfSensorPins, &mlSV, &cSV, &mrSV);
          midRightSensorArray[i] = mrSV;
          delay(10);
        }
        
        for (int i = 0; i <= SensorReadings; i++){
          sumMidRight = sumMidRight + (float)midRightSensorArray[i];
        }
         
        offsetBiasRight = sumMidRight/SensorReadings;
      return offsetBiasRight;
    }

    //--------------------------------------------------------------------------------------------------------------------------------------------------
    float CalibrateBlackLeft(int Line_Sensor_midLeft, int Line_Sensor_centre, int Line_Sensor_midRight, int TimeOut, int NumberOfSensorPins, float offsetBiasLeft){
     // Serial.println("Entered left calibration routine");
      int SensorReadings = 199;
      unsigned long midLeftSensorArray[200];
      unsigned long mlSV = 0;
      unsigned long cSV = 0;
      unsigned long mrSV = 0;
      float sumMidLeft = 0;
      float BlackSensorLeft = 0;
      float ScalingFactorLeft = 0;
      float ScalingFactorLeftGlobal = 0;
      
        //loop for recording black
        for (int i = 0; i <= SensorReadings; i++){
          //loop to record the current line sensor value at a given interval
          function(Line_Sensor_midLeft, Line_Sensor_centre, Line_Sensor_midRight, TimeOut, NumberOfSensorPins, &mlSV, &cSV, &mrSV);
          midLeftSensorArray[i] = mlSV;
          delay(10);
        }
        
        sumMidLeft = 0;
      
        for (int i = 0; i <= SensorReadings; i++){
          sumMidLeft = sumMidLeft + (float)midLeftSensorArray[i];
        }
        
        BlackSensorLeft = sumMidLeft/SensorReadings;
        ScalingFactorLeft = (1/(BlackSensorLeft - offsetBiasLeft))*1000;
        ScalingFactorLeftGlobal = ScalingFactorLeft;
        /*Serial.print("Black sensor value:");
        Serial.print(BlackSensorLeft);
        Serial.print("\n");
        Serial.println("Exited left calibration routine");*/
        return ScalingFactorLeftGlobal;
    }
//--------------------------------------------------------------------------------------------------------------------------------------------------
   float CalibrateBlackMiddle(int Line_Sensor_midLeft, int Line_Sensor_centre, int Line_Sensor_midRight, int TimeOut, int NumberOfSensorPins, float offsetBiasMiddle){
      //Serial.println("Entered middle calibration routine");
      int SensorReadings = 199;
      unsigned long MiddleSensorArray[200];
      unsigned long mlSV = 0;
      unsigned long cSV = 0;
      unsigned long mrSV = 0;
      float sumMiddle = 0;
      float BlackSensorMiddle = 0;
      float ScalingFactorMiddle = 0;
      float ScalingFactorMiddleGlobal = 0;
      
        //loop for recording black
        for (int i = 0; i <= SensorReadings; i++){
          //loop to record the current line sensor value at a given interval
          function(Line_Sensor_midLeft, Line_Sensor_centre, Line_Sensor_midRight, TimeOut, NumberOfSensorPins, &mlSV, &cSV, &mrSV);
          MiddleSensorArray[i] = cSV;
          delay(10);
        }
        
        sumMiddle = 0;
      
        for (int i = 0; i <= SensorReadings; i++){
          sumMiddle = sumMiddle + (float)MiddleSensorArray[i];
        }
        
        BlackSensorMiddle = sumMiddle/SensorReadings;
        ScalingFactorMiddle = (1/(BlackSensorMiddle - offsetBiasMiddle))*1000;
        ScalingFactorMiddleGlobal = ScalingFactorMiddle;
        return ScalingFactorMiddleGlobal;
    }

    //--------------------------------------------------------------------------------------------------------------------------------------------------

   float CalibrateBlackRight(int Line_Sensor_midLeft, int Line_Sensor_centre, int Line_Sensor_midRight, int TimeOut, int NumberOfSensorPins, float offsetBiasRight){
      //Serial.println("Entered right calibration routine");
      int SensorReadings = 199;
      unsigned long midRightSensorArray[200];
      unsigned long mlSV = 0;
      unsigned long cSV = 0;
      unsigned long mrSV = 0;
      float sumMidRight = 0;
      float BlackSensorRight = 0;
      float ScalingFactorRight = 0;
      float ScalingFactorRightGlobal = 0;
      
      //loop for recording black
      for (int i = 0; i <= SensorReadings; i++){
        //loop to record the current line sensor value at a given interval
        function(Line_Sensor_midLeft, Line_Sensor_centre, Line_Sensor_midRight, TimeOut, NumberOfSensorPins, &mlSV, &cSV, &mrSV);
        midRightSensorArray[i] = mrSV;
        delay(10);
      }
      
      sumMidRight = 0;
    
      for (int i = 0; i <= SensorReadings; i++){
        sumMidRight = sumMidRight + (float)midRightSensorArray[i];
      }
      
      BlackSensorRight = sumMidRight/SensorReadings;
      ScalingFactorRight = (1/(BlackSensorRight - offsetBiasRight))*1000;
      ScalingFactorRightGlobal = ScalingFactorRight;
      return ScalingFactorRightGlobal;
    }
};



#endif

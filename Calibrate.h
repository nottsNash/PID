// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _PID_H
#define _PID_H

// Class to contain generic PID algorithm.
class Calibrate_c {
  public:
  
    // Constructor, must exist.
    Calibrate_c() {
    }

      
    float ValueModification(float rawSensorValue, float OffsetBias, float ScalingFactor) {
      float NewSensorValue = 0;

      NewSensorValue = (abs(rawSensorValue - OffsetBias))*ScalingFactor;
      return NewSensorValue;
    }

    float WeightedController(float mlSV, float cSV, float mrSV){
      float SensorSummation = 0;
      float mlsvProportion = 0;
      float csvProportion = 0;
      float mrsvProportion = 0;
      float LeftWeight = 0;
      float RightWeight = 0;
      float SensorError = 0;

      SensorSummation = mlSV + cSV + mrSV;
      mlsvProportion = mlSV/SensorSummation;
      csvProportion = cSV/SensorSummation;
      mrsvProportion = mrSV/SensorSummation;

      LeftWeight = mlsvProportion + (csvProportion * 0.5);
      RightWeight = mrsvProportion + (csvProportion * 0.5);
      SensorError = LeftWeight - RightWeight;
      
      return SensorError;
    }

};



#endif

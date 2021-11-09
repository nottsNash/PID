// this #ifndef stops this file
// from being included mored than
// once by the compiler. 
#ifndef _PID_H
#define _PID_H

// Class to contain generic PID algorithm.
class PID_c {
  public:
  
    // Constructor, must exist.
    PID_c() {

    } 

    float Kp;
    float FeedbackSignal;
    float errorSignal;

    void initialisePIDs(){
      Kp = 2;
      FeedbackSignal = 0;
    }

    float updatePID(float demand, float measurement){
      errorSignal = demand - measurement;
      FeedbackSignal = Kp*errorSignal;
      return FeedbackSignal;
    }

};



#endif

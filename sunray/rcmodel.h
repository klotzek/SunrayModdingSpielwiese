// Ardumower Sunray 

// R/C model control

// use PCB pin 'mow' for R/C model control speed and PCB pin 'steering' for R/C model control steering, 
// also connect 5v and GND and activate model R/C control via PCB P20 start button for 3 sec.



#ifndef RCMODEL_H
#define RCMODEL_H



class RCModel {
    public:
      void begin();      
      void run(); 
      int rc_mowPWM;
      float rc_mowRPM;    
    protected:
      unsigned int lin_PWM ;                                            
      float rc_linear ;                                         
      unsigned int ang_PWM ;                                            
      float rc_angular ;
      unsigned int mow_PWM ;                                            
      
                                           
                                               
      unsigned long nextControlTime ;
    private:
#ifdef RC_DEBUG
        unsigned long nextOutputTime;
#endif
};


#endif

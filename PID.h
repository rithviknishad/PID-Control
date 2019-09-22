//
//  FILE:       PID.h
//  AUTHOR:     RITHVIK NISHAD (https://github.com/rithviknishad)
//  VERSION:    0.1.0
//  DESC:       PID Control Loop Mechanism
//  TAGS:       PID, Closed Control Loop, Open Source, Standard Library
//  URL:        
//

class PID
{
    protected:
    
    float* control_variable;
    float* process_variable;
    
    float error, last_error;


    public:

    float Kp = 0.1, Ki = 0.1, Kd = 0.1;



    protected:



    public:



};
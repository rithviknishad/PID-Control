//
//  FILE:       PID.h
//  AUTHOR:     RITHVIK NISHAD (https://github.com/rithviknishad)
//  VERSION:    0.1.0
//  DESC:       PID Control Loop Mechanism
//  TAGS:       PID, Closed Control Loop, Open Source, Standard Library
//  URL:        https://github.com/rithviknishad/PID-Control.git
//

class PID
{
    protected:
    
    float* control_variable, process_variable, output_variable;
    float error[];

    public:

    // PID tuning variables.
    float Kp = 0.1, Ki = 0.1, Kd = 0.1; // set value to 0, to disable appropriate control mechanism


    protected:

    static const float compute_p(float &error, float &kp) 
    { return error * kp; }

    static const float compute_i(float &accumulation_of_error, float &delta_time)
    { return accumulation_of_error * delta_time; }

    static const float compute_d(float &error, float &last_error, float &delta_time) 
    { return (error - last_error) / delta_time; }

    public:
    
};
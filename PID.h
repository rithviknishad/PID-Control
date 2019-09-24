//
//  FILE:       PID.h
//  AUTHOR:     RITHVIK NISHAD (https://github.com/rithviknishad)
//  VERSION:    0.1.0
//  DESC:       PID Control Loop Mechanism
//  TAGS:       PID, Closed Control Loop, Open Source, Standard Library
//  URL:        https://github.com/rithviknishad/PID-Control.git
//

typedef struct PID_Tuning
{
    // PID tuning variables.
    // set value to 0, to disable appropriate control mechanism
    float Kp, Ki, Kd;

    PID_Tuning(float kp, float ki, float kd);
    PID_Tuning(PID_Tuning & obj);
} PID_Tuning;


typedef struct PID_Output
{
    // PID output boundaries setting variables.
    float max_op, min_op;

    PID_Output(float max_value, float min_value);
    PID_Output(PID_Output & obj);
} PID_Output;


typedef struct PID_System : PID_Tuning, PID_Output
{
    PID_System(float kp, float ki, float kd, float max_op, float min_op);
    PID_System(PID_Tuning _pid_tuning, PID_Output _pid_output);
} PID_System;


class PID : PID_System
{
    private:
    float *process_ptr, *setpoint;

    public:
    float error, last_error, integral_error, last_time, delta_time, control;

    public:

    PID(float * var_setpoint, float * var_process, float kp, float ki, float kd, float op_max, float op_min);
    PID(float * var_setpoint, float * var_process, PID_System & pid_system);

    public:

    float update(const float time);
};
//
//  FILE:       PID.cpp
//  URL:        https://github.com/rithviknishad/PID-Control.git
//


//PID_Tuning Constructor(s)
PID_Tuning::PID_Tuning(float kp, float ki, float kd)
{ 
    Kp = kp; 
    Ki = ki; 
    Kd = kd; 
}

PID_Tuning::PID_Tuning(PID_Tuning & obj)
{
    Kp = obj.Kp;
    Ki = obj.Ki;
    Kd = obj.Kd;
}


//PID_Output Constructor(s)
PID_Output::PID_Output(float max_value, float min_value = 0.0f)
{ 
    max_op = max_value; 
    min_op = min_value; 
}

PID_Output::PID_Output(PID_Output & obj)
{
    max_op = obj.max_op;
    min_op = obj.min_op;
}


//PID_System Constructor(s)
PID_System::PID_System(float kp, float ki, float kd, float max_op, float min_op = 0.0f) : PID_Tuning(kp, ki, kd), PID_Output(max_op, min_op) { }

PID_System::PID_System(PID_Tuning _pid_tuning, PID_Output _pid_output) : PID_Tuning(_pid_tuning), PID_Output(_pid_output) { }


//PID Constructor(s)
PID::PID(float * var_setpoint, float * var_process, float kp, float ki, float kd, float op_max, float op_min) : PID_System(kp, ki, kd, op_max, op_min)
{
    process_ptr = var_process;
    setpoint = var_setpoint;
    error = last_error = integral_error = last_time = delta_time = control = 0;
}

PID::PID(float * var_setpoint, float * var_process, PID_System & pid_system) : PID_System(pid_system)
{
    process_ptr = var_process;
    setpoint = var_setpoint;
    error = last_error = integral_error = last_time = delta_time = control = 0;
}


float PID::update(const float time)
{
    //computes error, delta_time and then integral_error
    integral_error += ((error = (*setpoint) - (*process_ptr)) * (delta_time = time - last_time));
    //computes control op
    control = (Kp * error) + (Ki * integral_error) + (Kd * (error - last_error) / delta_time);
    //gets ready for next update
    last_error = error;
    last_time = time;

    return ((control < min_op) ? min_op : ((control > max_op) ? max_op : control));
}
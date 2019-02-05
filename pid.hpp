#ifndef PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.2.1


class PID
{

public:
    typedef enum
    {
        PID_FORWARD=0,
        PID_BACKWARD
    } pid_direction_t;


    typedef struct
    {
        double Kp;                              /** Proportional coefficient */
        double Ki;                              /** Integration coefficient */
        double Kd;                              /** Differential coefficient */

        float out_min;                          /** Minimum output signal */
        float out_max;                          /** Maximum output signal */

        pid_direction_t ControllerDirection;    /** Direction of a signal */
    } pid_config_t;

    PID(pid_config_t &);

    double compute(double, double);
    double compute(double);
    void update_setpoint(double);

    void set_limits(double, double);
    void set_tunings(double, double,
                    double);

    void set_direction(pid_direction_t);
    void get_config(pid_config_t &);

private:
    double compute_output(double, double);

    pid_config_t config;

    bool inAuto;
    double last_input;
    double integration_sum;

    double setpoint;
};
#endif


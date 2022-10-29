#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define RESISTANCE (1u)
//#define __DEBUG__
typedef struct VectTable VectTable;
typedef void (*pid_callback)(VectTable* self);
typedef void (*func)(float voltage);
typedef bool (*callback)(void);

typedef struct{

    /* For  PID*/
    float Kp;
    float Ki;
    float Kd;
    float p_term_V;
    float i_term_V;
    float d_term_V;
    float p_term_I;
    float i_term_I;
    float d_term_I;

    /* for internal reference*/
    float voltage_feedback;
    float voltage_reference;

    float minimum_current;
    float current_feedback;
    float current_reference;

}power_control;

struct VectTable{
    volatile bool enable_command;
    volatile power_control control;
    
    func            EPWM1_INT;
    pid_callback    state;
    callback        network_func;
};

/* static function*/
static void state_transition(VectTable* self , void* state);
static void constant_voltage(VectTable* self);
static void constant_current(VectTable* self);
static void idle(VectTable* self);
static void Initialization(VectTable* self);
/**
 * state trainsition
*/
static void state_transition(VectTable* self , void* state)
{
    if (state != NULL)
    {
        self->state = state;
    }
}

/**
 * state machine for constant voltage
*/
static void constant_voltage(VectTable* self)
{
    float err_value,voltage;

#if defined (__DEBUG__)
    printf("constant_voltage\n");
#endif

    /*simple pid*/
    err_value = (self->control.voltage_reference - self->control.voltage_feedback);
    self->control.p_term_V = self->control.Kp * err_value;
    self->control.i_term_V += err_value;

    voltage = (self->control.voltage_reference - self->control.p_term_V - self->control.i_term_V);

    /*control driver*/
    if (self->EPWM1_INT != NULL)
        self->EPWM1_INT(voltage);

    /* switch state*/
    if (self->control.current_feedback >= self->control.minimum_current)
    {
        self->enable_command = false;
        state_transition(self, idle);
    }
}

/**
 * state machine for constant current
*/
static void constant_current(VectTable* self)
{
    float err_value,voltage;
#if defined (__DEBUG__)
    printf("constant_current\n");
#endif
    /*simple pi control*/
    err_value = (self->control.current_reference - self->control.current_feedback);
    self->control.p_term_I = self->control.Kp * err_value;
    self->control.i_term_I += err_value;

    voltage = (self->control.current_reference - self->control.p_term_I - self->control.i_term_I) * RESISTANCE;

    /*control driver*/
    if (self->EPWM1_INT != NULL)
        self->EPWM1_INT(voltage);

    /* switch state*/
    if (self->control.voltage_feedback >= self->control.voltage_reference)
        state_transition(self, constant_voltage);
}

/**
 * state machine for idle
*/
static void idle(VectTable* self)
{
#if defined (__DEBUG__)
    printf("idle\n");
#endif
    /*wait for the enable command*/
    if (self->enable_command == true)
    {
        state_transition(self, constant_current);
    }
}

/*public function??*/
/**
 * Initialize the variables
*/
static void Initialization(VectTable* self)
{
    //initialize your variables here

    self->enable_command            = false;
    self->control.voltage_feedback  = 0.0f;     //Assume this is an adc readback
    self->control.voltage_reference = 3.3f;     //setpoint voltage
    self->control.current_feedback  = 0.0f;     //current readback
    self->control.current_reference = 1.0f;     //setpoint current

    /*PID parameters*/
    self->control.Kp                = 0.01;
    self->control.Ki                = 0.01;
    self->control.Kd                = 0.001;
    self->control.p_term_V          = 0;
    self->control.i_term_V          = 0;
    self->control.d_term_V          = 0;
    self->control.p_term_I          = 0;
    self->control.i_term_I          = 0;
    self->control.d_term_I          = 0;

}

/**
 * control routine -> assumed to be the lower driver layer.
*/
static void control_routine(float voltage){
    //run the control algorithm here

    // I = V/R
    // DAC
    //adjust_voltage(voltage);
}


/**
 * Main state Machine -> call from the main loop
*/
static void main_state_machine(VectTable* self){
    //run the state transition here

    /* Make sure the state is not NULL*/
    if (self->state != NULL)
        self->state(self);

}

/**
 * main loop
*/
extern void* network_construct(void);
int main(void)
{
    VectTable PieVectTable;
    Initialization(&PieVectTable);
    state_transition(&PieVectTable,idle);
    PieVectTable.EPWM1_INT = control_routine;
    PieVectTable.network_func = network_construct();
    while(true){
        main_state_machine(&PieVectTable);
        if (PieVectTable.network_func != NULL)
            PieVectTable.enable_command = PieVectTable.network_func();
    }
    return 0;
}
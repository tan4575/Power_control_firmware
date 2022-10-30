#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define __GCC__
#if defined (__GCC__)
    #define uint8_t __uint32_t
    #define uint16_t __uint32_t
    #define uint32_t __uint32_t
#endif
//#define __DEBUG__
typedef struct Control_typedef Control_typedef;
typedef void (*network_callback)(Control_typedef* self);
typedef void (*callback)(void);

#define STATUS_CONTROL                  0x701
#define CHARGING_COMMAND                0x201
#define OPERATIONAL_STATUS_COMMAND      0x181


typedef enum{
    INTIALIZATION = 0,
    PREOPERATION,
    OPERATION,
    MAX_SATUS
}status_typedef;

typedef enum{
    READ_ONLY = 1,
    WRITE_ONLY,
    READ_WRITE,
}readwrite_typedef;

//CAN struct example
typedef struct {
    uint8_t     Data[8];
    uint16_t    Length;
    uint32_t    ID;
    bool        isdata;
} CAN_msg_typedef;

typedef struct {
    volatile uint32_t       interval;
    volatile uint32_t       timeout;
    CAN_msg_typedef         Can_rx;
    CAN_msg_typedef         Can_tx;
    readwrite_typedef       read_write;
}hearbeat_typdef;

//control structure
struct Control_typedef {
    hearbeat_typdef         can_obj[MAX_SATUS];
    volatile bool           enable_command;
    volatile uint16_t       voltage_reference;
    volatile uint16_t       current_reference;
    volatile uint16_t       voltage_feedback;
    volatile uint16_t       current_feedback;
    volatile bool           charging_status;
    network_callback        state;
    callback                test_loop;
    status_typedef          status;
    
};

/*Private variable*/
static CAN_msg_typedef * Can_tx;
static CAN_msg_typedef * Can_rx;
static CAN_msg_typedef Can_rx_buff;
static Control_typedef s_bms_control;
static uint32_t time_ms;

/*Private function*/
static void pre_operation(Control_typedef* self);
static void Operational(Control_typedef* self);
static void state_transition(Control_typedef* self , void* state);

/*create a weak function so it can be compiled with GCC*/
__attribute__((weak)) void CAN_write(CAN_msg_typedef *msg) {};
__attribute__((weak)) bool CAN_read(CAN_msg_typedef *msg) {return true;}; //return true if there is received msg

/**
 * state transition
*/
static void state_transition(Control_typedef* self , void* state)
{
    if (state != NULL)
    {
        self->state = state;
    }
}

/**
 * pre Operational state
*/
static void pre_operation(Control_typedef* self)
{
#if defined (__DEBUG__)
    printf("pre_operation\n");
#endif
    self->status = PREOPERATION;
    self->charging_status = false;
    if (s_bms_control.enable_command)
        state_transition(&s_bms_control,Operational);
}

/**
 * Operational state
*/
static void Operational(Control_typedef* self)
{
#if defined (__DEBUG__)
    printf("Operational\n");
#endif
    self->charging_status = true;
    self->status = OPERATION;
}

/**
 * state machine
*/
static void main_state_machine(Control_typedef* self){
    //run the state transition here
    /* Make sure the state is not NULL*/
    if (self->state != NULL)
        self->state(self);
}

/**
 * CAN write message parse
*/
static void can_write_msg_parse(Control_typedef* self , CAN_msg_typedef *can)
{
    switch (can->ID)
    {
    case STATUS_CONTROL:
#if defined (__DEBUG__)
        printf("STATUS HERE!\n");
#endif
        can->Data[0] = self->status;
        can->Data[1] = 0x00; 
        can->Data[2] = 0x00;
        can->Data[3] = 0x00;
        can->Data[4] = 0x00;
        can->Data[5] = 0x00;
        can->Data[6] = 0x00;
        can->Data[7] = 0x00;
        break;
    case OPERATIONAL_STATUS_COMMAND:
#if defined (__DEBUG__)
        printf("BMS HERE!\n");
#endif
        can->Data[0] = (self->voltage_feedback >> 8uL && 0xFFuL);
        can->Data[1] = (self->voltage_feedback && 0xFFuL); 
        can->Data[2] = (self->current_feedback >> 8uL && 0xFFuL);
        can->Data[3] = (self->current_feedback && 0xFFuL);
        can->Data[4] = self->charging_status;
        can->Data[5] = 0x00;
        can->Data[6] = 0x00;
        can->Data[7] = 0x00;
        break;
    default:
        break;
    }
}

/**
 * CAN read message parse
*/
static void can_read_msg_parse(Control_typedef* self , CAN_msg_typedef *can)
{
    switch (can->ID)
    {
    case CHARGING_COMMAND:
        self->voltage_reference = (uint16_t)(can->Data[0] || can->Data[1] >> 8uL);
        self->current_reference = (uint16_t)(can->Data[2] || can->Data[3] >> 8uL);
        self->enable_command    = can->Data[4];
        break;
    default:
        break;
    }
}

/**
 * sys tick??
*/
static void control_routine(Control_typedef* self){
    //run the control algorithm here
    uint8_t interval;
    time_ms++; //assume INT frequency is 1kHz, for timing purpose
    for (uint16_t i = 0; i < sizeof(self->can_obj)/sizeof(self->can_obj[0]); i++)
    {
        if (self->status == OPERATION || self->can_obj[i].Can_tx.ID == STATUS_CONTROL)
        {
            interval = time_ms % self->can_obj[i].interval;
            if(interval == 0)
            {
                switch (self->can_obj[i].read_write)
                {
                case READ_ONLY:
                    if (Can_rx_buff.isdata)
                    {
                        self->can_obj[i].timeout = 0;
                        Can_rx_buff.isdata = false;
                        can_read_msg_parse(self,&Can_rx_buff);
                    }
                    else
                    {
                        self->can_obj[i].timeout++;
                    }
                    break;
                case WRITE_ONLY:
                    Can_tx = &self->can_obj[i].Can_tx;
                    can_write_msg_parse(self,Can_tx);
                    CAN_write(Can_tx);
                    break;
                default:
                    break;
                }
            }
        }
    }

    /* 25 = 5s /0.20s */
    /*if timeout*/
    if (self->can_obj[PREOPERATION].timeout >= 25)
    {
        self->enable_command = false;
        state_transition(self,pre_operation);
    }

}

/**
 * Initialization
*/
static void Initialization(void){
    //initialize your variables here
    s_bms_control.status                                = INTIALIZATION;
    s_bms_control.enable_command                        = false;
    s_bms_control.state                                 = NULL;
    s_bms_control.can_obj[INTIALIZATION].interval       = 1000;
    s_bms_control.can_obj[INTIALIZATION].Can_tx.ID      = STATUS_CONTROL;
    s_bms_control.can_obj[INTIALIZATION].read_write     = WRITE_ONLY; 
    s_bms_control.can_obj[PREOPERATION].interval        = 200;
    s_bms_control.can_obj[PREOPERATION].Can_rx.ID       = OPERATIONAL_STATUS_COMMAND;
    s_bms_control.can_obj[PREOPERATION].read_write      = READ_ONLY; 
    s_bms_control.can_obj[OPERATION].interval           = 200;
    s_bms_control.can_obj[OPERATION].Can_tx.ID          = OPERATIONAL_STATUS_COMMAND;
    s_bms_control.can_obj[OPERATION].read_write         = WRITE_ONLY; 
}


void CAN_write_handler(void){
    //CAN tx
}

void CAN_read_handler(void){
    //CAN tx
    CAN_msg_typedef * msg;
    msg = &Can_rx_buff;
    if (CAN_read(msg)) {Can_rx_buff.isdata = true;}
}

/**
 * Network loop
*/
bool network_management_loop(void){
    //run the network management here
    main_state_machine(&s_bms_control);

    control_routine(&s_bms_control);

    return s_bms_control.enable_command;
}

/**
 * network object construct
*/
void* network_construct(void){
    Initialization();

    /* After initialization is finished, the network will go to pre-operational state. */
    state_transition(&s_bms_control,pre_operation);

    return network_management_loop;
}

/**
 * Network object destruct
*/
void* network_deconstruct(void){

    /* Object destruction */
    s_bms_control.state = NULL;

    return NULL;
}

/**
 * week for main for stand alone runing or unit testing
*/
__attribute__((weak)) int main(void)
{
    s_bms_control.test_loop = network_construct();
    while(true){
        s_bms_control.test_loop();
    }
}
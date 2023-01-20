#include "kilolib.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "distribution_functions.c"

#define COLLISION_BITS 8
#define SECTORS_IN_COLLISION 2
#define ARGOS_SIMULATION
#define SCALING_STD 8

typedef enum
{ // Enum for different motion types
    TURN_LEFT = 1,
    TURN_RIGHT = 2,
    STOP = 3,
    FORWARD = 4,
} motion_t;

typedef enum
{ // Enum for boolean flags
    false = 0,
    true = 1,
} bool;

typedef enum
{ // Enum for the robot position wrt to areas
    LEFT = 1,
    RIGHT = 2,
} Free_space;

typedef enum
{
    kBLACK = 0,
    kWHITE = 2,
    kGRAY = 1,
    kLIGHTGRAY = 3,
} Light_sensor;

motion_t current_motion_type = STOP; // Current motion type
motion_t pivot = STOP;               // select the pivot, if always left or always right

/***********WALK PARAMETERS***********/
// const float std_motion_steps = 5 * 16; // variance of the gaussian used to compute forward motion
float std_motion_steps = 8 * 1;       // variance of the gaussian used to compute forward motion
float levy_exponent = 2.0;             // 2 is brownian like motion (alpha)
float crw_exponent = 0.0;              // higher more straight (rho)
uint32_t turning_ticks = 0;            // keep count of ticks of turning
const uint8_t max_turning_ticks = 125; /* constant to allow a maximum rotation of 180 degrees with \omega=\pi/5 */
unsigned int straight_ticks = 0;       // keep count of ticks of going straight
const uint16_t max_straight_ticks = 320;
uint32_t last_motion_ticks = 0;

/***********WALL AVOIDANCE***********/
// the kb is "equipped" with a proximity sensor

const uint8_t sector_base = (pow(2, COLLISION_BITS / 2) - 1);
uint8_t left_side = 0;
uint8_t right_side = 0;
uint8_t proximity_sensor = 0;
Free_space free_space = LEFT;
bool wall_avoidance_start = false;

/* ---------------------------------------------- */
// Robot messages to signal presence
message_t msg; //msg to send
// Variables for Smart Arena messages
int sa_type = 0;
int sa_payload = 0;

Light_sensor light_sensor = kWHITE;

uint32_t turning_ticks_threshold = (uint32_t)((0.18 / M_PI) * max_turning_ticks);    //threshold of 0.2 degrees

#ifndef ARGOS_SIMULATION
uint8_t start = 0; // waiting from ARK a start signal to run the experiment 0 : not received, 1 : received, 2 : not need anymore to receive
#endif

/*-------------------------------------------------------------------*/
/* Function for setting the led colour                               */
/*-------------------------------------------------------------------*/
void set_led()
{
    switch (light_sensor)
    {
    case kBLACK:
        set_color(RGB(3, 3, 3));
        break;
    case kGRAY:
        set_color(RGB(3, 0, 3));
        break;
    case kLIGHTGRAY:
        set_color(RGB(3, 3, 0));
        break;
    case kWHITE:
        set_color(RGB(0, 0, 0));
        break;

    default:
        break;
    }
}

/*-------------------------------------------------------------------*/
/* Function for setting the motor speed                              */
/*-------------------------------------------------------------------*/
void set_motion(motion_t new_motion_type)
{
    if (current_motion_type != new_motion_type)
    {
        switch (new_motion_type)
        {
        case FORWARD:
            // printf("Forward\n");
            spinup_motors();
            set_motors(kilo_straight_left, kilo_straight_right);
            break;
        case TURN_RIGHT:
            // printf("Right\n");
            spinup_motors();
            set_motors(0, kilo_turn_right);
            break;
        case TURN_LEFT:
            // printf("Left\n");
            spinup_motors();
            set_motors(kilo_turn_left, 0);
            break;
        case STOP:
        default:
            set_motors(0, 0);
        }
        current_motion_type = new_motion_type;
    }
}

/*-------------------------------------------------------------------*/
/* Parse ARK received messages                                       */
/*-------------------------------------------------------------------*/
void parse_smart_arena_message(uint8_t data[9], uint8_t kb_index)
{
    // index of first element in the 3 sub-blocks of data
    uint8_t shift = kb_index * 3;

    sa_type = data[shift + 1] >> 2 & 0x0F;
    sa_payload = ((data[shift + 1] & 0b11) << 8) | (data[shift + 2]);

    light_sensor = sa_type;


    // printf("sa_type: %d\n", sa_type);
    // printf("sa_payload: %d\n", sa_payload);
    /*
    2 Bits — 2_3678665
        std alpha0 rho0 alpha1 rho1
         1   1.99 0.00   1.01 0.98
    2 Bits — guess
        std alpha0 rho0 alpha1 rho1
         1   2.00  0.00   1.00 0.99

    3 Bits — 3_2679431
        std alpha0 rho0 alpha1 rho1 alpha2 rho2
         1   1.96 0.29   1.95 0.02      1 0.96
    3 Bits — guess
        std alpha0 rho0 alpha1 rho1 alpha2 rho2
         1   2.00  0.00   1.50  0.5  1.00  0.99

    4 Bits — 4_2680644
         std alpha0 rho0 alpha1 rho1 alpha2 rho2 alpha3 rho3
           1   1.99 0.31   1.95 0.00   1.02 0.05   1.01 0.96
    4 Bits — guess
        std alpha0 rho0 alpha1 rho1 alpha2 rho2 alpha3 rho3
         1   2.00  0.00  1.66  0.66  1.33  0.33  1.00  0.99
    
    */
    switch (sa_type)
    {
    case kBLACK:
        levy_exponent = 1.99;
        crw_exponent = 0.0;
        // printf("%u Black\n", kilo_uid);
        // printf("alpha: %f   rho: %f\n", levy_exponent, crw_exponent);
        break;
    case kGRAY:
        levy_exponent = 1.95;
        crw_exponent = 0.02;
        // printf("Gray\n");
        // printf("Error to many bits!!!!!\n");
        // printf("Error to many bits!!!!!\n");
        // printf("alpha: %f   rho: %f\n", levy_exponent, crw_exponent);
        break;
    case kLIGHTGRAY:
        levy_exponent = 1.95;
        crw_exponent = 0.02;
        // printf("LightGray\n");
        // printf("Error to many bits!!!!!\n");
        // printf("Error to many bits!!!!!\n");
        // printf("alpha: %f   rho: %f\n", levy_exponent, crw_exponent);
        break;
    case kWHITE:
        levy_exponent = 1.01;
        crw_exponent = 0.98;
        // printf("%d White\n", kilo_uid);
        // printf("alpha: %f   rho: %f\n", levy_exponent, crw_exponent);
        break;

    default:
        break;
    }

    if (sa_payload != 0)
    {
        // get rotation toward the center (if far from center)
        // avoid colliding with the wall
        // set_color(RGB(0, 0, 3));
        proximity_sensor = sa_payload;
        wall_avoidance_start = true;
    }
}

/*-------------------------------------------------------------------*/
/* Callback function for message reception                           */
/*-------------------------------------------------------------------*/
void rx_message(message_t *msg, distance_measurement_t *d)
{
    /* Unpack the message - extract ID, type and payload */
    if (msg->type == 0)
    {
        int id1 = msg->data[0] << 2 | (msg->data[1] >> 6);
        int id2 = msg->data[3] << 2 | (msg->data[4] >> 6);
        int id3 = msg->data[6] << 2 | (msg->data[7] >> 6);

        if (id1 == kilo_uid)
        {
            parse_smart_arena_message(msg->data, 0);
        }
        else if (id2 == kilo_uid)
        {
            parse_smart_arena_message(msg->data, 1);
        }
        else if (id3 == kilo_uid)
        {
            parse_smart_arena_message(msg->data, 2);
        }
    }

#ifndef ARGOS_SIMULATION
    /* start signal!*/
    else if (msg->type == 1 && start != 2)
    {
        start = 1;
    }

    /* ARK ID identification */
    else if (msg->type == 120)
    {
        int id = (msg->data[0] << 8) | msg->data[1];
        if (id == kilo_uid)
        {
            set_color(RGB(0, 0, 3));
        }
        else
        {
            set_color(RGB(3, 0, 0));
        }
    }
#endif
}

/*-------------------------------------------------------------------*/
/* Sending message                                                   */
/*-------------------------------------------------------------------*/
message_t *message_tx()
{
  return &msg;
}

/*-------------------------------------------------------------------*/
/* Function implementing the LMCRW random walk                       */
/*-------------------------------------------------------------------*/

void random_walk()
{
    switch (current_motion_type)
    {
    case TURN_LEFT:
    case TURN_RIGHT:
        /* if turned for enough time move forward */
        if (kilo_ticks > last_motion_ticks + turning_ticks)
        {
            /* start moving forward */
            last_motion_ticks = kilo_ticks;
            set_motion(FORWARD);
        }
        break;

    case FORWARD:
        /* if moved forward for enough time turn */
        if (kilo_ticks > last_motion_ticks + straight_ticks)
        {
            /* perform a random turn */
            last_motion_ticks = kilo_ticks;

            double angle = 0;
            if (crw_exponent == 0)
            {
                angle = (uniform_distribution(0, (M_PI)));
            }
            else
            {
                angle = fabs(wrapped_cauchy_ppf(crw_exponent));
            }
            // uncomment when single behaviour, comment when heterogeneous
            set_led();
            // printf("Taking decision!\n");
            // printf("levy_exponent %f\n\n", levy_exponent);
            straight_ticks = (uint32_t)(fabs(levy(std_motion_steps, levy_exponent)));
            turning_ticks = (uint32_t)((angle / M_PI) * max_turning_ticks);

            if(turning_ticks < turning_ticks_threshold) 
            {
                break;
            }
            
            /* 1st strategy */
            if (rand_soft() % 2)
            {
                turning_ticks = max_turning_ticks * 2 - turning_ticks;
            }

            set_motion(pivot);            
            
            // printf("Straight ticks %d,  angle:%f,  turning ticks:%d\n", straight_ticks, angle * (180.0/M_PI), turning_ticks);
            
        }
        break;

    case STOP:
    default:
        set_motion(STOP);
    }
}

/*-------------------------------------------------------------------*/
/* Init function                                                     */
/*-------------------------------------------------------------------*/
void setup()
{
  msg.type = 10;
  msg.data[0] = kilo_uid;
  msg.crc = message_crc(&msg);
  // Register the message_tx callback function.
  kilo_message_tx = message_tx;
    /* Initialise LED and motors */
#ifdef ARGOS_SIMULATION
    set_color(RGB(0, 0, 0));
#else
    set_color(RGB(0, 3, 0));
#endif
    set_motors(0, 0);

    /* Initialise random seed */
    uint8_t seed = rand_hard();
    rand_seed(seed);
    seed = rand_hard();
    srand(seed);

    if (rand_soft() % 2)
    {
        pivot = TURN_LEFT;
    }
    else
    {
        pivot = TURN_RIGHT;
    }

#ifdef ARGOS_SIMULATION
    set_motion(FORWARD);
#else
    set_motion(STOP);
#endif
    // printf("%d Environmental behaviour\n", kilo_uid);
}

/*-------------------------------------------------------------------*/
/* count 1s after decimal to binary conversion                       */
/*-------------------------------------------------------------------*/
uint8_t countOnes(uint8_t n)
{
    uint8_t count = 0;
    int i = 0;
    while (n > 0)
    {
        if ((n % 2) == 1)
            count++;
        n = n / 2;
        i++;
    }

    return count;
}

/*-------------------------------------------------------------------*/
/* Function implementing wall avoidance procedure                    */
/*-------------------------------------------------------------------*/
void wall_avoidance_procedure(uint8_t sensor_readings)
{
    right_side = sensor_readings & sector_base;
    left_side = (sensor_readings >> (COLLISION_BITS / 2)) & sector_base;

    uint8_t count_ones = countOnes(sensor_readings);
    if (count_ones > SECTORS_IN_COLLISION)
    {
        turning_ticks = (uint32_t)((M_PI / COLLISION_BITS) * max_turning_ticks);

        if (right_side < left_side)
        {
            set_motion(TURN_RIGHT);
            free_space = TURN_RIGHT;
        }
        else if (right_side > left_side)
        {
            set_motion(TURN_LEFT);
            free_space = TURN_LEFT;
        }
        else
        {
            set_motion(free_space);
        }
        if (kilo_ticks > last_motion_ticks + turning_ticks)
        {
            turning_ticks = (uint32_t)((M_PI / COLLISION_BITS) * max_turning_ticks);
            straight_ticks = SCALING_STD * (uint32_t)(fabs(levy(std_motion_steps, levy_exponent)));
        }
    }
}

/*-------------------------------------------------------------------*/
/* Main loop                                                         */
/*-------------------------------------------------------------------*/
void loop()
{
#ifndef ARGOS_SIMULATION
    if (start == 1)
    {
        /* Initialise motion variables */
        last_motion_ticks = rand() % max_straight_ticks;
        set_motion(FORWARD);
        start = 2;
    }
#endif

    if (wall_avoidance_start)
    {
        wall_avoidance_procedure(proximity_sensor);
        proximity_sensor = 0;
        wall_avoidance_start = false;
    }
    else
    {
        random_walk();
    }
    
    // This is only for heterogeneous robots
    // set_color(RGB(0, 0, 3));
}

int main()
{
    kilo_init();
    kilo_message_rx = rx_message;
    kilo_start(setup, loop);

    return 0;
}

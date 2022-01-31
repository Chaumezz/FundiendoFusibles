//***********************************************************************
// Hexapods Program
// Code for Arduino Mega
// by Francisco Carabaza 28/08/2021
// código basado en el trabajo de markwtech:
// https://markwtech.com/robots/hexapod/
// Se necesita un módulo bluetooth en el serial2 del Arduino Mega configurado a 115200 baudios
//***********************************************************************

//***********************************************************************
// Includes
//**********************************************************************
#include <Arduino.h>
#include <Servo.h>
#include <math.h>

//***********************************************************************
// Hexapod selection. Uncomment target hexapod.
//***********************************************************************

#define MINIHEX

//***********************************************************************
// Constant Declarations
//***********************************************************************

#define DEATHBAND 5

#define SMOOTH_FACTOR 0.05
#define SMOOTH_PREVIOUS_FACTOR 0.95

#define HEAD_SMOOTH_FACTOR 0.05
#define HEAD_SMOOTH_PREVIOUS_FACTOR 0.95

#ifdef MINIHEX
#include "Config_MiniHex.hpp"
#endif


const long A12DEG = 209440;   //12 degrees in radians x 1,000,000 (12 grados en millonésimas de radián)
const long A30DEG = 523599;   //30 degrees in radians x 1,000,000 (30 grados en millonésimas de radián)

const int FRAME_TIME_MS = 20; //frame time (20msec = 50Hz)

const int BATTERY_VOLTAGE = A14;  // Analog Pin

// pata 1 frontal derecha
// pata 2 media derecha
// pata 3 trasera derecha
// pata 4 trasera izquierda
// pata 5 media izquierda
// pata 6 frontal izquierda

//***********************************************************************
// Variable Declarations
//***********************************************************************
unsigned long currentTime;            //frame timer variables
unsigned long previousTime;

int temp;                             //mode and control variables
int mode;
int gait;
int gait_speed;
int gait_LED_color;
int reset_position;
int capture_offsets;

float L0, L3;                         //inverse kinematics variables
float gamma_femur;
float phi_tibia, phi_femur;
float theta_tibia, theta_femur, theta_coxa;

int leg1_IK_control, leg6_IK_control; //leg lift mode variables
float leg1_coxa, leg1_femur, leg1_tibia;
float leg6_coxa, leg6_femur, leg6_tibia;

int leg_num;                          //positioning and walking variables
int z_height_LED_color;
int totalX, totalY, totalZ;
int tick, numTicks;
int duration = 1080;
int z_height_left, z_height_right;
int commandedX, commandedY, commandedR;
int translateX, translateY, translateZ;
float step_height_multiplier;
float strideX, strideY, strideR;
float sinRotX, sinRotY, sinRotZ;
float cosRotX, cosRotY, cosRotZ;
float rotOffsetX, rotOffsetY, rotOffsetZ;
float amplitudeX, amplitudeY, amplitudeZ;
float offset_X[6], offset_Y[6], offset_Z[6];
float current_X[6], current_Y[6], current_Z[6];

int tripod_case[6]   = {1, 2, 1, 2, 1, 2}; //for tripod gait walking
int ripple_case[6]   = {2, 6, 4, 1, 3, 5}; //for ripple gait
int wave_case[6]     = {1, 2, 3, 4, 5, 6}; //for wave gait
int tetrapod_case[6] = {1, 3, 2, 1, 2, 3}; //for tetrapod gait

//Declare control variabes received by bluetooth
char BluetoothData; // the Bluetooth data received

int RX = 127; //X value from pad on the right
float RX_Previous = 127;
float RX_Smoothed = 127;

int RY = 127; //Y value from pad on the right
float RY_Previous = 127;
float RY_Smoothed = 127;

int RZ = 127; //Rotatión value from right joystick
float RZ_Previous = 127;
float RZ_Smoothed = 127;

int LX = 127; //X value from pad on the left
float LX_Previous = 127;
float LX_Smoothed = 127;

int LY = 127; //Y value from pad on the left
float LY_Previous = 127;
float LY_Smoothed = 127;

int LZ = 127; //Rotatión value from left joystick
float LZ_Previous = 127;
float LZ_Smoothed = 127;

//Head variables
int HX = 90; //Angulo de paneo del servo de la cabeza
float HX_Previous = 90;
float HX_Smoothed = 90;

int HY = 90; //Angulo de tilt del servo de la cabeza
float HY_Previous = 90;
float HY_Smoothed = 90;

float GR = 10; //Apertura de la pinza de la cabeza


int amplitude = 50;

float adcValue = 0;
float batteryVoltage = 0;

int update_interval = 1000; // time interval in ms for updating panel indicators
unsigned long last_time = 0; // time of last update


//***********************************************************************
// Object Declarations
//***********************************************************************
Servo coxa1_servo;
Servo femur1_servo;
Servo tibia1_servo;
Servo coxa2_servo;
Servo femur2_servo;
Servo tibia2_servo;
Servo coxa3_servo;
Servo femur3_servo;
Servo tibia3_servo;
Servo coxa4_servo;
Servo femur4_servo;
Servo tibia4_servo;
Servo coxa5_servo;
Servo femur5_servo;
Servo tibia5_servo;
Servo coxa6_servo;
Servo femur6_servo;
Servo tibia6_servo;

Servo head_tilt_servo;
Servo head_pan_servo;
Servo head_grip_servo;
//***********************************************************************
// Initialization Routine
//***********************************************************************
void setup()
{
  //start serials
  Serial.begin(115200);   //Serial para debug en el PC
  Serial2.begin(115200);  //Módulo bluetooth en Serial2. Es necesario ajustar esta velocidad a la que esté configurada en el módulo bluetooth.

  //attach servos
  coxa1_servo.attach(COXA1_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  femur1_servo.attach(FEMUR1_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  tibia1_servo.attach(TIBIA1_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  coxa2_servo.attach(COXA2_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  femur2_servo.attach(FEMUR2_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  tibia2_servo.attach(TIBIA2_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  coxa3_servo.attach(COXA3_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  femur3_servo.attach(FEMUR3_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  tibia3_servo.attach(TIBIA3_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  coxa4_servo.attach(COXA4_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  femur4_servo.attach(FEMUR4_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  tibia4_servo.attach(TIBIA4_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  coxa5_servo.attach(COXA5_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  femur5_servo.attach(FEMUR5_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  tibia5_servo.attach(TIBIA5_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  coxa6_servo.attach(COXA6_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  femur6_servo.attach(FEMUR6_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  tibia6_servo.attach(TIBIA6_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);

  head_pan_servo.attach(HEAD_PAN_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  head_pan_servo.write(90);
  head_tilt_servo.attach(HEAD_TILT_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  head_tilt_servo.write(90);

  head_grip_servo.attach(HEAD_GRIP_SERVO, MS_SERVO_MIN, MS_SERVO_MAX);
  head_grip_servo.write(10);


  //clear offsets
  for (leg_num = 0; leg_num < 6; leg_num++)
  {
    offset_X[leg_num] = 0.0;
    offset_Y[leg_num] = 0.0;
    offset_Z[leg_num] = 0.0;
  }

  capture_offsets = false;
  step_height_multiplier = 2.0; //modificado por paco, originalmente a 1.0

  //initialize mode and gait variables
  mode = 1; //1 walk, 2 translate control, 3 rotate control, 4 oneleglift, 99 set all 90
  gait = 2; //gate 0 tripod_gait, 1 wave gait, 2 ripple gait, 3 tetrapod gait
  gait_speed = 0;
  reset_position = true;
  leg1_IK_control = true;
  leg6_IK_control = true;
}


//***********************************************************************
// Main Program
//***********************************************************************
void loop()
{
  //read controller and process inputs
  read_bluetooth();

  UpdateIndicators();

  //set up frame time

  if ((millis() - previousTime) > FRAME_TIME_MS)
  {
    previousTime = millis();
    request_bluetooth_data();
    JamesBrutonSmoothing();
    move_head();
    print_debug(); //borrar luego
    previousTime = currentTime;

    //reset legs to home position when commanded
    if (reset_position == true)
    {
      Serial.println("Reset position");
      for (leg_num = 0; leg_num < 6; leg_num++)
      {
        current_X[leg_num] = HOME_X[leg_num];
        current_Y[leg_num] = HOME_Y[leg_num];
        current_Z[leg_num] = HOME_Z[leg_num];
      }
      reset_position = false;
    }

    //position legs using IK calculations - unless set all to 90 degrees mode
    if (mode < 99)
    {
      for (leg_num = 0; leg_num < 6; leg_num++)
        leg_IK(leg_num, current_X[leg_num] + offset_X[leg_num], current_Y[leg_num] + offset_Y[leg_num], current_Z[leg_num] + offset_Z[leg_num]);
    }

    //reset leg lift first pass flags if needed
    if (mode != 4)
    {
      leg1_IK_control = true;
      leg6_IK_control = true;
    }

    print_debug();                            //print debug data

    //process modes (mode 0 is default 'home idle' do-nothing mode)
    if (mode == 1)                            //walking mode
    {
      if (gait == 0) tripod_gait();           //walk using gait 0
      if (gait == 1) wave_gait();             //walk using gait 1
      if (gait == 2) ripple_gait();           //walk using gait 2
      if (gait == 3) tetrapod_gait();         //walk using gait 3
    }
    if (mode == 2) translate_control();       //joystick control x-y-z mode
    if (mode == 3) rotate_control();          //joystick control y-p-r mode
    if (mode == 4) one_leg_lift();            //one leg lift mode
    if (mode == 99) set_all_90();             //set all servos to 90 degrees mode
  }
}
void JamesBrutonSmoothing() {
  RX_Smoothed = (RX * SMOOTH_FACTOR) + (RX_Previous * SMOOTH_PREVIOUS_FACTOR);
  RX_Previous = RX_Smoothed;

  RY_Smoothed = (RY * SMOOTH_FACTOR) + (RY_Previous * SMOOTH_PREVIOUS_FACTOR);
  RY_Previous = RY_Smoothed;

  RZ_Smoothed = (RZ * SMOOTH_FACTOR) + (RZ_Previous * SMOOTH_PREVIOUS_FACTOR);
  RZ_Previous = RZ_Smoothed;

  LX_Smoothed = (LX * SMOOTH_FACTOR) + (LX_Previous * SMOOTH_PREVIOUS_FACTOR);
  LX_Previous = LX_Smoothed;

  LY_Smoothed = (LY * SMOOTH_FACTOR) + (LY_Previous * SMOOTH_PREVIOUS_FACTOR);
  LY_Previous = LY_Smoothed;

  LZ_Smoothed = (LZ * SMOOTH_FACTOR) + (LZ_Previous * SMOOTH_PREVIOUS_FACTOR);
  LZ_Previous = LZ_Smoothed;

  HX_Smoothed = (HX * HEAD_SMOOTH_FACTOR) + (HX_Previous * HEAD_SMOOTH_PREVIOUS_FACTOR);
  HX_Previous = HX_Smoothed;

  HY_Smoothed = (HY * HEAD_SMOOTH_FACTOR) + (HY_Previous * HEAD_SMOOTH_PREVIOUS_FACTOR);
  HY_Previous = HY_Smoothed;
}


//***********************************************************************
// Leg IK Routine
//***********************************************************************
void leg_IK(int leg_number, float X, float Y, float Z)
{
  //compute target femur-to-toe (L3) length
  L0 = sqrt(sq(X) + sq(Y)) - COXA_LENGTH;
  L3 = sqrt(sq(L0) + sq(Z));

  //process only if reach is within possible range (not too long or too short!)
  if ((L3 < (TIBIA_LENGTH + FEMUR_LENGTH)) && (L3 > (TIBIA_LENGTH - FEMUR_LENGTH)))
  {
    //compute tibia angle
    phi_tibia = acos((sq(FEMUR_LENGTH) + sq(TIBIA_LENGTH) - sq(L3)) / (2 * FEMUR_LENGTH * TIBIA_LENGTH));
    theta_tibia = phi_tibia * RAD_TO_DEG - 23.0 + TIBIA_CAL[leg_number];
    theta_tibia = constrain(theta_tibia, 0.0, 180.0);

    //compute femur angle
    gamma_femur = atan2(Z, L0);
    phi_femur = acos((sq(FEMUR_LENGTH) + sq(L3) - sq(TIBIA_LENGTH)) / (2 * FEMUR_LENGTH * L3));
    theta_femur = (phi_femur + gamma_femur) * RAD_TO_DEG + 14.0 + 90.0 + FEMUR_CAL[leg_number];
    theta_femur = constrain(theta_femur, 0.0, 180.0);

    //compute coxa angle
    theta_coxa = atan2(X, Y) * RAD_TO_DEG + COXA_CAL[leg_number];

#ifdef MINIHEX
#include "IK_MiniHex.hpp" // MiniHex IK Routine
#endif

  }
}


//***********************************************************************
// Tripod Gait
// Group of 3 legs move forward while the other 3 legs provide support
//***********************************************************************
void tripod_gait()
{
  //read commanded values from controller
  commandedX = map(RY_Smoothed, 0, 255, 127, -127);
  commandedY = map(RX_Smoothed, 0, 255, -127, 127);
  commandedR = map(LX_Smoothed, 255, 0, 127, -127);

  //if commands more than deadband then process
  if ((abs(commandedX) > DEATHBAND) || (abs(commandedY) > DEATHBAND) || (abs(commandedR) > DEATHBAND) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 2.0); //total ticks divided into the two cases
    for (leg_num = 0; leg_num < 6; leg_num++)
    {
      compute_amplitudes();
      switch (tripod_case[leg_num])
      {
        case 1:                               //move foot forward (raise and lower)
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / numTicks);
          if (tick >= numTicks - 1) tripod_case[leg_num] = 2;
          break;
        case 2:                               //move foot back (on the ground)
          current_X[leg_num] = HOME_X[leg_num] + amplitudeX * cos(M_PI * tick / numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] + amplitudeY * cos(M_PI * tick / numTicks);
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) tripod_case[leg_num] = 1;
          break;
      }
    }
    //increment tick
    if (tick < numTicks - 1) tick++;
    else tick = 0;
  }
}


//***********************************************************************
// Wave Gait
// Legs move forward one at a time while the other 5 legs provide support
//***********************************************************************
void wave_gait()
{
  //read commanded values from controller
  commandedX = map(RY_Smoothed, 0, 255, 127, -127);
  commandedY = map(RX_Smoothed, 0, 255, -127, 127);
  commandedR = map(LX_Smoothed, 255, 0, 127, -127);

  //if commands more than deadband then process
  if ((abs(commandedX) > DEATHBAND) || (abs(commandedY) > DEATHBAND) || (abs(commandedR) > DEATHBAND) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 6.0); //total ticks divided into the six cases
    for (leg_num = 0; leg_num < 6; leg_num++)
    {
      compute_amplitudes();
      switch (wave_case[leg_num])
      {
        case 1:                               //move foot forward (raise and lower)
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / numTicks);
          if (tick >= numTicks - 1) wave_case[leg_num] = 6;
          break;
        case 2:                               //move foot back one-fifth (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) wave_case[leg_num] = 1;
          break;
        case 3:                               //move foot back one-fifth (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) wave_case[leg_num] = 2;
          break;
        case 4:                               //move foot back one-fifth (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1)
            wave_case[leg_num] = 3;
          break;
        case 5:                               //move foot back one-fifth (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) wave_case[leg_num] = 4;
          break;
        case 6:                               //move foot back one-fifth (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.5;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.5;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) wave_case[leg_num] = 5;
          break;
      }
    }
    //increment tick
    if (tick < numTicks - 1) tick++;
    else tick = 0;
  }
}


//***********************************************************************
// Ripple Gait
// Left legs move forward rear-to-front while right also do the same,
// but right side is offset so RR starts midway through the LM stroke
//***********************************************************************
void ripple_gait()
{
  //read commanded values from controller
  commandedX = map(RY_Smoothed, 0, 255, 127, -127);
  commandedY = map(RX_Smoothed, 0, 255, -127, 127);
  commandedR = map(LX_Smoothed, 255, 0, 127, -127);

  //if commands more than deadband then process
  if ((abs(commandedX) > DEATHBAND) || (abs(commandedY) > DEATHBAND) || (abs(commandedR) > DEATHBAND) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 6.0); //total ticks divided into the six cases
    for (leg_num = 0; leg_num < 6; leg_num++)
    {
      compute_amplitudes();
      switch (ripple_case[leg_num])
      {
        case 1:                               //move foot forward (raise)
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / (numTicks * 2));
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / (numTicks * 2));
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / (numTicks * 2));
          if (tick >= numTicks - 1) ripple_case[leg_num] = 2;
          break;
        case 2:                               //move foot forward (lower)
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * (numTicks + tick) / (numTicks * 2));
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * (numTicks + tick) / (numTicks * 2));
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * (numTicks + tick) / (numTicks * 2));
          if (tick >= numTicks - 1) ripple_case[leg_num] = 3;
          break;
        case 3:                               //move foot back one-quarter (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) ripple_case[leg_num] = 4;
          break;
        case 4:                               //move foot back one-quarter (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) ripple_case[leg_num] = 5;
          break;
        case 5:                               //move foot back one-quarter (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) ripple_case[leg_num] = 6;
          break;
        case 6:                               //move foot back one-quarter (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks / 2.0;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks / 2.0;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) ripple_case[leg_num] = 1;
          break;
      }
    }
    //increment tick
    if (tick < numTicks - 1) tick++;
    else tick = 0;
  }
}


//***********************************************************************
// Tetrapod Gait
// Right front and left rear legs move forward together, then right
// rear and left middle, and finally right middle and left front.
//***********************************************************************
void tetrapod_gait()
{
  //read commanded values from controller
  commandedX = map(RY_Smoothed, 0, 255, 127, -127);
  commandedY = map(RX_Smoothed, 0, 255, -127, 127);
  commandedR = map(LX_Smoothed, 255, 0, 127, -127);

  //if commands more than deadband then process
  if ((abs(commandedX) > DEATHBAND) || (abs(commandedY) > DEATHBAND) || (abs(commandedR) > DEATHBAND) || (tick > 0))
  {
    compute_strides();
    numTicks = round(duration / FRAME_TIME_MS / 3.0); //total ticks divided into the three cases
    for (leg_num = 0; leg_num < 6; leg_num++)
    {
      compute_amplitudes();
      switch (tetrapod_case[leg_num])
      {
        case 1:                               //move foot forward (raise and lower)
          current_X[leg_num] = HOME_X[leg_num] - amplitudeX * cos(M_PI * tick / numTicks);
          current_Y[leg_num] = HOME_Y[leg_num] - amplitudeY * cos(M_PI * tick / numTicks);
          current_Z[leg_num] = HOME_Z[leg_num] + abs(amplitudeZ) * sin(M_PI * tick / numTicks);
          if (tick >= numTicks - 1) tetrapod_case[leg_num] = 2;
          break;
        case 2:                               //move foot back one-half (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) tetrapod_case[leg_num] = 3;
          break;
        case 3:                               //move foot back one-half (on the ground)
          current_X[leg_num] = current_X[leg_num] - amplitudeX / numTicks;
          current_Y[leg_num] = current_Y[leg_num] - amplitudeY / numTicks;
          current_Z[leg_num] = HOME_Z[leg_num];
          if (tick >= numTicks - 1) tetrapod_case[leg_num] = 1;
          break;
      }
    }
    //increment tick
    if (tick < numTicks - 1) tick++;
    else tick = 0;
  }
}


//***********************************************************************
// Compute walking stride lengths
//***********************************************************************
void compute_strides()
{
  //compute stride lengths
  strideX = STRIDE_X_MULTIPLIER * commandedX / 127;
  strideY = STRIDE_Y_MULTIPLIER * commandedY / 127;
  strideR = STRIDE_R_MULTIPLIER * commandedR / 127;

  //compute rotation trig
  sinRotZ = sin(radians(strideR));
  cosRotZ = cos(radians(strideR));
}


//***********************************************************************
// Compute walking amplitudes
//***********************************************************************
void compute_amplitudes()
{
  //compute total distance from center of body to toe
  totalX = HOME_X[leg_num] + BODY_X[leg_num];
  totalY = HOME_Y[leg_num] + BODY_Y[leg_num];

  //compute rotational offset
  rotOffsetX = totalY * sinRotZ + totalX * cosRotZ - totalX;
  rotOffsetY = totalY * cosRotZ - totalX * sinRotZ - totalY;

  //compute X and Y amplitude and constrain to prevent legs from crashing into each other
  amplitudeX = ((strideX + rotOffsetX) / 2.0);
  amplitudeY = ((strideY + rotOffsetY) / 2.0);
  amplitudeX = constrain(amplitudeX, -50, 50); //originalmente -50 a 50
  amplitudeY = constrain(amplitudeY, -50, 50); //originalmente -50 a 50

  //compute Z amplitude
  if (abs(strideX + rotOffsetX) > abs(strideY + rotOffsetY))
    amplitudeZ = step_height_multiplier * (strideX + rotOffsetX) / 4.0;
  else
    amplitudeZ = step_height_multiplier * (strideY + rotOffsetY) / 4.0;
}


//***********************************************************************
// Body translate with controller (xyz axes)
//***********************************************************************
void translate_control()
{
  //compute X direction move
  translateX = map(RY_Smoothed, 0, 255, -2 * TRAVEL, 2 * TRAVEL);
  for (leg_num = 0; leg_num < 6; leg_num++)
    current_X[leg_num] = HOME_X[leg_num] + translateX;

  //compute Y direction move
  translateY = map(RX_Smoothed, 0, 255, 2 * TRAVEL, -2 * TRAVEL);
  for (leg_num = 0; leg_num < 6; leg_num++)
    current_Y[leg_num] = HOME_Y[leg_num] + translateY;

  //compute Z direction move
  translateZ = LY_Smoothed;
  if (translateZ > 127)
    translateZ = map(translateZ, 128, 255, 0, TRAVEL);
  else
    translateZ = map(translateZ, 0, 127, -3 * TRAVEL, 0);
  for (leg_num = 0; leg_num < 6; leg_num++)
    current_Z[leg_num] = HOME_Z[leg_num] + translateZ;

  //lock in offsets if commanded
  if (capture_offsets == true)
  {
    for (leg_num = 0; leg_num < 6; leg_num++)
    {
      offset_X[leg_num] = offset_X[leg_num] + translateX;
      offset_Y[leg_num] = offset_Y[leg_num] + translateY;
      offset_Z[leg_num] = offset_Z[leg_num] + translateZ;
      current_X[leg_num] = HOME_X[leg_num];
      current_Y[leg_num] = HOME_Y[leg_num];
      current_Z[leg_num] = HOME_Z[leg_num];
    }
  }

  //if offsets were commanded, exit current mode
  if (capture_offsets == true)
  {
    capture_offsets = false;
    mode = 0;
  }
}



//***********************************************************************
// Body rotate with controller (xyz axes)
//***********************************************************************
void rotate_control()
{
  //compute rotation sin/cos values using controller inputs
  sinRotX = sin((map(RX_Smoothed, 0, 255, A12DEG, -A12DEG)) / 1000000.0);
  cosRotX = cos((map(RX_Smoothed, 0, 255, A12DEG, -A12DEG)) / 1000000.0);
  sinRotY = sin((map(RY_Smoothed, 0, 255, A12DEG, -A12DEG)) / 1000000.0);
  cosRotY = cos((map(RY_Smoothed, 0, 255, A12DEG, -A12DEG)) / 1000000.0);
  sinRotZ = sin((map(LX_Smoothed, 0, 255, -A30DEG, A30DEG)) / 1000000.0);
  cosRotZ = cos((map(LX_Smoothed, 0, 255, -A30DEG, A30DEG)) / 1000000.0);

  //compute Z direction move
  translateZ = LY_Smoothed;
  if (translateZ > 127)
    translateZ = map(translateZ, 128, 255, 0, TRAVEL);
  else
    translateZ = map(translateZ, 0, 127, -3 * TRAVEL, 0);

  for (int leg_num = 0; leg_num < 6; leg_num++)
  {
    //compute total distance from center of body to toe
    totalX = HOME_X[leg_num] + BODY_X[leg_num];
    totalY = HOME_Y[leg_num] + BODY_Y[leg_num];
    totalZ = HOME_Z[leg_num] + BODY_Z[leg_num];

    //perform 3 axis rotations
    rotOffsetX =  totalX * cosRotY * cosRotZ + totalY * sinRotX * sinRotY * cosRotZ + totalY * cosRotX * sinRotZ - totalZ * cosRotX * sinRotY * cosRotZ + totalZ * sinRotX * sinRotZ - totalX;
    rotOffsetY = -totalX * cosRotY * sinRotZ - totalY * sinRotX * sinRotY * sinRotZ + totalY * cosRotX * cosRotZ + totalZ * cosRotX * sinRotY * sinRotZ + totalZ * sinRotX * cosRotZ - totalY;
    rotOffsetZ =  totalX * sinRotY - totalY * sinRotX * cosRotY + totalZ * cosRotX * cosRotY                                  - totalZ;

    // Calculate foot positions to achieve desired rotation
    current_X[leg_num] = HOME_X[leg_num] + rotOffsetX;
    current_Y[leg_num] = HOME_Y[leg_num] + rotOffsetY;
    current_Z[leg_num] = HOME_Z[leg_num] + rotOffsetZ + translateZ;

    //lock in offsets if commanded
    if (capture_offsets == true)
    {
      offset_X[leg_num] = offset_X[leg_num] + rotOffsetX;
      offset_Y[leg_num] = offset_Y[leg_num] + rotOffsetY;
      offset_Z[leg_num] = offset_Z[leg_num] + rotOffsetZ + translateZ;
      current_X[leg_num] = HOME_X[leg_num];
      current_Y[leg_num] = HOME_Y[leg_num];
      current_Z[leg_num] = HOME_Z[leg_num];
    }
  }

  //if offsets were commanded, exit current mode
  if (capture_offsets == true)
  {
    capture_offsets = false;
    mode = 0;
  }
}

//***********************************************************************
// One leg lift mode
// also can set z step height using capture offsets
//***********************************************************************
void one_leg_lift()
{
  //read current leg servo 1 positions the first time
  if (leg1_IK_control == true)
  {
    leg1_coxa  = coxa1_servo.read();
    leg1_femur = femur1_servo.read();
    leg1_tibia = tibia1_servo.read();
    leg1_IK_control = false;
  }

  //read current leg servo 6 positions the first time
  if (leg6_IK_control == true)
  {
    leg6_coxa  = coxa6_servo.read();
    leg6_femur = femur6_servo.read();
    leg6_tibia = tibia6_servo.read();
    leg6_IK_control = false;
  }

  //process right joystick left/right axis
  temp = RX;
  temp = map(temp, 0, 255, 45, -45);
  coxa1_servo.write(constrain(int(leg1_coxa + temp), 45, 135));

  //process right joystick up/down axis
  temp = RY;
  if (temp < 117)                               //if joystick moved up
  {
    temp = map(temp, 116, 0, 0, 24);            //move leg 1
    femur1_servo.write(constrain(int(leg1_femur + temp), 0, 170));
    tibia1_servo.write(constrain(int(leg1_tibia + 4 * temp), 0, 170));
  }
  else                                          //if joystick moved down
  {
    z_height_right = constrain(temp, 140, 255); //set Z step height
    z_height_right = map(z_height_right, 140, 255, 1, 8);
  }

  //process left joystick left/right axis
  temp = LX;
  temp = map(temp, 0, 255, 45, -45);
  coxa6_servo.write(constrain(int(leg6_coxa + temp), 45, 135));

  //process left joystick up/down axis
  temp = LY;
  if (temp < 117)                               //if joystick moved up
  {
    temp = map(temp, 116, 0, 0, 24);            //move leg 6
    femur6_servo.write(constrain(int(leg6_femur + temp), 0, 170));
    tibia6_servo.write(constrain(int(leg6_tibia + 4 * temp), 0, 170));
  }
  else                                          //if joystick moved down
  {
    z_height_left = constrain(temp, 140, 255);  //set Z step height
    z_height_left = map(z_height_left, 140, 255, 1, 8);
  }

  //process z height adjustment
  if (z_height_left > z_height_right)
    z_height_right = z_height_left;             //use max left or right value

  if (capture_offsets == true)                  //lock in Z height if commanded
  {
    step_height_multiplier = 1.0 + ((z_height_right - 1.0) / 3.0);
    capture_offsets = false;
  }
}


void move_head() //Update head servos
{
  HX_Smoothed = map(HX_Smoothed, 0, 180, 180, 0);
  head_pan_servo.write(HX_Smoothed-HEAD_PAN);

  HY_Smoothed = map(HY_Smoothed, 0, 180, 0, 180);
  head_tilt_servo.write(HY_Smoothed-HEAD_TILT);

  head_grip_servo.write (GR-HEAD_GRIP);
}


//***********************************************************************
// Set all servos to 90 degrees
// Note: this is useful for calibration/alignment of the servos
// i.e: set COXA_CAL[6], FEMUR_CAL[6], and TIBIA_CAL[6] values in
//      constants section above so all angles appear as 90 degrees
//***********************************************************************
void set_all_90()
{
  coxa1_servo.write(90 + COXA_CAL[0]);
  femur1_servo.write(90 + FEMUR_CAL[0]);
  tibia1_servo.write(90 + TIBIA_CAL[0]);

  coxa2_servo.write(90 + COXA_CAL[1]);
  femur2_servo.write(90 + FEMUR_CAL[1]);
  tibia2_servo.write(90 + TIBIA_CAL[1]);

  coxa3_servo.write(90 + COXA_CAL[2]);
  femur3_servo.write(90 + FEMUR_CAL[2]);
  tibia3_servo.write(90 + TIBIA_CAL[2]);

  coxa4_servo.write(90 + COXA_CAL[3]);
  femur4_servo.write(90 + FEMUR_CAL[3]);
  tibia4_servo.write(90 + TIBIA_CAL[3]);

  coxa5_servo.write(90 + COXA_CAL[4]);
  femur5_servo.write(90 + FEMUR_CAL[4]);
  tibia5_servo.write(90 + TIBIA_CAL[4]);

  coxa6_servo.write(90 + COXA_CAL[5]);
  femur6_servo.write(90 + FEMUR_CAL[5]);
  tibia6_servo.write(90 + TIBIA_CAL[5]);

 

  while (true) {
  }
}

void clear_offsets()
{
  for (leg_num = 0; leg_num < 6; leg_num++)
  {
    offset_X[leg_num] = 0.0;
    offset_Y[leg_num] = 0.0;
    offset_Z[leg_num] = 0.0;
  }
}

//***********************************************************************
// Update indicators every second
//***********************************************************************
void UpdateIndicators()
{
  unsigned long t = millis();
  if ((t - last_time) > update_interval) {
    last_time = t;
    //adcValue = analogRead(A14);
    //batteryVoltage = (adcValue / 68.80);
    //Blynk.virtualWrite(V28, adcValue);
    //Blynk.virtualWrite(V29, String(batteryVoltage, 2));
    //Serial.println(analogRead(A14));
    //batteryVoltage = map((analogRead(A14)), 0, 1024, 0.0, 4.21);
    batteryVoltage = analogRead(A14) / 68.50;
    Serial2.println("*V" + String(batteryVoltage * 100) + "*");
    //Serial.println ("*V" + String(batteryVoltage) + "*");
  }
}

//***********************************************************************
// Read Bluetooth Data
//***********************************************************************
void read_bluetooth()
{
  if (Serial2.available()) {
    BluetoothData = Serial2.read(); //Get next character from bluetooth

    if (BluetoothData == 'W') {
      mode = 1; // Walk mode
    }

    if (BluetoothData == 'T') {
      mode = 2; // Trans mode
    }

    if (BluetoothData == 'R') {
      mode = 3; // Rotate mode
    }

    if (BluetoothData == 'A') {
      gait = 0; // Tripod gait
    }

    if (BluetoothData == 'B') {
      gait = 1; // Wave gait
    }

    if (BluetoothData == 'C') {
      gait = 2; // Ripple gait
    }

    if (BluetoothData == 'D') {
      gait = 3; // Tetrapod gait
    }

    if (BluetoothData == 'Z') { // Stop robot
      RX = 127;
      RY = 127;
      LX = 127;
      LY = 127;
    }

    if (BluetoothData == 's') {
      gait_speed = 0; // Low speed
      Serial2.print("*SLow");
    }

    if (BluetoothData == 'S') {
      gait_speed = 3; // High speed
      Serial2.print("*SHigh");
    }

    if (BluetoothData == 'E') { //Read Control Pad on Right (0-255) -  Sends 'EX__,Y___*
      if (Serial2.available()) {
        BluetoothData = Serial2.read(); //Get next character from bluetooth
        if (BluetoothData == 'X') {
          RX = Serial2.parseInt();
          while (BluetoothData != '*') {
            if (Serial2.available()) {
              BluetoothData = Serial2.read(); //Get next character from bluetooth
              if (BluetoothData == 'Y') RY = Serial2.parseInt();
            }
          }
        }
      }
    }
    RX = constrain(RX, 0, 255);
    RY = constrain(RY, 0, 255);

    if (BluetoothData == 'G') { //Read Control Pad on Left (0-255) -  Sends 'GX__,Y___*
      if (Serial2.available()) {
        BluetoothData = Serial2.read(); //Get next character from bluetooth
        if (BluetoothData == 'X') {
          LX = Serial2.parseInt();
          while (BluetoothData != '*') {
            if (Serial2.available()) {
              BluetoothData = Serial2.read(); //Get next character from bluetooth
              if (BluetoothData == 'Y') LY = Serial2.parseInt();
            }
          }
        }
      }
    }
    LX = constrain(LX, 0, 255);
    LY = constrain(LY, 0, 255);

    //             CONTROL CUELLO

    if (BluetoothData == 'H') {
      if (Serial2.available()) {
        BluetoothData = Serial2.read(); //Get next character from bluetooth
        if (BluetoothData == 'X') {
          HX = Serial2.parseInt();
          while (BluetoothData != '*') {
            if (Serial2.available()) {
              BluetoothData = Serial2.read(); //Get next character from bluetooth
              if (BluetoothData == 'Y') HY = Serial2.parseInt();
            }
          }
        }
      }
    }
    HX = constrain(HX, 0, 180);
    HY = constrain(HY, 0, 180);

   //        CONTROL  PINZA

    if (BluetoothData == 'P') GR = Serial2.parseInt();  
  }
}


//***********************************************************************
// Print Debug Data
//***********************************************************************
void request_bluetooth_data()
{
  Serial2.print("Hola!");
}


void print_debug()
{


  //output variable controls
  Serial2.print("RX:");
  Serial2.print(RX);
  Serial2.print(", RY:");
  Serial2.print(RY);
  Serial2.print(", LX:");
  Serial2.print(LX);
  Serial2.print(", LY:");
  Serial2.println(LY);
  Serial2.print("Mode ");
  Serial2.print(mode);
  Serial2.print("\t");
  Serial2.print("Gait ");
  Serial2.println(gait);

}

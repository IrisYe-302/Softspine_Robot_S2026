/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

// Trotting

#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB    
#else
  #define DEBUG_SERIAL Serial
#endif

const float DXL_PROTOCOL_VERSION = 2.0;

DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

long start, last_time, timestep;

float motor_rpm = 16;
int total_legs=4; //number of legs
uint8_t IDs[]={1,2,3,4,5,6};
int directions[]={1,-1,-1,1};
int leg_offset[]={0,90,-90,90};
int gait[]={0,180,0,180}; // diagonally alternating

float cycle_period = 2; // in seconds... syncs legs with spine
float spine_timing_offset = cycle_period*(3.0/4.0); // T/4 for left front leg down & spine arching left (destructive), 3T/4 for left front leg down and spine arching right (constructive)
float spine_magnitude = 106, spine_center = 160;
float servo6_timing_offset = -cycle_period*0.01;

/* Buehler*/
//clock parameters below calculated in clock_init()
float time_slow_start; // in seconds, time after the start of the period that the the leg enters the slow phase.
float time_slow_end; // in seconds, time after the start of the period that the the leg exits the slow phase.
float degree_slow_start; //in deg, the slow phase starting position
float degree_slow_end; //in deg, the slow phase starting position, if degree_slow_start=150 and degree_slow_end=210, this means whenever the leg's position is between 150 and 210, it will be in the slow phase.  

//Buehler clock timing parameters to change directly
//ϕ_s = 100 deg, ϕ_0= 90 deg, duty_cycle = 0.75
float phi_s=100; //in deg, ϕ_s is the angular extent of the slow phase
float phi_0=50; //in deg, ϕ_0 is the center of the slow phase in degrees
float duty_cycle =0.9;//d_c is the duty cycle of the slow phase (i.e. fraction of the period spent in the slow phase). 

float spine_center_offset[] = {-10, 10}; // adjust centers independently


void clock_init()
{

  degree_slow_start = phi_0 - phi_s / 2.0f;
  degree_slow_end   = phi_0 + phi_s / 2.0f;

  float T_slow = cycle_period * duty_cycle;
  float T_fast = cycle_period - T_slow;
  float w_f    = (360.0f - phi_s) / T_fast;

  // Time derived from position — guaranteed consistent
  time_slow_start = degree_slow_start / w_f;
  time_slow_end   = time_slow_start + T_slow;
  /*
    time_slow_start = cycle_period*(phi_0/360) - cycle_period*duty_cycle/2; //return the time after the start of the period that the the leg enters the slow phase
    time_slow_end = cycle_period*(phi_0/360) + cycle_period*duty_cycle/2; //return the time after the start of the period that the the leg exits the slow phase
    degree_slow_start = phi_0 - phi_s/2; //return the position that the the leg enters the slow phase
    degree_slow_end = phi_0 + phi_s/2; //return the position that the the leg exits the slow phase
  */

  return;
}

float triangleSignal(long elapsed, float spine_period) {
  float x = ((float)(elapsed) / spine_period) + 0.25f;
  float f = x - floor(x);                 
  float tri = 4.0f * fabs(f - 0.5f) - 1.0f;
  return spine_center + tri * spine_magnitude;
}

float angular_speed_fast(){  //return desired leg speed in fast phase in deg/s
  float w_f= (360.0f-phi_s)/(cycle_period-(time_slow_end-time_slow_start));
  return w_f; //desired speed in fast phase
}

float angular_speed_slow(){ //return desired leg speed in slow phase in deg/s
  float w_s= (phi_s)/(time_slow_end-time_slow_start);
  return w_s; //desired speed in slow phase
}

float get_desired_angle(int leg, long elapsed) // Robot's leg enum; time elapsed since start
{ // return the desired postion of the robot's leg given the leg number and the time elapsed in deg. It's an absolute position from 0 to 360. Don't use cumulative positions. It's ok to return positions in the deadzone (>300).    

  float s_elapsed = float(elapsed) / 1000.0f;

  // offset time based on leg phase
  float time_offset = 0;
  time_offset = (gait[leg]/360.0) * cycle_period;
  s_elapsed += time_offset;
  
  // whole number period
  int rotations = int(s_elapsed / cycle_period);

  // fractional period
  s_elapsed = fmod(s_elapsed, cycle_period);
  // whole rotations
  //float angle = rotations * 360.0f;
  float angle = rotations * 360.0; //we can't use cumulative positions
  
  // add fractional rotation
  if (s_elapsed <= time_slow_start)
    angle += angular_speed_fast() * s_elapsed;
  else if (s_elapsed <= time_slow_end)
    angle += degree_slow_start + ((s_elapsed - time_slow_start) * angular_speed_slow());
  else
    angle += degree_slow_end + ((s_elapsed - time_slow_end) * angular_speed_fast());
      
  //angle = fmod(angle, 360.0);
      
  return angle; //in deg
}

void setup() {
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  for (int i=0; i<total_legs; i++){
    dxl.ping(IDs[i]);
  }
  // Set op mode to normal op_position first for all to reset positions
  for (int i=0;i<6;i++){
    dxl.torqueOff(IDs[i]);
    dxl.setOperatingMode(IDs[i], OP_POSITION);
    dxl.torqueOn(IDs[i]);
    delay(100);
  } 

  // Reset each leg motor to zero position
  for (int i=0; i<total_legs; i++){
      float reset_angle= fmod((360.0 + directions[i] * get_desired_angle(i,0) + leg_offset[i]), 360.0);
      dxl.setGoalPosition(IDs[i], reset_angle, UNIT_DEGREE);   
  }

  // Reset spine motors to equal length
  for (int i=4; i<6; i++)
  {
      dxl.setGoalPosition(IDs[i], spine_center + spine_center_offset[i-4], UNIT_DEGREE);
  }

   delay(2000);

  // Change each leg servo mode to extended operating mode
  for (int i=0;i<total_legs;i++){
    dxl.torqueOff(IDs[i]);
    dxl.setOperatingMode(IDs[i], OP_EXTENDED_POSITION);
    dxl.torqueOn(IDs[i]);
    delay(100);
  }

  start = millis();
  timestep = 50;
  clock_init(); // for Buehler

}

void loop() {
  
  long elapsed = millis() - start;
  if (elapsed - last_time > timestep)
  {
    for (int i=0; i<total_legs; i++)
    {
      float desired_pos= 360.0 + directions[i] * get_desired_angle(i,elapsed) + leg_offset[i];
      //DEBUG_SERIAL.print("Desired leg pos : ");
      //DEBUG_SERIAL.println(desired_pos);
      dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
    }

    for (int i = 4; i < 6; i++) {
      float desired_angle = triangleSignal(elapsed + 1000*spine_timing_offset, 1000*cycle_period);
      desired_angle += spine_center_offset[i-4];
      dxl.setGoalPosition(IDs[i], desired_angle, UNIT_DEGREE);
    }
    last_time = elapsed;
  }
}

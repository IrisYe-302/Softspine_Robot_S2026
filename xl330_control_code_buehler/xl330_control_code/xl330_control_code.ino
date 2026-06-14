

#include <DynamixelShield.h>
#include <stdlib.h>
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

int total_legs=1; //number of legs

uint8_t IDs[]={1,2,3,4,5,6}; //default Leg ID-----[RF,LF,LR,RR]; if not changed, leg RF= ID 1, leg LF= ID 2, leg LR= ID 3, leg RR= ID 4; update the IDs using update_id()

uint8_t Directions[]={0,0,0,1,1,1}; //default Leg rotating direction -----[RF,LF,LR,RR]; value=0 then rotate CCW, value=1 then rotate CW; by default leg RF and leg RR will rotate CCW, and leg LF and leg LR will rotate CW; update the rotating direction using update_direction()

float gait[]={0,0,0,0,0}; //-----default gait; [LF-RF,LR-RF,RR-RF]; the gait coordinate is defined using 3 parameters (\phi_1, \phi_2, \phi_3), where \phi is the phase difference, and phase is defined as "TIME percentage within a gait cycle"; update the gait using update_gait()

int Leg_zeroing_offset[]={0,0,0,60,60,60}; //zeroing calibration offset in deg; leg angular position, ϕ, is defined as the angle measured clockwise about the axle from the upward vertical to the leg position, in radians. A zeroing calibration offset is needed because the servo’s zero position is not vertically downward and there is the offset between the servo-leg connector and the servo.  
                                        //by default is 60, which means when position command is 0, all legs should be pointing vertically upward.  

//clock parameters belows need to be calculated in clock_init()///////////////////////////////////////////////
float time_slow_start=1; // in seconds, time after the start of the period that the the leg enters the slow phase.
float time_slow_end=3; // in seconds, time after the start of the period that the the leg exits the slow phase.
float degree_slow_start=150; //in deg, the slow phase starting position
float degree_slow_end=210; //in deg, the slow phase starting position, if degree_slow_start=150 and degree_slow_end=210, this means whenever the leg's position is between 150 and 210, it will be in the slow phase.  
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//You may change the given Buehler clock timing parameters directly here
//ϕ_s = 0.85 rad, ϕ_0= 0.13 rad, d_c = 0.56
float phi_s=0.85; //in rad, ϕ_s is the angular extent of the slow phase
float phi_0=0.13+3.14; //in rad, ϕ_0 is the center of the slow phase 
float d_c=0.4;//d_c is the duty cycle of the slow phase (i.e. fraction of the period spent in the slow phase). 
float clock_period=4; //in seconds, time to complete 1 rotation


void clock_init(){ //change the value of the clock parameters
  //FIXME

  //you need to calculate time_slow_start,time_slow_end, degree_slow_start, degree_slow_end here
  //because of some technical constraint, you need to make sure 0<time_slow_start<time_slow_end<clock_period, and 0<degree_slow_start<degree_slow_end<300
  //in the servo deadzone, the position reading is incorrect, so the deadzone (300deg to 360deg) has to be fully in the fast phase
  //we are assuming that at t=0s, the leg is in position 0
  //notice degree_slow_start, degree_slow_end here are in deg   
  time_slow_start=clock_period*(phi_0/6.28)-clock_period*d_c/2; //return the time after the start of the period that the the leg enters the slow phase
  time_slow_end=clock_period*(phi_0/6.28)+clock_period*d_c/2; //return the time after the start of the period that the the leg exits the slow phase
  degree_slow_start=(phi_0-phi_s/2)/3.14*180; //return the position that the the leg enters the slow phase in deg
  degree_slow_end=(phi_0+phi_s/2)/3.14*180; //return the position that the the leg exits the slow phase in deg

  return;
}

float degree_slow(){ //return phi_s in deg
  //FIXME
  return  phi_s/3.14*180;
}

float omega_fast(){  //return desired leg speed in fast phase in deg/s
    //FIXME
  float o_f= (360.0f-degree_slow())/(clock_period-time_slow_end+time_slow_start);
  return o_f; //desired speed in fast phase
}

float omega_slow(){ //return desired leg speed in slow phase in deg/s
    //FIXME
  float o_s= (degree_slow())/(time_slow_end-time_slow_start);
  return o_s; //desired speed in slow phase
}

int dead_zone_speed_tuning=0; //adjustment to tune deadzone speed, MAGIC Variable as we are using position control outside the deadzone and speed control inside deadzone so the speed might be different; this variable is manually selected from observation, and it's ok that it's not working well.  
int different_direction_offset=-120; //adjustment to compensate position offset between 2 legs with different rotating direction. 


float get_desired_angle(int leg,                    // Robot's leg enum
                   long elapsed               // time elapsed since start
                   ) { // return the desired postion of the robot's leg given the leg number and the time elapsed in deg. It's an absolute position from 0 to 360. Don't use cumulative positions. It's ok to return positions in the deadzone (>300).    
    //FIXME

    float s_elapsed = float(elapsed) / 1000.0f;

    // offset time based on leg phase
    float time_offset = 0;
    if (leg>0) time_offset=(gait[leg-1]) * clock_period; // it's ok to use gait[] here, but remember gait[] only has 3 members
    s_elapsed += time_offset;
    
    // whole number period
    int rotations = int(s_elapsed / clock_period);

    // fractional period
    s_elapsed = fmod(s_elapsed, clock_period);
    // whole rotations
    //float angle = rotations * 360.0f;
    float angle =0; //we can't use cumulative positions
    
    // add fractional rotation
    if (s_elapsed <= time_slow_start)
        angle += omega_fast() * s_elapsed;
    else if (s_elapsed <= time_slow_end)
        angle +=
            degree_slow_start + ((s_elapsed - time_slow_start) * omega_slow());
    else
        angle +=
            degree_slow_end + ((s_elapsed - time_slow_end) * omega_fast());

    // add zeroing offset
    angle += Leg_zeroing_offset[leg];
    if (Directions[leg]==1) angle+=different_direction_offset; //Different direction offset
        
    angle = fmod(angle, 360.0);
       
    return angle; //in deg
}

void print_position(long t, int leg, float desired_pos){
        // Print desired position in degree value
        DEBUG_SERIAL.print(t);
        DEBUG_SERIAL.print(", ");    
        //DEBUG_SERIAL.print("Desired Position(degree) for leg#: ");
        //DEBUG_SERIAL.print(leg);
        //DEBUG_SERIAL.print(", ");        
        DEBUG_SERIAL.print(desired_pos);
        DEBUG_SERIAL.println(";"); 
        // Print present position in degree value
       // DEBUG_SERIAL.print(", ");    
        //DEBUG_SERIAL.print("Present Position(degree) for leg#: ");
        //DEBUG_SERIAL.print(leg);
        //DEBUG_SERIAL.print(" : ");        
       // DEBUG_SERIAL.println(dxl.getPresentPosition(IDs[leg], UNIT_DEGREE));
        
  return 0;
}





////////////////////////////////////////////// Do not change any code below this line/////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


long start;

void setup() {
  // put your setup code here, to run once:
  DEBUG_SERIAL.begin(115200);

  // Set Port baudrate to 1000000bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  // Get DYNAMIXEL information
  
  // Turn off torque when configuring items in EEPROM area
  for (int i=0;i<total_legs;i++){
    dxl.torqueOff(IDs[i]);
    dxl.setOperatingMode(IDs[i], OP_POSITION);
    dxl.torqueOn(IDs[i]);
    delay(100);
  }

  start = millis();
  clock_init();

}


long last_time=0;
int time_step=50;
bool in_dead_zone[]={0,0,0,0}; //0=not, 1=in



void loop() {
  // put your main code here, to run repeatedly:
  
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value. 
  // Set Goal Position in RAW value
  long elapsed = millis() - start;
  
  
  if (elapsed-last_time>time_step){
    last_time=elapsed;
    for (int i=0;i<total_legs;i++){
      float desired_pos=get_desired_angle(i,millis());
      if (Directions[i]==1) desired_pos=360.0-desired_pos;
      print_position(elapsed, i,desired_pos);
      
      if (Directions[i]==1){
        if (in_dead_zone[i]==0){

        
        if (desired_pos>10 )
          dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
        else{
          in_dead_zone[i]=1;

          float present_speed=dxl.getPresentVelocity(IDs[i]);
          dxl.torqueOff(IDs[i]);
          dxl.setOperatingMode(IDs[i], OP_VELOCITY);
          dxl.torqueOn(IDs[i]);
          //1 rpm=6 deg/s=9 unit
          //1 unit= 2/3 deg/s
          delay(10);
   
          dxl.setGoalVelocity(IDs[i], -omega_fast()/3, UNIT_RPM);
          
        }
        delay(10);   
      }
      else{
        int current_pos=dxl.getPresentPosition(IDs[i], UNIT_DEGREE);
        bool flag_temp=0;
        
        //DEBUG_SERIAL.println(i);
        
        if(Directions[i]==0)
            flag_temp=current_pos>20;
         else
            flag_temp=current_pos<350 && desired_pos<350;
          
        
        if (flag_temp && desired_pos>10 && desired_pos<300){
          in_dead_zone[i]=0;
          dxl.torqueOff(IDs[i]);
          dxl.setOperatingMode(IDs[i], OP_POSITION);
          dxl.torqueOn(IDs[i]);
          //1 rpm=6 deg/s=9 unit
          //1 unit= 2/3 deg/s
          delay(10);
          dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
        }
        
      }
      }
      
      else{
      if (in_dead_zone[i]==0){

        
        if (desired_pos<350 )
          dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
        else{
          in_dead_zone[i]=1;

          float present_speed=dxl.getPresentVelocity(IDs[i]);
          dxl.torqueOff(IDs[i]);
          dxl.setOperatingMode(IDs[i], OP_VELOCITY);
          dxl.torqueOn(IDs[i]);
          //1 rpm=6 deg/s=9 unit
          //1 unit= 2/3 deg/s
          delay(10);
   
          dxl.setGoalVelocity(IDs[i], omega_fast()/3, UNIT_RPM);
          
        }
        delay(10);   
      }
      else{
        int current_pos=dxl.getPresentPosition(IDs[i], UNIT_DEGREE);
        
        //DEBUG_SERIAL.println(i);
          
        
        if (current_pos>20 && desired_pos>20 && desired_pos<350){
          DEBUG_SERIAL.println(current_pos,desired_pos);
          in_dead_zone[i]=0;
          dxl.torqueOff(IDs[i]);
          dxl.setOperatingMode(IDs[i], OP_POSITION);
          dxl.torqueOn(IDs[i]);
          //1 rpm=6 deg/s=9 unit
          //1 unit= 2/3 deg/s
          delay(10);
          dxl.setGoalPosition(IDs[i], current_pos+15, UNIT_DEGREE);
        }
        
      }
    }
    }
  }
}
  

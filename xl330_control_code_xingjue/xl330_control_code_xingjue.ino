

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

int total_legs=6; //number of legs

uint8_t IDs[]={1,6,2,4,5,3}; //default Leg ID-----[RF,LF,LR,RR]; if not changed, leg RF= ID 1, leg LF= ID 2, leg LR= ID 3, leg RR= ID 4; update the IDs using update_id()

uint8_t Directions[]={1,1,1,0,0,0}; //default Leg rotating direction -----[RF,LF,LR,RR]; value=0 then rotate CCW, value=1 then rotate CW; by default leg RF and leg RR will rotate CCW, and leg LF and leg LR will rotate CW; update the rotating direction using update_direction()

float gait[]={0,0,0,0,0}; //-----default gait; [LF-RF,LR-RF,RR-RF]; the gait coordinate is defined using 3 parameters (\phi_1, \phi_2, \phi_3), where \phi is the phase difference, and phase is defined as "TIME percentage within a gait cycle"; update the gait using update_gait()

int Leg_zeroing_offset[]={150,60,240,60,60,60}; //zeroing calibration offset in deg; leg angular position, ϕ, is defined as the angle measured clockwise about the axle from the upward vertical to the leg position, in radians. A zeroing calibration offset is needed because the servo’s zero position is not vertically downward and there is the offset between the servo-leg connector and the servo.  
                                        //by default is 60, which means when position command is 0, all legs should be pointing vertically upward.  

float clock_period=1;

float get_desired_angle(int leg,                    // Robot's leg enum
                   long elapsed               // time elapsed since start
                   ) { // return the desired postion of the robot's leg given the leg number and the time elapsed in deg. It's an absolute position from 0 to 360. Don't use cumulative positions. It's ok to return positions in the deadzone (>300).    
    //FIXME

    float s_elapsed = float(elapsed) / 1000.0f;

    float temp_clock_period=1;

    if (leg>2)temp_clock_period=3;
    
    // offset time based on leg phase
    float time_offset = 0;
    if (leg>0) time_offset=(gait[leg-1]) * temp_clock_period; // it's ok to use gait[] here, but remember gait[] only has 3 members
    s_elapsed += time_offset;

    // whole number period
    int rotations = int(s_elapsed / temp_clock_period);

    // fractional period
    s_elapsed = fmod(s_elapsed, temp_clock_period);
    // whole rotations
    //float angle = rotations * 360.0f;
    float angle =0; //we can't use cumulative positions
    
    angle=360.0* s_elapsed / temp_clock_period;

          
    angle = fmod(angle, 360.0);
    angle=360*rotations+angle;
        
    return angle; //in deg
}



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

  
  for (int i=0;i<total_legs;i++){
      delay(10);
      dxl.setGoalPosition(IDs[i], 0, UNIT_DEGREE);   
  }

   delay(2000);
   
  for (int i=0;i<total_legs;i++){
    dxl.torqueOff(IDs[i]);
    dxl.setOperatingMode(IDs[i], OP_EXTENDED_POSITION);
    dxl.torqueOn(IDs[i]);
    delay(100);
  }

  for (int i=0;i<total_legs;i++){
      delay(10);
      float desired_pos=get_desired_angle(i,0);
      if (Directions[i]==1) desired_pos=360-desired_pos;
      
      dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
    
  }

  delay(10000);
  
  start = millis();

}


long last_time=0;
int time_step=50;

bool in_dead_zone[]={0,0,0,0,0,0};


void loop() {
  // put your main code here, to run repeatedly:
  
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value. 
  // Set Goal Position in RAW value
   long elapsed = millis() - start;
  
  
  if (elapsed>20000){
      
      for (int i=0;i<total_legs;i++){
        dxl.torqueOn(IDs[i]);
        delay(100);
      }
      
      for (int i=0;i<total_legs;i++){
        delay(10);
        dxl.setGoalPosition(IDs[i], 0, UNIT_DEGREE); 
      }  
  }
  else if (elapsed>10000){
    for (int i=0;i<total_legs;i++){
        dxl.torqueOff(IDs[i]);
        delay(100);
      }
  }   
  else if (elapsed-last_time>time_step){
    last_time=elapsed;
    for (int i=0;i<total_legs;i++){
      delay(10);
      float desired_pos=get_desired_angle(i,elapsed);
      if (Directions[i]==1) desired_pos=360-desired_pos;
      
      dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
     
    }
  }

 
}
  

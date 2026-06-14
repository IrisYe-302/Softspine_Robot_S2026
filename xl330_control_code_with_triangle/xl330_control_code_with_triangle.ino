

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

int total_legs=4; //number of legs

uint8_t IDs[]={1,2,3,4}; //default Leg ID-----[RF,LF,LR,RR]; if not changed, leg RF= ID 1, leg LF= ID 2, leg LR= ID 3, leg RR= ID 4; update the IDs using update_id()

uint8_t Directions[]={0,1,1,0}; //default Leg rotating direction -----[RF,LF,LR,RR]; value=0 then rotate CCW, value=1 then rotate CW; by default leg RF and leg RR will rotate CCW, and leg LF and leg LR will rotate CW; update the rotating direction using update_direction()

float gait[]={0.5, 0.5, 0}; //-----default gait; [LF-RF,LR-RF,RR-RF]; the gait coordinate is defined using 3 parameters (\phi_1, \phi_2, \phi_3), where \phi is the phase difference, and phase is defined as "TIME percentage within a gait cycle"; update the gait using update_gait()

int Leg_zeroing_offset[]={90, 0, 0, 0}; //zeroing calibration offset in deg; leg angular position, ϕ, is defined as the angle measured clockwise about the axle from the upward vertical to the leg position, in radians. A zeroing calibration offset is needed because the servo’s zero position is not vertically downward and there is the offset between the servo-leg connector and the servo.  
                                        //by default is 60, which means when position command is 0, all legs should be pointing vertically upward.  

float clock_period=4;

float motor5and6Center = 140;
float motor5Offset = 0;
float motor6Offset = 25;

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
    
    angle=360.0* s_elapsed / clock_period;

          
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

  dxl.torqueOff(5);
  dxl.setOperatingMode(5, OP_POSITION);
  dxl.torqueOn(5);

  dxl.torqueOff(6);
  dxl.setOperatingMode(6, OP_POSITION);
  dxl.torqueOn(6);
  
  for (int i=0;i<total_legs;i++){
      delay(10);
      dxl.setGoalPosition(IDs[i], Leg_zeroing_offset[i], UNIT_DEGREE);   
  }

   delay(200);
   
  for (int i=0;i<total_legs;i++){
    dxl.torqueOff(IDs[i]);
    dxl.setOperatingMode(IDs[i], OP_EXTENDED_POSITION);
    dxl.torqueOn(IDs[i]);
    delay(100);
  }

  dxl.torqueOff(5);
  dxl.setOperatingMode(5, OP_EXTENDED_POSITION); // or OP_POSITION
  dxl.torqueOn(5);

  dxl.torqueOff(6);
  dxl.setOperatingMode(6, OP_EXTENDED_POSITION); // or OP_POSITION
  dxl.torqueOn(6);

  for (int i=0;i<total_legs;i++){
      delay(10);
      float desired_pos=get_desired_angle(i,0);
      if (Directions[i]==1) desired_pos=360-desired_pos;
      desired_pos += Leg_zeroing_offset[i];     
      
      dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
    
  }

  dxl.setGoalPosition(5, motor5and6Center, UNIT_DEGREE);
  dxl.setGoalPosition(6, motor5and6Center + motor6Offset, UNIT_DEGREE);

  delay(100);
  
  start = millis();

}


long last_time=0;
int time_step=50;
float spineMotorTimeOffset = 0; // in seconds

bool in_dead_zone[]={0,0,0,0};

float triangleSignal(long s_elapsed, float clock_period, float spineMotorTimeOffset) {
  float x = ((float)(s_elapsed + spineMotorTimeOffset) / clock_period) + 0.25f;
  float f = x - floor(x);                 
  float tri = 4.0f * fabs(f - 0.5f) - 1.0f;
  return -tri;
}

float spineMotor5(long s_elapsed, float clock_period, float spineMotorTimeOffset) {
  float tri = triangleSignal(s_elapsed, clock_period, spineMotorTimeOffset);   
  return (motor5and6Center + motor5Offset) + 106.0f*tri;             
}

float spineMotor6(long s_elapsed, float clock_period, float spineMotorTimeOffset) {
  float tri = triangleSignal(s_elapsed, clock_period, spineMotorTimeOffset);   
  return (motor5and6Center + motor6Offset) + 106.0f*tri;           
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // Please refer to e-Manual(http://emanual.robotis.com/docs/en/parts/interface/dynamixel_shield/) for available range of value. 
  // Set Goal Position in RAW value
   long elapsed = millis() - start;
  
  
  if (elapsed>25000){
      
      for (int i=0;i<total_legs;i++){
        dxl.torqueOn(IDs[i]);
        dxl.torqueOn(5);
        dxl.torqueOn(6);
        delay(50);
      }
      
      for (int i=0;i<total_legs;i++){
        delay(10);
        dxl.setGoalPosition(IDs[i], Leg_zeroing_offset[i], UNIT_DEGREE);
        DEBUG_SERIAL.print("Present Position(degree) : ");
        DEBUG_SERIAL.println(dxl.getPresentPosition(2, UNIT_DEGREE));
  }
      dxl.setGoalPosition(5, motor5and6Center, UNIT_DEGREE);
      dxl.setGoalPosition(6, motor5and6Center + motor6Offset, UNIT_DEGREE);
  }
  else if (elapsed>15000){
    for (int i=0;i<total_legs;i++){
        dxl.torqueOff(IDs[i]);
        delay(50);
      }
  }   
  else if (elapsed-last_time>time_step){
    last_time=elapsed;

    float m5 = spineMotor5(elapsed, 1000*clock_period, 1000*spineMotorTimeOffset);
    float m6 = spineMotor6(elapsed, 1000*clock_period, 1000*spineMotorTimeOffset);

    dxl.setGoalPosition(6, m6, UNIT_DEGREE);
    dxl.setGoalPosition(5, m5, UNIT_DEGREE);
    
    for (int i=0;i<total_legs;i++){
      delay(10);
      float desired_pos=get_desired_angle(i,elapsed);
      if (Directions[i]==1) desired_pos=360-desired_pos;
      desired_pos += Leg_zeroing_offset[i];     
    
      dxl.setGoalPosition(IDs[i], desired_pos, UNIT_DEGREE);
      }
    }
  }  

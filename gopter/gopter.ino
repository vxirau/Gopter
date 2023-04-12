#include <Wire.h>


/*
PORT SETUP

VCC -- VCC
GND -- GND
SCL -- A5
SDA -- A4

*/

float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

int esc_1, esc_2, esc_3, esc_4;

double accelX,accelY,accelZ,temperature,gyroX,gyroY,gyroZ,gyro_x_cal,gyro_y_cal,gyro_z_cal; //Data que treguem de la IMU
double gyro_roll_input, gyro_pitch_input, gyro_yaw_input; //Els angles que extraiem del giroscopi, inputs del PID
double angle_pitch, angle_roll; //Els angles que extraiem del accelerometre, inputs del PID
uint32_t timer;
double acc_total_vector, angle_pitch_acc, angle_roll_acc, pitch_level_adjust, roll_level_adjust;
unsigned long loop_timer,tiempoInicio;
double roll, pitch ,yaw;
float rollangle,pitchangle;
int cal_int, start;
boolean gyro_angles_set;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, pid_output_yaw, pid_last_yaw_d_error;
//float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;

int throttle, battery_voltage;





void setup() {
  Serial.begin(115200);
  Wire.begin();

  setupMPU();   
  Serial.println("Calibrating Gyroscope......");
  for(cal_int=1;cal_int<=2000;cal_int++)
  { 
   recordRegisters();
   gyro_x_cal += gyroX;
   gyro_y_cal  += gyroY ;
   gyro_z_cal += gyroZ;
  }
  Serial.println("Calibration Done..!!!");
  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;
  start = 0;
  
  //start a timer
  timer = micros();
  tiempoInicio = micros();


  //Load the battery voltage to the battery_voltage variable.
  //65 is the voltage compensation for the diode.
  //12.6V equals ~5V @ Analog 0.
  //12.6V equals 1023 analogRead(0).
  //1260 / 1023 = 1.2317.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = (analogRead(0) + 65) * 1.2317;


}

/*
X - ROLL
Y - PITCH
Z - YAW
*/

void loop() {
  recordRegisters();

  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyroX / 65.5) * 0.3);    //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyroY / 65.5) * 0.3);  //Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyroZ / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  

  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyroY * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyroX * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr)                     The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyroZ * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyroZ * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.


  acc_total_vector = sqrt((accelX*accelX)+(accelY*accelY)+(accelZ*accelZ));       //Calculate the total accelerometer vector.
  
  if(abs(accelY) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    pitchangle = asin((float)accelY/acc_total_vector)* 57.296;          //Calculate the pitch angle.
  }
  if(abs(accelX) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    rollangle = asin((float)accelX/acc_total_vector)* -57.296;          //Calculate the roll angle.
  }


  angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
  angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.
  
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction



  if( start == 0 && (micros() - tiempoInicio > 5000000)) start = 1;         //Cooldown de 5 segons per a comencar a encendre els drones

  if(start == 1){
    start = 2;

    angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    gyro_angles_set = true;                                                 //Set the IMU started flag.

    //Reset the PID controllers for a bumpless start.
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;

  }

  //The PID set point in degrees per second is determined by the roll receiver input.
  //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_roll_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_1 > 1508)pid_roll_setpoint = receiver_input_channel_1 - 1508;
  else if(receiver_input_channel_1 < 1492)pid_roll_setpoint = receiver_input_channel_1 - 1492;

  pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
  pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


  //The PID set point in degrees per second is determined by the pitch receiver input.
  //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_pitch_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_2 > 1508)pid_pitch_setpoint = receiver_input_channel_2 - 1508;
  else if(receiver_input_channel_2 < 1492)pid_pitch_setpoint = receiver_input_channel_2 - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
  pid_pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

  //The PID set point in degrees per second is determined by the yaw receiver input.
  //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
  pid_yaw_setpoint = 0;
  //We need a little dead band of 16us for better results.
  if(receiver_input_channel_3 > 1050){ //Do not yaw when turning off the motors.
    if(receiver_input_channel_4 > 1508)pid_yaw_setpoint = (receiver_input_channel_4 - 1508)/3.0;
    else if(receiver_input_channel_4 < 1492)pid_yaw_setpoint = (receiver_input_channel_4 - 1492)/3.0;
  }
  
  calculate_pid();   

  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;


  //Engegar LED si la bateria esta per sota de 10V
  if(battery_voltage < 1000 && battery_voltage > 600)digitalWrite(12, HIGH);


  throttle = receiver_input_channel_3;       
  
  if(start == 2){
   if (throttle > 1800) throttle = 1800;                                   //We need some room to keep full control at full throttle.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 1 (front-right - CCW)
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 2 (rear-right - CW)
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw; //Calculate the pulse for esc 3 (rear-left - CCW)
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw; //Calculate the pulse for esc 4 (front-left - CW)

    if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
      esc_1 += esc_1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
      esc_2 += esc_2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
      esc_3 += esc_3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
      esc_4 += esc_4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    } 

    if (esc_1 < 1100) esc_1 = 1100;                                         //Keep the motors running.
    if (esc_2 < 1100) esc_2 = 1100;                                         //Keep the motors running.
    if (esc_3 < 1100) esc_3 = 1100;                                         //Keep the motors running.
    if (esc_4 < 1100) esc_4 = 1100;                                         //Keep the motors running.

    if(esc_1 > 2000)esc_1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc_2 > 2000)esc_2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc_3 > 2000)esc_3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc_4 > 2000)esc_4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  } else{
    esc_1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc_2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc_3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc_4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }                       




  Serial.print("BATTERY_VOLTAGE:");
  Serial.print(battery_voltage);
  Serial.print(",");
  Serial.print("PID_OUTPUT_PITCH:");
  Serial.print(pid_output_pitch);
  Serial.print(",");
  Serial.print("PID_OUTPUT_ROLL:");
  Serial.print(pid_output_roll);
  Serial.print(",");
  Serial.print("PID_OUTPUT_YAW:");
  Serial.println(pid_output_yaw);

  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  //Because of the angle calculation the loop time is getting very important. If the loop time is 
  //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure 
  //that the loop time is still 4000us and no longer! More information can be found on 
  //the Q&A page: 
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
    
  //if(micros() - loop_timer > 4050)digitalWrite(12, HIGH);                   //Turn on the LED if the loop time exceeds 4050us.

  while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
    loop_timer = micros();                                                    //Set the timer for the next loop.
  
   PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
  
  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }

}

void calculate_pid(){
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}


/*
FUNCIO QUE CONFIGURA LA IMU 
*/
void setupMPU(){
  Wire.beginTransmission(0b1101000);                                         //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B);                                                          //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000);                                                    //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000);                                         //I2C address of the MPU
  Wire.write(0x1B);                                                          //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x08);                                                          //Setting the gyro to full scale +/- 500deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000);                                         //I2C address of the MPU
  Wire.write(0x1C);                                                          //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0x10);                                                          //Setting the accel to +/- 8g
  Wire.endTransmission(); 
  
  Wire.beginTransmission(0b1101000);                                         //Start communication with the address found during search
  Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                    //End the transmission with the gyro    
}


void recordRegisters() {
  Wire.beginTransmission(0b1101000);                                         //I2C address of the MPU
  Wire.write(0x3B);                                                          //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,14);                                            //Request Accel Registers (3B - 40)
  while(Wire.available() < 14);
  accelX = Wire.read()<<8|Wire.read();                                       //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read();                                       //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read();                                       //Store last two bytes into accelZ
  temperature=Wire.read()<<8|Wire.read();
  gyroX = Wire.read()<<8|Wire.read();                                        //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read();                                        //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read();                                        //Store last two bytes into accelZ

  if(cal_int == 2000)
  {
    gyroX -= gyro_x_cal;
    gyroY -= gyro_y_cal;
    gyroZ -= gyro_z_cal;
  }

  
  // Valores hardcodeados del mando (valores van de 1000 a 2000), simulamos que los joysticks estan en medio
  receiver_input_channel_1 = 1500;//convert_receiver_channel(1);                 //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
  receiver_input_channel_2 = 1500;//convert_receiver_channel(2);                 //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
  receiver_input_channel_3 = 1500;//convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
  receiver_input_channel_4 = 1500;//convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

}
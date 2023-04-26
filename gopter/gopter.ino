#include <Wire.h>
#include <EEPROM.h>                        //Include the EEPROM.h library so we can store information onto the EEPROM


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
int aux=0;
int acc_axis[4], gyro_axis[4];


double accelX,accelY,accelZ,temperature,gyroX,gyroY,gyroZ,gyro_x_cal,gyro_y_cal,gyro_z_cal; //Data que treguem de la IMU
double gyro_roll_input, gyro_pitch_input, gyro_yaw_input; //Els angles que extraiem del giroscopi, inputs del PID
double angle_pitch, angle_roll; //Els angles que extraiem del accelerometre, inputs del PID
uint32_t timer;
double angle_pitch_acc, angle_roll_acc, pitch_level_adjust, roll_level_adjust;
unsigned long loop_timer,tiempoInicio;
double roll, pitch ,yaw;
float rollangle,pitchangle;
int cal_int, start;
boolean gyro_angles_set;
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, pid_output_yaw, pid_last_yaw_d_error;
int gyro_address;
double gyro_axis_cal[4];
long acc_x, acc_y, acc_z, acc_total_vector;

double gyro_pitch, gyro_roll, gyro_yaw;
//float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;

int throttle, battery_voltage;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_loop_timer;

int outputPin1 = 4; // PWM output pin
int outputPin2 = 5; // PWM output pin
int outputPin3 = 6; // PWM output pin
int outputPin4 = 7; // PWM output pin




byte eeprom_data[36];

boolean auto_level = true;                 //Auto level on (true) or off (false)


void setup() {
  Serial.begin(115200);
  Wire.begin();

 
  setupMPU();   
  
  start = 0;
  
  esc_1 = 2000;
  esc_2 = 2000;
  esc_3 = 2000;
  esc_4 = 2000;
  
  //start a timer
  timer = micros();
  tiempoInicio = micros();

  pinMode(outputPin1, OUTPUT);
  pinMode(outputPin2, OUTPUT);
  pinMode(outputPin3, OUTPUT);
  pinMode(outputPin4, OUTPUT);

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

  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3) ;   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.


  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr)                     The Arduino sin function is in radians
  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.


  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.
  
  if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
  }
  if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
  }


  angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
  angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.
  
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
  roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction

  if(!auto_level){                                                          //If the quadcopter is not in auto-level mode
    pitch_level_adjust = 0;                                                 //Set the pitch angle correction to zero.
    roll_level_adjust = 0;                                                  //Set the roll angle correcion to zero.
  }

  if( start == 0 && (micros() - tiempoInicio > 5000000)) start = 1;         //Cooldown de 5 segons per a comencar a encendre els drones

  if(start == 1){
    start = 2;
    tiempoInicio = micros();

    

  }

  
  if(start == 2){ 
    throttle = 2000;
    esc_1 = 2000;
    esc_2 = 2000;
    esc_3 = 2000;
    esc_4 = 2000;
  
    if(micros() - tiempoInicio > 100000){
      start=3;
      tiempoInicio = micros();
    }

  }

  if(start == 3){
    if(esc_1 > 1000){
      esc_1 = esc_1 - 100;
    }
     if(esc_2 > 1000){
      esc_2 = esc_2 - 100;
    }
     if(esc_3 > 1000){
      esc_3 = esc_3 - 100;
    }
     if(esc_4 > 1000){
      esc_4 = esc_4 - 100;
    }
    
    if(aux == 0){
      if(esc_1 == 1000 && esc_2 == 1000 && esc_3 == 1000 && esc_4 == 1000){
        tiempoInicio = micros();
        aux = 1;
      }
    }else{
      if(micros() - tiempoInicio > 2000000){
        start=4;
        tiempoInicio = micros();
      }
    }
    
    
   }

  if(start == 4){
    if(esc_1 < 1500){
      esc_1 = esc_1 + 10;
    }
     if(esc_2 < 1500){
      esc_2 = esc_2 + 10;
    }
     if(esc_3 < 1500){
      esc_3 = esc_3 + 10;
    }
     if(esc_4 < 1500){
      esc_4 = esc_4 + 10;
    }

    if(esc_4 == 1500){

      for(start = 0; start <= 35; start++)eeprom_data[start] = EEPROM.read(start);
      gyro_address = eeprom_data[32];                                           //Store the gyro address in the variable.

      //Check the EEPROM signature to make sure that the setup program is executed.
      while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B')delay(10);

      //The flight controller needs the MPU-6050 with gyro and accelerometer
      //If setup is completed without MPU-6050 stop the flight controller program  
      if(eeprom_data[31] == 2 || eeprom_data[31] == 3)delay(10);

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

      esc_1 = 1200;
      esc_2 = 1200;
      esc_3 = 1200;
      esc_4 = 1200;
      start = 5;
    }
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


  if(start == 5){

    throttle = receiver_input_channel_3;

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
  }   



  // Serial.print("EC4 (Front-Left):");
  // Serial.print(esc_4);
  // Serial.print(",");
  // Serial.print("EC1 (Front-Right):");
  // Serial.print(esc_1);
  // Serial.print(",");
  // Serial.print("EC3 (Back-Left):");
  // Serial.print(esc_3);
  // Serial.print(",");
  // Serial.print("EC2 (Back-Right):");
  // Serial.println(esc_2);
  //---------
  Serial.print("P:");
  Serial.print(gyro_pitch_input);
  Serial.print(",");
  Serial.print("R:");
  Serial.print(gyro_roll_input);
  Serial.print(",");
  Serial.print("Y:");
  Serial.println(gyro_yaw_input);
  //---------
  


  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  //Because of the angle calculation the loop time is getting very important. If the loop time is 
  //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure 
  //that the loop time is still 4000us and no longer! More information can be found on 
  //the Q&A page: 
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
    
  //if(micros() - loop_timer > 4050)digitalWrite(12, HIGH);                   //Turn on the LED if the loop time exceeds 4050us.

  while(micros() - loop_timer < 20700);                                      //We wait until 4000us are passed.
    loop_timer = micros();                                                    //Set the timer for the next loop.
  
  digitalWrite(4, HIGH);
  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);
  digitalWrite(7, HIGH);
  

  //BLANC DAVANT
  //Calculate the pulse for esc 1 (front-right - CCW)
  //Calculate the pulse for esc 2 (rear-right - CW)
  //Calculate the pulse for esc 3 (rear-left - CCW)
  //Calculate the pulse for esc 4 (front-left - CW)


  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer-100;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer-100;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer-100;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer-100;                                     //Calculate the time of the faling edge of the esc-4 pulse.
  
  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)digitalWrite(4, LOW);                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)digitalWrite(5, LOW);                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)digitalWrite(6, LOW);                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)digitalWrite(7, LOW);                //Set digital output 7 to low if the time is expired.
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
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(0b1101000);                                         //I2C address of the MPU
    Wire.write(0x3B);                                                          //Starting register for Accel Readings
    Wire.endTransmission();
    Wire.requestFrom(0b1101000,14);                                            //Request Accel Registers (3B - 40)
    while(Wire.available() < 14);                                           //Wait until the 14 bytes are received.
    acc_axis[1] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_x variable.
    acc_axis[2] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_y variable.
    acc_axis[3] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_z variable.
    temperature = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the temperature variable.
    gyro_axis[1] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_axis[2] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_axis[3] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
  }                                  //Store last two bytes into accelZ

  if(cal_int == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];                                       //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                                       //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];                                       //Only compensate after the calibration.
  }
  
  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011];                      //Set gyro_roll to the correct axis that was stored in the EEPROM.
  if(eeprom_data[28] & 0b10000000)gyro_roll *= -1;                          //Invert gyro_roll if the MSB of EEPROM bit 28 is set.
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011];                     //Set gyro_pitch to the correct axis that was stored in the EEPROM.
  if(eeprom_data[29] & 0b10000000)gyro_pitch *= -1;                         //Invert gyro_pitch if the MSB of EEPROM bit 29 is set.
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011];                       //Set gyro_yaw to the correct axis that was stored in the EEPROM.
  if(eeprom_data[30] & 0b10000000)gyro_yaw *= -1;                           //Invert gyro_yaw if the MSB of EEPROM bit 30 is set.

  acc_x = acc_axis[eeprom_data[29] & 0b00000011];                           //Set acc_x to the correct axis that was stored in the EEPROM.
  if(eeprom_data[29] & 0b10000000)acc_x *= -1;                              //Invert acc_x if the MSB of EEPROM bit 29 is set.
  acc_y = acc_axis[eeprom_data[28] & 0b00000011];                           //Set acc_y to the correct axis that was stored in the EEPROM.
  if(eeprom_data[28] & 0b10000000)acc_y *= -1;                              //Invert acc_y if the MSB of EEPROM bit 28 is set.
  acc_z = acc_axis[eeprom_data[30] & 0b00000011];                           //Set acc_z to the correct axis that was stored in the EEPROM.
  if(eeprom_data[30] & 0b10000000)acc_z *= -1;                              //Invert acc_z if the MSB of EEPROM bit 30 is set.
  
  // Valores hardcodeados del mando (valores van de 1000 a 2000), simulamos que los joysticks estan en medio
  receiver_input_channel_1 = 1500;//convert_receiver_channel(1);                 //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
  receiver_input_channel_2 = 1500;//convert_receiver_channel(2);                 //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
  receiver_input_channel_3 = 1200;//convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
  receiver_input_channel_4 = 1500;//convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

}
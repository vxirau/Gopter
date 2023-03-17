#include <Wire.h>


/*
PORT SETUP

VCC -- VCC
GND -- GND
SCL -- A5
SDA -- A4

*/

double accelX,accelY,accelZ,temperature,gyroX,gyroY,gyroZ,gyro_x_cal,gyro_y_cal,gyro_z_cal; //Data que treguem de la IMU
double gyro_roll_input, gyro_pitch_input, gyro_yaw_input; //Els angles que extraiem del giroscopi, inputs del PID
double angle_pitch, angle_roll; //Els angles que extraiem del accelerometre, inputs del PID
uint32_t timer;
double acc_total_vector, angle_pitch_acc, angle_roll_acc, pitch_level_adjust, roll_level_adjust;
unsigned long loop_timer;
double roll, pitch ,yaw;
float rollangle,pitchangle;
int cal_int;


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
  
  //start a timer
  timer = micros();
}

/*
X - ROLL
Y - PITCH
Z - YAW
*/

void loop() {
  recordRegisters();

  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyroX / 65.5) * 0.3);   //Gyro pid input is deg/sec.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyroY / 65.5) * 0.3);//Gyro pid input is deg/sec.
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyroZ / 65.5) * 0.3);      //Gyro pid input is deg/sec.

  
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += gyroY * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angle_roll += gyroX * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
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




/*
  accelX = accelX / 4096.0;
  accelY = accelY / 4096.0; 
  accelZ= accelZ / 4096.0;
  
  
  double dt = (double)(micros() - timer) / 1000000; //This line does three things: 1) stops the timer, 2)converts the timer's output to seconds from microseconds, 3)casts the value as a double saved to "dt".
  timer = micros(); //start the timer again so that we can calculate the next dt.

  //the next two lines calculate the orientation of the accelerometer relative to the earth and convert the output of atan2 from radians to degrees
  //We will use this data to correct any cumulative errors in the orientation that the gyroscope develops.
  rollangle=atan2(accelY,accelZ)*180/PI; // FORMULA FOUND ON INTERNET
  pitchangle=atan2(accelX,sqrt(accelY*accelY+accelZ*accelZ))*180/PI; //FORMULA FOUND ON INTERNET

  roll = 0.99 * (roll+ gyro_roll_input * dt) + 0.01 * rollangle;                      // Calculate the angle using a Complimentary filter
  pitch = 0.99 * (pitch + gyro_pitch_input * dt) + 0.01 * pitchangle; 
  yaw=gyro_yaw_input;

  */

  
  
  Serial.print("ROLL_(X):");
  Serial.print(gyro_roll_input);
  Serial.print(",");
  Serial.print("PITCH_(Y):");
  Serial.print(gyro_pitch_input);
  Serial.print(",");
  Serial.print("YAW_(Z):");
  Serial.print(gyro_yaw_input);
  Serial.print(",");
  Serial.print("PITCH_ANGLE:");
  Serial.print(pitchangle);
  Serial.print(",");
  Serial.print("ROLL_ANGLE:");
  Serial.println(rollangle);


  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  //Because of the angle calculation the loop time is getting very important. If the loop time is 
  //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure 
  //that the loop time is still 4000us and no longer! More information can be found on 
  //the Q&A page: 
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
    
  //if(micros() - loop_timer > 4050)digitalWrite(12, HIGH);                   //Turn on the LED if the loop time exceeds 4050us.

  //while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
  //  loop_timer = micros();                                                    //Set the timer for the next loop.
  

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
}
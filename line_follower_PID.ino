


float Kp=0,Ki=0,Kd=0;//change the value of kp ,ki and kd factors randomly and find a set of these value wich works good for your robot 
float error=0, P=0, I=0, D=0, PID_value=0;//defining the intial value 0
float previous_error=0, previous_I=0;//defining initially values of previous_error and previous_I 0 
int sensor[5]={0, 0, 0, 0, 0};//defining the sensor arrey of 5 
int initial_motor_speed=100;//defining the initial value of the motor speed as 100,can be changed

void read_sensor_values(void);//function that reads sensor values
void calculate_pid(void);//function that caluculates the pid value
void motor_control(void);//function that perform motor control action

void setup()
{
 pinMode(9,OUTPUT); //PWM Pin 1 ( first PWM pin on your motor driver that is connected to arduino pwm pin)
 pinMode(10,OUTPUT); //PWM Pin 2 (second PWM pin of your motor driver that is connected to arduino pwm pin )
 pinMode(4,OUTPUT); //Left Motor Pin 1(connected on arduino pin4)
 pinMode(5,OUTPUT); //Left Motor Pin 2(connected on arduino pin5)
 pinMode(6,OUTPUT); //Right Motor Pin 1(connected on arduino pin6)
 pinMode(7,OUTPUT);  //Right Motor Pin 2(connected on arduino pin7)
 Serial.begin(9600); //Enable Serial Communications
}

void loop()
{
    read_sensor_values();//sensor data is read
    calculate_pid();// pid is calculated
    motor_control();//motor speed is controlled
}

void read_sensor_values()
{
  sensor[0]=digitalRead(A0);//sensor data read from A0 arduino pin
  sensor[1]=digitalRead(A1);//sensor data read from A1 arduino pin
  sensor[2]=digitalRead(A2);//sensor data read from A2 arduino pin
  sensor[3]=digitalRead(A3);//sensor data read from A3 arduino pin
  sensor[4]=digitalRead(A4);//sensor data read from A4 arduino pin
  
  if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[4]==0)&&(sensor[4]==1))
  error=4;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[4]==1)&&(sensor[4]==1))
  error=3;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[4]==1)&&(sensor[4]==0))
  error=2;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[4]==1)&&(sensor[4]==0))
  error=1;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==1)&&(sensor[4]==0)&&(sensor[4]==0))
  error=0;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==1)&&(sensor[4]==0)&&(sensor[4]==0))
  error=-1;
  else if((sensor[0]==0)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[4]==0)&&(sensor[4]==0))
  error=-2;
  else if((sensor[0]==1)&&(sensor[1]==1)&&(sensor[2]==0)&&(sensor[4]==0)&&(sensor[4]==0))
  error=-3;
  else if((sensor[0]==1)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[4]==0)&&(sensor[4]==0))
  error=-4;
  else if((sensor[0]==0)&&(sensor[1]==0)&&(sensor[2]==0)&&(sensor[4]==0)&&(sensor[4]==0))
    if(error==-4) error=-5;
    else error=5;

}

void calculate_pid()//calculating pid 
{
    P = error;
    I = I + previous_I;
    D = error-previous_error;
    
    PID_value = (Kp*P) + (Ki*I) + (Kd*D);
    
    previous_I=I;
    previous_error=error;
}

void motor_control()//motor control
{
    // Calculating the effective motor speed:
    int left_motor_speed = initial_motor_speed-PID_value; 
    int right_motor_speed = initial_motor_speed+PID_value;
    
    // The motor speed should not exceed the max PWM value
    constrain(left_motor_speed,0,255);
    constrain(right_motor_speed,0,255);
  
  analogWrite(9,initial_motor_speed-PID_value);   //Left Motor Speed
    analogWrite(10,initial_motor_speed+PID_value);  //Right Motor Speed
    //following lines of code are to make the bot move forward
    /*The pin numbers and high, low values might be different
    depending on your connections */
    digitalWrite(4,HIGH);
    digitalWrite(5,LOW);
    digitalWrite(6,LOW);
    digitalWrite(7,HIGH);
}


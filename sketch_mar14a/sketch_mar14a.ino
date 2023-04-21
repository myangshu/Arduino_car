 #include<PID_v1.h>
 #include<Servo.h>

 //端口设置
 #define left_front_wheel_diroutput1 34
 #define left_front_wheel_diroutput2 35
 #define left_rear_wheel_diroutput1 37
 #define left_rear_wheel_diroutput2 36
 #define right_front_wheel_diroutput1 38
 #define right_front_wheel_diroutput2 39
 #define right_rear_wheel_diroutput1 40
 #define right_rear_wheel_diroutput2 41
 
 #define left_front_wheel_output 8
 #define left_rear_wheel_output 9
 #define right_front_wheel_output 7
 #define right_rear_wheel_output 6
 
 #define left_front_wheel_input 21
 #define left_rear_wheel_input 22
 #define right_front_wheel_input 23
 #define right_rear_wheel_input 24

 #define stepper_motor_1_stepPin 30
 #define stepper_motor_1_dirPin 31
 #define stepper_motor_2_stepPin 32
 #define stepper_motor_2_dirPin 33

#define servo_attach_pin 5

//PID数值初始化
int L1val=0,L2val=0,R1val=0,R2val=0;
double Kp = 0.10, Ki = 2, Kd = 0.02;
double LSetpoint,L1Input,L2Input,L1Output,L2Output;
double RSetpoint,R1Input,R2Input,R1Output,R2Output;
float L1last = 0,L2last = 0,R1last = 0,R2last = 0;
//PID数值设定和实例化对象
PID  L1PID(&L1Input,&L1Output,&LSetpoint,Kp,Ki,Kd,DIRECT);
PID  L2PID(&L2Input,&L2Output,&LSetpoint,Kp,Ki,Kd,DIRECT);
PID  R1PID(&R1Input,&R1Output,&RSetpoint,Kp,Ki,Kd,DIRECT);
PID  R2PID(&R2Input,&R2Output,&RSetpoint,Kp,Ki,Kd,DIRECT);

Servo myservo;

void setup() {
  
  //输出初始化
  pinMode(left_front_wheel_output,OUTPUT);
  pinMode(left_rear_wheel_output,OUTPUT);
  pinMode(right_front_wheel_output,OUTPUT);
  pinMode(right_rear_wheel_output,OUTPUT);

  pinMode(left_front_wheel_diroutput1,OUTPUT);
  pinMode(left_front_wheel_diroutput2,OUTPUT);
  pinMode(left_rear_wheel_diroutput1,OUTPUT);
  pinMode(left_rear_wheel_diroutput2,OUTPUT);
  pinMode(right_front_wheel_diroutput1,OUTPUT);
  pinMode(right_front_wheel_diroutput2,OUTPUT);
  pinMode(right_rear_wheel_diroutput1,OUTPUT);
  pinMode(right_rear_wheel_diroutput2,OUTPUT);
  
  //设置上拉电阻
  pinMode(left_rear_wheel_output,INPUT_PULLUP);
  pinMode(left_front_wheel_output,INPUT_PULLUP);
  pinMode(right_rear_wheel_output,INPUT_PULLUP);
  pinMode(right_front_wheel_output,INPUT_PULLUP);
  
  //设置外部中断
  attachInterrupt(digitalPinToInterrupt(left_rear_wheel_output), L2up, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_front_wheel_output), L1up, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_rear_wheel_output), R2up, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_front_wheel_output), R1up, CHANGE);

  //PID模式
  L1PID.SetMode(AUTOMATIC);
  L2PID.SetMode(AUTOMATIC);
  R1PID.SetMode(AUTOMATIC);
  R2PID.SetMode(AUTOMATIC);

  //设置PID阈值
  L1PID.SetOutputLimits(0, 255);
  L2PID.SetOutputLimits(0, 255);
  R1PID.SetOutputLimits(0, 255);
  R2PID.SetOutputLimits(0, 255);
  
  //步进电机接口定义
  pinMode(stepper_motor_1_stepPin, OUTPUT);
  pinMode(stepper_motor_1_dirPin, OUTPUT);
  pinMode(stepper_motor_2_stepPin, OUTPUT);
  pinMode(stepper_motor_2_dirPin, OUTPUT);

  //舵机初始化
  myservo.attach(servo_attach_pin);
  
  delay(0);
  Serial.begin(19200);
}

void loop() {
  delay(300);
  
  

  

  stopmotor();
  Serial.print("done\n");
  while(true){
    
    }
}
//舵机角度控制
void servo_set(int val){
  myservo.write(val);
}
//1号步进电机运行
void steppermotor1(int stepsPerRevolution,bool dir){
  digitalWrite(stepper_motor_1_dirPin,dir);
  for(int x = 0; x <= stepsPerRevolution; x++)
  {
    digitalWrite(stepper_motor_1_stepPin, HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepper_motor_1_stepPin, LOW);
    delayMicroseconds(2000);
  }  
}
//2号步进电机运行
void steppermotor2(int stepsPerRevolution,bool dir){
  digitalWrite(stepper_motor_2_dirPin,dir);
  for(int x = 0; x <= stepsPerRevolution; x++)
  {
    digitalWrite(stepper_motor_2_stepPin, HIGH);
    delayMicroseconds(2000);
    digitalWrite(stepper_motor_2_stepPin, LOW);
    delayMicroseconds(2000);
  }  
}

void forwardrun(double leftspeed,double rightspeed,long lasttime){
  
  digitalWrite(left_front_wheel_diroutput1, HIGH);
  digitalWrite(left_front_wheel_diroutput2, HIGH);
  digitalWrite(left_rear_wheel_diroutput1, HIGH);
  digitalWrite(left_rear_wheel_diroutput2, HIGH);
  digitalWrite(right_front_wheel_diroutput1, HIGH);
  digitalWrite(right_front_wheel_diroutput2, HIGH);
  digitalWrite(right_rear_wheel_diroutput1, HIGH);
  digitalWrite(right_rear_wheel_diroutput2, HIGH);
  
  LSetpoint=leftspeed;
  RSetpoint=rightspeed;
  long uptime = millis();
  while(millis()-uptime<lasttime){
    if ( millis() - L1last > 100)
    {
      double L1rpm = L1count_rpm();
      L1Input = map(L1rpm, 0, 105 , 0, 255);
    }
    
    if ( millis() - L2last > 100)
    {
      double L2rpm = L2count_rpm();
      L2Input = map(L2rpm, 0, 105 , 0, 255);
    }
    if ( millis() - R1last > 100)
    {
      double R1rpm = R1count_rpm();
      R1Input = map(R1rpm, 0, 105 , 0, 255);
    }
    if ( millis() - R2last > 100)
    {
      double R2rpm = R2count_rpm();
      R2Input = map(R2rpm, 0, 105 , 0, 255);
    }
    
    L1PID.Compute();
    analogWrite(left_front_wheel_output,L1Output);
    L2PID.Compute();
    analogWrite(left_rear_wheel_output,L2Output);
    R1PID.Compute();
    analogWrite(right_front_wheel_output,R1Output);
    R2PID.Compute();
    analogWrite(right_rear_wheel_output,R2Output);
    
    //Serial.print("\n");
    
    delay(100);
  }  
}

void stopmotor(){
  digitalWrite(left_front_wheel_diroutput1, HIGH);
  digitalWrite(left_front_wheel_diroutput2, HIGH);
  digitalWrite(left_rear_wheel_diroutput1, HIGH);
  digitalWrite(left_rear_wheel_diroutput2, HIGH);
  digitalWrite(right_front_wheel_diroutput1, HIGH);
  digitalWrite(right_front_wheel_diroutput2, HIGH);
  digitalWrite(right_rear_wheel_diroutput1, HIGH);
  digitalWrite(right_rear_wheel_diroutput2, HIGH);
}
//测速函数 X4
double L1count_rpm() {
  double L1rpm = L1val  / 24 * 1000/ (millis() - L1last);
  L1val = 0;
  L1last = millis();
  return L1rpm;
}
double L2count_rpm() {
  double L2rpm = L2val  / 24 * 1000/ (millis() - L2last);
  L2val = 0;
  L2last = millis();
  return L2rpm;
}
double R1count_rpm() {
  double R1rpm = R1val  / 24 * 1000/ (millis() - R1last);
  R1val = 0;
  R1last = millis();
  return R1rpm;
}
double R2count_rpm() {
  double R2rpm = R2val  / 24 * 1000/ (millis() - R2last);
  R2val = 0;
  R2last = millis();
  return R2rpm;
}
//计量函数 X4
void L2up(){
  L2val++;
}
void L1up(){
  L1val++;
}
void R2up(){
  R2val++;
}
void R1up(){
  R1val++;
}

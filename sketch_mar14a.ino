#include<PID_v1.h>


//int L1=0,L2=0,R1=0,R2=0;
double L1distance=0,L2distance=0,R1distance,R2distance;
double bal=0;
int L1val=0,L2val=0,R1val=0,R2val=0;
double Kp = 0.10, Ki = 2, Kd = 0.02;
double LSetpoint,L1Input,L2Input,L1Output,L2Output;
double RSetpoint,R1Input,R2Input,R1Output,R2Output;
float L1last = 0,L2last = 0,R1last = 0,R2last = 0;

PID  L1PID(&L1Input,&L1Output,&LSetpoint,Kp,Ki,Kd,DIRECT);
PID  L2PID(&L2Input,&L2Output,&LSetpoint,Kp,Ki,Kd,DIRECT);
PID  R1PID(&R1Input,&R1Output,&RSetpoint,Kp,Ki,Kd,DIRECT);
PID  R2PID(&R2Input,&R2Output,&RSetpoint,Kp,Ki,Kd,DIRECT);

void setup() {
  

  
  //输出初始化
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  
  //设置上拉电阻
  pinMode(18,INPUT_PULLUP);
  pinMode(19,INPUT_PULLUP);
  pinMode(20,INPUT_PULLUP);
  pinMode(21,INPUT_PULLUP);
  
  //设置外部中断
  attachInterrupt(digitalPinToInterrupt(18), L2up, CHANGE);
  attachInterrupt(digitalPinToInterrupt(19), L1up, CHANGE);
  attachInterrupt(digitalPinToInterrupt(20), R2up, CHANGE);
  attachInterrupt(digitalPinToInterrupt(21), R1up, CHANGE);

  
  L1PID.SetMode(AUTOMATIC);
  L2PID.SetMode(AUTOMATIC);
  R1PID.SetMode(AUTOMATIC);
  R2PID.SetMode(AUTOMATIC);

  //设置PID阈值
  L1PID.SetOutputLimits(0, 255);
  L2PID.SetOutputLimits(0, 255);
  R1PID.SetOutputLimits(0, 255);
  R2PID.SetOutputLimits(0, 255);
  
  delay(0);
  Serial.begin(19200);
}

void loop() {
  delay(300);
  //arun(100,200,2000);
  /*
  arun(450,450,3900);
  arun(450,150,1680);
  arun(450,450,3400);
  arun(450,285,6500);
  arun(450,450,2800);
  arun(150,450,1600);
  arun(450,450,800);
  */
  
  
  //左转
  
  arun(450,450,4000);
  arun(150,450,1650);
  arun(450,450,2700);
  arun(275,450,6000);
  arun(450,450,2900);
  arun(450,150,1600);
  arun(450,450,800);
  


  /*
  arun(450,450,4200);
  arun(150,450,1600);
  arun(450,450,4100);
  arun(150,450,1600);
  arun(450,450,1900);
  arun(150,450,1600);
  arun(450,450,3600);
  arun(450,150,1400);
  arun(450,450,1000);
  */
  /*
  arun(300,300,6500);
  arun(100,310,2100);
  arun(300,300,6500);
  arun(100,310,2000);
  arun(300,300,2900);
  arun(100,310,2000);
  arun(300,300,5000);
  arun(300,200,4000);
  */



  
  //赛道二（左转）
  /*
  arun(300,300,2500);
  arun(100,310,1800);
  arun(300,300,2000);
  arun(235,310,6300);
  */
  //赛道二（右转）
  /*
  arun(400,400,1500);
  arun(310,100,1750);
  arun(400,400,1800);
  arun(450,305,4500);
  */

  //赛道三（左转）
  /*
  arun(400,400,4020);
  arun(100,400,1300);
  arun(400,400,2500);
  arun(100,400,1300);
  arun(400,400,3700);
  arun(100,400,1350);
  // arun(-200,400,800);
  arun(400,410,3200);
  */
  

  //赛道三（右转）
  /*
  arun(300,300,5600);
  arun(310,100,1750);
  arun(310,300,2300);
  arun(310,100,1700);
  arun(300,300,6000);
  */

  //赛道一
  //arun(400,400,6300);

  
  analogWrite(8,0);
  analogWrite(9,0);
  analogWrite(6,0);
  analogWrite(7,0);
  delay(2000);
  //arun(100,300,2480);
  analogWrite(8,0);
  analogWrite(9,0);
  analogWrite(6,0);
  analogWrite(7,0);
  //arun(-20,-20,5000);
  Serial.print("done\n");
  while(true){
    
    }
  


  
  /*
  Serial.print("  L2val:");
  Serial.print(L2val);
  Serial.print("  L1val:");
  Serial.print(L1val);
  Serial.print("  R1val:");
  Serial.print(R1val);
  Serial.print("  R2val:");
  Serial.print(R2val);
  Serial.print("\n");
  
  L2val=0;
  L1val=0;
  R1val=0;
  R2val=0;
  
  */
  
  
  

}
void forward(int distime){
/*
  analogWrite(2,R1);//右前轮
  analogWrite(3,R2);//右后轮
  analogWrite(4,L2);//左后轮
  analogWrite(5,L1);//左前轮
*/
  
}
void arun(double leftspeed,double rightspeed,long lasttime){
  LSetpoint=leftspeed;
  RSetpoint=rightspeed;
  long uptime = millis();
  while(millis()-uptime<lasttime){
    bal=L1distance+L2distance-R1distance-R2distance;
    if ( millis() - L1last > 100)
    {
      double L1rpm = L1count_rpm();
      L1distance+=L1rpm;
      L1Input = map(L1rpm, 0, 105 , 0, 255);
    }
    
    if ( millis() - L2last > 100)
    {
      double L2rpm = L2count_rpm();
      L2distance+=L2rpm;
      L2Input = map(L2rpm, 0, 105 , 0, 255);
    }
    if ( millis() - R1last > 100)
    {
      double R1rpm = R1count_rpm();
      R1distance+=R1rpm;
      R1Input = map(R1rpm, 0, 105 , 0, 255);
    }
    if ( millis() - R2last > 100)
    {
      double R2rpm = R2count_rpm();
      R2distance+=R2rpm;
      R2Input = map(R2rpm, 0, 105 , 0, 255);
    }
    
    
    L1PID.Compute();
    analogWrite(7,L1Output);
    Serial.print("L1Input:");
    Serial.print(L1Input);
    Serial.print("L1Output:");
    Serial.print(L1Output);
    
    L2PID.Compute();
    analogWrite(6,L2Output);
    //Serial.print("L2Input:");
    //Serial.print(L2Input);
    //Serial.print("L2Output:");
    //Serial.print(L2Output);
    
    R1PID.Compute();
    analogWrite(8,R1Output);
    Serial.print("R1Input:");
    Serial.print(R1Input);
    Serial.print("R1Output:");
    Serial.print(R1Output);
    R2PID.Compute();
    analogWrite(9,R2Output);
    //Serial.print("R2Input:");
    //Serial.print(R2Input);
    //Serial.print("L1distance:");
   // Serial.print(L1distance);
    //Serial.print("  R1distance:");
    //Serial.print(R1distance);
     //Serial.print("L2distance:");
    //Serial.print(L2distance);
    //Serial.print("  R2distance:");
    //Serial.print(R2distance);
    //Serial.print("  bal:");
    //Serial.print(bal);
    
    Serial.print("\n");
    
    delay(100);
  }  
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

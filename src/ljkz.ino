#include <SimpleFOC.h>
#include "BluetoothSerial.h"
#include "valuepack.h"

#define RXD1 18
#define TXD1 23

unsigned long now_us = 0;
unsigned long now_us1 = 0;
////////////////////////云台角度控制///////////////////
struct{
    float Target;        //定义目标数值
    float TargetBias;        //定义目标数值偏差
    float Now;        //定义当前数值
    float Pre;        //定义上一个数值
    float Sensitivity;        //定义灵敏度
    float ActualOut;        //定义实际输出
    float Error;              //定义偏差值
    float Error_last;         //定义上一个偏差值
    float Error_last1;         //定义上一个偏差值
    float Kp,Ki,Kd;         //定义比例、积分、微分系数
    float PIDout;           //定义控制执行器的变量
    float Integral;         //定义积分值
    float Derivative;       //定义微分
    float Speed;     //角速度 
}Pitch,Yaw;
/////////////////////////////////////////////////////////

/////////////////////云台角度环蓝牙调试PID/////////////////
RxPack rxpack;
unsigned char buffer[50];
extern unsigned char vp_rxbuff[VALUEPACK_BUFFER_SIZE];
extern long rxIndex;
unsigned short vp_circle_rx_index;//环形缓冲区index
/////////////////////////////////////////////////////////

////////////////////////蓝牙主机连接从机///////////////////
BluetoothSerial SerialBT;
const char* deviceName = "HC-05";  // 要连接的设备名称
// RxPack rxpack;
// unsigned char buffer[50];
// extern unsigned char vp_rxbuff[VALUEPACK_BUFFER_SIZE];
// extern long rxIndex;
/////////////////////////////////////////////////////////

////////////////////////////双板通信/////////////////////////////////
unsigned char recstatu = 0;//表示是否处于一个正在接收数据包的状态
unsigned char ccnt = 0;//计数
uint8_t RxIndex = 9;//接收数据包字节数
uint8_t TxIndex = 9;//发送数据包字节数
unsigned char packerflag = 0;//是否接收到一个完整的数据包标志
unsigned char rxbuf[9] = {0,0,0,0,0,0,0,0,0};//接收数据的缓冲区
unsigned char txbuf[9] = {0,0,0,0,0,0,0,0,0};
unsigned char dat = 0;
////////////////////////////////////////////////////////////////////

////////////////////////////双机通信/////////////////////////////////
unsigned char recstatuBluetooth = 0;//表示是否处于一个正在接收数据包的状态
unsigned char ccntBluetooth = 0;//计数
uint8_t RxIndexBluetooth = 9;//接收数据包字节数
uint8_t TxIndexBluetooth = 9;//发送数据包字节数
uint8_t TryTime = 1;//尝试连接次数
unsigned char packerflagBluetooth = 0;//是否接收到一个完整的数据包标志
unsigned char rxbufBluetooth[9] = {0,0,0,0,0,0,0,0,0};//接收数据的缓冲区
unsigned char txbufBluetooth[9] = {0,0,0,0,0,0,0,0,0};
unsigned char datBluetooth = 0;
int CommunicationTime = 0;
int CommunicationGap = 50;
unsigned char X_Speed,Y_Speed;
////////////////////////////////////////////////////////////////////

float Amotor_speed = 0;   //A电机当前速度
float Bmotor_speed = 0;

int js = 0;

float setAsd = 0;//设置A电机速度
float setBsd = 0;//设置B电机速度

// MagneticSensorSPI(int cs, float _cpr, int _angle_register)
// config           - SPI config
//  cs              - SPI chip select pin 
MagneticSensorSPI sensor1 = MagneticSensorSPI(AS5147_SPI, 27);
MagneticSensorSPI sensor2 = MagneticSensorSPI(AS5147_SPI, 15);

//MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
//MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);
//TwoWire I2Cone = TwoWire(0);
//TwoWire I2Ctwo = TwoWire(1);

// these are valid pins (mosi, miso, sclk) for 2nd SPI bus on storm32 board (stm32f107rc)
SPIClass * hspi = NULL;

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(33, 25, 26, 32);

BLDCMotor motor2 = BLDCMotor(11);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(0, 16, 5, 17);

float target_angle = 0;      // 初始目标角度为0 


void setup() {
  // 编码器设置
  hspi = new SPIClass(HSPI);
  hspi->begin(14, 12, 13);//(sck, miso, mosi)
  //initialise magnetic sensor1 hardware
  sensor1.init(hspi);  
  sensor2.init(hspi);  

  //I2Cone.begin(12, 13, 400000);
  //I2Ctwo.begin(14, 15, 400000);   //SDA1,SCL1
  //sensor1.init(&I2Cone);
  //sensor2.init(&I2Ctwo);
  
  motor.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);
  // 驱动设置
  driver.pwm_frequency = 100000;
  driver.voltage_power_supply = 12;
  driver.init();
  motor.linkDriver(&driver);

  driver2.pwm_frequency = 100000;
  driver2.voltage_power_supply = 12;
  driver2.init();
  motor2.linkDriver(&driver2);

  // 选择调制方式为SVPWM
// 选择FOC调制类型
// FOCModulationType::SinePWM; （默认）
// FOCModulationType::SpaceVectorPWM;
// FOCModulationType::Trapezoid_120;
// FOCModulationType::Trapezoid_150;
motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
motor2.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // 控制模式为角度模式
  motor.controller = MotionControlType::velocity;
  motor2.controller = MotionControlType::velocity;
  
  ///*

  //motor.phase_resistance = 0.1; // 12.5 Ohms
  //motor.torque_controller = TorqueControlType::voltage;
  //motor.controller = MotionControlType::torque;
  
  //motor2.phase_resistance = 0.1; // 12.5 Ohms
  //motor2.torque_controller = TorqueControlType::voltage;
  //motor2.controller = MotionControlType::torque;
  //*/
  
  // PID参数
  motor.PID_velocity.P = 2;//0.1;
  motor.PID_velocity.I = 55;//4;
  motor.PID_velocity.D = 0.015;//0;
  motor.P_angle.P = 0.5;

  motor2.PID_velocity.P = 2;//0.1;
  motor2.PID_velocity.I = 55;//4;
  motor2.PID_velocity.D = 0.015;//0;
  motor2.P_angle.P = 0.5;
    
  //其他参数
  motor.voltage_limit = 12;    //最大电压
  motor.velocity_limit = 500;   //最大速度，rad/s
  motor.LPF_velocity.Tf = 0.01;  //速度的滤波时间常数

  motor2.voltage_limit = 12;    //最大电压
  motor2.velocity_limit = 500;   //最大速度，rad/s
  motor2.LPF_velocity.Tf = 0.01;  //速度的滤波时间常数
  // 串口设置
  Serial.begin(500000);
  Serial1.begin(500000, SERIAL_8N1, RXD1, TXD1);  

  //初始化
  motor.init(); 
  motor.initFOC();//(5.8794,Direction::CW);

  motor2.init();  
  motor2.initFOC();//(4.4493,Direction::CCW);

  
  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  _delay(1000);

   ////////////////////////蓝牙主机连接从机///////////////////////////////////
  SerialBT.begin("Head_Driver"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

////////////////////////WiFi连接////////////////////////////////////////////
//Wifi接收摄像头数据

///////////////////////////////////////////////////////////////////////////

//////////////////////////////////定时器读取蓝牙信息////////////////////////
  // hw_timer_t *timer1 = timerBegin(1, 80, true);    //启动定时器
  // timerAttachInterrupt(timer1, &TIME_INTERRUPT_2, true);
  // timerAlarmWrite(timer1, 1000, true);
  // timerAlarmEnable(timer1);
//////////////////////////////////////////////////////////////////////////

///////////////////////////角度控制代码//////////////////////////////////
Pitch.Sensitivity = 0.01;
Pitch.Target = 0;
Pitch.TargetBias = 0.22;
Pitch.Kp = 50;
Pitch.Ki = 0;
Pitch.Kd = 1;

Yaw.Sensitivity = 0.01;
Yaw.Target = 0;
Yaw.TargetBias = 1.19;
Yaw.Kp = 70;
Yaw.Ki = 0;
Yaw.Kd = 10;
////////////////////////////////////////////////////////////////////////
}

void loop() {  
  now_us = millis();

  // setAsd=50;
  // setBsd=50;

  motor.loopFOC();
  motor2.loopFOC();
  motor.move(setAsd);
  motor2.move(setBsd);
  
  Read_serial1(); //串口读数据
  if (millis() - CommunicationTime > CommunicationGap)
  {
    CommunicationTime = millis();
    Send_data_to_Slave(); //向主机发数据
  }
  
  Read_data_from_Slave(); //蓝牙从下板读命令数据

  // PIDTuning();//蓝牙调试PID参数
  // readValuePack(&rxpack);//接收遥控器数据包

  if(js>3)
  {
    setAsd = 0;
    setBsd = 0;
  } 
  if((now_us-now_us1)>=10)
  {
    js++;
    //Serial.println((now_us-now_us1));
    Pitch.Now = sensor1.getAngle();
    Yaw.Now = sensor2.getAngle();

    Pitch.Speed = (Pitch.Now - Pitch.Pre)/0.01;
    Pitch.Error = Pitch.Now - (Pitch.Target+Pitch.TargetBias);
    Pitch.Integral += Pitch.Ki * Pitch.Error;
    Pitch.Derivative = Pitch.Kd * Pitch.Speed;
    Pitch.PIDout = Pitch.Kp * Pitch.Error + Pitch.Integral + Pitch.Derivative;

    Yaw.Speed = (Yaw.Now - Yaw.Pre)/0.01;
    Yaw.Error = Yaw.Now - (Yaw.Target+Yaw.TargetBias);
    Yaw.Integral += Yaw.Ki * Yaw.Error;
    Yaw.Derivative = Yaw.Kd * Yaw.Speed;
    Yaw.PIDout = Yaw.Kp * Yaw.Error + Yaw.Integral + Yaw.Derivative;

    Pitch.Pre = Pitch.Now; 
    Yaw.Pre = Yaw.Now;    
    
    setAsd = Pitch.PIDout;
    setBsd = Yaw.PIDout;
    Read_motor_speed((int)(Amotor_speed*100),(int)(Bmotor_speed*100));
    now_us1 = now_us;
    // Serial.println(Amotor_speed,3);

    // 打印编码器角度
    Serial.print(Pitch.Now);
    Serial.print(" ");
    Serial.println(Yaw.Now);

    //打印PID控制效果
    // Serial.print(Pitch.Now);
    // Serial.print(",");
    // Serial.print(Pitch.Target);
    // Serial.print("\n");
  } 
  
}

void Send_data_to_Slave()
{   
  
  txbufBluetooth[0]=111;
  txbufBluetooth[1]=66;
  txbufBluetooth[2]=rxbuf[6];// X speed
  txbufBluetooth[3]=rxbuf[7];// Y speed
  txbufBluetooth[4]=0;
  txbufBluetooth[5]=0;
  txbufBluetooth[6]=0;
  txbufBluetooth[7]=0;
  txbufBluetooth[TxIndexBluetooth-1]=crc2(txbufBluetooth);
  SerialBT.write(txbufBluetooth,sizeof(txbufBluetooth));
  
}

void PIDTuning()//用蓝牙进行PID调试
{
  if (SerialBT.available()) 
  {
    vp_rxbuff[vp_circle_rx_index]=SerialBT.read();
	  vp_circle_rx_index++;
  	if(vp_circle_rx_index>=VALUEPACK_BUFFER_SIZE)
    	vp_circle_rx_index=0;
      rxIndex++;
  }
  Pitch.Kp = rxpack.floats[0];
  Pitch.Ki = rxpack.floats[1];
  Pitch.Kd = rxpack.floats[2];

  Yaw.Kp = rxpack.floats[3];
  Yaw.Ki = rxpack.floats[4];
  Yaw.Kd = rxpack.floats[5];

//打印蓝牙PID调试参数
  Serial.print(Pitch.Kp);
  Serial.print(" ");
  Serial.print(Pitch.Ki);
  Serial.print(" ");
  Serial.print(Pitch.Kd);
  Serial.print(" ");
  Serial.print(Yaw.Kp);
  Serial.print(" ");
  Serial.print(Yaw.Ki);
  Serial.print(" ");
  Serial.println(Yaw.Kd);
}

void Read_data_from_Slave() //串口读数据
{ 
  if (SerialBT.available() > 0) 
  {
     datBluetooth = SerialBT.read();  
     //Serial.print(datBluetooth);  
     if((ccntBluetooth==0)&&(datBluetooth == 111))
     {
       rxbufBluetooth[ccntBluetooth] = datBluetooth;
       ccntBluetooth = 1;
     }
     else if((ccntBluetooth==1)&&(datBluetooth == 66))
     {
       rxbufBluetooth[ccntBluetooth] = datBluetooth;
       recstatuBluetooth = 1;
       ccntBluetooth = 2; 
     }
     else if(recstatuBluetooth == 1) //表示是否处于一个正在接收数据包的状态
     {
          rxbufBluetooth[ccntBluetooth] = datBluetooth; 
          ccntBluetooth++;
          if(ccntBluetooth==RxIndexBluetooth)
          {
            if(crc1(rxbufBluetooth))
            {
                recstatuBluetooth = 0;
                packerflagBluetooth = 1;//用于告知系统已经接收成功
                ccntBluetooth = 0;  
                
                //Serial.println("Data from slave:");   
                // Serial.print(rxbufBluetooth[0]); 
                // Serial.print(" ");
                // Serial.print(rxbufBluetooth[1]); 
                // Serial.print(" ");
                // Serial.print(rxbufBluetooth[2]); 
                // Serial.print(" ");
                // Serial.print(rxbufBluetooth[3]); 
                // Serial.print(" ");
                // Serial.print(rxbufBluetooth[4]); 
                // Serial.print(" ");
                // Serial.print(rxbufBluetooth[5]); 
                // Serial.print(" ");
                // Serial.print(rxbufBluetooth[6]); 
                // Serial.print(" ");
                // Serial.println(rxbufBluetooth[7]);  
                
                Pitch.Target = Pitch.Sensitivity * (rxbufBluetooth[3]-127);
                if(Pitch.Target > 0.4) Pitch.Target = 0.4;
                if(Pitch.Target < -0.06) Pitch.Target = -0.06;
                Yaw.Target = Yaw.Sensitivity * (rxbufBluetooth[2]-127);
                if(Yaw.Target > 1) Yaw.Target = 1;
                if(Yaw.Target < -1) Yaw.Target = -1;
     
            }
            else
            {
                rxbufBluetooth[0] = 0;
                rxbufBluetooth[1] = 0;
                recstatuBluetooth = 0;
                packerflagBluetooth = 0;//用于告知系统已经接收失败
                ccntBluetooth = 0;       
                Serial.println("Communication error!");                     
            }                        
          }  
      }
      else
      {
          rxbufBluetooth[0] = 0;
          rxbufBluetooth[1] = 0;
          recstatuBluetooth = 0;
          packerflagBluetooth = 0;//用于告知系统已经接收失败
          ccntBluetooth = 0;
          datBluetooth = 0;
          Serial.println("on1.............................."); 
      }         
  }
}

void Read_motor_speed(int MA, int MB)
{   
  if(MA>30000)
  MA = 30000;
  if(MA<(-30000))
  MA = -30000;

  if(MB>30000)
  MB = 30000;
  if(MB<(-30000))
  MB = -30000;
  
  int ta = MA+32767;
  int tb = MB+32767;
  txbuf[0]=111;
  txbuf[1]=66;
  txbuf[2]=ta>>8;
  txbuf[3]=ta&0xff;
  txbuf[4]=tb>>8;
  txbuf[5]=tb&0xff;
  txbuf[6]=1;
  txbuf[7]=2;
  txbuf[TxIndex-1]=crc2(txbuf);
  Serial1.write(txbuf,sizeof(txbuf));
  
}


boolean crc1(unsigned char buffer[])
{
  unsigned int crc_bit1=0;
  unsigned int sum1=0;
  
  for (int j = 2; j <= (RxIndex-2); j++)
  {
    sum1 += buffer[j];
  }
  crc_bit1 = sum1 & 0xff;
  if ((unsigned char)crc_bit1 == buffer[RxIndex-1])
    return true;
  else
    return false;
}

unsigned char crc2(unsigned char buffer[])
{
  unsigned int crc_bit1=0;
  unsigned int sum1=0;
  
  for (int j = 2; j <= (TxIndex-2); j++)
  {
    sum1 += buffer[j];
  }
  crc_bit1 = sum1 & 0xff;

  return (unsigned char)crc_bit1;
}

void Read_serial1() //串口读数据
{ 
  if (Serial1.available() > 0) 
  {
     dat = Serial1.read();  
     //Serial.print(dat);  
     if((ccnt==0)&&(dat == 111))
     {
       rxbuf[ccnt] = dat;
       ccnt = 1;
     }
     else if((ccnt==1)&&(dat == 66))
     {
       rxbuf[ccnt] = dat;
       recstatu = 1;
       ccnt = 2; 
     }
     else if(recstatu == 1) //表示是否处于一个正在接收数据包的状态
     {
          rxbuf[ccnt] = dat; 
          ccnt++;
          if(ccnt==RxIndex)
          {
            if(crc1(rxbuf))
            {
                recstatu = 0;
                packerflag = 1;//用于告知系统已经接收成功
                ccnt = 0;  
                int x1=rxbuf[2];
                x1<<=8;
                x1+=rxbuf[3];
                int x2=rxbuf[4];
                x2<<=8;
                x2+=rxbuf[5];
      
                // X_Speed = rxbuf[6];
                // X_Speed = rxbuf[7];
                // setAsd = (x1-32767)*0.01;
                // setBsd = (x2-32767)*0.01;

                // Serial.println("Data from controller:");
                // Serial.print(sensor1.getAngle());
                // Serial.print(" ");
                // Serial.print(sensor2.getAngle());
                // Serial.print(" ");
                // Serial.print(setAsd);
                // Serial.print(" ");
                // Serial.print(setBsd); 
                // Serial.print(" ");
                // Serial.print(rxbuf[6]); 
                // Serial.print(" ");
                // Serial.println(rxbuf[7]); 
                js = 0;     
     
            }
            else
            {
                rxbuf[0] = 0;
                rxbuf[1] = 0;
                recstatu = 0;
                packerflag = 0;//用于告知系统已经接收失败
                ccnt = 0;       
                Serial.println("Communication error!");                     
            }                        
          }  
      }
      else
      {
          rxbuf[0] = 0;
          rxbuf[1] = 0;
          recstatu = 0;
          packerflag = 0;//用于告知系统已经接收失败
          ccnt = 0;
          dat = 0;
          //Serial.println("on1.............................."); 
      }         
  }
}

// void TIME_INTERRUPT_2()//定时中断 中断服务函数
// {
//   Read_data_from_Slave();
// }
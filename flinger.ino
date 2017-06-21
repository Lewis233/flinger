
/*Controlling unmanned vehical by gestures based on arduino
	2017/5/28 by Lewis233
	Changelog:
		2017/5/28 initial release
*/
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "HMC5883L.h"

#define FIRST_FINGER 2
#define MIDDLE_FINGER 3
#define RING_FINGER 4
#define BUZZER 5

#define CE 8
#define CSN 7
#define MOSI 11
#define MISO 12
#define SCK 13
#define uchar unsigned char

//*********************************************NRF24L01*************************************
uchar const TX_ADDRESS[5]={0x00,0x00,0x00,0x00,0xA2};//transmiss address
uchar const RX_ADDRESS[5]={0x00,0x00,0x00,0x00,0xA1};//receive address
unsigned char TxBuf[12]; //transmission buffer
unsigned char RxBuf[12]; //receiver buffer
//***************************************NRF24L01_Register*******************************************************
#define ADR_WIDTH       5   	// 3-5  address length
#define PLOAD_WIDTH     12    	// 1-32 data package length
#define READ_REG        0x00  	//
#define WRITE_REG       0x20 	//
#define RD_RX_PLOAD     0x61  	// length of data
#define WR_TX_PLOAD     0xA0  	// length of data
#define FLUSH_TX        0xE1 	// clear all transmission data
#define FLUSH_RX        0xE2  	// clear all receiver data
#define REUSE_TX_PL     0xE3  	//
#define NOP             0xFF  	// reserve
//*************************************SPI(nRF24L01)Register****************************************************
#define CONFIG          0x00  // working mode
#define EN_AA           0x01  // enable pipe
#define EN_RXADDR       0x02  // enable address
#define SETUP_AW        0x03  //
#define SETUP_RETR      0x04  //
#define RF_CH           0x05  // setup the frequence
#define RF_SETUP        0x06  // setup the pipe
#define STATUS          0x07  //
#define OBSERVE_TX      0x08  //
#define CD              0x09  //
#define RX_ADDR_P0      0x0A  //
#define RX_ADDR_P1      0x0B  //
#define RX_ADDR_P2      0x0C  //
#define RX_ADDR_P3      0x0D  //
#define RX_ADDR_P4      0x0E  //
#define RX_ADDR_P5      0x0F  //
#define TX_ADDR         0x10  // transmission address
#define RX_PW_P0        0x11  // receiver address 0
#define RX_PW_P1        0x12  //
#define RX_PW_P2        0x13  //
#define RX_PW_P3        0x14  //
#define RX_PW_P4        0x15  //
#define RX_PW_P5        0x16  //
#define FIFO_STATUS     0x17  //

#define NTD0 -1
#define NTD1 294
#define NTD2 330
#define NTD3 350
#define NTD4 393
#define NTD5 441
#define NTD6 495
#define NTD7 556
#define NTDL1 147
#define NTDL2 165
#define NTDL3 175
#define NTDL4 196
#define NTDL5 221
#define NTDL6 248
#define NTDL7 278
#define NTDH1 589
#define NTDH2 661
#define NTDH3 700
#define NTDH4 786
#define NTDH5 882
#define NTDH6 990
#define NTDH7 112

#define WHOLE 1
#define HALF 0.5
#define QUARTER 0.25
#define EIGHTH 0.25
#define SIXTEENTH 0.625
//******************************************************************************************
uchar sta = 0;
uchar	RX_DR	=sta^6;
uchar	TX_DS	=sta^5;
uchar	MAX_RT	=sta^4;
uchar SSLL=0;//to judge whether the vehicle is out of control by the signal received
uchar SZML=1;//unlock the vehicle

MPU6050 accelgyro;
HMC5883L compass;

int status = 0;
bool flag = 0;//to judge whether need ajust the compass or not
bool flag_land = 0;//to judge whether the plane is returning
int ax, ay, az;
int gx, gy, gz;
float init_angle = 0.0;
float present_angle = 0.0;
float init_depre = 0.0;//depression angle
float error = 30.0;//the acceptible error of the compass

void setup(){
	pinMode(MOSI, OUTPUT);
	pinMode(MISO, INPUT);
	pinMode(SCK, OUTPUT);
	pinMode(CSN, OUTPUT);
	pinMode(CE, OUTPUT);
	//Set the finger switch
	pinMode(FIRST_FINGER, INPUT);
	pinMode(MIDDLE_FINGER, INPUT);
	pinMode(RING_FINGER, INPUT);
	//Set the buzzer
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER,HIGH);

	Wire.begin();
  delay(2000);
	Serial.begin(9600);

	// initialize MPU6050
  //Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
	// verify connection
  //Serial.println("Testing device connections...");
  //Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

	// Initialize Initialize HMC5883L
  //Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    //Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);

	init_NRF24L01() ;
	nRF24L01_RX_Mode(RxBuf);//receive data and put it into RxBuf
	delay(1);

	adjust();
	buff_init();
	music3();

 while(readSta == 0){
  SZML = 0;
  sendWifi(10);
  delay(10);
 }
}

void loop(){
	accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	status = readSta();
	if(status == 0){
		SZML = 3;
		sendWifi(10);//transmiss
		delay(1);
	}
	else if(status == 1){
		SZML = 1;
		readCompass();
	}
		else if(status == 2){
			SZML = 1;
			if(az < 0){
				delay(50);
				if(az < 0) ready_land();
			}
			else{
				SZML = 1;
				accelerate(calDep());
			}
		}
}

void front(){
	Serial.print("front\n");
	TxBuf[5] = 188;
  TxBuf[6] = 128;
	sendWifi(20);//transmiss
	delay(1);
}

void left(){
	Serial.print("left\n");
	TxBuf[6] = 68;
	TxBuf[5] = 128;
	sendWifi(20);//transmiss
	delay(1);
}

void right(){
	Serial.print("right\n");
	TxBuf[6] = 188;
	TxBuf[5] = 128;
	sendWifi(20);//transmiss
	delay(1);
}

void back(){
	Serial.print("back\n");
	TxBuf[5] = 68;
	TxBuf[6] = 128;
	sendWifi(20);//transmiss
	delay(1);
}

void notpoint(){
	//Serial.print("notpoint\n");
  TxBuf[5] = 128;
  TxBuf[6] = 128;
	sendWifi(10);//transmiss
	delay(50);
}

void accelerate(float v){
	//Serial.print("The accelerator goes to ");
	//Serial.println(v);
	//Serial.print(v);
	//Serial.print(' ');
	int val = int(350 + 8*v);
	if(val > 1000) val = 650;
	if(val < 0) val = 0;
	//Serial.print(val);
  //Serial.print(' ');
	TxBuf[2] = val/256;
	TxBuf[3] = val%256;
  TxBuf[5] = 128;
  TxBuf[6] = 128;
 //Serial.print(TxBuf[2]);
 //Serial.print(' ');
 //Serial.println(TxBuf[3]);
 
	sendWifi(20);//transmiss
	delay(1);
}

char readSta(){
	int f = digitalRead(FIRST_FINGER);
	int m = digitalRead(MIDDLE_FINGER);
	int r = digitalRead(RING_FINGER);

	if(f == 0 && m == 0) return 0;
	if(f == 1 && m == 0) return 1;
	if(f == 1 && m == 1) return 2;
}

void readCompass(){
	Vector raw = compass.readRaw();//.XAxis .YAxis
  present_angle = angle_ceil(calAn(raw.XAxis, raw.YAxis) - init_angle);
  //Serial.print(present_angle);
  if(angle_ceil(0-error) < present_angle || present_angle < angle_ceil(0+error))
	  front();
	else if(angle_ceil(90-error) < present_angle && present_angle < angle_ceil(90+error))
			right();
		else if(angle_ceil(180-error) < present_angle && present_angle < angle_ceil(180+error))
				back();
			else if(angle_ceil(270-error) < present_angle && present_angle < angle_ceil(270+error))
					left();
	else notpoint();

	delay(1);
}

//calculate the depression angle
float calDep(){
	float x = float(ax / 1000);
	float z = float(az / 1000);
	float a = asin(x/sqrt(x*x + z*z))/(2*3.14)*360;
	return (a - init_depre);
}

//generate different sound
void bee(char type){
	switch(type){
		case 0:{
			for(int i = 20; i >= 0; --i){
				tone(BUZZER, 589);
				delay(40);
				noTone(BUZZER);
			}
		};break;
		case 1:{
			for(int i = 20; i >= 0; --i){
				tone(BUZZER, 380);
				delay(40);
				noTone(BUZZER);
			}
		};break;
		case 2:{
			for(int i = 20; i >= 0; --i){
				tone(BUZZER, 180);
				delay(40);
				noTone(BUZZER);
			}
		};break;
	}
}

float calAn(int x, int y){
	float theta = atan(abs(float(y))/abs(float(x)))/(2*3.14)*360;
	if (x > 0 && y > 0)
		return theta;
	if (x < 0 && y > 0)
		return (180 - theta);
	if (x < 0 && y < 0)
		return (180 + theta);
	if (x > 0 && y < 0)
		return (360 - theta);

		return theta;
}

float angle_ceil(float x){
	while(x > 360.0){
		x = x - 360.0;
	}
	while(x < 0.0){
		x = x + 360.0;
	}
 return x;
}

void adjust(){
	//Serial.println("Adusting the compass");
	music2();
	int x_sum = 0;
	int y_sum = 0;
	int angle_length = 0;
	//wait to be stable
	for(int i = 20; i >= 0; --i){
		Vector raw = compass.readRaw();//.XAxis .YAxis
		//Serial.println(calAn(raw.XAxis, raw.YAxis));
		delay(100);
	}
	for(int j = 20; j >= 0; --j){
		Vector raw = compass.readRaw();
		x_sum += raw.XAxis;
		y_sum += raw.YAxis;
		angle_length += 1;
		delay(100);
	}
	init_angle = calAn(float(x_sum)/angle_length, float(y_sum)/angle_length);
	//Serial.println("Adjusting the accelerator");
	float depre = 0.0;
	int depre_length = 0;
	for(int k = 20; k >= 0; --k){
		accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		delay(100);
	}
	for(int l = 20; l >= 0; --l){
		accelgyro.getMotion6(&az,&ay,&az,&gx,&gy,&gz);
		float x = ax / 1000.0;
		float z = az / 1000.0;
		depre += asin(x/sqrt(x*x + z*z))/(2*3.14)*360;
		++depre_length;
	}
	init_depre = depre/depre_length;
	//Serial.println(init_depre);
	//Serial.println("Adjustment finished");
}

void ready_land(){
  TxBuf[5] = 128;
  TxBuf[6] = 128;
  sendWifi(20);
	music1();
	delay(5000);
}

void buff_init(){
	TxBuf[0] = 0;
	TxBuf[1] = 0;
	TxBuf[2] = 0;
	TxBuf[3] = 50;
	TxBuf[4] = 128;
	TxBuf[5] = 128;
	TxBuf[6] = 128;
	TxBuf[7] = 0;
	TxBuf[8] = 123;
	TxBuf[9] = 117;
	TxBuf[10] = 128;
}

void sendWifi(char times){
	TxBuf[1] = SZML;
	for(char i = times; i > 0; --i){
		if(SSLL > 253){SSLL = 0;}
		++SSLL;
		TxBuf[0] = SSLL;
		nRF24L01_TX_Mode(TxBuf);
		delay(1);
		nRF24L01_RX_Mode(RxBuf);//receive
		delay(1);
		for(char k = 0; k < 12; ++k){
    Serial.print(TxBuf[k]);
    Serial.print(' ');
		}
    Serial.println();
	}
}

uchar SPI_RW(uchar reg)
{
	uchar bit_ctr;
   	for(bit_ctr=0;bit_ctr<8;bit_ctr++) // output 8-bit
   	{
		digitalWrite(MOSI, (reg & 0x80));         // output 'uchar', MSB to MOSI
		reg = (reg << 1);           // shift next bit into MSB..
		digitalWrite(SCK, HIGH);                      // Set SCK high..
		reg |= digitalRead(MISO);       		  // capture current MISO bit
		digitalWrite(SCK, LOW);            		  // ..then set SCK low again
   	}
    return(reg);           		  // return read uchar
}
/****************************************************************************************************
/*SPI read
/****************************************************************************************************/
uchar SPI_Read(uchar reg)
{
	uchar reg_val;
	digitalWrite(CSN, LOW);                // CSN low, initialize SPI communication...
	SPI_RW(reg);            // Select register to read from..
	reg_val = SPI_RW(0);    // ..then read registervalue
	digitalWrite(CSN, HIGH);                // CSN high, terminate SPI communication
	return(reg_val);        // return register value
}
/****************************************************************************************************/
/*SPI read&write
/****************************************************************************************************/
uchar SPI_RW_Reg(uchar reg, uchar value)
{
	uchar status;
	digitalWrite(CSN, LOW);                   // CSN low, init SPI transaction
	status = SPI_RW(reg);      // select register
	SPI_RW(value);             // ..and write value to it..
	digitalWrite(CSN, HIGH);                   // CSN high again
	return(status);            // return nRF24L01 status uchar
}
/****************************************************************************************************/
/*SPI read
/****************************************************************************************************/
void SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars)
{
	uchar i;
	digitalWrite(CSN, LOW);                    		// Set CSN low, init SPI tranaction
	SPI_RW(reg);       		// Select register to write to and read status uchar
	for(i=0;i<uchars;i++)
	pBuf[i] = SPI_RW(0);    //
	digitalWrite(CSN, HIGH);
}
/*********************************************************************************************************
/*SPI write
/*********************************************************************************************************/
void SPI_Write_Buf(uchar reg, uchar *pBuf, uchar uchars)
{
	uchar status,i;
	digitalWrite(CSN, LOW);            //set CSN low
	status = SPI_RW(reg);
	for(i=0; i<uchars;i++) //
		SPI_RW(*pBuf++);
	digitalWrite(CSN, HIGH);           //set CSN high
}
/***********************************************************************************************************
/*transmiss the buffer data
/**********************************************************************************************************/
void nRF24L01_TX_Mode(unsigned char * TX_buf)
{
	digitalWrite(CE, LOW);			//StandBy
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, RX_ADDRESS, ADR_WIDTH);
	SPI_Write_Buf(WR_TX_PLOAD, TX_buf, PLOAD_WIDTH);
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);
	digitalWrite(CE, HIGH);
	delay(2);
//	Delay(250);
}
/****************************************************************************************************/
/*receive buffer data
/****************************************************************************************************/
void nRF24L01_RX_Mode(unsigned char * RX_buf)
{
	digitalWrite(CE, LOW);
    SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, ADR_WIDTH);
	SPI_RW_Reg(WRITE_REG + CONFIG, 0x0f);
	sta=SPI_Read(STATUS);
	if(RX_DR)
	SPI_Read_Buf(RD_RX_PLOAD,RX_buf,PLOAD_WIDTH);
	SPI_RW_Reg(WRITE_REG+STATUS,sta);
	digitalWrite(CE, HIGH);
    delay(2);
}
/****************************************************************************************
  init nrf24l01
//***************************************************************************************/
void init_NRF24L01(void)
{

	digitalWrite(CE, LOW);        // set CE low
	digitalWrite(CSN, HIGH);        // set CSN high
	digitalWrite(SCK, LOW);        // set SCK low
	delay(2);
 	digitalWrite(CE, LOW);    // chip enable
 	digitalWrite(CSN, HIGH);   // Spi  disable
 	digitalWrite(SCK, LOW);   //
   delay(2);
	SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS,ADR_WIDTH);    // write transmission address
	SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, RX_ADDRESS,ADR_WIDTH); // write receiver address
	SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      //  enable pipe
	SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  //  enable receiver address 0
	SPI_RW_Reg(WRITE_REG + RF_CH, 0);        //   set frequence 2.4G
	SPI_RW_Reg(WRITE_REG + RX_PW_P0, PLOAD_WIDTH); // write the length of data
	SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);   		//set the intensity of the signal
    delay(2);
}

void music1(){
  int tune[]=
{
NTDH5,NTDH3,NTDH4,NTDH5,NTDH3,NTDH4,NTDH5,NTD5,NTD6,NTD7,NTDH1,NTDH2,NTDH3,NTDH4,
NTDH3,NTDH1,NTDH2,NTDH3,NTD3,NTD4,NTD5,NTD6,NTD5,NTD4,NTD5,NTD3,NTD4,NTD5,
NTD4,NTD6,NTD5,NTD4,NTD3,NTD2,NTD3,NTD2,NTD1,NTD2,NTD3,NTD4,NTD5,NTD6
};
float durt[]=
{
0.5,0.25,0.25,0.5,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,
0.5,0.25,0.25,0.5,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,
0.5,0.25,0.25,0.5,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25
};
int length;
length=sizeof(tune)/sizeof(tune[0]);

for(int x=0;x<length;x++)
{
tone(BUZZER,tune[x]);
delay(400*durt[x]);
delay(100*durt[x]);
noTone(BUZZER);
}
digitalWrite(BUZZER,HIGH);
}

void music2(){
  int tune[]=
{
NTDH3,NTDH2,NTDH3,NTDH2,NTDH3,NTD7,NTDH2,NTDH1,NTD6
};
float durt[]=
{
0.5,0.5,0.5,0.5,0.5,0.5,0.5,0.5,1
};
int length;
length=sizeof(tune)/sizeof(tune[0]);
for(int x=0;x<length;x++)
{
tone(BUZZER,tune[x]);
delay(400*durt[x]);
delay(100*durt[x]);
noTone(BUZZER);
}
digitalWrite(BUZZER, HIGH);
}

void music3(){
  int tune[]=
{
NTD7,NTD6,NTD5,NTD6,
NTDH1,NTD0,NTDH2,NTDH1,NTD7,NTDH1,
NTDH3,NTD0,NTDH4,NTDH3,NTDH2,NTDH3,
NTDH7,NTDH6,NTDH5,NTDH6,NTDH7,NTDH6,NTDH5,NTDH6,1248,NTDH6,1248,
NTDH7,NTDH6,NTDH5,NTDH6,
NTDH7,NTDH6,NTDH5,NTDH6,
NTDH7,NTDH6,NTDH5,NTDH4,
NTDH3
};
float durt[]=
{
0.25,0.25,0.25,0.25,
0.5,0.5,0.25,0.25,0.25,0.25,
0.5,0.5,0.25,0.25,0.25,0.25,
0.25,0.25,0.25,0.25,0.25,0.25,0.25,0.25,1,0.5,0.5,
0.5,0.5,0.5,0.5,
0.5,0.5,0.5,0.5,
0.5,0.5,0.5,0.5,
1
};
int length;
length=sizeof(tune)/sizeof(tune[0]);

for(int x=0;x<length;x++)
{
tone(BUZZER,tune[x]);
delay(400*durt[x]);
delay(100*durt[x]);
noTone(BUZZER);
}
digitalWrite(BUZZER, HIGH);
}

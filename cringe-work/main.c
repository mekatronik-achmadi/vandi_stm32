/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "STMSerial.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "stdbool.h"
#include <string.h>
char USART2_read(void);
int USART2_write(int ch);
void _penyelarasan(char *input,char *temp,char *nilai);
void _pembacaan(char* input,char* temp);
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

USART_HandleTypeDef husart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
int state=0;
int a=0,b=0,z,kec,banyakString,awal,tengah, akhir,v1,v3,kode,kecepatan,k=0;
char txData[10];
char rxData [50];
char kod;
char v2,v;
char dataS1[10];
char dataS2[10];
char dataS3[10];
char dataS12[10];
char dataS22[10];
char dataS32[10];
char dataSS12[10];
char dataSS22[10];
char dataSS32[10];
char* token;
char** rest[30];
char ch;
char kec1[64];
char kec2[64];
char kec3[64];
char xx[10];
char yy[10];

/* Private variables ---------------------------------------------------------*/
//=================== Nilai Encoder 1, 2, dan 3==============================
int en1,en2,en3;
int TotalEnc1,TotalEnc2,TotalEnc3;
int tPulsa_P=110;
int Current_P;
int Prev_p;

//================== Variable untuk pewaktu===========================
uint32_t Previous;
uint32_t Current;
int count1,count2;
int Interval = 100;
int Interval2 = 100;
int inter=0;
int range=0;
int WaktuMs;

//=================Variable Persamaan PID motor 1======================
int Error1;
int Last_Error1;
double Sum_Error1;
double SetPoint1 = 0;
double Real;
float PID_term1;
float KP1=4.5;
float KI1=0.01;
float KD1=0.5;

//=================Variable Persamaan PID motor 2======================
int Error2;
int Last_Error2;
double Sum_Error2;
double SetPoint2 = 0;
double Real;
float PID_term2;
float KP2=4.5;
float KI2=0.01;
float KD2=0.5;

//=================Variable Persamaan PID motor 3======================
int Error3;
int Last_Error3;
double Sum_Error3;
double SetPoint3 = 0;
double Real;
float PID_term3;
float KP3=4.5;
float KI3=0.01;
float KD3=0.5;
float RPM1,RPM2,RPM3;

//=================Variable Moving Avereage====================
int ke1,ke2,ke3;
int data[10];
int hasil;

//===================Variable Motor=============================
int Duty1,Duty2,Duty3;
float Wx,Wy,Ww;

//========================== Variable Koordinat  X dan Y=================
float Xt,X0 ;
float Yt,Y0;
float Wt,W0;
float Tht = 0;

float Dt;
float EM1, EM2, EM3;
float Vx,Vy,V0;
float limit_Vx,limit_Vy,limit_Vw;
bool X;
int pe=0;

float VRx,VRy,VRw;
float wI1,wI2,wI3;
//============================Data Regresi================================

int data_1[105],data_2[105],data_3[105];
int RPM1_F,PWM1_F,RPM1_B,PWM1_B,RPM2_F,PWM2_F,RPM2_B,PWM2_B,RPM3_F,PWM3_F,RPM3_B,PWM3_B;

//============================Trajectory Tracking======================
float VX0,VY0,VTh0,VXt,VYt,VTht;
int eX,eY,eth;
float kx  = 0;
float ky  = 0;
float kth = 0;
int l;


//=======================jarak X,Y dan 0============
int jarak_M1,jarak_M2,jarak_M3;
float kor_X,kor_Y;
int n,xT,yT;
double jarak_tempuhX,jarak_sebelumX,jarak_totalX;
double jarak_tempuhY,jarak_sebelumY,jarak_totalY;
double jarak_tempuh0,jarak_sebelum0,jarak_total0;


//======================= kecepatan V ms =================
float Vxms,Vyms,Vwms;

//====================== Trajectory Tracking ===================
double Zxref,Zxt,Zx0,Vzx0,Vzxref;
double Zyref,Zyt,Zy0,Vzy0,Vzyref;
double Z0ref,Z0t,Z00,Vz00,Vz0ref;
double Ztf,Zt,t,DeltaX,DeltaY,Delta0;

//====================== Error Trajectory =================
double EZx_Sekarang,EZx_Sebelum,EZx_Total;
double EZy_Sekarang,EZy_Sebelum,EZy_Total;
double EZ0_Sekarang,EZ0_Sebelum,EZ0_Total;

//===================== Trajetory Tracking ============
float ax0,ax1,ax2,ax3;
float ay0,ay1,ay2,ay3;
float a00,a01,a02,a03;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
float Moving_Average(int input){

	float Hasil_f;
	data[ke1] = input;
	ke1++;
	if(ke1 == 10){
		ke1 =0;
	}

	Hasil_f = (data[0] + data[1] + data[2] + data[3] + data[4] + data[5] + data[6] + data[7] + data[8] + data[9] )/10;

   return 	Hasil_f ;

}

void PID_TERM_1(){
	Error1 = (SetPoint1) - RPM1;
	PID_term1 = (KP1 * Error1) + (KI1 * Sum_Error1) + ((KD1/Interval) * (Error1 - Last_Error1));
	Sum_Error1 += Error1;
	Last_Error1 = Error1;
}

void PID_TERM_2(){
	Error2 = (SetPoint2) - RPM2;
	PID_term2 = (KP2 * Error2) + (KI2 * Sum_Error2) + ((KD2/Interval) * (Error2 - Last_Error2));
	Sum_Error2 += Error2;
	Last_Error2 = Error2;
}

void PID_TERM_3(){
	Error3 = (SetPoint3) - RPM3;
	PID_term3 = (KP3 * Error3) + (KI3 * Sum_Error3) + ((KD3/Interval) * (Error3 - Last_Error3));
	Sum_Error3 += Error3;
	Last_Error3 = Error3;
}

void interval(int Interval){
	/*fungsi ini digunakan untuk mengambil data dalam jangka waktu tertentu.
	 * variable Interval dimasukan dalam orde miliSecond
	 *
	 */
	count1 = __HAL_TIM_GET_COUNTER(&htim1);
	if(count1 == Interval-1){
		   //STM_Serial_PrintInt(count1);
		  // STM_Serial_Print("\n\r");
		   __HAL_TIM_SET_COUNTER(&htim1,0);

		  }
}

void Motor_2(int Kecepatan_1){
	if(Kecepatan_1 > 0){
		htim5.Instance->CCR2 = Kecepatan_1;
		htim5.Instance->CCR1 = 0;
	}
	else if(Kecepatan_1 < 0){
		htim5.Instance->CCR2 = 0;
		htim5.Instance->CCR1 = Kecepatan_1*-1;
	}
	else if(Kecepatan_1 == 0){
		htim5.Instance->CCR2 = 0;
		htim5.Instance->CCR1 = 0;
	}
}

void Motor_1(int Kecepatan_2){
	if(Kecepatan_2 > 0){
		htim3.Instance->CCR3 = Kecepatan_2;
		htim3.Instance->CCR4 = 0;
	}
	else if(Kecepatan_2 < 0){
		htim3.Instance->CCR3 = 0;
		htim3.Instance->CCR4 = Kecepatan_2*-1;
	}
	else if(Kecepatan_2 == 0){
		htim3.Instance->CCR3 = 0;
		htim3.Instance->CCR4 = 0;
	}
}

void Motor_3(int Kecepatan_3){
	if(Kecepatan_3 > 0){
		htim8.Instance->CCR1 = Kecepatan_3;
		htim8.Instance->CCR2 = 0;
	}
	else if(Kecepatan_3 < 0){
		htim8.Instance->CCR1 = 0;
		htim8.Instance->CCR2 = Kecepatan_3*-1;
	}
	else if(Kecepatan_3 == 0){
		htim8.Instance->CCR1 = 0;
		htim8.Instance->CCR2 = 0;
	}
}

void Forward_kinematic(float w1, int w2, int w3){

//	 if (wI1 <0){
//		 w1 = w1*-1;
//	 	 }
//	 if (wI2 <0){
//	 		 w2 = w2*-1;
//	 	 }
//	 if (wI3 <0){
//	 		 w3 = w3*-1;
//	 	}
	Wx = 0.866 * w1  - 0.866 * w2;
	Wy = 0.5   * w1  + 0.5   * w2 - w3;
	Ww = w1/14 + w2/14 + w3/14;

	VRx = (3*2*3.14) * Wx;
	VRy = (3*2*3.14) * Wy;
	VRw = (3*2*3.14) * Ww;
//Punyaku
//	Wx = -0.866 * w1  - 0.866 * w2 - w3;
//	Wy = 0.5   * w1  - 0.5   * w2;
//	Ww = w1/14 + w2/14 + w3/14;
//
//	VRx = (3*2*3.14) * Wx;
//	VRy = (3*2*3.14) * Wy;
//	VRw = (3*2*3.14) * Ww;

}

void Inverse_Kinematic(float x, float y, float w ){
	/*============Persamaan Forward Kinematik  3 Roda===================
	 * th = 30;
	 *
	 * Vx = ( cos(th)*V1 - cos(th)*V2 +  0    )/r
	 * Vy = ( sin(th)*V1 +  sin(th)*V2  -V3   )/r
	 * V0 = (    V1/14   +    V2/14   + V3/14 )/r
	 *
	 * ===========Persamaan Inverse Kinematik 3 Roda=====================
	 *
	 */


	wI1 = ( 0.333*y + 0.5774*x +  4.667*w);
	wI3 = ( 0.333*y - 0.5774*x +  4.667*w);
	wI2 = (-0.667*y + 0        +  4.667*w);

	SetPoint1 = wI1/(3*2*3.14);
	SetPoint2 = wI2/(3*2*3.14);
	SetPoint3 = wI3/(3*2*3.14);

//punyaku
//	wI1 = ( 1*y - 0.268*x +  3.7537*w);
//	wI3 = ( -1*y - 0.268*x +  3.7537*w);
//	wI2 = (0 + 0.536*x        +  6.501*w);
//
//	SetPoint1 = wI1/(3*2*3.14);
//	SetPoint2 = wI2/(3*2*3.14);
//	SetPoint3 = wI3/(3*2*3.14);

}


void koordinat(float S_M1,float S_M2, float S_M3){
	int X,Y;

	X = -0.866*S_M1  -0.866*S_M3 + S_M2;
	Y = -0.5 *S_M1 + 0.5* S_M3;

	kor_X = X;
	kor_Y = Y;
}


void Jarak_Tempuh(float SX, float SY){
	/*Persamaan Jarak tempuh
	 * th = 120;
	 * S1 = -0.866*SX  + 0.5493*SY
	 * S2 = -0.866*SX  - 0.5493*SY
	 * S3 =  1.7321*SX
//	 */
//	S_M1t = -0.866*SX + 0.866*SY;
//	S_M2t = -0.866*SX - 0.5493*SY;
//	S_M3t =  1.7321*SX           ;
//
//	Vx = SX;
//	Vy = SY;
//	V0 = 0;


}


float Scaling(float in, uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2 ){
	float  m;
	float  out;
	m = (y2 - y1)/ (x2 - x1);
	out = m*in + y1;
	return out;
}


float constrain( float in, uint16_t up, uint8_t down){
	float out;
	if(in < down){
		out = down;
	}
	else if ( in > up){
		out = up;
	}
	else
		out = in;

	return out;
}


void persamaan_regresi(){
	/*==================Persamaan Regresi Linear========================
	 * y = RPM (Output)
	 * x = PWM (Input)
	 *
	 * Persmaan liner Motor 1 Maju
	 * y = 0.28*x-10
	 *
	 * Persmaan liner Motor 1 Mundur
	 * y = 0.28*x-10
	 *
	 * Persmaan linear motor 2 maju
	 * y = 0.28*x-17
	 *
	 * Persamaan linear motor 2 Mundur
	 * y = 0.28*x-15
	 *
	 * Persamaan liner motor 3 maju
	 * y = 0.28*x-11
	 *
	 * Persamaan liner motor 3 Mundur
	 * y = 0.28*x -11
	 */
	//====================RPM Motor 1=============================
		PWM1_F = ((100*RPM1_F)/28)+10;

		PWM1_B = ((100*RPM1_B)/28)+10;

	//====================RPM Motor 2=============================
		PWM2_F = ((100*RPM2_F)/28)+17;

		PWM2_B = ((100*RPM2_B)/28)+15;

	//====================RPM Motor 3=============================
		PWM3_F = ((100*RPM3_F)/28)+11;

		PWM3_B = ((100*RPM3_B)/28)+11;


}


void Tampilan_Serial(int x){
	switch(x){

	case 1:
		   STM_Serial_Print("count :");
		   STM_Serial_PrintInt(count1);
		   STM_Serial_Print(" | ");
		   STM_Serial_Print("Setpoint 1 :");
		   STM_Serial_PrintInt(SetPoint1);
		   STM_Serial_Print(" | ");
		   STM_Serial_Print(" PWM1 :");
		   STM_Serial_PrintInt(Duty1);
		   STM_Serial_Print(" |   ");
		   STM_Serial_Print("Setpoint 2 :");
		   STM_Serial_PrintInt(SetPoint2);
		   STM_Serial_Print(" | ");
		   STM_Serial_Print(" PWM2 :");
		   STM_Serial_PrintInt(Duty2);
		   STM_Serial_Print(" |   ");
		   STM_Serial_Print("Setpoint 3 :");
		   STM_Serial_PrintInt(SetPoint3);
		   STM_Serial_Print(" | ");
		   STM_Serial_Print(" PWM3 :");
		   STM_Serial_PrintInt(Duty3);
		   STM_Serial_Print(" | \r\n");
		  break;
	case 2:
			STM_Serial_PrintInt(Error1);
			STM_Serial_Print(" | ");
			STM_Serial_PrintInt(Error2);
			STM_Serial_Print(" | ");
			STM_Serial_PrintInt(Error3);
			STM_Serial_Print(" | ");
			STM_Serial_PrintInt(data[4]);
			STM_Serial_Print(" | ");
			STM_Serial_PrintInt(data[5]);
			STM_Serial_Print(" | ");
			STM_Serial_PrintInt(data[6]);
			STM_Serial_Print(" | ");
			STM_Serial_PrintInt(data[7]);
			STM_Serial_Print(" | ");
			STM_Serial_PrintInt(data[8]);
			STM_Serial_Print(" | ");
			STM_Serial_PrintInt(data[9]);
			STM_Serial_Print(" | \r\n");
			break;
	case 3:
		STM_Serial_Print("Nilai VX ms :");
		STM_Serial_PrintInt(Vxms);
		STM_Serial_Print(" | ");
		STM_Serial_Print("Nilai VY ms :");
		STM_Serial_PrintInt(Vyms);
		STM_Serial_Print(" | ");
		STM_Serial_Print("Nilai Vw ms :");
		STM_Serial_PrintInt(Vwms);
		STM_Serial_Print(" | \r\n");
		break;

	case 4 :
		STM_Serial_PrintInt(VRx);
		STM_Serial_Print(" | ");
		STM_Serial_PrintInt(VRy);
		STM_Serial_Print(" | ");
		STM_Serial_PrintInt(VRw);
		STM_Serial_Print(" | ");
		STM_Serial_Print("Data Motor1 RPM :");
	    STM_Serial_PrintInt(RPM1);
		STM_Serial_Print("Data Motor2 RPM :");
	    STM_Serial_PrintInt(RPM2);
		STM_Serial_Print("Data Motor3 RPM :");
	    STM_Serial_PrintInt(RPM3);
		STM_Serial_Print(" | \r\n");
		break;

	case 5:

		STM_Serial_PrintInt(jarak_M1);
		STM_Serial_Print(" | ");
		STM_Serial_PrintInt(jarak_M2);
		STM_Serial_Print(" | ");
		STM_Serial_PrintInt(jarak_M3);
		STM_Serial_Print(" | \r\n");
		break;

	case 6 :
		   STM_Serial_Print("Data Motor1 RPM :");
		   STM_Serial_PrintInt(RPM1);
		   STM_Serial_Print(" | ");
		   STM_Serial_Print(" PWM1 :");
		   STM_Serial_PrintInt(Duty1);
		   STM_Serial_Print(" |   ");
		   STM_Serial_Print("Data Motor2 RPM :");
		   STM_Serial_PrintInt(RPM2);
		   STM_Serial_Print(" | ");
		   STM_Serial_Print(" PWM2 :");
		   STM_Serial_PrintInt(Duty2);
		   STM_Serial_Print(" |   ");
		   STM_Serial_Print("Data Motor3 RPM :");
		   STM_Serial_PrintInt(RPM3);
		   STM_Serial_Print(" | ");
		   STM_Serial_Print(" PWM3 :");
		   STM_Serial_PrintInt(Duty3);
		   STM_Serial_Print(" | \r\n");
		  break;

	case 7 :
//		STM_Serial_Print("Jarak Tempuh X :");
//		STM_Serial_PrintInt(jarak_tempuhX);
//		STM_Serial_Print("|");
//		STM_Serial_Print("Jarak Total X :");
		STM_Serial_PrintInt(jarak_totalX);
		STM_Serial_Print("|");
//		STM_Serial_Print("Jarak sebelum X :");
//		STM_Serial_PrintInt(jarak_sebelumX);
//		STM_Serial_Print("  |  |  ");
//		STM_Serial_Print("Jarak Tempuh Y :");
//		STM_Serial_PrintInt(jarak_tempuhY);
//		STM_Serial_Print("|");
//		STM_Serial_Print("Jarak Total Y :");
		STM_Serial_PrintInt(jarak_totalY);
//		STM_Serial_Print("|");
//		STM_Serial_Print("Jarak sebelum Y :");
//		STM_Serial_PrintInt(jarak_sebelumY);
		STM_Serial_Print("\r\n");
		break;

	case 8 :
		STM_Serial_Print("Jarak Tempuh X :");
		STM_Serial_PrintInt(Vx);
		STM_Serial_Print("|");
		STM_Serial_Print(" X :");
		STM_Serial_PrintInt(Zxt);
		STM_Serial_Print("|");
		STM_Serial_Print("|\r\n");
		break;

	case 9 :
		STM_Serial_ReadBegin(4);
//		STM_Serial_Print("Hasil Bacaan :");
//		STM_Serial_PrintInt(Vx);
		break;
	case 10 :
		STM_Serial_PrintInt(Zxt);
		STM_Serial_Print("x");
		STM_Serial_Print("?");
		STM_Serial_PrintInt(Zyt);
		STM_Serial_Print("y");
		STM_Serial_Print("?");
//		STM_Serial_Print(" 0 :");
//		STM_Serial_PrintInt(Z0t);
//		STM_Serial_Print("|");
//		STM_Serial_Print("|\r\n");
	case 11 :
//		STM_Serial_PrintInt(RPM1);
//		STM_Serial_Print(" | ");
//		STM_Serial_PrintInt(RPM2);
//		STM_Serial_Print(" | ");
//		STM_Serial_PrintInt(RPM3);
//		STM_Serial_Print(" |\n ");
		STM_Serial_PrintInt(EZx_Sekarang/10); // nilai real robot
		//STM_Serial_Print("x");
		STM_Serial_Print(",");
		STM_Serial_PrintInt(EZy_Sekarang/10); // nilai real robot
		//STM_Serial_Print("y");
		STM_Serial_Print("?");
	case 12 :
		STM_Serial_PrintInt(RPM1);
		STM_Serial_Print(",");
		STM_Serial_PrintInt(RPM2);
		STM_Serial_Print(",");
		STM_Serial_PrintInt(RPM3);
		STM_Serial_Print("?");

	}
}

void riset_error(){
	Error1 = 0;
	Sum_Error1 = 0;
	Last_Error1 =0;

	Error2 = 0;
	Sum_Error2 = 0;
	Last_Error2 =0;

	Error3 = 0;
	Sum_Error3 = 0;
	Last_Error3 =0;
}


void TF_Motor1(){
	  if(SetPoint1 > 0){
		  RPM1_F = SetPoint1/6;
		  PWM1_F = ((100/28)*RPM1_F)+10;
		  Duty1 = PWM1_F;
		  Motor_1(Duty1);
	  }

	  else if(SetPoint1 <= 0){
		  RPM1_B = -1*SetPoint1/6;
		  PWM1_B = ((100/28)*RPM1_B)+10;
		  Duty1 = PWM1_B;
		  Motor_1(-1*Duty1);
	  }
}


void TF_Motor2(){
	  if(SetPoint2 > 0){
		  RPM2_F = SetPoint2/6;
		  PWM2_F = ((100/28)*RPM2_F)+17;
		  Duty2 = PWM2_F;
		  Motor_2(Duty2);
	  }

	  else if(SetPoint2 <= 0){
		  RPM2_B = -1*SetPoint2/6;
		  PWM2_B = ((100/28)*RPM2_B)+15;
		  Duty2 = PWM2_B;
		  Motor_2(-1*Duty2);
	  }
}


void TF_Motor3(){
	  if(SetPoint3 > 0){
		  RPM3_F = SetPoint3/6;
		  PWM3_F = ((100/28)*RPM3_F)+17;
		  Duty3 = PWM3_F;
		  Motor_3(Duty3);
	  }

	  else if(SetPoint3 <= 0){
		  RPM3_B = -1*SetPoint3/6;
		  PWM3_B = ((100/28)*RPM3_B)+15;
		  Duty3 = PWM3_B;
		  Motor_3(-1*Duty3);
	  }
}
void readVelocity(){
	if(rxData[4] == '1'){
			//  strcpy(dataS1,"");
			strncpy(dataS1,rxData,4);
			if (atoi(dataS1)==0 && strstr(dataS1,"-")){
				a = - atoi(dataS1);
			}else{
				a = atoi(dataS1);
			}
		}

	if(rxData[10] == '2'){
			//  strcpy(dataS2,"");
			strncpy(dataS2,(rxData)+6,4);
			if (atoi(dataS2)==0 && strstr(dataS2,"-")){
				b = - atoi(dataS2);
			}else{
				b = atoi(dataS2);
			}
		}
//		else if(rxData[4] == '3'){
//			//  strcpy(dataS3,"");
//			strncpy(dataS3,rxData,4);
//			if (atoi(dataS3)==0 && strstr(dataS3,"-")){
//				z = - atoi(dataS3);
//			}else{
//				z = atoi(dataS3);
//			}
//		}
//		else{
//			Motor_1(0);
//			Motor_2(0);
//			Motor_3(0);
//			//STM_Serial_Print("undefined");
//		}


//----------------------------------------------------------------------------
//	if(rxData[4] == '1'){
//		//  strcpy(dataS1,"");
//		strncpy(dataS1,rxData,4);
//		if (atoi(dataS1)==0 && strstr(dataS1,"-")){
//			a = - atoi(dataS1);
//		}else{
//			a = atoi(dataS1);
//		}
//
//		//STM_Serial_PrintInt(x);
//		//Motor_1(x);
//		//STM_Serial_PrintInt(x+"1");
//	}
//	else if(rxData[4] == '2'){
//		//  strcpy(dataS2,"");
//		strncpy(dataS2,rxData,4);
//		if (atoi(dataS2)==0 && strstr(dataS2,"-")){
//			b = - atoi(dataS2);
//		}else{
//			b = atoi(dataS2);
//		}
//		//y = atoi(dataS2);
//		//STM_Serial_PrintInt(y);
//	//				  STM_Serial_Print("Y ");
//	//				  STM_Serial_Print("\n");
//		//Motor_2(y);
//		//STM_Serial_PrintInt(y+"2");
//	}
//	else if(rxData[4] == '3'){
//		//  strcpy(dataS3,"");
//		strncpy(dataS3,rxData,4);
//		if (atoi(dataS3)==0 && strstr(dataS3,"-")){
//			z = - atoi(dataS3);
//		}else{
//			z = atoi(dataS3);
//		}
//		//z = atoi(dataS3);
//		//STM_Serial_PrintInt(z);
//	//				  STM_Serial_Print("O ");
//	//			  STM_Serial_Print("\n");
//				  //Motor_3(z);
//				  //STM_Serial_PrintInt(z+"3");
//	}
//	else{
//		Motor_1(0);
//		Motor_2(0);
//		Motor_3(0);
//		//STM_Serial_Print("undefined");
//	}
//------------------------------------------------------------------------------
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  //=================== inisialisasi Timer( PWM ) motor 1, 2, dan 3 ===========================
   HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
   HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
   HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
   HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);

   //===================== Untuk Membuat Interval waktu ======================================
   HAL_TIM_Base_Start(&htim1);
   HAL_TIM_Base_Start(&htim2);

  /* USER CODE END 2 */
//   for(int i = 0; i<=30000;i++){
//	   HAL_UART_Receive_DMA(&huart2,(uint8_t*)rxData,100);
//	  // HAL_USART_Receive_DMA(&husart1,(uint8_t*)rxData,100);
//	   //readVelocity();
////	   if(rxData[4] == '1'){
////		   //  strcpy(dataS1,"");
////		   strncpy(dataS1,rxData,4);
////		   if (atoi(dataS1)<=0 && strstr(dataS1,"-")){
////			   a = - atoi(dataS1);
////		   }else{
////			   a = atoi(dataS1);
////		   }
////	   }
////
////	   if(rxData[10] == '2'){
////		   //  strcpy(dataS2,"");
////		   strncpy(dataS2,(rxData)+6,4);
////		   if (atoi(dataS2)<=0 && strstr(dataS2,"-")){
////			   b = - atoi(dataS2);
////		   }else{
////			   b = atoi(dataS2);
////		   }
////	   }
//
//
//  //	  if (a != 0 && b != 0){
//  //		   STM_Serial_Print(" ready! ");
//  //		   break;
//  //	  }
//     }
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */
	  //========================== Pengaturan Pergerakan Robot =========================
	  //  if(x != 0 && y!= 0){

	  count1 = __HAL_TIM_GET_COUNTER(&htim1);
//	  HAL_UART_Receive_DMA(&huart2,(uint8_t*)rxData,100);
//	  readVelocity();
	  //	if(a != 0 && b!= 0){
	  //	 for(int i=0; i<=120; i++){
	  //	  if(t >= 0 && t <= 2.5){
	  if(t >= 0 && t <= 2.5){
		  //Zxref = a;//Zxref = 100; //**Zxref = a;
		  Zxref = 100;
		  Zx0   = Zx0;
		  Zyref = 0;//*Zyref = 0;//**Zyref = b;
		  Zy0   = Zy0;
		  Ztf   = 2.5;
		  Zt    = t;
	  }

	  if(t >= 2.5 && t <= 5){

		  //Zxref = a;//*Zxref = a;//Zxref = 100;//** Zxref = 0
		  Zxref = 100;
		  //Zx0   = a;//Zx0   = 100;//**Zx0   = a;
		  Zx0 = Zxt; // posisi sekarang trajectory tracking
		  //Zyref = b;//Zyref = 100;//**Zyref = b;
		  Zyref = 100;
		  Zy0   = Zyt;//*Zy0   = 0;//**Zy0   = b;
		  Ztf   = 2.5;
		  Zt    = t - 2.5;
	  }

	  if(t >= 5.5 && t <= 7.5){

		  Zxref = 0;
		  //Zx0   = a;//*Zx0   = a;//Zx0   = 100;//**Zx0   = 0;
		  Zx0 = Zxt; //posisi sekarang trajectory tracking
		  Zyref = 100;//*Zyref = b;//Zyref = 100;//**Zyref = 0;
		  //Zy0   = b;//Zy0   = 100;//**Zy0   = b;
		  Zy0 = Zyt;//posisi sekarang trajectory tracking
		  Ztf   = 2.5;
		  Zt    = t - 5.5;
	  }

	  if(t >= 7.5 && t <= 10){

		  Zxref = 0;
		  Zx0   = Zxt;
		  Zyref = 0;
		  Zy0   = Zyt;//Zy0   = 100;
		  Ztf   = 2.5;
		  Zt    = t - 7.5;
	  }

	  if(t >= 10){
		  //if(t >= 2.5){
		  t = 0;
		  Vx =0;
		  Vy =0;
		  Zx0 = jarak_totalX*0.1;
		  Zy0 = jarak_totalY*0.1;
	  }





	  //=========================== Define Velocity =====================================
	  Vx = ((ax1) + (ax2*Zt*2) + (ax3*Zt*Zt*3))*60;

	  Vy = ((ay1) + (ay2*Zt*2) + (ay3*Zt*Zt*3))*60;

	  V0 = ((a01) + (a02*Zt*2) + (a03*Zt*Zt*3))*60;

	  //==================================================================================

	  //========================= Pembatasan Nilai Kecepatan =============================

	  DeltaX = ((ax2*2)*(ax2*2)) - (4*(ax3*3)*(ax1));
	  DeltaY = ((ay2*2)*(ay2*2)) - (4*(ay3*3)*(ay1));
	  Delta0 = ((a02*2)*(a02*2)) - (4*(a03*3)*(a01));

	  limit_Vx = DeltaX /-2*(ax3*3);

	  limit_Vy = DeltaY /-2*(ay3*3);

	  limit_Vw = Delta0 /-2*(a03*3);

	  if(Vx >= limit_Vx *60){
		  Inverse_Kinematic(0,0,0 );
	  }

	  if(Vy >= limit_Vy * 60){
		  Inverse_Kinematic(0,0,0 );
	  }

	  if(V0 >= limit_Vw* 60){
		  Inverse_Kinematic(0,0,0 );
	  }


	  //==================================================================================
	  if(Zt >= Ztf){
		  Vx =0;
		  Vy =0;
	  }
	  Inverse_Kinematic((Vx - EZx_Sekarang),(Vy - EZy_Sekarang),0 );

	  	  //Inverse_Kinematic((Vx),(Vy),0 );

	  	  //===================Sampling waktu dalam 100ms===================
	  // untuk mendapatkan data dari bluetooth
	  if(count1 == Interval){
		 // if(a==0 || b==0){
			  HAL_UART_Receive_DMA(&huart2,(uint8_t*)rxData,100);
			  //readVelocity();
			  if(rxData[4] == '1'){
				  //  strcpy(dataS1,"");
				  strncpy(dataS1,rxData,4);
				  if (atoi(dataS1)==0 && strstr(dataS1,"-")){
					  Vxms = - atoi(dataS1);
				  }else{
					  Vxms = atoi(dataS1);
				  }
			  }

			  if(rxData[10] == '2'){
				  //  strcpy(dataS2,"");
				  strncpy(dataS2,(rxData)+6,4);
				  if (atoi(dataS2)==0 && strstr(dataS2,"-")){
					  Vyms = - atoi(dataS2);
				  }else{
					  Vyms = atoi(dataS2);
				  }
			  }
		 // }
		  //Menentukan nilai RPM1, RPM2, dan RPM3 dalam 100ms
		  TotalEnc1 = en1;
		  TotalEnc2 = en2;
		  TotalEnc3 = en3;

		  //================ Menghitung nilai total encoder========================
		  jarak_M1 +=en1;
		  jarak_M2 +=en2;
		  jarak_M3 +=en3;

		  //================== Mendapatkan nilai RPM =========================
		  RPM1 = (TotalEnc1)*(600/110);
		  RPM2 = (TotalEnc2)*(600/110);
		  RPM3 = (TotalEnc3)*(600/110);

		  //==================== Menghitung kecepatan Linear =====================
		  Forward_kinematic(RPM1,RPM3,RPM2);

		  //==================== Mengubah nilai kecepatan  V cm/m => V cm/s ==========
		  Vxms = VRx/60;
		  Vyms = VRy/60;
		  Vwms = VRw/60;

		  jarak_tempuhX  = Vxms*0.1;//harus dikali 0,1 sebagai sampling time
		  jarak_totalX   = jarak_sebelumX + jarak_tempuhX;
		  jarak_sebelumX = jarak_totalX;

		  jarak_tempuhY  = Vyms*0.1;//harus dikali 0,1 sebagai sampling time
		  jarak_totalY   = jarak_sebelumY + jarak_tempuhY;
		  jarak_sebelumY = jarak_totalY;

		  jarak_tempuh0  = Vwms;//harus dikali 0,1 sebagai sampling time
		  jarak_total0   = jarak_sebelum0 + jarak_tempuh0;
		  jarak_sebelum0 = jarak_total0;

		  //======================= Permasamaan Error Trajectory ========================
		  EZx_Sekarang = jarak_totalX - Zxt ;
		  EZx_Total    = EZx_Sekarang + EZx_Sebelum;
		  EZx_Sebelum  = EZx_Total;

		  EZy_Sekarang = jarak_totalY - Zyt ;
		  EZy_Total    = EZy_Sekarang + EZy_Sebelum;
		  EZy_Sebelum  = EZy_Total;

		  EZ0_Sekarang = jarak_total0 - Z0t ;
		  EZ0_Total    = EZ0_Sekarang + EZ0_Sebelum;
		  EZ0_Sebelum  = EZ0_Total;
		  //======================= Persmaan Posisi trajectory Tracking===================

		  //======================= Persmaan untuk koordinat X,Y dan 0 =========================
		  ax0 = Zx0; // d
		  ax1 = 0  ; // c
		  ax2 =  3*(Zxref - Zx0)*(1/(Ztf*Ztf)); //b
		  ax3 = -2*(Zxref - Zx0)*(1/(Ztf*Ztf*Ztf)); //a

		  Zxt  = (ax0) + (ax1*Zt) + (ax2*(Zt*Zt)) + (ax3*(Zt*Zt*Zt));

		  ay0 = Zy0;
		  ay1 = 0  ;
		  ay2 =  3*(Zyref - Zy0)*(1/(Ztf*Ztf));
		  ay3 = -2*(Zyref - Zy0)*(1/(Ztf*Ztf*Ztf));

		  Zyt  = (ay0) + (ay1*Zt) + (ay2*(Zt*Zt)) + (ay3*(Zt*Zt*Zt));

		  a00 = Z00;
		  a01 = 0  ;
		  a02 =  3*(Z0ref - Z00)*(1/(Ztf*Ztf));
		  a03 = -2*(Z0ref - Z00)*(1/(Ztf*Ztf*Ztf));

		  Z0t  = (a00) + (a01*Zt) + (a02*(Zt*Zt)) + (a03*(Zt*Zt*Zt));

		  //===============================================================================

		  //======================= Persamaan Error Trajectory ============================
		  EZx_Sekarang = (Vxms * 0.1) - Vx;
		  EZy_Sekarang = (Vyms * 0.1) - Vy;
		  EZ0_Sekarang = (jarak_tempuh0 * 0.1) - Z0t;

		  //================================================================================
		  //Memanggil fungsi PID tiap Motor
		  PID_TERM_1();
		  PID_TERM_2();
		  PID_TERM_3();

		  //membataskan nilai output dari persamaan PID
		  Duty1 = constrain(abs(PID_term1),1024,0);
		  Duty2 = constrain(abs(PID_term2),1024,0);
		  Duty3 = constrain(abs(PID_term3),1024,0);


		  //Menampilkan nilai
		  /*Subfungsi Tampilan_Serial(x)
		   * x = 1 untuk menampilkan RPM dan PWM pada setiap motor
		   * x = 2 untuk menampilkan data filter
		   * x = 3 untuk menampilkan nilai Vms
		   * x = 4 untuk menampilkan nilai VR
		   * x = 5 untuk menampilkan data motor
		   * x = 6 untuk menampilkan perbandinga RPM dan dutyClycle
		   * x = 7 untuk menampilkan jarak tempuh x,y dan 0
		   */
		  STM_Serial_PrintInt(Vx); //mengirim ref v x
		  STM_Serial_Print(",");
		  STM_Serial_PrintInt(Vy); // mengirim ref v y
		  STM_Serial_Print(",");
		  STM_Serial_PrintInt(Vxms); // mengirim v x sekarang
		  STM_Serial_Print(",");
		  STM_Serial_PrintInt(Vyms); // mengirim v y sekarang
		  STM_Serial_Print("?");
		 // STM_Serial_PrintInt(jarak_totalX/10); // nilai real robot
		 // STM_Serial_Print(",");
		 // STM_Serial_PrintInt(jarak_totalY/10); // nilai real robot
		 // STM_Serial_Print("?");
//		  Tampilan_Serial(11);
		  __HAL_TIM_SET_COUNTER(&htim1,0);
		  //					   STM_Serial_PrintInt(t);
	  	  //					   STM_Serial_Print("\r\n");


		  //Mengreset nilai encoder pada nilai nol
		  en1 = 0;
		  en2 = 0;
		  en3 = 0;

		  t = t + 0.1;

	  }



	  //========================Untuk menjalankan semua motor DC maju atau mundur==========================
	  if(PID_term1 >= 0){
		  Motor_1(Duty1);
	  }
	  else if (PID_term1 < 0){
		  Motor_1(Duty1*-1);
	  }
	  else if(SetPoint1 == 0){
		  Motor_1(0);
		  Error1 = 0;
		  Sum_Error1 = 0;
		  Last_Error1 =0;
	  }


	  if(PID_term2 >= 0){
		  Motor_2(Duty2);
	  }
	  else if (PID_term2 < 0){
		  Motor_2(Duty2*-1);
	  }
	  else if(SetPoint2 == 0){
		  Motor_2(0);
		  Error2 = 0;
		  Sum_Error2 = 0;
		  Last_Error2 =0;
	  }

	  if(PID_term3 >= 0){
		  Motor_3(Duty3);
	  }
	  else if (PID_term3 < 0){
		  Motor_3(Duty3*-1);
	  }
	  else if(SetPoint3 == 0){
		  Motor_3(0);
		  Error3 = 0;
		  Sum_Error3 = 0;
		  Last_Error3 =0;
	  }

	  //====================================================================================================


   //}
   /* USER CODE END 3 */
   //  }

/* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */

  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
  if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == GPIO_PIN_SET) en3--;
  else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == GPIO_PIN_RESET) en3++;
//  STM_Serial_Print("Encoder3 :");
//  STM_Serial_PrintInt(en3);
//  STM_Serial_Print(" | \n\r");
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */

  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_SET) en2--;
    else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_RESET) en2++;
//    STM_Serial_Print("Encoder2 :");
//    STM_Serial_PrintInt(en2);
//    STM_Serial_Print(" | \n\r");
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_SET) en1--;
    else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_5) == GPIO_PIN_RESET) en1++;
//    STM_Serial_Print("Encoder1 :");
//    STM_Serial_PrintInt(en1);
//    STM_Serial_Print(" | \n\r");
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1024;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 16;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1024;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 16;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1024;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  husart1.Instance = USART1;
  husart1.Init.BaudRate = 115200;
  husart1.Init.WordLength = USART_WORDLENGTH_8B;
  husart1.Init.StopBits = USART_STOPBITS_1;
  husart1.Init.Parity = USART_PARITY_NONE;
  husart1.Init.Mode = USART_MODE_TX_RX;
  husart1.Init.CLKPolarity = USART_POLARITY_LOW;
  husart1.Init.CLKPhase = USART_PHASE_1EDGE;
  husart1.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PE4 PE5 PE11 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  __NOP();
	//UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
//  HAL_UART_Receive_DMA(&huart2,(uint8_t*)rxData,10);
  HAL_UART_Transmit(&huart2, (uint8_t*)rxData,strlen(rxData),100);
  HAL_UART_Transmit(&huart3, (uint8_t*)rxData,strlen(rxData),100);
  //STM_Serial_Print(rxData);


}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  __NOP();
	//UNUSED(huart);
  /* NOTE: This function should not be modified, when the callback is needed,
           the HAL_UART_RxCpltCallback could be implemented in the user file
   */
  HAL_UART_Receive_DMA(&huart2,(uint8_t*)rxData,100);
  HAL_UART_Receive_DMA(&huart3,(uint8_t*)rxData,100);
//  HAL_UART_Transmit(&huart2, (uint8_t*)rxData,strlen(rxData),10);
  //STM_Serial_Print(rxData);


}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
s

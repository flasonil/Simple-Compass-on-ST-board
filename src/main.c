//PROGETTO FINALE
//TERZO ESERCIZIO SUL MAGNETOMETRO
//GRUPPO 16
//COPPOLA ILARIO, SAKI OSSAMA.

#include "stm32f4xx.h"
#include "stm32f401_discovery.h"
#include "stm32f401_discovery_lsm303dlhc.h"
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define TX_TIMEOUT_MS 5000
#define STREAM_PERIOD_MS 20
#define TOGGLE_PERIOD_LED3 250
#define TOGGLE_PERIOD_LED6 25

void I2C_Config(void);
void USART_Config(void);
void LEDs_Config(void);
ErrorStatus Mag_ReadData(int16_t* dataOut);
void Mag_Config(void);

u8 streamActive = 0;
u8 dataReady = 0;
u8 flag = 0;
u8 mode = 0;
volatile int timer_ms = 0;
volatile int counter_ms = 0;



int main(void)
{
	int16_t magData[3]; // X,Y and Z Acc data
	ErrorStatus result = ERROR;
	int8_t j = 0;

	int16_t X[4], Y[4];
	float outX[4] = {0,0,0,0}, outY[4] = {0,0,0,0};
	float teta = 0;

	/* LEDs configuration */
	LEDs_Config();

	/* I2C configuration */
	I2C_Config();

	/* USART configuration */
	USART_Config();

	/* Accelerometer configuration */
	Mag_Config();

	/* SysTick configuration */
	if (SysTick_Config(SystemCoreClock / 1000)) {
	/* Capture error */
		while(1);
	}

	/* print welcome string */
	printf(" Welcome! press 's' to start/stop streaming Magnetometer data\r\n");


	while(1)
	{

		if ((streamActive == 1) && (dataReady == 1))
		{
			/* read Acc data registers */
			result = Mag_ReadData(magData);
			//Vettore di quattro elementi che funge da finestra mobile: il campione più recente nella
			//posizione 0, quello più vecchio nella posizione 3.
			//Traslo i campioni degli assi x e y non filtrati.
			X[3] = X[2];
			Y[3] = Y[2];
			X[2] = X[1];

			Y[2] = Y[1];
			X[1] = X[0];
			Y[1] = Y[0];
			//Salvo i dati letti.
			X[0] = magData[0];
			Y[0] = magData[1];
			//Vettori contenenti i campioni filtrati.
			//E' stato implementato un filtro a media mobile.
			//Traslo prima i campioni di una posizione ed in out[0] calcolo la media.
			outX[3] = outX[2];
			outX[2] = outX[1];
			outX[1] = outX[0];
			outX[0] = 0;
			for(j=0;j<4;j++)
				outX[0] = outX[0] + X[j];
			outX[0] =(float) outX[0]/(4);
			outY[3] = outY[2];
			outY[2] = outY[1];
			outY[1] = outY[0];
			outY[0] = 0;
			for(j=0;j<4;j++)
				outY[0] = outY[0] + Y[j];
			outY[0] =(float) outY[0]/(4);
			//MODALITA' DATI
			if(mode == 0){
				if (result == SUCCESS)
					printf("Mag Data: \t%d\t%d\t%d\n\r", magData[0], magData[1], magData[2]);
				else
					printf("ERROR reading Mag data\n\r");
			}
			//MODALITA' ORIENTAMENTO
			else if(mode == 1){

				teta =(float) ((atan(outY[0]/outX[0]))*180)/3.14;
				if((teta < 0) && (outX[0] < 0))
					teta = (2*(90 + teta)) - teta ;
				else if((outX[0] < 0) && (outY[0] < 0))
					teta += 180 ;
				else if((outX[0] > 0) && (outY[0] < 0)){
					teta += 360 ;
				}
				//Nella modalità orientamento invio i dati filtrati
				//degli assi x e y e l'angolo teta che misura l'angolo tra
				//l'asse x del sensore ed il vettore del campo magnetico terrestre.
				printf("Mag Data: \t%f\t%f\t%f\n\r", outX[0], outY[0], teta);

				if((330 < teta) || (teta < 30)){
					STM_EVAL_LEDOff(LED4);
					STM_EVAL_LEDOff(LED5);
					STM_EVAL_LEDOff(LED6);
					STM_EVAL_LEDOn(LED3);
				}
				else if((30 < teta) && (teta < 60)){
					STM_EVAL_LEDOn(LED4);
					STM_EVAL_LEDOff(LED5);
					STM_EVAL_LEDOff(LED6);
					STM_EVAL_LEDOn(LED3);
				}
				else if((60 < teta) && (teta < 120)){
					STM_EVAL_LEDOn(LED4);
					STM_EVAL_LEDOff(LED5);
					STM_EVAL_LEDOff(LED6);
					STM_EVAL_LEDOff(LED3);
				}
				else if((120 < teta) && (teta < 150)){
					STM_EVAL_LEDOn(LED4);
					STM_EVAL_LEDOff(LED5);
					STM_EVAL_LEDOn(LED6);
					STM_EVAL_LEDOff(LED3);
				}
				else if((150 < teta) && (teta < 210)){
					STM_EVAL_LEDOff(LED4);
					STM_EVAL_LEDOff(LED5);
					STM_EVAL_LEDOn(LED6);
					STM_EVAL_LEDOff(LED3);
				}
				else if((210 < teta) && (teta < 240)){
					STM_EVAL_LEDOff(LED4);
					STM_EVAL_LEDOn(LED5);
					STM_EVAL_LEDOn(LED6);
					STM_EVAL_LEDOff(LED3);
				}
				else if((240 < teta) && (teta < 300)){
					STM_EVAL_LEDOff(LED4);
					STM_EVAL_LEDOn(LED5);
					STM_EVAL_LEDOff(LED6);
					STM_EVAL_LEDOff(LED3);
				}
				else if((300 < teta) && (teta < 330)){
					STM_EVAL_LEDOff(LED4);
					STM_EVAL_LEDOn(LED5);
					STM_EVAL_LEDOff(LED6);
					STM_EVAL_LEDOn(LED3);
				}
			}
			dataReady = 0;
		}
	}
}

void I2C_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef  I2C_InitStructure;

	/* Enable the I2C periph */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	/* Enable SCK and SDA GPIO clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* I2C SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* I2C SDA pin configuration */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* I2C configuration -------------------------------------------------------*/
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 100000;

	/* Apply LSM303DLHC_I2C configuration after enabling it */
	I2C_Init(I2C1, &I2C_InitStructure);

	/* LSM303DLHC_I2C Peripheral Enable */
	I2C_Cmd(I2C1, ENABLE);
}

void USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Enable UART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* Connect PXx to USARTx_Tx*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

	/* Connect PXx to USARTx_Rx*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the RX Interrupt */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	/* USARTx configured as follows:
		- BaudRate = 115200 baud
		- Word Length = 8 Bits
		- One Stop Bit
		- No parity
		- Hardware flow control disabled (RTS and CTS signals)
		- Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART initialization */
	USART_Init(USART2, &USART_InitStructure);

	/* Enable USART */
	USART_Cmd(USART2, ENABLE);

}

void Mag_Config(void)
{
	LSM303DLHCMag_InitTypeDef LSM303DLHC_InitStruct;

	LSM303DLHC_InitStruct.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;
	LSM303DLHC_InitStruct.MagOutput_DataRate = LSM303DLHC_ODR_75_HZ;
	LSM303DLHC_InitStruct.MagFull_Scale = LSM303DLHC_FS_1_9_GA;
	LSM303DLHC_InitStruct.Temperature_Sensor=LSM303DLHC_TEMPSENSOR_DISABLE;

	LSM303DLHC_MagInit(&LSM303DLHC_InitStruct);

}

ErrorStatus Mag_ReadData(int16_t* dataOut)
{
	ErrorStatus res = ERROR;
	u8 readBuffer[6];

	//I2C registers are 8 bit long, it is thus necessary to concatenate MSB and LSB
	//Read one register at a time from Acc, following the register order (first MSB, then LSB)
	res = LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, 1, &readBuffer[0]);
	if (res != SUCCESS)
		return ERROR;

	res = LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_L_M, 1, &readBuffer[1]);
	if (res != SUCCESS)
		return ERROR;

	res = LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M, 1, &readBuffer[2]);
	if (res != SUCCESS)
		return ERROR;

	res = LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_M, 1, &readBuffer[3]);
	if (res != SUCCESS)
		return ERROR;

	res = LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M, 1, &readBuffer[4]);
	if (res != SUCCESS)
		return ERROR;

	res = LSM303DLHC_Read(MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_M, 1, &readBuffer[5]);
	if (res != SUCCESS)
		return ERROR;


	dataOut[0] = ((int16_t)readBuffer[0]<<8) + readBuffer[1];
	dataOut[1] = ((int16_t)readBuffer[2]<<8) + readBuffer[3];
	dataOut[2] = ((int16_t)readBuffer[4]<<8) + readBuffer[5];

	return SUCCESS;

}

void LEDs_Config(void){

	/* Initialize all Leds mounted on STM32F401C-DISCO board */
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);

	/* Turn off LED4, LED3, LED5 and LED6 */
	STM_EVAL_LEDOff(LED3);
	STM_EVAL_LEDOff(LED4);
	STM_EVAL_LEDOff(LED5);
	STM_EVAL_LEDOff(LED6);
}

void SysTick_Handler(void)
{
	counter_ms++;
	//Lampeggio iniziale del led con frequenza 2Hz
	if((counter_ms == TOGGLE_PERIOD_LED3) && (flag == 0)){
		STM_EVAL_LEDToggle(LED3);
		counter_ms = 0;
	}
	if (streamActive == 1){
		//Lampeggio del led nella modalità dati con frequenza 20Hz
		if(((counter_ms % TOGGLE_PERIOD_LED6) == 0) && (mode == 0))
			STM_EVAL_LEDToggle(LED6);
		//Campionamento
		if ((counter_ms % STREAM_PERIOD_MS) == 0)
			dataReady = 1;
	}
	timer_ms++;

}

void USART2_IRQHandler(void)
{
	char ch = 0;

	/* RX interrupt */
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		ch = USART_ReceiveData(USART2);
		//Ricezione carattere 's'
		if (ch == 's'){
			streamActive = 1 - streamActive;
			printf("Start/Stop\r\n");
			//Disabilito il lampeggio iniziale
			//Si attiva il flag che discrimina la configurazione iniziale del
			//programma con il lampeggio del led a 2Hz dalla modalità operativa.
			flag = 1;
			STM_EVAL_LEDOff(LED3);
			STM_EVAL_LEDOff(LED4);
			STM_EVAL_LEDOff(LED5);
			STM_EVAL_LEDOff(LED6);
			counter_ms = 0;
		}
		//Ricezione carattere 'd': cambio modalità
		if((ch == 'd') && (streamActive == 0)){
			mode = 1 - mode ;
			printf("Changed Mode\r\n");
			if(!mode)
				printf("Dati\r\n");
			else
				printf("Orientamento\r\n");
		}
		if(((ch != 'd') && (ch != 's')) || ((ch == 'd') && (streamActive == 1))){
			printf("Comando non riconosciuto!\r\n");
		}
	}

}

int __io_putchar(int ch)
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART2, (uint8_t)ch);

  timer_ms = 0;

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
  {
	  /* Waiting timer expired, return */
	  if (timer_ms > TX_TIMEOUT_MS)
		  return 0;
  }

  return ch;
}

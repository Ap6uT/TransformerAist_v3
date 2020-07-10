
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l0xx_hal.h"

/* USER CODE BEGIN Includes */

#define amplDetectorPort GPIOB
#define amplDetectorPin GPIO_PIN_1
#define amplDetectorPinSet() HAL_GPIO_WritePin(amplDetectorPort,amplDetectorPin,GPIO_PIN_SET)
#define amplDetectorPinReset() HAL_GPIO_WritePin(amplDetectorPort,amplDetectorPin,GPIO_PIN_RESET)

#define attOnPort GPIOA
#define attOnPin GPIO_PIN_13
#define attOnPinSet() HAL_GPIO_WritePin(attOnPort,attOnPin,GPIO_PIN_SET)
#define attOnPinReset() HAL_GPIO_WritePin(attOnPort,attOnPin,GPIO_PIN_RESET)

#define FlAdr (uint32_t)(0x08007D00)

#define AttPause 1
#define AttPauseAfter 2

#define Kf 50

#define MBReinitTime 60

const unsigned char auchCRCHi[] =
{
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
0x40
};

const unsigned char auchCRCLo[] =
{
0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
0x40
};



const uint32_t USART_const [9] = {2400,4800,9600,14400,19200,38400,56000,57600,115200};
//const uint8_t  TIMER_const [9] = {4,8,4,3,2,2,2,2,2};
const uint8_t  TIMER_const [9] = {37,73,37,25,19,10,7,7,4};

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim21;
TIM_HandleTypeDef htim22;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

ADC_AnalogWDGConfTypeDef awdgc1;

volatile uint8_t mbReinitCnt=0;
volatile uint8_t needFlashWrite=0;

uint16_t CRCCod;

uint16_t max=0;
uint16_t min=0xFFFF;

uint16_t MBPause=0;
uint16_t MBPauseCnt=0;

uint32_t zeros=0;

uint8_t MBBusy=0;


#define res_buff_size 80
unsigned char res_buffer[res_buff_size];					// приемный буфер
unsigned char write_buffer[300];					// буфер для передачи
volatile unsigned char res_wr_index;


float testy;

static volatile uint16_t reg_MB[77];
uint16_t reg_MB2[8];

uint16_t reg_MB3[40];

float reg_MBf[7];

volatile uint8_t NeedChangeSpeed=0;

uint8_t Sin=0;
uint32_t Dot=0;


uint32_t Dooot=0;
uint32_t prevDooot=0;

double Dot12,DotMax2,DotMin2;
	


uint16_t ChCnt=0;
uint8_t ChF=0;



uint8_t wtf=0;

uint16_t MaxS[3]={0,0,0};
uint16_t RMSS[3]={0,0,0};
uint16_t Max_Max[3]={0,0,0};


uint16_t TT1=0xC8;
uint16_t TT2=0xC8;
uint16_t TT3=0xC8;

double ampl,rms;

volatile uint8_t A90=0;

volatile uint8_t FlagA90=0;

volatile uint8_t AttP=0;
volatile uint8_t AttPAft=0;

volatile uint8_t AttPF=0;
volatile uint8_t AttPAftF=0;


uint16_t adc1[6];

uint8_t dmaflg=0;



volatile uint8_t Flag=1;
volatile uint8_t Flag_timer=1;
uint8_t FlagMB=1;
uint8_t FlagMB2=1;

uint32_t FBI[3][25];
uint16_t abcd[8];
uint32_t FBI0;
uint32_t FBI1;
uint32_t FBI2;
uint32_t FBI3;

uint32_t OHBOY = 0;


	




#define MB_RMS reg_MB2[0]
#define MB_RMSMAX reg_MB2[1]
#define MB_RMS2 reg_MB2[2]
#define MB_RMSMAX2 reg_MB2[3]
#define MB_RMS3 reg_MB2[4]
#define MB_RMSMAX3 reg_MB2[5]
#define MB_HZ reg_MB2[6]








#define MB_AMPL_NOW  reg_MB[0x01]	
#define MB_RMS_NOW   reg_MB[0x02]
#define MB_AMPL_NOW2 reg_MB[0x03]	
#define MB_RMS_NOW2  reg_MB[0x04]
#define MB_AMPL_NOW3 reg_MB[0x05]	
#define MB_RMS_NOW3  reg_MB[0x06]

#define MB_HZ_NOW reg_MB[0x07]

#define MB_AMPL_ZERO  reg_MB[0x08]	
#define MB_RMS_ZERO   reg_MB[0x09]
#define MB_AMPL_ZERO2 reg_MB[0x0A]	
#define MB_RMS_ZERO2  reg_MB[0x0B]
#define MB_AMPL_ZERO3 reg_MB[0x0C]	
#define MB_RMS_ZERO3  reg_MB[0x0D]

#define MB_NAME  reg_MB[0x20]
#define MB_ADR   reg_MB[0x21]
#define MB_SPEED reg_MB[0x22]

#define MB_STATUS  reg_MB[0x23]

#define MB_TELE       reg_MB[0x24]
#define MB_LOG_INPUT  reg_MB[0x25]
#define MB_LOG_INPUT2 reg_MB[0x26]

#define MB_AMPL_N_I  reg_MB[0x30]
#define MB_RMS_N_I   reg_MB[0x31]
#define MB_AMPL_O_I  reg_MB[0x32]	
#define MB_RMS_O_I   reg_MB[0x33]	

#define MB_AMPL_N_I2 reg_MB[0x34]
#define MB_RMS_N_I2  reg_MB[0x35]
#define MB_AMPL_O_I2 reg_MB[0x36]	
#define MB_RMS_O_I2  reg_MB[0x37]	

#define MB_AMPL_N_I3 reg_MB[0x38]
#define MB_RMS_N_I3  reg_MB[0x39]
#define MB_AMPL_O_I3 reg_MB[0x3A]	
#define MB_RMS_O_I3  reg_MB[0x3B]	

#define MB_HZ_I reg_MB[0x3C]

#define MB_ATT_ON  reg_MB[0x3D]
#define MB_ATT_OFF reg_MB[0x3E]

#define MB_FLOAT1_N_F reg_MB[0x41]
#define MB_FLOAT2_N_F reg_MB[0x40]

#define MB_FLOAT1_O_F reg_MB[0x43]
#define MB_FLOAT2_O_F reg_MB[0x42]

#define MB_FLOAT1_N_F2 reg_MB[0x45]
#define MB_FLOAT2_N_F2 reg_MB[0x44]

#define MB_FLOAT1_O_F2 reg_MB[0x47]
#define MB_FLOAT2_O_F2 reg_MB[0x46]

#define MB_FLOAT1_N_F3 reg_MB[0x49]
#define MB_FLOAT2_N_F3 reg_MB[0x48]

#define MB_FLOAT1_O_F3 reg_MB[0x4B]
#define MB_FLOAT2_O_F3 reg_MB[0x4A]

#define MB_HZ_F reg_MB[0x4C]


#define MB_RMS_T_fl  	  reg_MBf[0]
#define MB_RMSMAX_T_fl  reg_MBf[1]
#define MB_RMS_T_fl2 	  reg_MBf[2]
#define MB_RMSMAX_T_fl2 reg_MBf[3]
#define MB_RMS_T_fl3    reg_MBf[4]
#define MB_RMSMAX_T_fl3 reg_MBf[5]
#define MB_HZ_T_fl      reg_MBf[6]


volatile uint8_t FORCE_ATT=0;



uint16_t MB_HZ_1=0;

union {
	uint16_t int1[2];
	float float1;
} union1;





const double tens[16]={1,10,100,1000,10000,100000,1000000,10000000,100000000,1000000000,10000000000,100000000000,
1000000000000,10000000000000,100000000000000,1000000000000000};


	uint8_t hz_st=0;
	uint8_t hz_st2=0;
	uint16_t hz_f=0;
	uint16_t hz_f2=0;


	volatile uint8_t bl_flag = 0;
	volatile uint8_t reset_flag = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(uint8_t bd);
static void MX_TIM21_Init(void);
static void MX_TIM22_Init(void);
static void MX_IWDG_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
unsigned int CRC16 (unsigned char *pucFrame, unsigned int usLen)
													// pucFrame - указатель на начало буфера (адрес)
													// usLen - длина пакета
{
	volatile unsigned char   ucCRCHi = 0xFF;					// старший байт црц
	volatile unsigned char   ucCRCLo = 0xFF;					// младший байт црц
	volatile int             iIndex;
	int i=0;

	while (usLen--)									// цикл, usLen уменьшается на 1 при каждой итерации
	{
		iIndex = ucCRCLo ^ pucFrame[i];				// ксорим ucCRCLo  и байт, на который указывает pucFrame.
		i++;										// полученное значение будет индексом iIndex в таблицах. pucFrame инкрементируется.
		ucCRCLo = ucCRCHi ^ (auchCRCHi[iIndex] );	// ксорим ucCRCHi и значение из таблицы aucCRCHi с индексом iIndex.
		ucCRCHi = ( auchCRCLo[iIndex] );			// ucCRCHi равно значению из таблицы aucCRCLo с индексом iIndex
	}
	return ( ucCRCHi << 8 | ucCRCLo );				// Возвращаем старший и младший байт CRC в виде 16-разрядного слова
}


static void MX_TIM2_Init(uint8_t bd)
{
	__HAL_RCC_TIM2_CLK_ENABLE();
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = TIMER_const[bd];
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	
	TIM2->PSC = 3200 - 1; 
	TIM2->ARR = TIMER_const[bd]; 
	TIM2->DIER |= TIM_DIER_UIE; 
	TIM2->CR1 |= TIM_CR1_OPM;
	
	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 1); 
	NVIC_EnableIRQ(TIM2_IRQn);
}

static void MX_TIM21_Init(void)
{
	__HAL_RCC_TIM21_CLK_ENABLE();
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 32000-1;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 500;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	TIM21->PSC = 32000-1; 
	TIM21->ARR = 500 ; 
	TIM21->DIER |= TIM_DIER_UIE; 
	//TIM21->CR1 |= TIM_CR1_OPM;
	TIM21->CR1 |= TIM_CR1_CEN; 
	HAL_NVIC_SetPriority(TIM21_IRQn, 0, 3); 
	NVIC_EnableIRQ(TIM21_IRQn);
}


static void MX_TIM22_Init(void)
{	
	__HAL_RCC_TIM22_CLK_ENABLE();
  htim22.Instance = TIM22;
  htim22.Init.Prescaler = 32-1;
  htim22.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim22.Init.Period = 10000;
  htim22.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim22) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
	TIM22->PSC = 32 - 1; 
	TIM22->ARR = 65000 ; 
	TIM22->DIER |= TIM_DIER_UIE; 
	//TIM22->CR1 |= TIM_CR1_OPM;
	TIM22->CR1 |= TIM_CR1_CEN; 
	HAL_NVIC_SetPriority(TIM22_IRQn, 0, 0); 
	NVIC_EnableIRQ(TIM22_IRQn);
}


static void USART2_ReInit(uint8_t bd)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = USART_const[bd];
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

void Erase_Flash(void)
{
	uint32_t PgError = 0;
	HAL_FLASH_Unlock();
	
	FLASH_EraseInitTypeDef Flash_eraseInitStruct;
	Flash_eraseInitStruct.TypeErase     = FLASH_TYPEERASE_PAGES;
	Flash_eraseInitStruct.PageAddress  = FlAdr;
	Flash_eraseInitStruct.NbPages        = 2;

	if(HAL_FLASHEx_Erase(&Flash_eraseInitStruct, &PgError) != HAL_OK)
	{
		 HAL_FLASH_Lock();
	}
	HAL_FLASH_Lock();
}

uint32_t check_calc(uint32_t *buffer, uint32_t buff_len) {
	uint32_t result, i;
	result=0;
	for(i = 0; i < buff_len; i++) {
    result ^= buffer[i]; 
  }
	return result;
}

void Write_Flash_Adr(uint32_t Adr)
{
	uint16_t i=0;
	uint32_t Buf[20];
	uint32_t PgError = 0;
	Buf[0]=(uint32_t)(MB_ADR*0x10000+MB_SPEED);
	Buf[1]=(uint32_t)(MB_RMS_N_I*0x10000+MB_AMPL_N_I);
	Buf[2]=(uint32_t)(MB_RMS_O_I*0x10000+MB_AMPL_O_I);
	Buf[3]=(uint32_t)(MB_RMS_N_I2*0x10000+MB_AMPL_N_I2);
	Buf[4]=(uint32_t)(MB_RMS_O_I2*0x10000+MB_AMPL_O_I2);
	Buf[5]=(uint32_t)(MB_RMS_N_I3*0x10000+MB_AMPL_N_I3);
	Buf[6]=(uint32_t)(MB_RMS_O_I3*0x10000+MB_AMPL_O_I3);
	Buf[7]=(uint32_t)(MB_HZ_I*0x10000+MB_HZ_F);
	Buf[8]=(uint32_t)(MB_FLOAT1_N_F*0x10000+MB_FLOAT2_N_F);
	Buf[9]=(uint32_t)(MB_FLOAT1_O_F*0x10000+MB_FLOAT2_O_F);
	Buf[10]=(uint32_t)(MB_AMPL_ZERO*0x10000+MB_RMS_ZERO);
	Buf[11]=(uint32_t)(MB_AMPL_ZERO2*0x10000+MB_RMS_ZERO2);
	Buf[12]=(uint32_t)(MB_AMPL_ZERO3*0x10000+MB_RMS_ZERO3);
	Buf[13]=(uint32_t)(MB_NAME);
	Buf[14]=(uint32_t)(MB_ATT_OFF*0x10000+MB_ATT_ON);
	Buf[15]=(uint32_t)(MB_FLOAT1_N_F2*0x10000+MB_FLOAT2_N_F2);
	Buf[16]=(uint32_t)(MB_FLOAT1_O_F2*0x10000+MB_FLOAT2_O_F2);
	Buf[17]=(uint32_t)(MB_FLOAT1_N_F3*0x10000+MB_FLOAT2_N_F3);
	Buf[18]=(uint32_t)(MB_FLOAT1_O_F3*0x10000+MB_FLOAT2_O_F3);
	Buf[19]=check_calc(Buf,19);
	
	HAL_FLASH_Unlock();
	
	FLASH_EraseInitTypeDef Flash_eraseInitStruct;
	Flash_eraseInitStruct.TypeErase     = FLASH_TYPEERASE_PAGES;
	Flash_eraseInitStruct.PageAddress  = FlAdr;
	Flash_eraseInitStruct.NbPages        = 1;

	if(HAL_FLASHEx_Erase(&Flash_eraseInitStruct, &PgError) != HAL_OK)
	{
		 HAL_FLASH_Lock();
	}
	for(i=0;i<20;i++)
	{
		HAL_FLASH_Program(TYPEPROGRAM_WORD, Adr+i*4,Buf[i]);
	}	
	HAL_FLASH_Lock();
}

void Write_Flash(void)
{
	/*__disable_irq ();
	MBBusy=1;
	Write_Flash_Adr(FlAdr);
	HAL_Delay(100);
	Write_Flash_Adr(FlAdr+0x100);
	HAL_Delay(100);
	Write_Flash_Adr(FlAdr+0x200);
	MBBusy=0;
	__enable_irq();*/
	uint16_t i=0;
	uint32_t Buf[20];
	uint32_t PgError = 0;
	Buf[0]=(uint32_t)(MB_ADR*0x10000+MB_SPEED);
	Buf[1]=(uint32_t)(MB_RMS_N_I*0x10000+MB_AMPL_N_I);
	Buf[2]=(uint32_t)(MB_RMS_O_I*0x10000+MB_AMPL_O_I);
	Buf[3]=(uint32_t)(MB_RMS_N_I2*0x10000+MB_AMPL_N_I2);
	Buf[4]=(uint32_t)(MB_RMS_O_I2*0x10000+MB_AMPL_O_I2);
	Buf[5]=(uint32_t)(MB_RMS_N_I3*0x10000+MB_AMPL_N_I3);
	Buf[6]=(uint32_t)(MB_RMS_O_I3*0x10000+MB_AMPL_O_I3);
	Buf[7]=(uint32_t)(MB_HZ_I*0x10000+MB_HZ_F);
	Buf[8]=(uint32_t)(MB_FLOAT1_N_F*0x10000+MB_FLOAT2_N_F);
	Buf[9]=(uint32_t)(MB_FLOAT1_O_F*0x10000+MB_FLOAT2_O_F);
	Buf[10]=(uint32_t)(MB_AMPL_ZERO*0x10000+MB_RMS_ZERO);
	Buf[11]=(uint32_t)(MB_AMPL_ZERO2*0x10000+MB_RMS_ZERO2);
	Buf[12]=(uint32_t)(MB_AMPL_ZERO3*0x10000+MB_RMS_ZERO3);
	Buf[13]=(uint32_t)(MB_NAME);
	Buf[14]=(uint32_t)(MB_ATT_OFF*0x10000+MB_ATT_ON);
	Buf[15]=(uint32_t)(MB_FLOAT1_N_F2*0x10000+MB_FLOAT2_N_F2);
	Buf[16]=(uint32_t)(MB_FLOAT1_O_F2*0x10000+MB_FLOAT2_O_F2);
	Buf[17]=(uint32_t)(MB_FLOAT1_N_F3*0x10000+MB_FLOAT2_N_F3);
	Buf[18]=(uint32_t)(MB_FLOAT1_O_F3*0x10000+MB_FLOAT2_O_F3);
	Buf[19]=check_calc(Buf,19);
	
	HAL_FLASH_Unlock();
	
	FLASH_EraseInitTypeDef Flash_eraseInitStruct;
	Flash_eraseInitStruct.TypeErase     = FLASH_TYPEERASE_PAGES;
	Flash_eraseInitStruct.PageAddress  = FlAdr;
	Flash_eraseInitStruct.NbPages        = 3;

	if(HAL_FLASHEx_Erase(&Flash_eraseInitStruct, &PgError) != HAL_OK)
	{
		 HAL_FLASH_Lock();
	}
	for(i=0;i<20;i++)
	{
		HAL_FLASH_Program(TYPEPROGRAM_WORD, FlAdr+i*4,Buf[i]);
	}	
	{
		HAL_FLASH_Program(TYPEPROGRAM_WORD, FlAdr+0x100+i*4,Buf[i]);
	}	
	{
		HAL_FLASH_Program(TYPEPROGRAM_WORD, FlAdr+0x200+i*4,Buf[i]);
	}	
	HAL_FLASH_Lock();
}

uint32_t FLASH_Read(uint32_t address)
{
	return (*(__IO uint32_t*)address);
}

uint16_t IntToBCD (uint32_t d)
{
	uint8_t st=0;
	uint8_t t1,t2,t3;
	uint16_t tt;
	while(d/1000>0)
	{
		d=d/10;
		st++;
	}
	tt=d;
	t1=tt%10;
	t2=tt/10%10;
	t3=tt/100;
	return t3*0x1000+t2*0x100+t1*0x10+st;
}

uint16_t BCDToInt (uint16_t d)
{
	uint8_t t1,t2,t3;
	d=d>>4;
	t1=d&0x000F;
	t2=(d>>4)&0x000F;
	t3=(d>>8)&0x000F;
	return t1+t2*10+t3*100;
}

void MB04(void) //поменяны местами регистры
{
	float rms_f=0;
	float max_f=0;

	uint8_t i;
	
	uint32_t MB_RMS_T[3];
	uint32_t MB_RMSMAX_T[3];
	uint32_t MB_HZ_T=0;
	
	float RMS_T_fl[3];
	float RMSMAX_T_fl[3];
	
	uint16_t AMPL_N_I[3];
	uint16_t RMS_N_I[3];
	uint16_t AMPL_O_I[3];
	uint16_t RMS_O_I[3];
	uint16_t AMPL_ZERO[3];
	uint16_t RMS_ZERO[3];
	for(i=0;i<3;i++)
	{
		AMPL_N_I[i]=reg_MB[i*4+0x30];
		RMS_N_I[i]=reg_MB[i*4+0x31];
		AMPL_O_I[i]=reg_MB[i*4+0x32];
		RMS_O_I[i]=reg_MB[i*4+0x33];
		AMPL_ZERO[i]=reg_MB[i*2+8];
		RMS_ZERO[i]=reg_MB[i*2+9];
	}
	
	for(i=0;i<3;i++)
	{
		uint8_t Max_Max_Att=0;
		Max_Max_Att=Max_Max[i]>>12;
		
		Max_Max[i]&=0x0FFF;
		ampl=Max_Max[i]-AMPL_ZERO[i];
		if(ampl<0){ampl=0.0;}
		if(Max_Max_Att)
		{
			/*union1.int1[1]=MB_FLOAT2_O_F; 
			union1.int1[0]=MB_FLOAT1_O_F;*/
			
			union1.int1[1]=reg_MB[0x42+4*i]; 
			union1.int1[0]=reg_MB[0x43+4*i]; 

			max_f=union1.float1;
				
			RMSMAX_T_fl[i]=((float)ampl*max_f/(float)AMPL_O_I[i]);
			MB_RMSMAX_T[i]=(int)(RMSMAX_T_fl[i]*100000.0);
		}
		else
		{
			/*union1.int1[1]=MB_FLOAT2_N_F; 
			union1.int1[0]=MB_FLOAT1_N_F;*/
			union1.int1[1]=reg_MB[0x40+4*i]; 
			union1.int1[0]=reg_MB[0x41+4*i]; 
			max_f=union1.float1;

			RMSMAX_T_fl[i]=(float)ampl*max_f/(float)AMPL_N_I[i];
			MB_RMSMAX_T[i]=(int)(RMSMAX_T_fl[i]*100000.0);		
		}
		
		if(FlagA90)
		{
			rms=MaxS[i]-AMPL_ZERO[i];
			if(rms<0){rms=0.0;}

			if(A90)
			{
				/*union1.int1[1]=MB_FLOAT2_O_F;
				union1.int1[0]=MB_FLOAT1_O_F;*/
				
				union1.int1[1]=reg_MB[0x42+4*i]; 
				union1.int1[0]=reg_MB[0x43+4*i]; 
				rms_f=union1.float1;
				
				RMS_T_fl[i]=((float)rms*rms_f/(float)AMPL_O_I[i]);
				MB_RMS_T[i]=(int)(RMS_T_fl[i]*100000.0);
			}
			else
			{
				/*union1.int1[1]=MB_FLOAT2_N_F;
				union1.int1[0]=MB_FLOAT1_N_F;*/
				union1.int1[1]=reg_MB[0x40+4*i]; 
				union1.int1[0]=reg_MB[0x41+4*i];
				rms_f=union1.float1;

				RMS_T_fl[i]=((float)rms*rms_f/(float)AMPL_N_I[i]);
				MB_RMS_T[i]=(int)(RMS_T_fl[i]*100000.0);
			}	
		}
		else
		{
			rms=RMSS[i]-RMS_ZERO[i];
			if(rms<0){rms=0.0;}


			if(A90)
			{
				/*union1.int1[1]=MB_FLOAT2_O_F;
				union1.int1[0]=MB_FLOAT1_O_F;*/
				union1.int1[1]=reg_MB[0x42+4*i]; 
				union1.int1[0]=reg_MB[0x43+4*i]; 
				rms_f=union1.float1;
				RMS_T_fl[i]=((float)rms*rms_f/(float)RMS_O_I[i]);
				MB_RMS_T[i]=(int)(RMS_T_fl[i]*100000.0);
			}
			else
			{
				/*union1.int1[1]=MB_FLOAT2_N_F;
				union1.int1[0]=MB_FLOAT1_N_F;*/
				union1.int1[1]=reg_MB[0x40+4*i]; 
				union1.int1[0]=reg_MB[0x41+4*i];
				rms_f=union1.float1;
				//reg_MB[0]=(int)rms_f;
				RMS_T_fl[i]=(float)rms*rms_f/(float)RMS_N_I[i];
				MB_RMS_T[i]=(int)(RMS_T_fl[i]*100000.0);
			}	
		}
	}
	
	MB_RMS=IntToBCD(MB_RMS_T[0]);
	MB_RMS2=IntToBCD(MB_RMS_T[1]);
	MB_RMS3=IntToBCD(MB_RMS_T[2]);

	MB_RMSMAX=IntToBCD(MB_RMSMAX_T[0]);
	MB_RMSMAX2=IntToBCD(MB_RMSMAX_T[1]);
	MB_RMSMAX3=IntToBCD(MB_RMSMAX_T[2]);
	
	if(Dooot>0&&MB_RMS>0)
	{
		hz_f=BCDToInt(MB_HZ_F);
		hz_st=MB_HZ_F&0x000F;
		hz_f2=BCDToInt(MB_HZ_I);
		hz_st2=MB_HZ_I&0x000F;
		
		

		//MB_HZ_T=(int)((22470000.0/(float)Dooot)*(tens[hz_st]/tens[hz_st2])*((float)hz_f/(float)hz_f2));
		
		//16.10.2019
		//MB_HZ_1=(int)(22470000.0/(float)Dooot);
		MB_HZ_1=(int)(10000000.0/(float)Dooot);
		
		
		
		//MB_HZ_T=MB_HZ_1*(tens[hz_st]/tens[hz_st2])*((float)hz_f/(float)hz_f2);
		
		MB_HZ_T_fl=MB_HZ_1*(tens[hz_st]/tens[hz_st2])*((float)hz_f/(float)hz_f2);
		MB_HZ_T=(int)MB_HZ_T_fl;
		
		MB_HZ_1=IntToBCD(MB_HZ_1);
		MB_HZ=IntToBCD(MB_HZ_T);
	}
	else
	{
		MB_HZ=0;
		MB_HZ_T_fl=0.0;
	}

	MB_HZ_T_fl/=100000.0;
	
	
	for(i=0;i<3;i++)
	{
		union1.float1=RMS_T_fl[i];
		reg_MB3[i*4]=union1.int1[0];
		reg_MB3[i*4+1]=union1.int1[1];

		union1.float1=RMSMAX_T_fl[i];
		reg_MB3[i*4+2]=union1.int1[0];
		reg_MB3[i*4+3]=union1.int1[1];
	}
	union1.float1=MB_HZ_T_fl;
	reg_MB3[12]=union1.int1[0];
	reg_MB3[13]=union1.int1[1];
	
	Max_Max[0]=0;	
	Max_Max[1]=0;
	Max_Max[2]=0;
}


void Flash_Jump_Adress(uint32_t adress)															// Переход в область другой программы во флеш памяти
{
	__set_PRIMASK(1);																							// Отключаем глобальные прерывания(обязательно перед переходом)																						 					
	
	typedef 	void (*pFunction)(void);														// Объявляем тип функции-ссылки
	pFunction Jump_To_Application;																// Объявляем функцию-ссылку

  uint32_t JumpAddress = *(__IO uint32_t*) (adress + 4); 						// Адрес перехода на вектор (Reset_Handler) 		
  Jump_To_Application = (pFunction) JumpAddress;  							// Указатель на функцию перехода
	__set_MSP(*(volatile uint32_t*) adress);														// Указываем адрес вектора стека(Stack Pointer)	
  Jump_To_Application();                          							// Переходим на основную программу
}	

void My_System_Reset(void)
{
	if(reset_flag)// Регистр - флаг сделать ресет контроллеру
	{
		HAL_NVIC_SystemReset();
	}
}

void My_Jump_Boatloader(void)
{
	if(bl_flag)																	// Проверяем регистр перехода в загрузчик
	{
		// Останавливаем преобразования АЦП и отключаем его
		HAL_ADC_Stop_DMA(&hadc);
		
		// Останавливаем все задействованные таймеры
		HAL_TIM_Base_Stop_IT(&htim2);					 										// остановить таймер 2
		HAL_TIM_Base_Stop_IT(&htim21);					 			
		HAL_TIM_Base_Stop_IT(&htim22);					 							 			
		
		// Отключаем все прерывания
		HAL_NVIC_DisableIRQ(TIM2_IRQn);				  									// отключить прерывания таймера 2
		HAL_NVIC_DisableIRQ(TIM21_IRQn);				  	
		HAL_NVIC_DisableIRQ(TIM22_IRQn);				  									
		HAL_NVIC_DisableIRQ(USART2_IRQn);		 		  								// отключить прерывания USART 2		  			 		  			

		HAL_NVIC_DisableIRQ(RCC_IRQn);				  									// отключить прерывания Осцилятора
		HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
	
		
		// Деинициализация портов
		HAL_GPIO_DeInit(GPIOA, 0xff);
		HAL_GPIO_DeInit(GPIOB, 0xff);
		HAL_GPIO_DeInit(GPIOC, 0xff);

		
		// Деинициализация таймеров
		HAL_TIM_Base_DeInit(&htim2);
		HAL_TIM_Base_DeInit(&htim21);
		HAL_TIM_Base_DeInit(&htim22);
		
		// Деинициализация задействованного АЦП_1
		HAL_ADC_DeInit(&hadc);
		
		// Деинициализация приемо-передатчиков
		HAL_UART_DeInit(&huart2);
		HAL_RCC_DeInit();
		HAL_DeInit();
		
		__HAL_RCC_DMA1_CLK_DISABLE();
		
		// Выключаем тактирование портов GPIO
		//__GPIOA_CLK_DISABLE();
		//__GPIOB_CLK_DISABLE();
		//__GPIOC_CLK_DISABLE();
		//__GPIOD_CLK_DISABLE();
		
		Flash_Jump_Adress(0x08000000);														// Переходим в область памяти загрузчика
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	__set_PRIMASK(1);									  											// Отключаем глобальные прерывания	
	SCB->VTOR = (uint32_t)0x08002800;  												// Переопределяем начало таблицы векторов прерываний 
	__set_PRIMASK(0);
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_ADC_Init();
  //MX_TIM2_Init();
  MX_TIM21_Init();
  MX_TIM22_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

	
	MB_RMS_N_I=1;
	MB_AMPL_N_I=1;
	MB_RMS_N_I2=1;
	MB_AMPL_N_I2=1;
	MB_RMS_N_I3=1;
	MB_AMPL_N_I3=1;
	MB_RMS_O_I=1;
	MB_AMPL_O_I=1;
	MB_RMS_O_I2=1;
	MB_AMPL_O_I2=1;
	MB_RMS_O_I3=1;
	MB_AMPL_O_I3=1;
	
	MB_FLOAT1_N_F=0x3f80;
	MB_FLOAT2_N_F=0x3f80;
	MB_FLOAT1_N_F2=0x3f80;
	MB_FLOAT2_N_F2=0x3f80;
	MB_FLOAT1_N_F3=0x3f80;
	MB_FLOAT2_N_F3=0x3f80;
	MB_FLOAT1_O_F=0x3f80;
	MB_FLOAT2_O_F=0x3f80;
	MB_FLOAT1_O_F2=0x3f80;
	MB_FLOAT2_O_F2=0x3f80;
	MB_FLOAT1_O_F3=0x3f80;
	MB_FLOAT2_O_F3=0x3f80;
	
	MB_AMPL_ZERO=0;
	MB_RMS_ZERO=0;
	MB_AMPL_ZERO2=0;
	MB_RMS_ZERO2=0;
	MB_AMPL_ZERO3=0;
	MB_RMS_ZERO3=0;
	MB_ADR=247;
	MB_SPEED=2;
	//reg_MB[17]=10;
	
	MB_HZ_I=0x5001;
	MB_HZ_F=0x5001;
	
	MB_NAME=0;
	
	MB_ATT_ON=0x0FA0;
	MB_ATT_OFF=0x00FA;
	uint8_t j;
	FBI[0][0]=FLASH_Read(FlAdr);
	uint8_t k, rightData;
	
	if((FBI[0][0]==0)||(FBI[0][0]==0xFFFFFFFF))
	{
		rightData=4; //wrong data - need write default
		Write_Flash();
	}
	else
	{
		for(k=0;k<20;k++)
		{
			FBI[0][k]=FLASH_Read(FlAdr + k*4);
			FBI[1][k]=FLASH_Read(FlAdr + 0x100 + k*4);
			FBI[2][k]=FLASH_Read(FlAdr + 0x200 + k*4);
		}
		uint32_t crc32[3];
		for(k=0;k<3;k++)
		{
			crc32[k]=check_calc(FBI[k],19);
		}
		if (crc32[0]==FBI[0][19] && crc32[1]==FBI[1][19] && FBI[0][19]==FBI[1][19])
		{
			rightData=0;
		}
		else if (crc32[0]==FBI[0][19] && crc32[2]==FBI[2][19] && FBI[0][19]==FBI[2][19])
		{
			rightData=0;
		}
		else if (crc32[1]==FBI[1][19] && crc32[2]==FBI[2][19] && FBI[1][19]==FBI[2][19])
		{
			rightData=1;
		}
		else
		{
			rightData=4; //wrong data - need write default
			Write_Flash();		
		}
	}
	
	
	if(rightData<3)
	{
		MB_ADR=FBI[rightData][0]>>16;
		MB_SPEED=FBI[rightData][0]&0x0000FFFF;
		
		MB_RMS_N_I=FBI[rightData][1]>>16;
		MB_AMPL_N_I=FBI[rightData][1]&0x0000FFFF;
		MB_RMS_O_I=FBI[rightData][2]>>16;
		MB_AMPL_O_I=FBI[rightData][2]&0x0000FFFF;
		
		MB_RMS_N_I2=FBI[rightData][3]>>16;
		MB_AMPL_N_I2=FBI[rightData][3]&0x0000FFFF;
		MB_RMS_O_I2=FBI[rightData][4]>>16;
		MB_AMPL_O_I2=FBI[rightData][4]&0x0000FFFF;
		
		MB_RMS_N_I3=FBI[rightData][5]>>16;
		MB_AMPL_N_I3=FBI[rightData][5]&0x0000FFFF;
		MB_RMS_O_I3=FBI[rightData][6]>>16;
		MB_AMPL_O_I3=FBI[rightData][6]&0x0000FFFF;
		
		MB_HZ_I=FBI[rightData][7]>>16;
		MB_HZ_F=FBI[rightData][7]&0x0000FFFF;
		
		MB_FLOAT1_N_F=FBI[rightData][8]>>16;
		MB_FLOAT2_N_F=FBI[rightData][8]&0x0000FFFF;
		
		MB_FLOAT1_O_F=FBI[rightData][9]>>16;
		MB_FLOAT2_O_F=FBI[rightData][9]&0x0000FFFF;
		
		MB_AMPL_ZERO=FBI[rightData][10]>>16;
		MB_RMS_ZERO=FBI[rightData][10]&0x0000FFFF;
		
		MB_AMPL_ZERO2=FBI[rightData][11]>>16;
		MB_RMS_ZERO2=FBI[rightData][11]&0x0000FFFF;
		
		MB_AMPL_ZERO3=FBI[rightData][12]>>16;
		MB_RMS_ZERO3=FBI[rightData][12]&0x0000FFFF;
		
		MB_NAME=FBI[rightData][13]&0x0000FFFF;
		
		MB_ATT_OFF=FBI[rightData][14]>>16;
		MB_ATT_ON=FBI[rightData][14]&0x0000FFFF;
		
		MB_FLOAT1_N_F2=FBI[rightData][15]>>16;
		MB_FLOAT2_N_F2=FBI[rightData][15]&0x0000FFFF;
		
		MB_FLOAT1_O_F2=FBI[rightData][16]>>16;
		MB_FLOAT2_O_F2=FBI[rightData][16]&0x0000FFFF;
		
		MB_FLOAT1_N_F3=FBI[rightData][17]>>16;
		MB_FLOAT2_N_F3=FBI[rightData][17]&0x0000FFFF;
		
		MB_FLOAT1_O_F3=FBI[rightData][18]>>16;
		MB_FLOAT2_O_F3=FBI[rightData][18]&0x0000FFFF;
	}
	if(MB_HZ_I==0)
	{
		MB_HZ_I=8001;
		MB_HZ_F=8001;
		Write_Flash();
	}
	
	HAL_NVIC_SetPriority(USART2_IRQn, 0, 2);
  NVIC_EnableIRQ(USART2_IRQn);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE); 
	

	USART2_ReInit(MB_SPEED);
  MX_TIM2_Init(MB_SPEED);

	HAL_ADC_Start_DMA(&hadc,(uint32_t*)&adc1,6);
	Flag=0;
	TIM22->CR1 |= TIM_CR1_CEN; 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		uint8_t i;
		HAL_IWDG_Refresh(&hiwdg);
		
		if(mbReinitCnt>MBReinitTime)
		{
			mbReinitCnt=0;
			//usart reinit
			if(MB_SPEED>8){MB_SPEED=2;}
			USART2_ReInit(MB_SPEED);
			MX_TIM2_Init(MB_SPEED);
			HAL_NVIC_SetPriority(USART2_IRQn, 0, 1);
			HAL_NVIC_EnableIRQ(USART2_IRQn);
			__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
		}
		/*if(needFlashWrite)
		{
			needFlashWrite=0;
			Write_Flash();
		}*/
		
		My_Jump_Boatloader();
		
		if(Flag || Flag_timer/*Dot > 30000*/)
		{
			Flag=0;
			
			
			//MB_HZ_NOW=Dooot;
			if(Flag_timer) 
			{
				Dooot=0;
			}
			Flag_timer=0;
			
			
			MB_AMPL_NOW=adc1[0];
			MB_RMS_NOW=adc1[1];
			MB_AMPL_NOW2=adc1[2];
			MB_RMS_NOW2=adc1[3];
			MB_AMPL_NOW3=adc1[4];
			MB_RMS_NOW3=adc1[5];
			amplDetectorPinSet();
			if(A90)
			{
				MB_RMS_NOW|=0x1000;
				MB_AMPL_NOW|=0x1000;
				MB_RMS_NOW2|=0x1000;
				MB_AMPL_NOW2|=0x1000;
				MB_RMS_NOW3|=0x1000;
				MB_AMPL_NOW3|=0x1000;
			}
			HAL_IWDG_Refresh(&hiwdg);
			for(i=0;i<3;i++) 
			{
				RMSS[i]=adc1[i*2+1]; 
				MaxS[i]=adc1[i*2];
				if(MaxS[i]>(Max_Max[i] & 0x0FFF))
				{
					Max_Max[i]=reg_MB[i*2+1];
				}
			}
			if((MaxS[0]>MB_ATT_ON||MaxS[1]>MB_ATT_ON||MaxS[2]>MB_ATT_ON||FORCE_ATT>0)&&A90==0)
			{
				attOnPinSet();
				A90=1;
				FlagA90=1;
				AttPAftF=1;
				//AttPF=1;
			}
			
			if(MaxS[0]<MB_ATT_OFF&&MaxS[1]<MB_ATT_OFF&&MaxS[2]<MB_ATT_OFF&&A90&&AttPAftF==0&&FORCE_ATT==0)
			{
				attOnPinReset();
				A90=0;
				FlagA90=1;
			}
			
			amplDetectorPinReset();
			
			
		}
		

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
	hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_160CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}



/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA13 PA14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 2);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
	
	HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* USER CODE BEGIN 4 */

void EXTI0_1_IRQHandler(void)
{
	if((EXTI->PR & EXTI_PR_PR0) == EXTI_PR_PR0)
	{
		HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
		Dooot=TIM22->CNT;
		MB_HZ_NOW=Dooot;
		TIM22->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
		TIM22->CNT = 0;
		TIM22->CR1 |= TIM_CR1_CEN; 
		Flag=1;
	}
}

void EXTI4_15_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15))
	{
		attOnPinSet();
		if(A90==0)
		{
			attOnPinSet();
			A90=1;
			FlagA90=1;
			AttPAftF=1;
		}
	}
}

void TIM22_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim22);
	Dooot=0;
	MB_HZ_NOW=TIM22->CNT;
	TIM22->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
	TIM22->CNT = 0;
	TIM22->CR1 |= TIM_CR1_CEN; 
	Flag_timer=1;

}



void USART2_IRQHandler(void)
{
	if((USART2->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
	{	
		
			TIM2->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
			TIM2->CNT=0;
			res_buffer[res_wr_index]=(uint8_t)(USART2->RDR);
			//HAL_UART_Receive(&huart2, &x, 1, 100);
			if(res_wr_index<res_buff_size-1)
			{
				res_wr_index++;			
			}
			FlagMB=1;
			TIM2->CR1 |= TIM_CR1_CEN; 
	}
	HAL_UART_IRQHandler(&huart2);
}

void TIM2_IRQHandler(void)
{
	TIM2->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
	HAL_TIM_IRQHandler(&htim2);

	FlagMB=1;
	
	
		
	uint16_t ind,dt;
	
	uint8_t snd_cnt=0;
	int i;
	if (res_buffer[0]==MB_ADR||res_buffer[0]==247)

  {
		mbReinitCnt=0;
	  CRCCod=CRC16(res_buffer, (res_wr_index));	// Расчет СRC
	  if (CRCCod==0)								// Проверка CRC в посылке
	  {							

			if (MBBusy==0) 
			{
				switch (res_buffer[1]) {
				case 0x03:							// Чтение регистров
				{
					if (res_buffer[0]==247&&(res_buffer[2]<1) && ((res_buffer[3]+res_buffer[4]*256+res_buffer[5]-1)<0x4D))
					{
						write_buffer[0]=res_buffer[0];					// Адрес устройства
						write_buffer[1]=0x03;						// Та-же функция
						write_buffer[2]=res_buffer[5]*2;			// Счетчик байт

						for (i=0; i<res_buffer[5]; i++)				// Значения регистров
						{
							write_buffer[4+(2*i)]=(reg_MB[res_buffer[2]*0x100+res_buffer[3]+i])& 0x00FF;//%256;		// Младший байт (2-ой)
							write_buffer[3+(2*i)]=(reg_MB[res_buffer[2]*0x100+res_buffer[3]+i])>> 8;///256;	// Старший байт (1-ый)
						}		
						snd_cnt=write_buffer[2]+3;
					}
					else if ((res_buffer[2]<1) && ((res_buffer[3]+res_buffer[4]*256+res_buffer[5]-0x25)<13  ||
						(res_buffer[3]+res_buffer[4]*256+res_buffer[5]-0x20)<4 || (res_buffer[3]+res_buffer[4]*256+res_buffer[5]-0x23)<2))
					{
						write_buffer[0]=res_buffer[0];					// Адрес устройства
						write_buffer[1]=0x03;						// Та-же функция
						write_buffer[2]=res_buffer[5]*2;			// Счетчик байт

						for (i=0; i<res_buffer[5]; i++)				// Значения регистров
						{
							write_buffer[4+(2*i)]=(reg_MB[res_buffer[2]*0x100+res_buffer[3]+i])& 0x00FF;//%256;		// Младший байт (2-ой)
							write_buffer[3+(2*i)]=(reg_MB[res_buffer[2]*0x100+res_buffer[3]+i])>> 8;///256;	// Старший байт (1-ый)
						}		
						snd_cnt=write_buffer[2]+3;
					}
					else
					{
						write_buffer[0]=res_buffer[0];					// адрес блока
						write_buffer[1]=0x83;						// та-же функция + взведенный бит ошибки
						write_buffer[2]=0x02;				// код ошибки - недопустимый адрес
						snd_cnt=3;
					}
					break;
				}
				case 0x04:							// Чтение регистров
				{
					if ((res_buffer[2]==1) && ((res_buffer[3]+res_buffer[4]*256+res_buffer[5]-1)<8))
					{
						//ampl=MaxS;
						MB04();

						write_buffer[0]=res_buffer[0];					// Адрес устройства
						write_buffer[1]=0x04;						// Та-же функция
						write_buffer[2]=res_buffer[5]*2;			// Счетчик байт

						for (i=0; i<res_buffer[5]; i++)				// Значения регистров
						{
							write_buffer[4+(2*i)]=(reg_MB2[res_buffer[2]*0x100+res_buffer[3]+i-256])%256;		// Младший байт (2-ой)
							write_buffer[3+(2*i)]=(reg_MB2[res_buffer[2]*0x100+res_buffer[3]+i-256])/256;	// Старший байт (1-ый)
						}


						
						snd_cnt=write_buffer[2]+3;
					}
					else if ((res_buffer[2]==2) && ((res_buffer[3]+res_buffer[4]*256+res_buffer[5]-1)<15))
					{
						//ampl=MaxS;
						MB04();

						write_buffer[0]=res_buffer[0];					// Адрес устройства
						write_buffer[1]=0x04;						// Та-же функция
						write_buffer[2]=res_buffer[5]*2;			// Счетчик байт

						for (i=0; i<res_buffer[5]; i++)				// Значения регистров
						{
							write_buffer[4+(2*i)]=(reg_MB3[res_buffer[2]*0x100+res_buffer[3]+i-512])%256;		// Младший байт (2-ой)
							write_buffer[3+(2*i)]=(reg_MB3[res_buffer[2]*0x100+res_buffer[3]+i-512])/256;	// Старший байт (1-ый)
						}
						
						snd_cnt=write_buffer[2]+3;

					}
					else
					{
						write_buffer[0]=res_buffer[0];					// адрес блока
						write_buffer[1]=0x84;						// та-же функция + взведенный бит ошибки
						write_buffer[2]=0x02;				// код ошибки - недопустимый адрес
						snd_cnt=3;
						

					}
					break;
				}
				case 0x06:						//запись регистра
				{
						if ((res_buffer[2]*0x100+res_buffer[3]>0x20)&&(res_buffer[2]*0x100+res_buffer[3]<0x25))//если возможна запись регистра
						{
							//uint16_t ind,dt;
							write_buffer[0]=res_buffer[0];					// адрес блока
							write_buffer[1]=0x06;						// та-же функция
							write_buffer[2]=res_buffer[2];				// те же данные
							write_buffer[3]=res_buffer[3];
							write_buffer[4]=res_buffer[4];
							write_buffer[5]=res_buffer[5];
							snd_cnt=6;
							ind=res_buffer[2]*0x100+res_buffer[3];
							dt=res_buffer[4]*0x100+res_buffer[5];					
							if(ind==0x22)
							{
								if(dt>8){dt=8;}
								NeedChangeSpeed=1;
								//USART2_ReInit(dt);
								//MX_TIM2_Init(dt);
							}
							reg_MB[ind]=dt;	
							needFlashWrite=1;
							//Write_Flash();
						}
						//zero
						else if(res_buffer[0]==247&&((res_buffer[2]*0x100+res_buffer[3]>7)&&(res_buffer[2]*0x100+res_buffer[3]<0x0E)))
						{	
							MB_AMPL_ZERO=MB_AMPL_NOW&0x0FFF;
							MB_RMS_ZERO=MB_RMS_NOW&0x0FFF;
							MB_AMPL_ZERO2=MB_AMPL_NOW2&0x0FFF;
							MB_RMS_ZERO2=MB_RMS_NOW2&0x0FFF;
							MB_AMPL_ZERO3=MB_AMPL_NOW3&0x0FFF;
							MB_RMS_ZERO3=MB_RMS_NOW3&0x0FFF;
							

							write_buffer[0]=res_buffer[0];					// адрес блока
							write_buffer[1]=0x06;						// та-же функция
							write_buffer[2]=res_buffer[2];				// те же данные
							write_buffer[3]=res_buffer[3];
							write_buffer[4]=res_buffer[4];
							write_buffer[5]=res_buffer[5];
							snd_cnt=6;
						
							needFlashWrite=1;
							//Write_Flash();
						}
						
						else if(res_buffer[0]==247&&((res_buffer[2]*0x100+res_buffer[3]>0x2F)&&(res_buffer[2]*0x100+res_buffer[3]<0x4D)))
						{
							MB04();
							
							

							reg_MB[res_buffer[3]]=res_buffer[4]*0x100+res_buffer[5];


							if(res_buffer[3]==0x4C)
							{
								MB_HZ_I=MB_HZ_1;
							}

							write_buffer[0]=res_buffer[0];					// адрес блока
							write_buffer[1]=0x06;						// та-же функция
							write_buffer[2]=res_buffer[2];				// те же данные
							write_buffer[3]=res_buffer[3];
							write_buffer[4]=res_buffer[4];
							write_buffer[5]=res_buffer[5];
							snd_cnt=6;
						
							needFlashWrite=1;
							//Write_Flash();
						}
						
						else if(res_buffer[0]==247&&res_buffer[2]==0x00&&res_buffer[3]==0x50&&res_buffer[4]==0x50&&res_buffer[5]==0x50)
						{
							write_buffer[0]=res_buffer[0];					// адрес блока
							write_buffer[1]=0x06;						// та-же функция
							write_buffer[2]=res_buffer[2];				// те же данные
							write_buffer[3]=res_buffer[3];
							write_buffer[4]=res_buffer[4];
							write_buffer[5]=res_buffer[5];
							snd_cnt=6;

							
							FORCE_ATT = 0x01;
						}
						
						else if(res_buffer[0]==247&&res_buffer[2]==0x00&&res_buffer[3]==0x50&&res_buffer[4]==0xA0&&res_buffer[5]==0xA0)
						{
							write_buffer[0]=res_buffer[0];					// адрес блока
							write_buffer[1]=0x06;						// та-же функция
							write_buffer[2]=res_buffer[2];				// те же данные
							write_buffer[3]=res_buffer[3];
							write_buffer[4]=res_buffer[4];
							write_buffer[5]=res_buffer[5];
							snd_cnt=6;

							
							FORCE_ATT = 0x00;
						}			
						else if(res_buffer[0]==247&&res_buffer[2]==0x55&&res_buffer[3]==0x55&&res_buffer[4]==0x55&&res_buffer[5]==0x55)
						{
							write_buffer[0]=res_buffer[0];					// адрес блока
							write_buffer[1]=0x06;						// та-же функция
							write_buffer[2]=res_buffer[2];				// те же данные
							write_buffer[3]=res_buffer[3];
							write_buffer[4]=res_buffer[4];
							write_buffer[5]=res_buffer[5];
							snd_cnt=6;	
							bl_flag = 1;
						}	
						else if(res_buffer[0]==247&&res_buffer[2]==0xFF&&res_buffer[3]==0xFF&&res_buffer[4]==0xFF&&res_buffer[5]==0xFF)
						{
							write_buffer[0]=res_buffer[0];					// адрес блока
							write_buffer[1]=0x06;						// та-же функция
							write_buffer[2]=res_buffer[2];				// те же данные
							write_buffer[3]=res_buffer[3];
							write_buffer[4]=res_buffer[4];
							write_buffer[5]=res_buffer[5];
							snd_cnt=6;						
							Erase_Flash();
						}	
						
						else
						{
							write_buffer[0]=res_buffer[0];					// адрес устройства
							write_buffer[1]=0x86;						// та-же функция
							write_buffer[2]=0x02;				// код ошибки - недопустимый адрес
							snd_cnt=3;



						}

						break;
				}
				case 0x10:
				{
					if(res_buffer[0]==247&&res_buffer[2]==0&&
						(res_buffer[3]==0x40||res_buffer[3]==0x42||res_buffer[3]==0x44||res_buffer[3]==0x46||res_buffer[3]==0x48||res_buffer[3]==0x4A)
						&&res_buffer[4]==0&&res_buffer[5]==2
						&&res_buffer[6]==4)
					{
						write_buffer[0]=res_buffer[0];					// Адрес устройства
						write_buffer[1]=res_buffer[1];						// Та-же функция
						write_buffer[2]=res_buffer[2];		
						write_buffer[3]=res_buffer[3];
						write_buffer[4]=res_buffer[4];
						write_buffer[5]=res_buffer[5];
						if(res_buffer[3]==0x40)
						{
							MB_AMPL_N_I=(MB_AMPL_NOW&0x0FFF)-MB_AMPL_ZERO;
							MB_RMS_N_I=(MB_RMS_NOW&0x0FFF)-MB_RMS_ZERO;
						}
						else if(res_buffer[3]==0x44)
						{
							MB_AMPL_N_I2=(MB_AMPL_NOW2&0x0FFF)-MB_AMPL_ZERO2;
							MB_RMS_N_I2=(MB_RMS_NOW2&0x0FFF)-MB_RMS_ZERO2;
						}
						else if(res_buffer[3]==0x48)
						{
							MB_AMPL_N_I3=(MB_AMPL_NOW3&0x0FFF)-MB_AMPL_ZERO3;
							MB_RMS_N_I3=(MB_RMS_NOW3&0x0FFF)-MB_RMS_ZERO3;
						}
						else if(res_buffer[3]==0x42)
						{
							MB_AMPL_O_I=(MB_AMPL_NOW&0x0FFF)-MB_AMPL_ZERO;
							MB_RMS_O_I=(MB_RMS_NOW&0x0FFF)-MB_RMS_ZERO;
						}
						else if(res_buffer[3]==0x46)
						{
							MB_AMPL_O_I2=(MB_AMPL_NOW2&0x0FFF)-MB_AMPL_ZERO2;
							MB_RMS_O_I2=(MB_RMS_NOW2&0x0FFF)-MB_RMS_ZERO2;
						}
						else if(res_buffer[3]==0x4A)
						{
							MB_AMPL_O_I3=(MB_AMPL_NOW3&0x0FFF)-MB_AMPL_ZERO3;
							MB_RMS_O_I3=(MB_RMS_NOW3&0x0FFF)-MB_RMS_ZERO3;
						}
						//reg_MB[res_buffer[3]+9]=(MB_RMS_NOW&0x0FFF)-MB_RMS_ZERO;
						//reg_MB[res_buffer[3]+8]=(MB_AMPL_NOW&0x0FFF)-MB_AMPL_ZERO;
						reg_MB[res_buffer[3]]=res_buffer[7]*0x100+res_buffer[8];
						reg_MB[res_buffer[3]+1]=res_buffer[9]*0x100+res_buffer[10];
						snd_cnt=6;
						needFlashWrite=1;
							//Write_Flash();
					}
					else
					{
						write_buffer[0]=res_buffer[0];					// адрес устройства
						write_buffer[1]=0x90;						// та-же функция
						write_buffer[2]=0x02;				// код ошибки - недопустимый адрес
						snd_cnt=3;

					}	
					break;
				}

				case 0x45:
				{
					
					write_buffer[0]=res_buffer[0];					// адрес блока
					write_buffer[1]=res_buffer[1];						// та-же функция 
					write_buffer[2]=res_buffer[2];						// та-же функция 
					snd_cnt=3;

					if(res_buffer[0]==247)
					{
						FORCE_ATT = res_buffer[2];
					}
					
					//HAL_UART_Transmit(&huart2,write_buffer,5,100);
					break;
				}
				default:
				{
					write_buffer[0]=res_buffer[0];					// адрес блока
					write_buffer[1]=res_buffer[1]+0x80;						// та-же функция + взведенный бит ошибки
					write_buffer[2]=0x01;				// код ошибки - недопустимая функция
					snd_cnt=3;


					break;
				}
			}
		}
		else 
		{
			write_buffer[0]=res_buffer[0];					// адрес блока
			write_buffer[1]=res_buffer[1]+0x80;						// та-же функция + взведенный бит ошибки
			write_buffer[2]=0x06;				// код ошибки - busy
			snd_cnt=3;
		}
		
			CRCCod=CRC16(write_buffer, snd_cnt);				// расчет CRC

			write_buffer[snd_cnt] = CRCCod & 0x00FF;			// мл. байт CRC
			write_buffer[snd_cnt+1] = CRCCod >> 8;				// ст. байт CRC
			HAL_UART_Transmit(&huart2,write_buffer,snd_cnt+2,100);
			//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
		
			if(NeedChangeSpeed)
			{
				NeedChangeSpeed=0;
				USART2_ReInit(dt);
				MX_TIM2_Init(dt);
			}
			
			if(needFlashWrite)
			{
				needFlashWrite=0;
				Write_Flash();
			}
	  }



  }
//HAL_UART_Transmit(&huart2,res_buffer,res_wr_index,100);
	

  res_wr_index=0;
	 HAL_NVIC_SetPriority(USART2_IRQn, 0, 2);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

}

void TIM21_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim21);

	mbReinitCnt++;
	
	if(FlagA90)
	{
		AttP++;
		if(AttP>AttPause)
		{
			FlagA90=0;
			AttP=0;
		}
	}
	if(AttPAftF)
	{
		AttPAft++;
		if(AttPAft>AttPauseAfter)
		{
			AttPAftF=0;
			AttPAft=0;
		}
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

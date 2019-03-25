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
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define AVANCE 	GPIO_PIN_SET
#define RECULE  GPIO_PIN_RESET
#define POURCENT 640
#define Seuil_Dist_4 1600 // corespond à 10 cm.
#define Seuil_Dist_3 1600
#define Seuil_Dist_1 1600
#define Seuil_Dist_2 1600
#define V1 38
#define V2 56
#define V3 76
#define Vmax 95
#define T_2_S 1000 //( pwm période = 2 ms )
#define T_1_S 500
#define T_100_MS 50
#define T_200_MS 100
#define T_2000_MS 1000
#define CKp_D 100  //80 Robot1
#define CKp_G 100  //80 Robot1
#define CKi_D 80  //50 Robot1
#define CKi_G 80  //50 Robot1
#define CKd_D 0
#define CKd_G 0
#define DELTA 0x50

enum CMDE {
	START,
	STOP,
	AVANT,
	ARRIERE,
	DROITE,
	GAUCHE
};
volatile enum CMDE CMDE;
enum MODE {
	SLEEP, ACTIF
};

enum MOOD {
	PARK, MOV_PARK, STOP_PARK
};
volatile enum MOOD Mood = STOP_PARK;

volatile enum MODE Mode;
volatile unsigned char New_CMDE = 0;
volatile uint16_t Dist_ACS_1, Dist_ACS_2, Dist_ACS_3, Dist_ACS_4;
volatile unsigned int Time = 0;
volatile unsigned int Tech = 0;
uint16_t adc_buffer[8];
uint16_t Buff_Dist[8];
uint8_t BLUE_RX;

uint16_t _DirG, _DirD, CVitG, CVitD, DirD, DirG;
uint16_t _CVitD = 0;
uint16_t _CVitG = 0;
uint16_t VitD, VitG, DistD, DistG;
uint16_t DistD_old = 0;
uint16_t DistG_old = 0;
int Cmde_VitD = 0;
int Cmde_VitG = 0;
unsigned long Dist_parcours = 0;
volatile uint32_t Dist_Obst;
volatile uint32_t  Distance_Sonar;
uint32_t Dist_Obst_;
uint32_t Dist_Obst_cm;
uint32_t Dist;
uint8_t UNE_FOIS = 1;
uint32_t OV = 0;
volatile unsigned int Tech_Park;	//compteur temps fonction park
volatile unsigned int Tech_Rot = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Gestion_Commandes(void);
void regulateur(void);
void controle(void);
void Calcul_Vit(void);
void ACS(void);
void Lecture_Sonar(uint32_t xi, uint32_t yi, uint32_t zi);
void Park(void);
void Mov_Park(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  Dist_Obst = 0;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
  	HAL_SuspendTick(); // suppression des Tick interrupt pour le mode sleep.

  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  // Start PWM servo-motor
  	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  	CMDE = STOP;
  	New_CMDE = 1;
  	HAL_TIM_Base_Start_IT(&htim2);	// Start IT sur font montant PWM
  	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  	HAL_UART_Receive_IT(&huart3, &BLUE_RX, 1);

  	HAL_ADC_Start_IT(&hadc1);
  	//Init IT surveillance batterie
	HAL_TIM_Base_Start_IT(&htim1);
  	HAL_TIM_IC_Start_IT (&htim1, TIM_CHANNEL_1);
  	HAL_TIM_IC_Start_IT (&htim1, TIM_CHANNEL_2);	//Init IT lecture sonar
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  	 HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);		//start servo
  	 //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 5000);		//bouger servo 0°
  while (1)
  {	 Gestion_Commandes();
	 controle();
		Park();
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

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
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

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI15_10_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* USART3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
}

/* USER CODE BEGIN 4 */
void Gestion_Commandes(void) {
	enum ETAT {
		VEILLE,
		ARRET,
		AV1,
		AV2,
		AV3,
		RV1,
		RV2,
		RV3,
		DV1,
		DV2,
		DV3,
		GV1,
		GV2,
		GV3
	};
	static enum ETAT Etat = VEILLE;

if (New_CMDE) {
		New_CMDE = 0;
	switch (CMDE) {
		case STOP: {
			_CVitD = _CVitG = 0;
			// Mise en sommeil: STOP mode , réveil via IT BP1
			Etat = VEILLE;
			Mode = SLEEP;

			break;
		}
		case START: {
			// réveil sytème grace à l'IT BP1
			Etat = ARRET;
			Mode = SLEEP;

			break;
		}
		case AVANT: {
			switch (Etat) {
			case VEILLE: {
				Etat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = AV1;
				Mode = ACTIF;
				break;
			}
			case AV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case AV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3 ;
				Etat = AV3;
				Mode = ACTIF;
				break;
			}
			case AV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3 ;
				Etat = AV3;
				Mode = ACTIF;
				break;
			}
			case RV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = 0;
				_CVitD = 0;
				Etat = ARRET;
				Mode = SLEEP;

				break;
			}
			case RV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = RV1;
				Mode = ACTIF;
				break;
			}
			case RV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = RV2;
				Mode = ACTIF;
				break;
			}
			case DV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = AV1;
				Mode = ACTIF;
				break;
			}
			case DV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case DV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = AV3;
				Mode = ACTIF;
				break;
			}
			case GV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case GV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case GV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = AV3;
				Mode = ACTIF;
				break;
			}
			}
			break;
		}
		case ARRIERE: {
			switch (Etat) {
			case VEILLE: {
				Etat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = RV1;
				Mode = ACTIF;
				break;
			}
			case AV1: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = 0;
				_CVitD = 0;
				Etat = ARRET;
				Mode = SLEEP;

				break;
			}
			case AV2: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = AV1;
				Mode = ACTIF;
				break;
			}
			case AV3: {
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = AV2;
				Mode = ACTIF;
				break;
			}
			case RV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = RV2;
				Mode = ACTIF;
				break;
			}
			case RV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = RV3;
				Mode = ACTIF;
				break;
			}
			case RV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = RV3;
				Mode = ACTIF;
				break;
			}
			case DV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = RV1;
				Mode = ACTIF;
				break;
			}
			case DV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = RV2;
				Mode = ACTIF;
				break;
			}
			case DV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = RV3;
				Mode = ACTIF;
				break;
			}
			case GV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = RV1;
				Mode = ACTIF;
				break;
			}
			case GV2: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = RV2;
				Mode = ACTIF;
				break;
			}
			case GV3: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = RV3;
				Mode = ACTIF;
				break;
			}
			}
			break;
		}
		case DROITE: {
			switch (Etat) {
			case VEILLE: {
				Etat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = DV1;
				Mode = ACTIF;
				break;
			}
			case AV1: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = DV1;
				Mode = ACTIF;
				break;
			}
			case AV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = DV2;
				Mode = ACTIF;
				break;
			}
			case AV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = DV3;
				Mode = ACTIF;
				break;
			}
			case RV1: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = DV1;
				Mode = ACTIF;
				break;
			}
			case RV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = DV2;
				Mode = ACTIF;
				break;
			}
			case RV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = DV3;
				Mode = ACTIF;
				break;
			}
			case DV1: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = DV2;
				Mode = ACTIF;
				break;
			}
			case DV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = DV3;
				Mode = ACTIF;
				break;
			}
			case DV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = DV3;
				Mode = ACTIF;
				break;
			}
			case GV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = 0;
				_CVitD = 0;
				Etat = ARRET;
				Mode = SLEEP;

				break;
			}
			case GV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = GV1;
				Mode = ACTIF;
				break;
			}
			case GV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = GV2;
				Mode = ACTIF;
				break;
			}
			}
			break;
		}
		case GAUCHE: {
			switch (Etat) {
			case VEILLE: {
				Etat = VEILLE;
				Mode = SLEEP;
				break;
			}
			case ARRET: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = GV1;
				Mode = ACTIF;
				break;
			}
			case AV1: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = GV1;
				Mode = ACTIF;
				break;
			}
			case AV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = GV2;
				Mode = ACTIF;
				break;
			}
			case AV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = GV3;
				Mode = ACTIF;
				break;
			}
			case RV1: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = GV1;
				Mode = ACTIF;
				break;
			}
			case RV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = GV2;
				Mode = ACTIF;
				break;
			}
			case RV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = GV3;
				Mode = ACTIF;
				break;
			}
			case DV1: {
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = 0;
				_CVitD = 0;
				Etat = ARRET;
				Mode = SLEEP;

				break;
			}
			case DV2: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V1;
				_CVitD = V1;
				Etat = DV1;
				Mode = ACTIF;
				break;
			}
			case DV3: {
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = DV2;
				Mode = ACTIF;
				break;
			}
			case GV1: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				Etat = GV2;
				Mode = ACTIF;
				break;
			}
			case GV2: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = GV3;
				Mode = ACTIF;
				break;
			}
			case GV3: {
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V3;
				_CVitD = V3;
				Etat = GV3;
				Mode = ACTIF;
				break;
			}
			break;
		}
	}
}
}
}

void controle(void) {

	if (Tech >= T_200_MS) {
		Tech = 0;
		ACS();
		Calcul_Vit();
		regulateur();
	}

}

void ACS(void) {
	enum ETAT {
		ARRET, ACTIF
	};
	static enum ETAT Etat = ARRET;
	static uint16_t Delta1 = 0;
	static uint16_t Delta2 = 0;
	static uint16_t Delta3 = 0;
	static uint16_t Delta4 = 0;

	switch (Etat) {
	case ARRET: {
		if (Mode == ACTIF )
			Etat = ACTIF;
		else{
			CVitD = _CVitD;
			CVitG = _CVitG;
			DirD = _DirD;
			DirG = _DirG;
		}
		break;
	}
	case ACTIF: {
		if (Mode == SLEEP)
			Etat = ARRET;
		if (_DirD == AVANCE && _DirG == AVANCE) {
			if ((Dist_ACS_1 < Seuil_Dist_1 - Delta1)
					&& (Dist_ACS_2 < Seuil_Dist_2 - Delta2)) {
				CVitD = _CVitD;
				CVitG = _CVitG;
				DirD = _DirD;
				DirG = _DirG;
				Delta1 = Delta2 = 0;
			} else if ((Dist_ACS_1 < Seuil_Dist_1)
					&& (Dist_ACS_2 > Seuil_Dist_2)) {
				CVitD = V1;
				CVitG = V1;
				DirG = AVANCE;
				DirD = RECULE;
				Delta2 = DELTA;
			} else if ((Dist_ACS_1 > Seuil_Dist_1)
					&& (Dist_ACS_2 < Seuil_Dist_2)) {
				CVitD = V1;
				CVitG = V1;
				DirD = AVANCE;
				DirG = RECULE;
				Delta1 = DELTA;
			} else if ((Dist_ACS_1 > Seuil_Dist_1)
					&& (Dist_ACS_2 > Seuil_Dist_2)) {
				CVitD = 0;
				CVitG = 0;
				DirD = RECULE;
				DirG = RECULE;
			}
		} else if (_DirD == RECULE && _DirG == RECULE) {
			if ((Dist_ACS_3 < Seuil_Dist_3 - Delta3)
					&& (Dist_ACS_4 < Seuil_Dist_4 - Delta4)) {
				CVitD = _CVitD;
				CVitG = _CVitG;
				DirD = _DirD;
				DirG = _DirG;
				Delta3 = Delta4 = 0;
			} else if ((Dist_ACS_3 > Seuil_Dist_3)
					&& (Dist_ACS_4 < Seuil_Dist_4)) {
				CVitD = V1;
				CVitG = V1;
				DirD = AVANCE;
				DirG = RECULE;
				Delta3 = DELTA;
			} else if ((Dist_ACS_3 < Seuil_Dist_3)
					&& (Dist_ACS_4 > Seuil_Dist_4)) {
				CVitD = V1;
				CVitG = V1;
				DirG = AVANCE;
				DirD = RECULE;
				Delta4 = DELTA;
			} else if ((Dist_ACS_3 > Seuil_Dist_3)
					&& (Dist_ACS_4 > Seuil_Dist_4)) {
				CVitD = 0;
				CVitG = 0;
				DirD = RECULE;
				DirG = RECULE;
			}
		} else {
			CVitD = _CVitD;
			CVitG = _CVitG;
			DirD = _DirD;
			DirG = _DirG;
		}
		break;
	}
	}
}

void Calcul_Vit(void) {

	DistD = __HAL_TIM_GET_COUNTER(&htim3);
	DistG = __HAL_TIM_GET_COUNTER(&htim4);
	VitD = abs(DistD - DistD_old);
	VitG = abs(DistG - DistG_old);
	DistD_old = DistD;
	DistG_old = DistG;
	if (DirD == DirG) {
		Dist_parcours = Dist_parcours + ((VitD + VitG) >> 1);
	}
}

void regulateur(void) {
	enum ETAT {
		ARRET, ACTIF
	};
	static enum ETAT Etat = ARRET;
	uint16_t Kp_D = CKp_D;
	uint16_t Kp_G = CKp_G;
	uint16_t Ki_D = CKi_D;
	uint16_t Ki_G = CKi_G;
	uint16_t Kd_D = CKd_D;
	uint16_t Kd_G = CKd_G;

	static int16_t ErreurD = 0;
	static int16_t ErreurG = 0;
	static int16_t ErreurD_old = 0;
	static int16_t ErreurG_old = 0;
	static int16_t S_erreursD = 0;
	static int16_t S_erreursG = 0;
	static int16_t V_erreurD = 0;
	static int16_t V_erreurG = 0;

	switch (Etat) {
	case ARRET: {
		if (Mode == ACTIF)
			Etat = ACTIF;
		else {
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_4);
			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(IR3_out_GPIO_Port, IR3_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR4_out_GPIO_Port, IR4_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR1_out_GPIO_Port, IR1_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR2_out_GPIO_Port, IR2_out_Pin, GPIO_PIN_RESET);

			HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON,
					PWR_SLEEPENTRY_WFI);

			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			Time = 0;
		}
		break;
	}
	case ACTIF: {
		if ((CVitD != 0) && (CVitG != 0))
			Time = 0;
		if ((Mode == SLEEP) && (VitD == 0) && (VitG == 0) && Time > T_2_S)
			Etat = ARRET;
		else {
			ErreurD = CVitD - VitD;
			ErreurG = CVitG - VitG;
			S_erreursD += ErreurD;
			S_erreursG += ErreurG;
			V_erreurD = ErreurD - ErreurD_old;
			V_erreurG = ErreurG - ErreurG_old;
			ErreurD_old = ErreurD;
			ErreurG_old = ErreurG;
			Cmde_VitD = (unsigned int) Kp_D * (int) (ErreurD)
					+ (unsigned int) Ki_D * ((int) S_erreursD)
					+ (unsigned int) Kd_D * (int) V_erreurD;
			Cmde_VitG = (unsigned int) Kp_G * (int) (ErreurG)
					+ (unsigned int) Ki_G * ((int) S_erreursG)
					+ (unsigned int) Kd_G * (int) V_erreurG;

			//Cmde_VitD = _CVitD*640;
			//Cmde_VitG = _CVitG*640;
			//	DirD = _DirD;
			//	DirG= _DirG;

			if (Cmde_VitD < 0)
				Cmde_VitD = 0;
			if (Cmde_VitG < 0)
				Cmde_VitG = 0;
			if (Cmde_VitD > 100 * POURCENT)
				Cmde_VitD = 100 * POURCENT;
			if (Cmde_VitG > 100 * POURCENT)
				Cmde_VitG = 100 * POURCENT;
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint16_t ) Cmde_VitG);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (uint16_t ) Cmde_VitD);
			HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, (GPIO_PinState) DirD);
			HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, (GPIO_PinState) DirG);

		}
		break;
	}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {

		switch (BLUE_RX) {
		case 'F': {
			CMDE = AVANT;
			//New_CMDE = 1;
			break;
		}

		case 'B': {
			CMDE = ARRIERE;
			//New_CMDE = 1;
			break;
		}

		case 'L': {
			CMDE = GAUCHE;
			//New_CMDE = 1;
			break;
		}

		case 'R': {
			CMDE = DROITE;
			//New_CMDE = 1;
			break;
		}

		case 'D':{
			// disconnect bluetooth
			break;
		}
		case 'X': {
			Mood = PARK;
			//New_CMDE = 1;
			break;
		}
		case 'W':{
			Mood = MOV_PARK;
			Tech_Rot = 0;
			//New_CMDE = 1;
			break;
		}
		default:
			New_CMDE = 1;
		}

		HAL_UART_Receive_IT(&huart3, &BLUE_RX, 1);

	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	Dist_ACS_3 = adc_buffer[0] - adc_buffer[4];
	Dist_ACS_4 = adc_buffer[3] - adc_buffer[7];
	Dist_ACS_1 = adc_buffer[1] - adc_buffer[5];
	Dist_ACS_2 = adc_buffer[2] - adc_buffer[6];
	HAL_ADC_Stop_DMA(hadc);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim) {
	static unsigned char cpt = 0;

	if ( htim->Instance == TIM2) {
		cpt++;
		Time++;
		Tech++;
		Tech_Park++;
		Tech_Rot++;

		switch (cpt) {
		case 1: {
			HAL_GPIO_WritePin(IR3_out_GPIO_Port, IR3_out_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(IR4_out_GPIO_Port, IR4_out_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(IR1_out_GPIO_Port, IR1_out_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(IR2_out_GPIO_Port, IR2_out_Pin, GPIO_PIN_SET);
			break;
		}
		case 2: {
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_buffer, 8);
			break;
		}
		case 3: {
			HAL_GPIO_WritePin(IR3_out_GPIO_Port, IR3_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR4_out_GPIO_Port, IR4_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR1_out_GPIO_Port, IR1_out_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(IR2_out_GPIO_Port, IR2_out_Pin, GPIO_PIN_RESET);
			break;
		}
		case 4: {
			HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adc_buffer, 8);
			break;
		}
		default:
			cpt = 0;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	static unsigned char TOGGLE = 0;

	if (TOGGLE)
		CMDE = STOP;
	else
		CMDE = START;
	TOGGLE = ~TOGGLE;
	New_CMDE = 1;

}


void Park(void){
	enum STATE{
		ACQ, ROTG, ROTD, AVANCE_S, RECULE_S
	};
	enum CAS {
		UN, DEUX
	};
	static enum STATE state = ROTG;
	static enum CAS cas = UN;
	static uint32_t x_p = 0;
	static uint32_t y_p = 0;
	static uint32_t z_p = 0;	//Coordonnées du robot à l'état park
	static uint32_t y = 0;				//Distance instantanée mesurée par le sonar
	static uint32_t  dist = 0;


	if (Tech_Park >= T_100_MS){
		Tech_Park = 0;
		switch (Mood){
		case PARK:{
				//Lecture de x (gauche)
			  	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 5000);		//bouger servo 90°
			  	 while (Tech_Park <= T_1_S);
			  	 Tech_Park = 0;
			  	 HAL_GPIO_WritePin(GPIOB, TRIG_SONAR_Pin, GPIO_PIN_SET);	//Met le trigger du sonar à 1
			  	while (Tech_Park <= T_100_MS);
			  	Tech_Park = 0;
			  	 x_p = Distance_Sonar;
			  	 HAL_GPIO_WritePin(GPIOB, TRIG_SONAR_Pin, GPIO_PIN_RESET);	//Met le trigger du sonar à 0

			  	 //Lecture de y (tout droit)
			  	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2800);		//bouger servo 0°
			  	while (Tech_Park <= T_1_S);
			  	Tech_Park = 0;
				 HAL_GPIO_WritePin(GPIOB, TRIG_SONAR_Pin, GPIO_PIN_SET);	//Met le trigger du sonar à 1
				 while (Tech_Park <= T_100_MS);
				Tech_Park = 0;
				 y_p = Distance_Sonar;
				 HAL_GPIO_WritePin(GPIOB, TRIG_SONAR_Pin, GPIO_PIN_RESET);	//Met le trigger du sonar à 0

				//Lecture de z (droite)
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 900);			//bouger servo -90°
				while (Tech_Park <= T_1_S);
				Tech_Park = 0;
				 HAL_GPIO_WritePin(GPIOB, TRIG_SONAR_Pin, GPIO_PIN_SET);	//Met le trigger du sonar à 1
				 while (Tech_Park <= T_1_S);
				 Tech_Park = 0;
				 z_p = Distance_Sonar;
				 HAL_GPIO_WritePin(GPIOB, TRIG_SONAR_Pin, GPIO_PIN_RESET);	//Met le trigger du sonar à 0

			  	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2800);		//bouger servo 0°

				Mood = STOP_PARK;
				break;
		}

		case MOV_PARK :{
			switch (state){
			case ROTG :{
				//Rotation à gauche
				Mode = ACTIF;
				_DirG = RECULE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				if (Tech_Rot > 1100){
					Mode = SLEEP;
					_CVitG = 0;
					_CVitD = 0;
					state = ACQ;
					Tech_Rot = 0;
				}
				break;
			}

			case ROTD :{
				//Rotation à droite
				Mode = ACTIF;
				_DirG = AVANCE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				if (Tech_Rot > 1100){
					Mode = SLEEP;
					_CVitG = 0;
					_CVitD = 0;
					state = ACQ;
					Tech_Rot = 0;
					cas = DEUX;
				}
				break;
			}

			case ACQ :{
			  	 //Lecture de y (tout droit)
			  	 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 2800);		//bouger servo 0°
			  	while (Tech_Park <= T_1_S);
			  	Tech_Park = 0;
				 HAL_GPIO_WritePin(GPIOB, TRIG_SONAR_Pin, GPIO_PIN_SET);	//Met le trigger du sonar à 1
				 while (Tech_Park <= T_100_MS);
				 Tech_Park = 0;
				 y = Distance_Sonar;
				 HAL_GPIO_WritePin(GPIOB, TRIG_SONAR_Pin, GPIO_PIN_RESET);	//Met le trigger du sonar à 0

				switch (cas){
				case UN :{
					dist = x_p;
					break;
				}

				case DEUX :{
					dist = y_p;	//cas de la seconde mesure
					break;
				}
				}

				if (y > dist)
					state = AVANCE_S;
				/*else if (dist - y > 500)
					state = RECULE_S;*/
				else {
					Mode = SLEEP;
					_CVitG = 0;
					_CVitD = 0;
					switch(cas){
					case UN :{
						state = ROTD;
						break;
					}
					case DEUX :{
						Mood = STOP_PARK;
						break;
					}
					}
				}

				Tech_Rot = 0;
				break;
			}

			case AVANCE_S :{
				//Avance
				Mode = ACTIF;
				_DirG = AVANCE;
				_DirD = AVANCE;
				_CVitG = V2;
				_CVitD = V2;
				state = ACQ;
				break;
			}

			case RECULE_S :{
				//Recule
				Mode = ACTIF;
				_DirG = RECULE;
				_DirD = RECULE;
				_CVitG = V2;
				_CVitD = V2;
				state = ACQ;
				break;
			}
			}
			break;
		}

		case STOP_PARK :{
			Mode = SLEEP;
			_CVitG = 0;
			_CVitD = 0;
			break;
		}
		}
	}
}

//IT surveillance batterie
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc)
{
  HAL_GPIO_WritePin( LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
}

//IT front descendant sonar
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
	  Distance_Sonar = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);
  }
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
	  HAL_GPIO_WritePin(GPIOB, TRIG_SONAR_Pin, GPIO_PIN_RESET);
    }
  /* NOTE : This function Should not be modified, when the callback is needed,
            the __HAL_TIM_IC_CaptureCallback could be implemented in the user file
   */
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

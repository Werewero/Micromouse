
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include <math.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//rejestry imu9
#define LSM6DS33_gyro_adress (0x6A<<1) // adres czujnika: 1101010x
#define LSM6DS33_CTRL2_G_register 0x11 // rejestr ustawien zyroskopu
#define LSM6DS33_OUTX_L_G 0x22 // rejestr bajtu nizszego osi x
#define LSM6DS33_OUTX_H_G 0x23 // rejestr bajtu wyzszego osi x
#define LSM6DS33_OUTY_L_G 0x24 // rejestr bajtu nizszego osi y
#define LSM6DS33_OUTY_H_G 0x25 // rejestr bajtu wyzszego osi y
#define LSM6DS33_OUTZ_L_G 0x26 // rejestr bajtu nizszego osi z
#define LSM6DS33_OUTZ_H_G 0x27 // rejestr bajtu wyzszego osi z
//#define LSM6DS33_INT1_CTRL 0x0D// rejestr przygotowania danych do odbioru?
#define LSM6DS33_WHO_AM_I 0x0F // rejestr who am i
#define LSM6DS33_TEMPERATURE 0x21 // rejsetr temperatury czujnika

//maski bitowe
//CTRL2_G = [ODR_G3][ODR_G2][ODR_G1][ODR_G0][ FS_G1][ FS_G0][ FS_125][0] rejsetr ustawien zyroskopu
#define LSM6DS33_G_104Hz 0x40 //  0100 0000
#define LSM6DS33_G_125dps 0x2 //0000 0010
#define LSM6DS33_G_500dps 0x4 //0000 0100
#define LSM6DS33_G_416Hz 0x60 // 0110 0000

//wlaczenie interrupta na INT1 (nie u¿ywane)
#define LSM6DS33_INT1_DRDY_G 0x02 // 0000 0010

//zmienne zyroskop
uint8_t whoami; // test, czy zyroskop czujnik dziala i czy dobrze jest identyfikowany
uint8_t DataX1 = 0; // Zmienna do bezposredniego odczytu z zyroskopu bajtu wyzszego
uint8_t DataX2 = 0; //Zmienna do bezposredniego odczytu z zyroskopu bajtu nizszego
int16_t Xaxis = 0; // Zawiera przeksztalcona forme odczytanych danych

//zmienne do liczenia k¹tu
float Xw;


//k¹t okreœlaj¹cy po³o¿enie robota
int pomiary=1;
int forward = 1;
int left = 0;
int right = 0;
float angle=0.0;
float main_angle=0.0;

//zmienne pomiar odleglosci
//wartosci robocze
uint32_t time1;
uint32_t time2;
uint32_t time3;

 float dystans1;
 float dystans2;
 float dystans3;

//wartosci przetworzone
float srednia1=0;
float srednia2=0;
float srednia3=0;

float odleglosc1;
float odleglosc2;
float odleglosc3;

//regulator PID

int u=0;
float e=0.0; //error
float e_sum=0.0;
float e_last=0.0;
float kp= 10.0;
float kd =-0.3;
float ki = 1.0;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM6_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void HAL_SYSTICK_Callback(void){
if(fabsf(Xw)>0.1){
			angle=angle + Xw*0.01;
			e= angle-main_angle;
			e_sum=e_sum +e;
			if(e_sum>100.0) e_sum=100.0;
			if(e_sum<-100.0) e_sum=-100.0;

			u = kp*e + ki*e_sum + kd*(e - e_last); //PID
			 if(dystans1<=15){
				 for(int v = 180; v > 0;v = v - 30)

				 {
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, v);
				__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, v);

				}
			 }
			 else{

			if(fabsf(e)<80.0){
				if(e>0){
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 200);//lewe
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 200+fabsf(u));//prawe
			}
				if(e<0){
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 200+fabsf(u));
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 200);
			}
			}

			/*if(fabs(e)>=80.0){
				if(e>0){
					HAL_GPIO_WritePin(L_ENGINE_1_GPIO_Port, L_ENGINE_1_Pin, RESET);
					HAL_GPIO_WritePin(L_ENGINE_2_GPIO_Port, L_ENGINE_2_Pin, SET);
					HAL_GPIO_WritePin(R_ENGINE_1_GPIO_Port, R_ENGINE_1_Pin, SET);
					HAL_GPIO_WritePin(R_ENGINE_2_GPIO_Port, R_ENGINE_2_Pin, RESET);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 400);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 400+u);
				}
				if(e<0){
					HAL_GPIO_WritePin(L_ENGINE_1_GPIO_Port, L_ENGINE_1_Pin, SET);
					HAL_GPIO_WritePin(L_ENGINE_2_GPIO_Port, L_ENGINE_2_Pin, RESET);
					HAL_GPIO_WritePin(R_ENGINE_1_GPIO_Port, R_ENGINE_1_Pin, RESET);
					HAL_GPIO_WritePin(R_ENGINE_2_GPIO_Port, R_ENGINE_2_Pin, SET);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 400+u);
					__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 400);
				}
*/





			e_last=e;


	}
}
	/*if(fabsf(Xaxis)<=0.1){
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 500);
		__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 500);
	}*/

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	static uint16_t i = 0; // licznik
	if (htim->Instance == TIM17){ //przerwanie wysylajace sygnal na pin trigger czujnika odleglosci

	switch (i)
	{
	case 0 :
		HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, SET);
		HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, SET);
		HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, SET);
		break;
	case 1:
		HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, RESET);
		HAL_GPIO_WritePin(TRIG2_GPIO_Port, TRIG2_Pin, RESET);
		HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, RESET);
		break;
	}
	i++;
	if (i==6000) i=0;
	}

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) //przerwanie odbierajace sygnal z echo czujnika odleglosci
{	static uint16_t i = 0;
	static uint16_t j = 0;
	static uint16_t k = 0;
//if(pomiary==1){
if(GPIO_Pin == ECHO1_Pin){ //czujnik 1
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)==GPIO_PIN_SET)
		__HAL_TIM_SET_COUNTER(&htim16, 0);
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0)==GPIO_PIN_RESET)
	{
		time:
		time1 = __HAL_TIM_GET_COUNTER(&htim16);
		dystans1 = time1/58.;
		if(dystans1>300.0 || dystans1<3.5) goto time; //wstepna redukcja szumow

		//if(dystans1<= 10.0) forward =0;
		//drugi etap redukcji szumów
		srednia1=srednia1+dystans1;
		i++;
		if(i==10){
			i=0;
			odleglosc1=srednia1/10;
			if(dystans1<= 10.0) forward =0;
			srednia1=0;
		}


	}


}
if(GPIO_Pin == ECHO2_Pin){ //czujnik 2
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)==GPIO_PIN_SET)
		__HAL_TIM_SET_COUNTER(&htim7, 0);
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)==GPIO_PIN_RESET)
		{
			time2:
			time2 = __HAL_TIM_GET_COUNTER(&htim7);
			dystans2 = time2/58.;
			if(dystans2>300.0 || dystans2<3.5) goto time2;
			//if(dystans2>10.0) right=1;
			srednia2=srednia2+dystans2;
			j++;
			if(j==10){
				odleglosc2 = srednia2/10;

				j=0;
				srednia2=0;
			}
		}
}
if(GPIO_Pin == ECHO3_Pin) { //czujnik 3
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6)==GPIO_PIN_SET)
		__HAL_TIM_SET_COUNTER(&htim6, 0);
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6)==GPIO_PIN_RESET)
		{
			time3:
			time3 = __HAL_TIM_GET_COUNTER(&htim6);
			dystans3 = time3/58.;
			if(dystans3>200.0 || dystans3<3.5) goto time3;
			//if(dystans3>10.0) left = 1;
			srednia3=srednia3+dystans3;
			k++;
			if(k==10){
				odleglosc3 = srednia3/10;
				if(odleglosc3>10.0) left = 1;
				k=0;
				srednia3=0;
			}
		}

}

//}

}
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM17_Init();
  MX_TIM16_Init();
  MX_TIM7_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  //inicjalizacja timerow
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_Base_Start(&htim6);
    HAL_TIM_Base_Start(&htim7);
    HAL_TIM_Base_Start(&htim16);
    HAL_TIM_Base_Start_IT(&htim17);

    //inicjalizacja polaczenia i2c z zyroskopem
    uint8_t Settings = LSM6DS33_G_104Hz | LSM6DS33_G_125dps; // ustawienia rejsetsru zyroskopu (suma bitowa)
    HAL_I2C_Mem_Write(&hi2c1, LSM6DS33_gyro_adress, LSM6DS33_CTRL2_G_register, 1, &Settings, 1, 100);// wpisanie wartosci do rejsetru stawien zyroskopu

    //bazowy kierunek obrotu silnikow
      HAL_GPIO_WritePin(L_ENGINE_1_GPIO_Port, L_ENGINE_1_Pin, RESET);
      HAL_GPIO_WritePin(L_ENGINE_2_GPIO_Port, L_ENGINE_2_Pin, SET);
      HAL_GPIO_WritePin(R_ENGINE_1_GPIO_Port, R_ENGINE_1_Pin, SET);
      HAL_GPIO_WritePin(R_ENGINE_2_GPIO_Port, R_ENGINE_2_Pin, RESET);

      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //identyfikacja czujnika (who am i)
		  	  HAL_I2C_Mem_Read(&hi2c1, LSM6DS33_gyro_adress, LSM6DS33_WHO_AM_I, 1, &whoami, 1 ,100);

		  	  //odczytanie wartosci z zyroskopu
		  	  HAL_I2C_Mem_Read(&hi2c1, LSM6DS33_gyro_adress, LSM6DS33_OUTX_H_G, 1, &DataX1, 1 ,10);
		  	  HAL_I2C_Mem_Read(&hi2c1, LSM6DS33_gyro_adress, LSM6DS33_OUTX_L_G, 1, &DataX2, 1 ,10);
		  	  Xaxis = (DataX1 << 8) + DataX2 - 1260;
		  	  Xw= Xaxis*125.0/(float)INT16_MAX/8;



		  	 /* if(fabsf(e)<20.0) pomiary=1;

		  	  if(forward==0 && left==1){

		  		main_angle=main_angle -90.0;
		  		left=0;
		  		forward=1;
		  	  }

		  	  if(forward==0 && left == 0 && right== 1){

		  		  main_angle=main_angle+90.0;
		  		  right=0;
		  		  forward=1;
		  	  }
		  	  if(forward==0 && left==0 && right == 0){

		  		  main_angle=main_angle + 180.0;
		  		  forward=1;
		  	  }*/



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
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
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

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_TIM16|RCC_PERIPHCLK_TIM17
                              |RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  PeriphClkInit.Tim17ClockSelection = RCC_TIM17CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
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

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 71;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM7 init function */
static void MX_TIM7_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 71;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM16 init function */
static void MX_TIM16_Init(void)
{

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 71;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM17 init function */
static void MX_TIM17_Init(void)
{

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 71;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 9;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TRIG1_Pin|TRIG2_Pin|L_ENGINE_2_Pin|L_ENGINE_1_Pin 
                          |R_ENGINE_1_Pin|R_ENGINE_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG3_GPIO_Port, TRIG3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : ECHO1_Pin ECHO2_Pin ECHO3_Pin */
  GPIO_InitStruct.Pin = ECHO1_Pin|ECHO2_Pin|ECHO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG1_Pin TRIG2_Pin L_ENGINE_2_Pin L_ENGINE_1_Pin 
                           R_ENGINE_1_Pin R_ENGINE_2_Pin */
  GPIO_InitStruct.Pin = TRIG1_Pin|TRIG2_Pin|L_ENGINE_2_Pin|L_ENGINE_1_Pin 
                          |R_ENGINE_1_Pin|R_ENGINE_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG3_Pin */
  GPIO_InitStruct.Pin = TRIG3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG3_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

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

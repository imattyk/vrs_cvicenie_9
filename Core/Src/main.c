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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm6ds0.h"
#include "hts221.h"
#include "lps25hb.h"
#include "lis3mdl.h"
uint8_t text_to_display[] = "____";
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
enum abeceda{
	A = 0b1110111,
	a = 0b1111101,
	b = 0b0011111,
	C = 0b1001110,
	c = 0b0001101,
	d = 0b0111101,
	E = 0b1001111,
	F = 0b1000111,
	G = 0b1011110,
	H = 0b0110111,
	h = 0b0010111,
	I = 0b0000110,
	J = 0b0111100,
	L = 0b0001110,
	n = 0b0010101,
	O = 0b1111110,
	o = 0b0011101,
	P = 0b1100111,
	q = 0b1110011,
	r = 0b0000101,
	S = 0b1011011,
	t = 0b0001111,
	U = 0b0111110,
	u = 0b0011100,
	y = 0b0111011,

	K = 0b1010111,
	M = 0b1101010,
	W = 0b0111111,
	X = 0b1001001,
	Z = 0b1101101,
	V = 0b0101010,

	num_1 = 0b0110000,
	num_2 = 0b1101101,
	num_3 = 0b1111001,
	num_4 = 0b0010011,
	num_5 = 0b1011011,
	num_6 = 0b1011111,
	num_7 = 0b1110000,
	num_8 = 0b1111111,
	num_9 = 0b1111011,
	num_0 = 0b1111110,
};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
float mag[3], acc[3];
uint16_t buffer;
int16_t temperature = 0;
uint16_t humidity = 0;
float pressure = 0;
uint16_t altitude = 0;
float azimut = 0;
uint8_t status = 0;
uint8_t button_state = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  LL_TIM_EnableIT_UPDATE(TIM2);
  LL_TIM_EnableCounter(TIM2);

  lsm6ds0_init();
  hts221_init();
  lps25hb_init();
  lis3mdl_init();
  //

   // VYPISUJEME TENTO RETAZEC, AKOKOLVEK DLHY, COKOLVEK V NOM JE, JE TO SPRAVENE DYNAMICKY, NIE HARDCODED
   char vypis[15];

   int index = 0;
   int flag = 0;

   char mag_vypis[12];
   char azimut_str[10];

   char teplota_vypis[12];
   char teplota_str[10];

   char vlhkost_vypis[12];
   char vlhkost_str[10];

   char tlak_vypis[15];
   char tlak_str[10];

   char vyska_vypis[12];
   char vyska_str[10];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  	  strncpy(text_to_display, &vypis[index], 4);
	  	  if(flag == 0){
	  		  index++;
	  	  }
	  	  if(flag == 1){
	  		  index--;
	  	  }

	  	  if(index == sizeof(vypis)-6){
	  		  flag = 1;
	  	  }
	  	  if(index == 0){
	  		  flag = 0;
	  	  }

	  	  //LL_mDelay(500);

	  	  //lsm6ds0_get_acc(acc, (acc+1), (acc+2));
	  	  //LL_mDelay(50);
	  	  //i2c_master_write(0, 0x0F, 0x5F << 1, 0);

	  	  //buffer = *(i2c_master_read(&data, 1, 0xf, 0xbe, 0));

	  	  humidity = HTS221_Get_Humidity();
	  	  temperature = HTS221_Get_Temperature();
	  	  pressure = lps25hb_getPressure();
	  	  altitude = ((powf((1013.25/pressure),1/5.257)-1)*((float)temperature+273.15))/0.0065;
	  	  azimut = lis3mdl_get_mag_z();

	  	  switch(button_state){
	  	  	case 0:
	  	  	memset(mag_vypis, '\0',12);
	  	  	gcvt(azimut, 5, azimut_str);
	  	  	strcat(mag_vypis, " MAG_");
	  	  	strcat(mag_vypis, azimut_str);
	  	  	memset(vypis,'\0',15);
	  	  	strcat(vypis, mag_vypis);
	  		break;

	  	  	case 1:
	  	  	memset(teplota_vypis, '\0',12);
	  	  	sprintf(teplota_str, "%d", temperature);
	  	  	strcat(teplota_vypis, " TEMP_");
	  	  	strncat(teplota_vypis, teplota_str, 2);
	  	  	strcat(teplota_vypis, ".");
	  	  	strncat(teplota_vypis, &teplota_str[2],1);
	  	  	memset(vypis,'\0',15);
	  	  	strcpy(vypis, teplota_vypis);
	  	  	break;

	  	  	case 2:
	  	  	memset(vlhkost_vypis,'\0',12);
	  	  	sprintf(vlhkost_str, "%d", humidity);
	  	  	strcat(vlhkost_vypis, " HUM_");
	  	  	strncat(vlhkost_vypis, vlhkost_str,2);
	  	  	memset(vypis,'\0',15);
	  	  	strcpy(vypis, vlhkost_vypis);
	  	  	break;

	  	  	case 3:
	  	  	memset(tlak_vypis,'\0',15);
	  	  	gcvt(pressure, 6, tlak_str);
	  	  	strcat(tlak_vypis, " BAR_");
	  	  	strcat(tlak_vypis, tlak_str);
	  	  	memset(vypis,'\0',15);
	  	  	strcpy(vypis, tlak_vypis);
	  		break;

	  	  	case 4:
	  	  	memset(vyska_vypis, '\0',12);
	  	  	sprintf(vyska_str, "%d", altitude);
	  	  	strcat(vyska_vypis, " ALT_");
	  	  	strcat(vyska_vypis, vyska_str);
	  	  	memset(vypis,'\0',15);
	  	  	strcpy(vypis, vyska_vypis);
	  	  	break;
	  	  }

	  	  LL_mDelay(200);


  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_Init1msTick(8000000);
  LL_SetSystemCoreClock(8000000);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
}

/* USER CODE BEGIN 4 */
void write_character(unsigned short ch)
{
static char bin[8];
static char inverted_bin[8];

	// write bin number into an array
	for (int i = 0;i < 8; i++)
	{
		bin[i] = ch & 0x80 ? '1' : '0';
		ch <<= 1;
	}

	// invert characters in bin array
	for (int i = 0;i < 8; i++)
	{
		if (bin[i] == '1'){inverted_bin[i] = 0;}
		else if (bin[i] == '0'){inverted_bin[i] = 1;}
	}

	if(inverted_bin[1] == 1 && inverted_bin[2] == 1 && inverted_bin[3] == 1 && inverted_bin[4] == 1 && inverted_bin[5] == 1 && inverted_bin[6] == 0 && inverted_bin[7] == 1){
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_1);

	}else{
		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_1);

		if(inverted_bin[1] == 1){
			LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1);
		}
		if(inverted_bin[1] == 0){
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1);
		}
		//--------------------------------------------------
		if(inverted_bin[2] == 1){
				LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_0);
			}
		if(inverted_bin[2] == 0){
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_0);
		}
		//--------------------------------------------------
		if(inverted_bin[3] == 1){
				LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);
		}
		if(inverted_bin[3] == 0){
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_8);
		}
		//--------------------------------------------------
		if(inverted_bin[4] == 1){
				LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
		}
		if(inverted_bin[4] == 0){
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_5);
		}
		//--------------------------------------------------
		if(inverted_bin[5] == 1){
				LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_11);
		}
		if(inverted_bin[5] == 0){
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_11);
		}
		//--------------------------------------------------
		if(inverted_bin[6] == 1){
				LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);
		}
		if(inverted_bin[6] == 0){
			LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_3);
		}
		//--------------------------------------------------
		if(inverted_bin[7] == 1){
				LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);
		}
		if(inverted_bin[7] == 0){
			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);
		}
	}

	/*
	 HAL_GPIO_WritePin(GPIOA, LL_GPIO_PIN_1, inverted_bin[1]);
	 HAL_GPIO_WritePin(GPIOA, LL_GPIO_PIN_0, inverted_bin[2]);
	 HAL_GPIO_WritePin(GPIOA, LL_GPIO_PIN_8, inverted_bin[3]);
	 HAL_GPIO_WritePin(GPIOB, LL_GPIO_PIN_5, inverted_bin[4]);

	 HAL_GPIO_WritePin(GPIOA, LL_GPIO_PIN_11, inverted_bin[5]);
	 HAL_GPIO_WritePin(GPIOA, LL_GPIO_PIN_3, inverted_bin[6]);
	 HAL_GPIO_WritePin(GPIOB, LL_GPIO_PIN_4, inverted_bin[7]);
	 //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0); // desatinna ciara
	  */
}

void turnON_digit(int seg1,int seg2, int seg3, int seg4)
{
	if (seg1 == 1){
		LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_0);
	}else{
		LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_0);
	}
	if (seg2 == 1){
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);
	}else{
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4);
	}
	if (seg3 == 1){
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_12);
	}else{
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_12);
	}
	if (seg4 == 1){
		LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_2);
	}else{
		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2);
	}
}

void write_4_chars(uint8_t text[]){

	for(uint8_t i = 0; i<4; i++){
		if(i == 0){
			write_character(getCharNumber(text[i]));
			turnON_digit(1,0,0,0);
		}
		if(i == 1){
			write_character(getCharNumber(text[i]));
			turnON_digit(0,1,0,0);
		}
		if(i == 2){
			write_character(getCharNumber(text[i]));
			turnON_digit(0,0,1,0);
		}
		if(i == 3){
			write_character(getCharNumber(text[i]));
			turnON_digit(0,0,0,1);
		}
		resetSegments();
		resetDigits();
	}
}

unsigned short getCharNumber(uint8_t single_char){

	if(single_char == 'A'){
	        return 0b1110111;
	    }
	if(single_char == 'a'){
	        return 0b1111101;
	    }
	if(single_char == 'b'){
	        return 0b0011111;
	    }
	if(single_char == 'C'){
	        return 0b1001110;
	    }
	if(single_char == 'c'){
	        return 0b0001101;
	    }
	if(single_char == 'd'){
	        return 0b0111101;
	    }
	if(single_char == 'E'){
	        return 0b1001111;
	    }
	if(single_char == 'F'){
	        return 0b1000111;
	    }
	if(single_char == 'G'){
	        return 0b1011110;
	    }
	if(single_char == 'H'){
	        return 0b0110111;
	    }
	if(single_char == 'h'){
	        return 0b0010111;
	    }
	if(single_char == 'I'){
	        return 0b0000110;
	    }
	if(single_char == 'J'){
	        return 0b0111100;
	    }
	if(single_char == 'L'){
	        return 0b0001110;
	    }
	if(single_char == 'n'){
	        return 0b0010101;
	    }
	if(single_char == 'O'){
	        return 0b1111110;
	    }
	if(single_char == 'o'){
	        return 0b0011101;
	    }
	if(single_char == 'P'){
	        return 0b1100111;
	    }
	if(single_char == 'q'){
	        return 0b1110011;
	    }
	if(single_char == 'r'){
	        return 0b0000101;
	    }
	if(single_char == 'S'){
	        return 0b1011011;
	    }
	if(single_char == 't'){
	        return 0b0001111;
	    }
	if(single_char == 'U'){
	        return 0b0111110;
	    }
	if(single_char == 'u'){
	        return 0b0011100;
	    }
	if(single_char == 'y'){
	        return 0b0111011;
	    }
	if(single_char == 'K'){
	        return 0b1010111;
	    }
	if(single_char == 'M'){
	        return 0b1101010;
	    }
	if(single_char == 'W'){
	        return 0b0111111;
	    }
	if(single_char == 'X'){
	        return 0b1001001;
	    }
	if(single_char == 'Z'){
	        return 0b1101101;
	    }
	if(single_char == 'V'){
	        return 0b0101010;
	    }
	if(single_char == '1'){
	        return 0b0110000;
	    }
	if(single_char == '2'){
	        return 0b1101101;
	    }
	if(single_char == '3'){
	        return 0b1111001;
	    }
	if(single_char == '4'){
	        return 0b0010011;
	    }
	if(single_char == '5'){
	        return 0b1011011;
	    }
	if(single_char == '6'){
	        return 0b1011111;
	    }
	if(single_char == '7'){
	        return 0b1110000;
	    }
	if(single_char == '8'){
	        return 0b1111111;
	    }
	if(single_char == '9'){
	        return 0b1111011;
	    }
	if(single_char == '0'){
	        return 0b1111110;
	    }
	if(single_char == '_'){
	        return 0b0001000;
	    }
	if(single_char == ' '){
		        return 0b0000000;
		    }
	if(single_char == '.'){
		return 0b0000010;
	}
	if(single_char == '-'){
		return 0b0000001;
	}
}

void resetDigits(){
	turnON_digit(0, 0, 0, 0);
}
void resetSegments(){
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_1);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_0);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_8);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_5);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_11);
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_3);
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);
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

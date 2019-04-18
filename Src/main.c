/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "hx711.h"

/* Global Variables ----------------------------------------------- */
uint32_t debouncer = 0; // initialize debouncer variable for button input
char outputDataChar[30]; // variable for storing integer data to be printed to the serial debug window
HX711 loadCellOneData; // struct for passing setup info for load cell library
int weightValue = 0; // load cell measurement variable

void sendChar(char character)
{
	while(1)
	{
		if(USART3->ISR & USART_ISR_TXE) // if the transmit register is empty, leave the endless while loop
		{
			break;
		}
	}
	USART3->TDR = character; // place the input character into the transmit register
}

void sendStr (char string[])
{
	int i = 0;
	sendChar('\n'); // new line
	sendChar('\r'); // carriage return
	for(i = 0; i <= 1000; i++) // loop over each char in the char[]
	{
		if(string[i] == 0) // test for null character at end of char[], and leave if found
		{
			break;
		}
		else
		{
			sendChar(string[i]); // transmit the current character
		}
	}
}

void sendStrNoNewline (char string[])
{
	int i = 0;
	//sendChar('\t'); // tab over to show second value
	for(i = 0; i <= 1000; i++) // loop over each char in the char[]
	{
		if(string[i] == 0) // test for null character at end of char[], and leave if found
		{
			break;
		}
		else
		{
			sendChar(string[i]); // transmit the current character
		}
	}
}

void sendInt(int input)
{
	sprintf(outputDataChar,"%d",input);
	sendStr(outputDataChar);
}

void sendIntNoNewLine(int input)
{
	sprintf(outputDataChar,"%d",input);
	sendStrNoNewline(outputDataChar);
}

void setupSerialDebug()
{
	//USART Setup for debugging
	//Setup PC4 (USART3_TX) and PC5 (USART3_RX)
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // enable peripheral clock for GPIOC
	GPIOC->MODER |= (1 << 9) | (1 << 11);	// Set pins PC4 (USART3_TX) and PC5 (USART3_RX) into alternate function mode
	GPIOC->AFR[0] |= (1 << 16) | (1 << 20);	// Program alternate function 1 for PC4 and PC5; AFR[1] is  AFRH; AFR[0] is AFRL
	
	// USART3 Setup
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN; // Enable RCC peripheral clock for USART3
	USART3->BRR = HAL_RCC_GetHCLKFreq()/115200; // Set baud rate for USART3
	//USART3->BRR = 71; // Set baud frate for USART3 using divider of 71; 8M/71 ~= 112500
	USART3->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE; // Enable USART transmitter, receiver and entire peripheral;
}



void testButtonSetup(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // enable peripheral clock for GPIOA
	GPIOA->MODER 		&= ~((1 >> 0) | (1 >> 1)); // set digital input mode for user button
	GPIOA->OSPEEDR	|= ~((1 >> 0) | (1 >> 1)); // set low speed for user button
	GPIOA->PUPDR 		|= (1 << 1); // set pull-down resistor for user button
}

void motorPinSetup(void)
{
	// setup pin PC1 to enable; PC2 to IN1; PC3 to IN2
	
	// Enable clock for GPIOC
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // enable peripheral clock for GPIOC
	
	// Clear GPIOC MODER for enable, 
	GPIOC->MODER &= 0xFFFFFF03;
	
	// Enable pin setup
	GPIOC->MODER 		|= (1 << GPIO_MODER_MODER13_Pos); // set output mode for enable pin output
	GPIOC->OTYPER		&= (1 << 13); // set push-pull output type
	GPIOC->OSPEEDR	&= ~(3 << GPIO_OSPEEDR_OSPEEDR13_Pos); // set low speed for enable pin output
	GPIOC->PUPDR 		|= (1 << GPIO_PUPDR_PUPDR13_Pos); // set pull-down resistor for enable pin output
	GPIOC->BRR		|= (1 << 13); // initially set h bridge enable pin low; 

	// IN1 pin setup
	GPIOC->MODER 		|= (1 << GPIO_MODER_MODER14_Pos); // set output mode for h bridge input 1 out (STM32 output)
	GPIOC->OTYPER		&= (1 << 14); // set push-pull output type
	GPIOC->OSPEEDR	&= ~(3 << GPIO_OSPEEDR_OSPEEDR14_Pos); // set low speed for h bridge input 1 out (STM32 output)
	GPIOC->PUPDR 		|= (1 << GPIO_PUPDR_PUPDR14_Pos); // set pull-down resistor for h bridge input 1 out (STM32 output)
	GPIOC->BSRR		|= (1 << 14); // initially set h bridge input 1 pin high;

	// IN2 pin setup
	GPIOC->MODER 		|= (1 << GPIO_MODER_MODER15_Pos); // set output mode for h bridge input 2 out (STM32 output)
	GPIOC->OTYPER		&= (1 << 15); // set push-pull output type
	GPIOC->OSPEEDR	&= ~(1 << GPIO_OSPEEDR_OSPEEDR15_Pos); // set low speed for h bridge input 2 out (STM32 output)
	GPIOC->PUPDR 		|= (1 << GPIO_PUPDR_PUPDR15_Pos); // set pull-down resistor for h bridge input 2 out (STM32 output)
	GPIOC->BRR		|= (1 << 15); // initially set h bridge input 2 pin low;
}

void enablePinHigh(void)
{
	GPIOC->BSRR		|= (1 << 13); // set h bridge enable pin high;
	//sendStr("Enable pin set high. ODR value for pin 1:");
	//sendInt(GPIOC->ODR & (1 << 13));
}

void enablePinLow(void)
{
	GPIOC->BRR		|= (1 << 13); // set h bridge enable pin low;
	//sendStr("Enable pin set low. ODR value for pin 1:");
	//sendInt(GPIOC->ODR & (1 << 1));
}

void setMotorDirectionForward(void)
{
	GPIOC->BSRR		|= (1 << 14); // set h bridge input 1 pin high; 
	GPIOC->BRR		|= (1 << 15); // set h bridge input 2 pin low;
	//sendStr("Motor set to forward. ODR value for pin 2:");
	//sendInt(GPIOC->ODR & (1 << 14));
	//sendStr("Motor set to forward. ODR value for pin 3:");
	//sendInt(GPIOC->ODR & (1 << 15));
}

void setMotorDirectionBackward(void)
{
	GPIOC->BRR		|= (1 << 14); // set h bridge input 1 pin low; 
	GPIOC->BSRR		|= (1 << 15); // set h bridge input 2 pin high; 
	//sendStr("Motor set to backward. ODR value for pin 2:");
	//sendInt(GPIOC->ODR & (1 << 14));
	//sendStr("Motor set to backward. ODR value for pin 3:");
	//sendInt(GPIOC->ODR & (1 << 15));
}

void motorForward(void)
{
	setMotorDirectionForward();
	enablePinHigh();
}

void motorBackward(void)
{
	setMotorDirectionBackward();
	enablePinHigh();
}

void motorStop(void)
{
	enablePinLow();
}

void motorButton(void)
{
	debouncer = (debouncer << 1); // Always shift every loop iteration
	
	if (GPIOA->IDR & 0x00000001) // If input signal is set/high
	{ 
		debouncer |= 0x01; // Set lowest bit of bit-vector
	}
	if (debouncer == 0xFFFFFFFF) { // If high, do this
		motorForward();
	}
	
	if (debouncer == 0x00000000) { // If low, do this
		motorStop();
	}
}

void SystemClock_Config(void);
/**
  * @brief  The application entry point.
  * @retval int
  */

void loadCellSetup(void)
{
	loadCellOneData.gpioSck = GPIOA; // use GPIOA for clock
	loadCellOneData.gpioData = GPIOA; // use GPIOA for data
	loadCellOneData.pinSck = GPIO_PIN_1; // use pin 1 for clock
	loadCellOneData.pinData = GPIO_PIN_2; // use pin 2 for data
	loadCellOneData.offset = 0; // set offset
	loadCellOneData.gain = 1; // set gain
	
	HX711_Init(loadCellOneData);
};

void motorTest(void)
{
	sendStr("Running motor test...");
	motorForward();
	HAL_Delay(1000);
	motorStop();
	HAL_Delay(1000);
	motorBackward();
	HAL_Delay(1000);
	motorStop();
	sendStr("Motor test complete.");
}


void spam(void)
{
	for (int i = 0; i < 25; i++)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_Delay(1);
	}
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	setupSerialDebug(); // setup serial debugging interface via UART
	testButtonSetup(); // setup for test button input
	motorPinSetup();
	loadCellSetup();
	
	motorTest();
	
			
	//sendStr("Running weight test...");
	//sendStr("Weight measured is:");
	//weightValue = HX711_Value(loadCellOneData);
	//sendStr("Weight test complete.");
	
  while (1)
  {
		//motorButton();
		weightValue = HX711_Value(loadCellOneData);
		sendInt(weightValue);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

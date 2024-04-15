/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd16x2.h"
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
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//uint16_t temps_precedent;
//uint16_t temps_actuel;

int nombrePeriodeCompte = 0;
// permet de garder le compt des periodes et savoir quand on atteind nombrePeriodesMax

int nombrePeriodesMax = 3; // on compte sur 3 periodes

int pertPourAfficher = 0; // 0 si on prend des valeurs,
// 1 si on affiche, permet de ne pas changer des variables utlisée dans le main pendant qu'on les utilisent

double coef = 0.015; // coefficient egale a c/(2*f)

// conteur pour les temps des periodes
// timer : 1tic = 1 microsec 10^-6
uint16_t tempsPeriodes;
uint16_t tempsDernierePeriode;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{ // on vérifie que l'interuption vient bien de la bonne entrée
    if (GPIO_Pin == ENTREE_Pin){

    	// on vérifie que les valeurs ne sont pas prettent à être affichée.
    	if (pertPourAfficher == 0){

    		// on recupère la valeur du timer
    		tempsDernierePeriode = __HAL_TIM_GET_COUNTER(&htim16);

    		// on remet le timer a 0 pour la prochaine prise de valeur
    	    __HAL_TIM_SET_COUNTER(&htim16, 0);

    	    // si on a pris la 1er valeur, elle n'est pas cohérente, on ne la garde pas
    	    if (nombrePeriodeCompte == -1){
    	       	nombrePeriodeCompte ++;
    	    }
    	    else{// on recupère la valeur
    	    	nombrePeriodeCompte ++; // on a une période de plus de comptée
    	    	tempsPeriodes += tempsDernierePeriode; // le temps de toutes les périodes est incrémenté

    	    	// si on a compté nombrePeriodesMax période on envoie le signal pour afficher
    	    	// et on arrête de compter
    	    	if (nombrePeriodeCompte >= nombrePeriodesMax){
    	    		nombrePeriodeCompte = -1;
    	    		pertPourAfficher = 1;
    	    	}
    	    }
    	}
    }
}




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
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start(&htim16);

  // initialisation de l'afficheur à LED
  lcd16x2_init_4bits(LCD_RS_GPIO_Port, LCD_RS_Pin, LCD_E_Pin,
  		  	  LCD_D4_GPIO_Port, LCD_D4_Pin, LCD_D5_Pin, LCD_D6_Pin, LCD_D7_Pin);


  lcd16x2_cursorShow(0);
  lcd16x2_clear();
  lcd16x2_setCursor(0,0);
  lcd16x2_printf("Radar Doppler");
  lcd16x2_setCursor(1, 0);
  lcd16x2_printf("Group - B");
  HAL_Delay(1000);
  lcd16x2_clear();

  double vitessePrecedente = 0;
  int nombrePasse = 0;
  double vitessekmh;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // si on doit afficher
	  if (pertPourAfficher == 1){
		  // calcule de la vitesse

		  // de face
		  //double vitesse = (double)100000. * coef * nombrePeriodesMax / tempsPeriodes;

		  // angle de 30°
		  double vitesse = (double)100000. * coef * 0.866 * nombrePeriodesMax / tempsPeriodes;

		  // angle de 45°
		  //double vitesse = (double)100000. * coef * 0.71 * nombrePeriodesMax / tempsPeriodes;


		  // on relance l'aquisition
		  tempsPeriodes = 0;
		  nombrePeriodeCompte = -1;
		  pertPourAfficher = 0;
		  // on verifie que la vitesse n'est pas trop basse
		  if ((vitesse >= 1.)&&(vitesse >= vitessePrecedente)){
			  vitessekmh = vitesse * 3.6;
			  // on affiche la vitesse
			  lcd16x2_clear();
			  lcd16x2_setCursor(0,0);
			  lcd16x2_printf("%.2f km.h^-1", (float)vitessekmh);
			  lcd16x2_setCursor(1, 0);
			  lcd16x2_printf("%.2f m.s^-1", (float)vitesse);

			  vitessePrecedente = vitesse;
		  }
		  else {
			  nombrePasse ++;
			  if (nombrePasse >= 1){
				  vitessePrecedente = 0;
				  nombrePasse =0;
			  }
		  }
		  // on attend 500 millisecondes pour eviter que l'ecrant clignotte trop
		  HAL_Delay(500);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 80-1;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RS_Pin|LCD_E_Pin|LCD_D4_Pin|LCD_D5_Pin
                          |LCD_D6_Pin|LCD_D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LCD_RS_Pin LCD_E_Pin LCD_D4_Pin LCD_D5_Pin
                           LCD_D6_Pin LCD_D7_Pin */
  GPIO_InitStruct.Pin = LCD_RS_Pin|LCD_E_Pin|LCD_D4_Pin|LCD_D5_Pin
                          |LCD_D6_Pin|LCD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ENTREE_Pin */
  GPIO_InitStruct.Pin = ENTREE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENTREE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
  __disable_irq();
  while (1)
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

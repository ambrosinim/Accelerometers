/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "crc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "stm32f4xx.h"
#include<string.h>
#include "stm32f4_discovery_lis3dsh.h"

#include "ai_datatypes_defines.h"
#include "ai_platform.h"
#include "network.h"
#include "network_data.h"

#include "network_normalized.h"
#include "network_normalized_data.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEST_LENGTH_SAMPLES  25 //==SIZE
#define SNR_THRESHOLD_F32    140.0f
#define BLOCK_SIZE            25
#define NUM_TAPS              25
#define MAX_BLOCKSIZE     	25
#define DELTA           (0.000001f)

#define CIRCLE 100
#define TICK   001
#define NONE   010
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void Acc_Config(void);

uint8_t chRX = 0;
uint8_t dataReceived = 0;
uint8_t streamActive = 1;
uint8_t dataReady = 0;

volatile uint16_t ADC_Value;
extern uint32_t readyToSend;
extern uint8_t bufferRX[BLOCK_SIZE];
extern uint8_t bufferTX[BLOCK_SIZE];
extern uint8_t idxRX;
extern uint8_t idxTX;
extern uint8_t nBytesTX;


uint16_t idx = 0;
int16_t accData[SIZE][3][1];

/*** To make the samples coming from the accelerometers compatible with the ARM function "arm_fir_f32",
 *** we separate and transpose the columns of the matrix accData  ***/

float accDataTransposeX[SIZE];
float accDataTransposeY[SIZE];
float accDataTransposeZ[SIZE];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int my_circle[][3][1] = { //100 expected
	{114,     -61,      974},
    {108,     -68,      936},
    { -2,     -27,      924},
    { 81,    -103,     1226},
    { 922,    139,      805},
    { 552,    428,      624},
    { 441,     443,     422},
    {  95,    454,      573},
    {-544,     87,      786},
    {-405,    -99,     1361},
    {-102,   -205,     1179},
    { 247,   -150,     1129},
    { 295,     47,      831},
    {  93,      3,      942},
    {  75,     11,     1004},
    {  69,     10,      945},
    {  76,     10,      960},
    {  59,     13,      960},
    { 122,     20,      973},
    {  97,      4,      993},
    {  13,    -45,      960},
    { 116,     98,     1021},
    {  71,     61,      888},
    {  88,    157,      937},
    {  39,     62,     1037}};

int my_tick[][3][1] = { // expected
    { 20,      375,     912},
    { -19,     361,     798},
    { 98,      284,      68},
    { 408,    -111,    1402},
    { -45,    -115,    1352},
    { 57,     -151,    1950},
    { 120,    -151,     952},
    { -122,   -210,     297},
    { -951,    146,     413},
    { -353,    306,     741},
    { -345,    320,     779},
    { -379,    298,     826},
    { -432,    287,     813},
    { -396,    286,     825},
    { -417,    293,     821},
    { -415,    289,     866},
    { -366,    309,     793},
    { -361,    284,     813},
    { -399,    308,     874},
    { -380,    289,     789},
    { -406,    291,     842},
    { -382,    291,     845},
    { -379,    309,     809},
    { -417,    288,     843},
    { -405,    284,     798}};

float my_circle_normalized[][3][1] = { //100 expected
	{0.114,     -0.061,      0.974},
    {0.108,     -0.068,      0.936},
    { -0.02,     -0.027,      0.924},
    { 0.081,    -0.103,     0.926},
    { 0.922,    0.139,      0.805},
    { 0.552,    0.428,      0.624},
    { 0.441,     0.443,     0.422},
    {  0.095,    0.454,      0.573},
    {-0.544,     0.087,      0.786},
    {-0.405,    -0.099,     0.961},
    {-0.102,   -0.205,     0.979},
    { 0.247,   -0.150,     0.929},
    { 0.295,     0.047,      0.831},
    {  0.093,      0.003,      0.942},
    {  0.075,     0.011,     0.904},
    {  0.069,     0.010,      0.945},
    {  0.076,     0.010,      0.960},
    {  0.059,     0.013,      0.960},
    { 0.122,     0.020,      0.973},
    {  0.097,      0.004,      0.993},
    {  0.013,    -0.045,      0.960},
    { 0.116,     0.098,     0.921},
    {  0.071,     0.061,      0.888},
    {  0.088,    0.157,      0.937},
    {  0.039,     0.062,     0.937}};





float my_circle_normalized_1[][3][1] = { //100 expected

		{0.05834186,-0.03121801,0.49846469},

		{0.05527124,-0.03480041,0.4790174},

		{-0.00102354,-0.01381781,0.47287615},

		{0.04145343,-0.05271238,0.62743091},

		{0.47185261,0.07113613,0.41197544},

		{0.28249744,0.21903787,0.31934493},

		{0.22569089,0.22671443,0.21596725},

		{0.04861822,0.23234391,0.29324463},

		{-0.27840328,0.04452405,0.40225179},

		{-0.20726714,-0.0506653 ,0.69651996},

		{-0.05220061,-0.104913  ,0.60337769},

		{0.12640737,-0.07676561,0.57778915},

		{0.15097236,0.02405322,0.42528147},

		{0.04759468,0.00153531,0.48208802},

		{0.0383828 ,0.00562948,0.51381781},

		{0.03531218,0.00511771,0.48362334},

		{0.03889458,0.00511771,0.4912999} ,

		{0.03019447,0.00665302,0.4912999 },

		{0.06243603,0.01023541,0.49795292},

		{0.04964176,0.00204708,0.50818833},

		{0.00665302,-0.02302968,0.4912999 },

		{0.0593654 ,0.05015353,0.52251791},

		{0.03633572,0.03121801,0.45445241},

		{0.04503582,0.080348,0.47952917},

		{0.01995906,0.03172979,0.53070624}};

float my_circle_normalized_2[][3][1] = {
	{0.123,-0.335,0.886},
	{0.123,-0.335,0.886},
	{0.119,-0.374,0.884},
	{0.095,-0.335,0.914},
	{0.212,-0.346,0.851},
	{0.167,-0.352,0.941},
	{0.284,-0.315,0.980},
	{0.551,-0.370,0.835},
	{0.716,-0.222,0.922},
	{0.796,0.079,0.831},
	{0.372,0.307,0.940},
	{-0.183,0.405,0.317},
	{0.043,0.155,0.608},
	{-0.350,-0.074,1.00},
	{-0.237,-0.509,1.269},
	{0.040,-0.458,1.249},
	{0.453,-0.284,0.855},
	{0.318,-0.249,0.874},
	{0.207,-0.195,0.796},
	{0.039,-0.248,0.942},
	{0.203,-0.253,0.949},
	{0.027,-0.281,0.921},
	{0.165,-0.254,0.915},
	{0.142,-0.266,0.889},
	{0.180,-0.298,0.913}};


float my_tick_normalized[][3][1] = { // expected
    { 0.020,      0.375,     0.912},
    { -0.019,     0.361,     0.798},
    { 0.098,      0.284,      0.068},
    { 0.408,    -0.111,    0.902},
    { -0.045,    -0.115,    0.952},
    { 0.057,     -0.151,    0.950},
    { 0.120,    -0.151,     0.952},
    { -0.122,   -0.210,     0.297},
    { -0.951,    0.146,     0.413},
    { -0.353,    0.306,     0.741},
    { -0.345,    0.320,     0.779},
    { -0.379,    0.298,     0.826},
    { -0.432,    0.287,     0.813},
    { -0.396,    0.286,     0.825},
    { -0.417,    0.293,     0.821},
    { -0.415,    0.289,     0.866},
    { -0.366,    0.309,     0.793},
    { -0.361,    0.284,     0.813},
    { -0.399,    0.308,     0.874},
    { -0.380,    0.289,     0.789},
    { -0.406,    0.291,     0.842},
    { -0.382,    0.291,     0.845},
    { -0.379,    0.309,     0.809},
    { -0.417,    0.288,     0.843},
    { -0.405,    0.284,     0.798}};


float my_tick_normalized_1[][3][1] = // expected

		{{-0.00716479, 0.29682702,0.37922211},

		{-0.0209826 ,0.290174  ,0.43039918},

		{-0.05834186,0.2988741,0.41351075},

		{-0.0383828,0.30603889,0.38485159},

		{-0.03070624,0.30962129,0.39252815},

		{-0.04554759,0.28915046,0.30040942},

		{0.01125896,0.18423746,0.09825998},

		{0.16888434,0.04247697,0.47952917},

		{-0.06090072,-0.15455476,0.50102354},

		{-0.06345957,-0.18372569,0.70573183},

		{-0.0419652,-0.14636643,0.41504606},

		{0.01842375,-0.1345957 , 0.64380757},

		{-0.1468782 ,-0.16018424,0.48055271},

		{-0.29324463,0.02047083,0.25383828},

		{-0.20829069,0.10184237,0.32753327},

		{-0.22876151,0.16888434,0.42579324},

		{-0.19549642,0.21340839,0.29119754},

		{-0.26100307,0.2185261,0.314739},

		{-0.24872057,0.22978506,0.38741044},

		{-0.2221085,0.22927329,0.35823951},

		{-0.23336745,0.23899693,0.36796315},

		{-0.24616172,0.22773797,0.38587513},

		{-0.23183214,0.23592631,0.34288639},

		{-0.23490276,0.22569089,0.3828045},

		{-0.22876151,0.23592631,0.35056295}};


float my_none_normalized_1[][3][1] = {
	{-0.030,-0.048,0.917},
	{-0.009,-0.028,0.984},
	{-0.022,-0.046,0.972},
	{-0.007,-0.044,0.934},
	{-0.034,-0.058,0.974},
	{-0.018,-0.056,0.960},
	{-0.005,-0.055,0.967},
	{-0.021,-0.068,0.946},
	{0.017,-0.055,0.980},
	{-0.001,-0.078,0.956},
	{0.022,-0.068,0.990},
	{-0.01,-0.055,0.973},
	{-0.011,-0.062,0.979},
	{-0.004,-0.064,0.970},
	{0.014,-0.064,0.950},
	{-0.021,-0.059,0.983},
	{-0.07,-0.059,0.967},
	{-0.04,-0.060,0.987},
	{-0.08,-0.041,0.972},
	{-0.020,-0.077,0.988},
	{-0.015,-0.055,0.973},
	{0.011,-0.057,0.967},
	{0.053,-0.078,0.970},
	{0.001,-0.073,0.973},
	{0.034,-0.056,0.933}};



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char *classes[]= {'c','t','n'};


volatile uint16_t ADC_Value;
extern uint32_t readyToSend;
//extern uint8_t bufferRX[RX_BUFFER_SIZE];
//extern uint8_t bufferTX[TX_BUFFER_SIZE];
extern uint8_t idxRX;
extern uint8_t idxTX;
extern uint8_t nBytesTX;

void Acc_Config(void);


/*** To make the samples coming from the accelerometers compatible with the ARM function "arm_fir_f32",
 *** we separate and transpose the columns of the matrix accData  ***/

/* ----------------------------------------------------------------------
* Declare Global variables
* ------------------------------------------------------------------- */

uint32_t blockSize = 25;


/******** Cycle counter defines  **********/
volatile unsigned int cyc[2];
volatile unsigned int *DWT_CYCCNT = (volatile unsigned int *)0xE0001004; 	// Cycle counter
volatile unsigned int *DWT_CONTROL= (volatile unsigned int *)0xE0001000;	// counter control
volatile unsigned int *SCB_DEMCR  = (volatile unsigned int *)0xE000EDFC;
#define STOPWATCH_START {cyc[0]=*DWT_CYCCNT;} 								// start counting
#define STOPWATCH_STOP  {cyc[1]=*DWT_CYCCNT; cyc[1]=cyc[1]-cyc[0];}			// stop counting, result is in cyc[1]

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	// init, reset and start the cycle counter
		*SCB_DEMCR = *SCB_DEMCR | 0x01000000;
		*DWT_CYCCNT = 0; 							// reset the counter
		*DWT_CONTROL = *DWT_CONTROL | 1 ; 			// enable the counter



		u_int8_t k;
		char buf[50]; //	just 4 strings
		  int buf_len = 0;
		  ai_error ai_err;
		  ai_i32 nbatch;
		  uint32_t timestamp;
		  float *y_val;

		  // Chunk of memory used to hold intermediate values for neural network
		  AI_ALIGNED(4) ai_u8 activations[AI_NETWORK_NORMALIZED_DATA_ACTIVATIONS_SIZE];

		  // Buffers used to store input and output tensors
		  AI_ALIGNED(4) ai_float in_data[AI_NETWORK_NORMALIZED_IN_1_SIZE_BYTES];
		  AI_ALIGNED(4) ai_float out_data[AI_NETWORK_NORMALIZED_OUT_1_SIZE_BYTES];
		  ////////////////////////////////AI_ALIGNED(4) ai_i8 my_samples =
		  // Pointer to our model
		  ai_handle network_normalized = AI_HANDLE_NULL;

		  // Initialize wrapper structs that hold pointers to data and info about the
		  // data (tensor height, width, channels)
		  ai_buffer ai_input[AI_NETWORK_NORMALIZED_IN_NUM] = AI_NETWORK_NORMALIZED_IN;
		  ai_buffer ai_output[AI_NETWORK_NORMALIZED_OUT_NUM] = AI_NETWORK_NORMALIZED_OUT;

		  // Set working memory and get weights/biases from model
		  /*ai_network_params ai_params1 = {
		    AI_NETWORK_DATA_WEIGHTS(ai_myacc_model_data_weights_get()),
		    AI_NETWORK_DATA_ACTIVATIONS(activations)
		  };*/
		  ai_network_params ai_params = AI_NETWORK_PARAMS_INIT(AI_NETWORK_NORMALIZED_DATA_WEIGHTS(ai_network_normalized_data_weights_get()),
				   AI_NETWORK_NORMALIZED_DATA_ACTIVATIONS(activations));
		  // Set pointers wrapper structs to our data buffers
		  ai_input[0].n_batches = 1;
		  ai_input[0].data = AI_HANDLE_PTR(in_data);
		  ai_output[0].n_batches = 1;
		  ai_output[0].data = AI_HANDLE_PTR(out_data);
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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_CRC_Init();

  //LL_SPI_Enable(LIS3DSH_SPI);
    Acc_Config();

    // Start timer/counter
    HAL_TIM_Base_Start(&htim3);

    // Greetings!
    buf_len = sprintf(buf, "\r\n\r\nSTM32 X-Cube-AI test\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);

    // Create instance of neural network
    ai_err = ai_network_normalized_create(&network_normalized, AI_NETWORK_NORMALIZED_DATA_CONFIG);
    if (ai_err.type != AI_ERROR_NONE)
    {
      buf_len = sprintf(buf, "Error: could not create NN instance\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);
      while(1);
    }

    // Initialize neural network
    if (!ai_network_normalized_init(network_normalized, &ai_params))
    {
      buf_len = sprintf(buf, "Error: could not initialize NN\r\n");
      HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);
      while(1);
    }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  buf_len = sprintf(buf, "Done: 25 samples Acc\r\n");
	  HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);

	  if (dataReady) {

		  LIS3DSH_ReadACC(accData[idx++]);

		  if (idx == SIZE)
		  {
	  	        buf_len = sprintf(buf, "Done: 25 samples %d\r\n", accData[idx-1][0][0]);
	  	        HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);
			  idx = 0;
			  }
			  dataReady = 0;
		  }
	  /* Fill input buffer (use test value) */
	  for (uint32_t i = 0; i < AI_NETWORK_NORMALIZED_IN_1_HEIGHT; i++)
	  {
		  for (uint32_t j = 0; j < AI_NETWORK_NORMALIZED_IN_1_WIDTH; j++)  {
			  ((ai_float*)in_data)[(i+j)] = ((ai_float)my_tick_normalized[i][j][0]); //my_samples
		  }
	  }


  	      // Get current timestamp
  	      timestamp = htim3.Instance->CNT;
  	      // Perform inference
  	      nbatch = ai_network_normalized_run(network_normalized, &ai_input[0], &ai_output[0]);
  	      if (nbatch != 1) {
  	        buf_len = sprintf(buf, "Error: could not run inference\r\n");
  	        HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);
  	      }

  	      // Read output (predicted y) of neural network
  	    	  y_val = ((float *)out_data);

  		      k = 0;
  		      float max = y_val[k];
  			  if (y_val[1] > max){
  				  max = (float)y_val[1];
  				  k = 1;
  			  }
  			  if (y_val[2] > max){
  				  max = (int)y_val[2];
  				  k = 2;
  			  }


  	      // Print output of neural network along with inference time (microseconds)
  	      buf_len = sprintf(buf,
  	                        "Output: %f, %f, %f CLASS: %c\r\n",
  	                        y_val[0],y_val[1],y_val[2],
  	                        classes[k]);   //| Duration: %lu\r  htim3.Instance->CNT - timestamp,


  	      HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);



  	      buf_len = sprintf(buf,
  	                        "input: %f, %f, %f \r\n",
							(ai_float)my_tick_normalized[0][0][0],(ai_float)my_circle_normalized[0][1][0],(ai_float)my_circle_normalized[0][2][0]);   //| Duration: %lu\r  htim3.Instance->CNT - timestamp,


  	      HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);

  	      buf_len = sprintf(buf,
  	                        "AccData: %d, %d, %d CLASS: %c\r\n",
							accData[0][0][0],accData[0][1][0],accData[0][2][0],
  	                        classes[k]);   //| Duration: %lu\r  htim3.Instance->CNT - timestamp,


  	      HAL_UART_Transmit(&huart2, (uint8_t *)buf, buf_len, 100);

  	      // Wait before doing it again
  	      HAL_Delay(500);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Acc_Config(void)
{
	LIS3DSH_InitTypeDef AccInitStruct;

	AccInitStruct.Output_DataRate = LIS3DSH_DATARATE_12_5 ; // 10Hz SAMPLING
	AccInitStruct.Axes_Enable = LIS3DSH_XYZ_ENABLE;
	AccInitStruct.SPI_Wire = LIS3DSH_SERIALINTERFACE_4WIRE;
	AccInitStruct.Self_Test = LIS3DSH_SELFTEST_NORMAL;
	AccInitStruct.Full_Scale = LIS3DSH_FULLSCALE_2;
	AccInitStruct.Filter_BW = LIS3DSH_FILTER_BW_800;

	LIS3DSH_Init(&AccInitStruct);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

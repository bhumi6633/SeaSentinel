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
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RX_BUFFER_SIZE 50       // UART receive buffer size
#define HOURLY_BUFFER_SIZE 168   // 168 hourly readings for one week
#define WEEKLY_BUFFER_SIZE 52    // 52 weekly averages for a year

uint8_t rxBuffer[RX_BUFFER_SIZE];  // UART receive buffer
uint16_t rxIndex = 0;              // Index for UART receive buffer

// Circular buffers
float hourlyBuffer[HOURLY_BUFFER_SIZE];   // Stores 168 hourly readings
uint16_t hourlyIndex = 0;                 // Index for hourly buffer
uint16_t hourlyCount = 0;                 // Tracks the number of hourly readings

float weeklyBuffer[WEEKLY_BUFFER_SIZE];   // Stores 52 weekly averages
uint16_t weeklyIndex = 0;                 // Index for weekly buffer

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
const int DIGIT_REPS[10][8][3] = {
    {{0, 0, 0}, {1, 1, 1}, {1, 0, 1}, {1, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 0, 0}, {0, 0, 0}}, // 0
    {{0, 0, 0}, {0, 1, 0}, {1, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 0, 0}, {0, 0, 0}}, // 1
    {{0, 0, 0}, {1, 1, 1}, {0, 0, 1}, {1, 1, 1}, {1, 0, 0}, {1, 1, 1}, {0, 0, 0}, {0, 0, 0}}, // 2
    {{0, 0, 0}, {1, 1, 1}, {0, 0, 1}, {1, 1, 1}, {0, 0, 1}, {1, 1, 1}, {0, 0, 0}, {0, 0, 0}}, // 3
    {{0, 0, 0}, {1, 0, 1}, {1, 0, 1}, {1, 1, 1}, {0, 0, 1}, {0, 0, 1}, {0, 0, 0}, {0, 0, 0}}, // 4
    {{0, 0, 0}, {1, 1, 1}, {1, 0, 0}, {1, 1, 1}, {0, 0, 1}, {1, 1, 1}, {0, 0, 0}, {0, 0, 0}}, // 5
    {{0, 0, 0}, {1, 1, 1}, {1, 0, 0}, {1, 1, 1}, {1, 0, 1}, {1, 1, 1}, {0, 0, 0}, {0, 0, 0}}, // 6
    {{0, 0, 0}, {1, 1, 1}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {0, 0, 0}, {0, 0, 0}}, // 7
    {{0, 0, 0}, {1, 1, 1}, {1, 0, 1}, {1, 1, 1}, {1, 0, 1}, {1, 1, 1}, {0, 0, 0}, {0, 0, 0}}, // 8
    {{0, 0, 0}, {1, 1, 1}, {1, 0, 1}, {1, 1, 1}, {0, 0, 1}, {1, 1, 1}, {0, 0, 0}, {0, 0, 0}}  // 9
};

const int CHAR_C[8][3] = {{0, 0, 0}, {0, 1, 1}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {0, 1, 1}, {0, 0, 0}, {0, 0, 0}};
const int CHAR_K[8][3] = {{0, 0, 0}, {1, 0, 1}, {1, 1, 0}, {1, 0, 0}, {1, 1, 0}, {1, 0, 1}, {0, 0, 0}, {0, 0, 0}};
const int CHAR_F[8][3] = {{0, 0, 0}, {1, 1, 1}, {1, 0, 0}, {1, 1, 1}, {1, 0, 0}, {1, 0, 0}, {0, 0, 0}, {0, 0, 0}};
const int CHAR_DOT[8][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {1, 0}, {0, 0}, {0, 0}};
const int CHAR_STAR[8][2] = {{0, 0}, {1, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
const int CHAR_PLUS[8][3] = {{0, 0, 0}, {0, 0, 0}, {0, 1, 0}, {1, 1, 1}, {0, 1, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
const int CHAR_MINUS[8][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {1, 1, 1}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Column pin definitions
#define C1_PIN GPIO_PIN_6  // d10
#define C1_GPIO_Port GPIOB
#define C2_PIN GPIO_PIN_7  // d9
#define C2_GPIO_Port GPIOC
#define C3_PIN GPIO_PIN_9   // d8
#define C3_GPIO_Port GPIOA
#define C4_PIN GPIO_PIN_8   // d7
#define C4_GPIO_Port GPIOA
#define C5_PIN GPIO_PIN_10   // d6
#define C5_GPIO_Port GPIOB
#define C6_PIN GPIO_PIN_4   // d5
#define C6_GPIO_Port GPIOB
#define C7_PIN GPIO_PIN_5   // d4
#define C7_GPIO_Port GPIOB
#define C8_PIN GPIO_PIN_3   // d3
#define C8_GPIO_Port GPIOB

// Row pin definitions
#define R8_PIN GPIO_PIN_10   // d2
#define R8_GPIO_Port GPIOA
#define R7_PIN GPIO_PIN_8   // d15
#define R7_GPIO_Port GPIOB
#define R6_PIN GPIO_PIN_0   // a0
#define R6_GPIO_Port GPIOA
#define R5_PIN GPIO_PIN_1   // a1
#define R5_GPIO_Port GPIOA
#define R4_PIN GPIO_PIN_4   // a2
#define R4_GPIO_Port GPIOA
#define R3_PIN GPIO_PIN_0   // a3
#define R3_GPIO_Port GPIOB
#define R2_PIN GPIO_PIN_1   // a4
#define R2_GPIO_Port GPIOC
#define R1_PIN GPIO_PIN_0   // a5
#define R1_GPIO_Port GPIOC

// Mode definitions
typedef enum {
    MODE_CURRENT_CELSIUS = 0,
    MODE_CURRENT_FAHRENHEIT,
    MODE_CURRENT_KELVIN,
    MODE_WEEKLY_CELSIUS,
    MODE_WEEKLY_FAHRENHEIT,
    MODE_WEEKLY_KELVIN,
    MODE_YEARLY_CELSIUS,
    MODE_YEARLY_FAHRENHEIT,
    MODE_YEARLY_KELVIN
} Mode_t;

// Initialize the modes array correctly
Mode_t modes[9] = {
    MODE_CURRENT_CELSIUS,
    MODE_CURRENT_FAHRENHEIT,
    MODE_CURRENT_KELVIN,
    MODE_WEEKLY_CELSIUS,
    MODE_WEEKLY_FAHRENHEIT,
    MODE_WEEKLY_KELVIN,
    MODE_YEARLY_CELSIUS,
    MODE_YEARLY_FAHRENHEIT,
    MODE_YEARLY_KELVIN
};

// Set the initial current mode
Mode_t currentMode = MODE_CURRENT_CELSIUS;
//Mode_t currentMode = MODE_CURRENT_FAHRENHEIT;
//Mode_t currentMode = MODE_CURRENT_KELVIN;


// Temperature variable
//float currentTempCelsius = 0.0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

// Temperature conversion functions
float celsiusToFahrenheit(float celsius);
float celsiusToKelvin(float celsius);

// Circular buffer management
void addHourlyTemperature(float temperature);
void addWeeklyAverage(float weeklyAverage);
float calculateAverage(float *buffer, int size);

// Get averages
float getWeeklyAverage(void);
float getYearlyAverage(void);

// Float to String
void floatToString(float temperature, char *tempStr);

//help function


// Display functions
void updateDisplay(float currentTempCelsius);
void convertToMatrix(float temperature, char unit);
void displayMatrix(int binaryMatrix[8][45]);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Conversion functions
float celsiusToFahrenheit(float celsius) {
    return (celsius * 9.0 / 5.0) + 32.0;
}

float celsiusToKelvin(float celsius) {
    return celsius + 273.15;
}

// Add new temperature to the hourly buffer
void addHourlyTemperature(float temperature) {
    hourlyBuffer[hourlyIndex] = temperature;
    hourlyIndex = (hourlyIndex + 1) % HOURLY_BUFFER_SIZE;

    if (hourlyCount < HOURLY_BUFFER_SIZE) {
        hourlyCount++;
    }

    // If 168 readings are complete, calculate weekly average
    if (hourlyCount == HOURLY_BUFFER_SIZE) {
        float weeklyAverage = calculateAverage(hourlyBuffer, HOURLY_BUFFER_SIZE);
        addWeeklyAverage(weeklyAverage);
    }
}

// Add weekly average to the weekly buffer
void addWeeklyAverage(float weeklyAverage) {
    weeklyBuffer[weeklyIndex] = weeklyAverage;
    weeklyIndex = (weeklyIndex + 1) % WEEKLY_BUFFER_SIZE;
}

// Calculate average of a buffer
float calculateAverage(float *buffer, int size) {
    float sum = 0.0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return size > 0 ? (sum / size) : 0.0;
}

// Get weekly average (from hourly buffer)
float getWeeklyAverage(void) {
    return calculateAverage(hourlyBuffer, hourlyCount);
}

// Get yearly average (from weekly buffer)
float getYearlyAverage(void) {
    int validWeeks = (hourlyCount == HOURLY_BUFFER_SIZE) ? WEEKLY_BUFFER_SIZE : weeklyIndex;
    return calculateAverage(weeklyBuffer, validWeeks);
}
float pow(float number, int power) {
	float temp=number;
	if (power >0) {
		for (int k = 1; k < power; k++) {
			number*=temp;
		}
	} else if(power<0) {
		number=1/number;
		power+=1;
		power = -power;
		for (int k = 1; k < power; k++) {
			number/=temp;
		}
	}else{
		number=1;
	}
//	for (int k = 1; k < power; k++) {
//		number*=temp;
//	}
	return number;
}
float charArrToFloat(char* buffer) {
	float returnValue = 0.0;
	float addValue = 0.0;
	float negative = 1.0;
	float decimalLocation;
	for (int i = 0.0; i<6.0; i++) {
		if (buffer[i] == '.') {
			decimalLocation = i;
			break;
		}
	}

	for (int j = 0; j<6; j++) {
		addValue = 0.0;
		switch (buffer[j]) {
			case '-':
				negative *= -1.0;
				break;
			case '1':
				addValue = 1.0;
				break;
			case '2':
				addValue = 2.0;
				break;
			case '3':
				addValue = 3.0;
				break;
			case '4':
				addValue = 4.0;
				break;
			case '5':
				addValue = 5.0;
				break;
			case '6':
				addValue = 6.0;
				break;
			case '7':
				addValue = 7.0;
				break;
			case '8':
				addValue = 8.0;
				break;
			case '9':
				addValue = 9.0;
				break;
			case '0':
				addValue = 0.0;
				break;
			case '.':
				break;
		}
		addValue *= pow(10.0,decimalLocation-j-1);
		returnValue += addValue;
	}
	return returnValue *= negative;
}
void floatToString(float temperature, char *tempStr) {
    // Multiply by 100 to shift decimal point two places to the right
    int tempInt = (int)(temperature * 100);

    // Convert the integer part (before the decimal)
    int integerPart = tempInt / 100;  // Get the integer part
    int decimalPart = tempInt % 100;  // Get the decimal part

    // Build the string manually
    if (temperature < 0) {
        // Handle negative values
        integerPart = -integerPart;
        decimalPart = -decimalPart;
    }

    // Convert integer and decimal parts to strings
    int index = 0;
    if (temperature < 0) index = 1;  // Adjust the index for negative numbers

    // Add the integer part to the string
    index += sprintf(tempStr + index, "%d", integerPart);  // Convert integer to string

    // Add the decimal point
    tempStr[index++] = '.';

    // Add the decimal part (with leading zero if necessary)
    if (decimalPart < 10) {
        tempStr[index++] = '0';  // Add leading zero if necessary
    }
    sprintf(tempStr + index, "%d", decimalPart);  // Convert decimal to string

    // Null-terminate the string
    tempStr[index + 2] = '\0';
}

// Update display based on the current mode
void updateDisplay(float currentTempCelsius) {
	reset();
    float valueToDisplay = 0.0;
    char unit = 'C';  // Default to Celsius

    switch (currentMode) {
        case MODE_CURRENT_CELSIUS:
            valueToDisplay = currentTempCelsius;
            unit = 'C';
            break;
        case MODE_CURRENT_FAHRENHEIT:
            valueToDisplay = celsiusToFahrenheit(currentTempCelsius);
            unit = 'F';
            break;
        case MODE_CURRENT_KELVIN:
            valueToDisplay = celsiusToKelvin(currentTempCelsius);
            unit = 'K';
            break;
        case MODE_WEEKLY_CELSIUS:
            valueToDisplay = getWeeklyAverage();
            unit = 'C';
            break;
        case MODE_WEEKLY_FAHRENHEIT:
            valueToDisplay = celsiusToFahrenheit(getWeeklyAverage());
            unit = 'F';
            break;
        case MODE_WEEKLY_KELVIN:
            valueToDisplay = celsiusToKelvin(getWeeklyAverage());
            unit = 'K';
            break;
        case MODE_YEARLY_CELSIUS:
            valueToDisplay = getYearlyAverage();
            unit = 'C';
            break;
        case MODE_YEARLY_FAHRENHEIT:
            valueToDisplay = celsiusToFahrenheit(getYearlyAverage());
            unit = 'F';
            break;
        case MODE_YEARLY_KELVIN:
            valueToDisplay = celsiusToKelvin(getYearlyAverage());
            unit = 'K';
            break;
    }
    convertToMatrix(valueToDisplay, unit);
}

void convertToMatrix(float temperature, char unit) {
    // Initialize an 8x35 matrix of zeros
    int matrix[8][45] = {0};

    // Step 2: Format temperature as a string
    char tempStr[10];
    floatToString(temperature, tempStr);

    // Step 3: Place the sign (either + or -) in the first 3 columns
    int col = 8;
    if (temperature < 0) {
        // Negative sign representation
        for (int row = 0; row < 8; row++) {
            for (int j = 0; j < 3; j++) {
                matrix[row][col + j] = CHAR_MINUS[row][j];
            }
        }
    } else {
        // Positive sign representation
       for (int row = 0; row < 8; row++) {
            for (int j = 0; j < 3; j++) {
                matrix[row][col + j] = CHAR_PLUS[row][j];
            }
        }
    }
    col += 4;  // Move to the next column after the sign

    // Step 4: Process each character in the temperature string
    for (int i = 0; i < strlen(tempStr); i++) {
        char currentChar = tempStr[i];

        if (currentChar == '.') {
            // Place decimal point, it takes 2 columns
            for (int row = 0; row < 8; row++) {
                for (int j = 0; j < 2; j++) {
                    matrix[row][col + j] = CHAR_DOT[row][j];
                }
            }
            col += 2;  // Move 2 columns after the decimal point
        } else {
            // For digits (0-9), place their 3-column representation
            int digit = currentChar - '0'; // Convert character to digit
            if (digit >= 0 && digit <= 9) {
                for (int row = 0; row < 8; row++) {
                    for (int j = 0; j < 3; j++) {
                        matrix[row][col + j] = DIGIT_REPS[digit][row][j];
                    }
                }
                col += 3;  // Move 3 columns after placing the digit
            }
        }

        col++;  // Move one more column for the space after each digit or decimal
    }

    // Step 5: Place the star symbol (2 columns)
    for (int row = 0; row < 8; row++) {
        for (int j = 0; j < 2; j++) {
            matrix[row][col + j] = CHAR_STAR[row][j];
        }
    }
    col += 2;  // Move 2 columns after the star

    // Step 6: Place the unit character (C, F, or K)
    const int (*charRep)[3] = {0};
    if (unit == 'C') {
        charRep = CHAR_C;
    } else if (unit == 'F') {
        charRep = CHAR_F;
    } else if (unit == 'K') {
        charRep = CHAR_K;
    }

    // Place the unit character
    for (int row = 0; row < 8; row++) {
        for (int j = 0; j < 3; j++) {
            matrix[row][col + j] = charRep[row][j];
        }
    }
    displayMatrix(matrix);
}
 #include "stm32f4xx_hal.h"
 #include <stdio.h>
 #include <sys/time.h>

 #define NUM_ROWS 8
 #define NUM_COLS 45
 #define WINDOW_WIDTH 8

// Define GPIO Pins for rows (r1 to r8)
 #define ROW_PINS {R1_PIN, R2_PIN, R3_PIN, R4_PIN, R5_PIN, R6_PIN, R7_PIN, R8_PIN}
 const uint16_t rowPins[] = ROW_PINS;

 // Define GPIO Pins for columns (c1 to c8)
 #define ROW_PORTS {R1_GPIO_Port, R2_GPIO_Port, R3_GPIO_Port, R4_GPIO_Port, R5_GPIO_Port, R6_GPIO_Port, R7_GPIO_Port, R8_GPIO_Port}
 const GPIO_TypeDef* rowPorts[] = ROW_PORTS;

 // Define GPIO Pins for rows (r1 to r8)
 #define COL_PINS {C1_PIN, C2_PIN, C3_PIN, C4_PIN, C5_PIN, C6_PIN, C7_PIN, C8_PIN}
 const uint16_t colPins[] = COL_PINS;

 // Define GPIO Pins for columns (c1 to c8)
 #define COL_PORTS {C1_GPIO_Port, C2_GPIO_Port, C3_GPIO_Port, C4_GPIO_Port, C5_GPIO_Port, C6_GPIO_Port, C7_GPIO_Port, C8_GPIO_Port}
 const GPIO_TypeDef* colPorts[] = COL_PORTS;

 int scanCount = 0;

 // function that resets all the pins
 void reset(void) {
	 HAL_GPIO_WritePin(C1_GPIO_Port,C1_PIN, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(C2_GPIO_Port,C2_PIN, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(C3_GPIO_Port,C3_PIN, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(C4_GPIO_Port,C4_PIN, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(C5_GPIO_Port,C5_PIN, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(C6_GPIO_Port,C6_PIN, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(C7_GPIO_Port,C7_PIN, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(C8_GPIO_Port,C8_PIN, GPIO_PIN_SET);
	 HAL_GPIO_WritePin(R1_GPIO_Port,R1_PIN, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(R2_GPIO_Port,R2_PIN, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(R3_GPIO_Port,R3_PIN, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(R4_GPIO_Port,R4_PIN, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(R5_GPIO_Port,R5_PIN, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(R6_GPIO_Port,R6_PIN, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(R7_GPIO_Port,R7_PIN, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(R8_GPIO_Port,R8_PIN, GPIO_PIN_RESET);
 }

 // Function to display the matrix on the 8x8 matrix display
 void displayMatrix(int binaryMatrix[NUM_ROWS][NUM_COLS]) {
//	   HAL_GPIO_WritePin(colPorts[0],colPins[0], GPIO_PIN_RESET);
//	   HAL_GPIO_WritePin(rowPorts[0],rowPins[0], GPIO_PIN_SET);
     // Loop through the matrix columns with a window size of 8 columns at a time
     for (int colStart = 0; colStart <= NUM_COLS - WINDOW_WIDTH; colStart++) {
         scanCount = 0;
    	 while (scanCount <= 25) {
			 // Loop through each row (r1 to r8)
			 for (int row = 0; row < NUM_ROWS; row++) {
				 // Set the row pins to high for the current row

				 HAL_GPIO_WritePin(rowPorts[row],rowPins[row], GPIO_PIN_SET);

				 for (int col = 0; col < WINDOW_WIDTH; col++) {
					 if (binaryMatrix[row][col + colStart]) {
						 HAL_GPIO_WritePin(colPorts[col],colPins[col], GPIO_PIN_RESET);
					 }
				 }
				 HAL_Delay(1);
				 reset();
			 }
			 scanCount++;
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
   char RX_buffer[6];
   float data_r;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

   while (1) {



// 	  if (HAL_SPI_Receive(&hspi1, rxBuffer, RX_BUFFER_SIZE,1000)==HAL_OK){
// 		  HAL_UART_Transmit(&huart2, temp_buffer,6,HAL_MAX_DELAY);
// 	  } else if (!(HAL_SPI_Receive(&hspi1, rxBuffer, RX_BUFFER_SIZE,1000))){
// 		  HAL_UART_Transmit
//	  int start_time = get_current_time_ms();

	  updateDisplay(25.50);

//	  if (HAL_SPI_Receive(&hspi1,(uint8_t*)RX_buffer,6,1000)==HAL_OK) {
//		  HAL_UART_Transmit(&huart2,(uint8_t*)RX_buffer,7,1000);
////		  updateDisplay(atof(RX_buffer));
//		  data_r = charArrToFloat(RX_buffer);
//		  updateDisplay(data_r);
//		  for (int i=0;i<6;i++){
//			  RX_buffer[i]='\0';
//		  }
////
//	  }
//	  if (0) {
//		  if (m<8) {
//			  m++;
//		  } else {
//			  m = 0;
//		  }
//		   Mode_t currentMode = modes[m];
//	  }



	  // if (HAL_SPI_Receive(&hspi1, rxBuffer, RX_BUFFER_SIZE,1000)==HAL_OK){
	  // 		  HAL_UART_Transmit(&huart2, temp_buffer,6,HAL_MAX_DELAY);
	  // 	  }
//
////	  int elapsed_time = get_current_time_ms() - start_time;
//	  HAL_Delay(5000 - 1000);
//   	  currentTempCelsius = atof(RX_buffer);


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 PA8
                           PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB10 PB3 PB4
                           PB5 PB6 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

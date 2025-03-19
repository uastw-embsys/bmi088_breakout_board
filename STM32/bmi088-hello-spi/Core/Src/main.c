/* USER CODE BEGIN Header */

/** ***************************************************************************
 * Author: Christian Fibich <fibich@technikum-wien.at>
 * 
 * Stadt Wien Kompetenzteam für Drohnentechnik in der Fachhochschulausbildung
 * Kompetenzfeld Embedded Systems
 * Fachhochschule Technikum Wien
 *
 * All rights reserved.
 *
 * Date: 19.3.2025
 *
 * BMI088 rudimentary bring-up code (via SPI).
 * Implements a complementary filter for orienation estimation.
 *
 ******************************************************************************/
 
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

struct imu_measurement {
	float acc[3];
	float gyr[3];
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* attention: left shift included for HAL */

#define BMI088_GYROSCOPE_I2C_ADDR(x) ((0x68 | ((x) & 1)) << 1)
#define BMI088_ACCELEROMETER_I2C_ADDR(x) ((0x18 | ((x) & 1)) << 1)

#define BMI088_REG_ADDR_LEN 1

#define BMI088_REG_ACC_CHIP_ID                      0x00
#define BMI088_REG_ACC_CHIP_ID_VALUE                0x1E
#define BMI088_REG_ACC_ERR_REG                      0x02
#define BMI088_REG_ACC_STATUS                       0x03
#define BMI088_REG_ACC_DATA0                        0x12
#define BMI088_REG_ACC_CONF                         0x40
#define BMI088_REG_ACC_CONF_BWP_NORMAL              (0x0A << 4)
#define BMI088_REG_ACC_CONF_BWP_OSR2                (0x09 << 4)
#define BMI088_REG_ACC_CONF_BWP_OSR4                (0x08 << 4)
#define BMI088_REG_ACC_CONF_ODR_100Hz               (0x08)
#define BMI088_REG_ACC_CONF_ODR_200Hz               (0x09)
#define BMI088_REG_ACC_CONF_ODR_400Hz               (0x0A)
#define BMI088_REG_ACC_CONF_ODR_800Hz               (0x0B)
#define BMI088_REG_ACC_CONF_ODR_1600Hz              (0x0C)
#define BMI088_REG_ACC_RANGE                        0x41
#define BMI088_REG_ACC_RANGE_3G                     0x00
#define BMI088_REG_ACC_RANGE_6G                     0x01
#define BMI088_REG_ACC_RANGE_12G                    0x02
#define BMI088_REG_ACC_RANGE_24G                    0x03
#define BMI088_REG_ACC_PWR_CTRL                     0x7D
#define BMI088_REG_ACC_PWR_CTRL_ACCELEROMETER_ON    0x04
#define BMI088_REG_ACC_PWR_CONF                     0x7C
#define BMI088_REG_ACC_PWR_CONF_ACTIVE_MODE         0x00
#define BMI088_REG_ACC_INT1_IO_CONF                 0x53
#define BMI088_REG_ACC_INT1_IO_CONF_INPUT           (1<<4)
#define BMI088_REG_ACC_INT1_IO_CONF_OUTPUT          (1<<3)
#define BMI088_REG_ACC_INT1_IO_CONF_OD              (1<<2)
#define BMI088_REG_ACC_INT1_IO_CONF_ACTIVE_HIGH     (1<<1)
#define BMI088_REG_ACC_INT2_IO_CONF                 0x54
#define BMI088_REG_ACC_INT2_IO_CONF_INPUT           (1<<4)
#define BMI088_REG_ACC_INT2_IO_CONF_OUTPUT          (1<<3)
#define BMI088_REG_ACC_INT2_IO_CONF_OD              (1<<2)
#define BMI088_REG_ACC_INT2_IO_CONF_ACTIVE_HIGH     (1<<1)
#define BMI088_REG_ACC_INT1_INT2_MAP_DATA           0x58
#define BMI088_REG_ACC_INT1_INT2_MAP_DATA_INT1_DRDY (1<<2)

#define ACC_RANGE BMI088_REG_ACC_RANGE_24G

#define BMI088_REG_GYR_CHIP_ID              0x00
#define BMI088_REG_GYR_CHIP_VALUE           0x0F
#define BMI088_REG_GYR_GYRO_RANGE           0x0F
#define BMI088_REG_GYR_GYRO_RANGE_2000DPS   0x00
#define BMI088_REG_GYR_GYRO_BANDWIDTH       0x10
#define BMI088_REG_GYR_GYRO_BANDWIDTH_23Hz  0x04
#define BMI088_REG_GYR_GYRO_BANDWIDTH_47Hz  0x03
#define BMI088_REG_GYR_GYRO_BANDWIDTH_116Hz 0x02
#define BMI088_REG_GYR_DATA0                0x02
#define BMI088_REG_GYR_INT3_INT4_IO_CONF    0x16
#define BMI088_REG_GYR_INT3_INT4_IO_CONF_INT3_OD          (1<<1)
#define BMI088_REG_GYR_INT3_INT4_IO_CONF_INT3_ACTIVE_HIGH (1<<0)
#define BMI088_REG_GYR_INT3_INT4_IO_MAP     0x18
#define BMI088_REG_GYR_INT3_INT4_IO_MAP_INT3  0x01
#define BMI088_REG_GYR_INT_CTRL             0x15
#define BMI088_REG_GYR_INT_CTRL_ON          0x80

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void huart2_printf(const char *format, ...) {
	static char linebuf[1024];
	va_list args;
	va_start(args, format);
	int n = vsnprintf(linebuf, 1024, format, args);
	HAL_UART_Transmit(&huart2, (uint8_t*) linebuf, n, HAL_MAX_DELAY);
	va_end(args);
}

int huart2_fgets(char *buf, int len) {
	HAL_UART_AbortReceive(&huart2);
	while (len > 1) {
		HAL_UART_Receive(&huart2, (uint8_t*) buf, 1, HAL_MAX_DELAY);
		HAL_UART_Transmit(&huart2, (uint8_t*) buf, 1, HAL_MAX_DELAY);
		if (*buf == '\r') {
			break;
		}
		buf++;
		len--;
	}
	buf[1] = '\0';
	return (len == 0) ? -1 : 0;
}

float get_dt(void) {
	static uint32_t last_tick = 0;
	uint32_t current_tick = HAL_GetTick();
	float dt = 0;

	if (current_tick < last_tick) {
		dt = ((0xffffffff-last_tick)+current_tick)/1000.0;
	} else {
		dt = (current_tick-last_tick)/1000.0;
	}
	last_tick = current_tick;

	return dt;
}

#define BMI088_ACCEL_CS_PIN GPIO_PIN_0
#define BMI088_GYRO_CS_PIN GPIO_PIN_1
#define BMI088_CS_GPIO GPIOB

enum sensor {
	BMI088_GYRO  = BMI088_GYRO_CS_PIN,
	BMI088_ACCEL = BMI088_ACCEL_CS_PIN
};

int bmi088_spi_write(uint8_t addr, uint8_t *data, int len, enum sensor sensor) {
	static uint8_t buf[17];
	if (len > sizeof(buf)-1) return -1;
	buf[0] = addr;
	memcpy(buf+1,data,len);
	/* set CS */
	HAL_GPIO_WritePin(BMI088_CS_GPIO, sensor, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, buf, len+1, HAL_MAX_DELAY);
	/* unset CS */
	HAL_GPIO_WritePin(BMI088_CS_GPIO, BMI088_GYRO_CS_PIN|BMI088_ACCEL_CS_PIN, GPIO_PIN_SET);
	return 0;
}

int bmi088_spi_read(uint8_t addr, uint8_t *data, int len, enum sensor sensor) {
	static uint8_t obuf[18];
	static uint8_t ibuf[18];
	if (len > sizeof(obuf)-2) return -1;
	memset(obuf,0,sizeof(obuf));
	memset(ibuf,0,sizeof(ibuf));
	int offset = sensor == BMI088_GYRO ? 1 : 2;

	obuf[0] = addr | 0x80;
	/* set CS */
	HAL_GPIO_WritePin(BMI088_CS_GPIO, sensor, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, obuf, ibuf, len+offset, HAL_MAX_DELAY);
	/* unset CS */
	HAL_GPIO_WritePin(BMI088_CS_GPIO, BMI088_GYRO_CS_PIN|BMI088_ACCEL_CS_PIN, GPIO_PIN_SET);
	memcpy(data,ibuf+offset,len);
	return 0;
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	/* identify BMI088 by searching address space*/

	int ai = 0;
	int gi = 0;
	int rv_acc = 1;
	int rv_gyr = 1;
	uint8_t acc_chip_id = 0;
	uint8_t gyr_chip_id = 0;

	HAL_Delay(1); // Datasheet PG 13

	(void) bmi088_spi_read(BMI088_REG_ACC_CHIP_ID, &acc_chip_id, 1, BMI088_ACCEL); // dummy read to wake ACCEL
	rv_acc = bmi088_spi_read(BMI088_REG_ACC_CHIP_ID, &acc_chip_id, 1, BMI088_ACCEL);
	(void) bmi088_spi_read(BMI088_REG_GYR_CHIP_ID, &gyr_chip_id, 1, BMI088_GYRO); // dummy read to wake GYRO
	rv_gyr = bmi088_spi_read(BMI088_REG_GYR_CHIP_ID, &gyr_chip_id, 1, BMI088_GYRO);

	int gci = gyr_chip_id;
	int aci = acc_chip_id;

	huart2_printf("ACC: %02x (%d)\r\n",aci,rv_acc);
	huart2_printf("GYR: %02x (%d)\r\n",gci,rv_gyr);

	if (rv_acc || (acc_chip_id != BMI088_REG_ACC_CHIP_ID_VALUE)) {
		huart2_printf("I2C accelerometer not present.\r\n");
		return -1;
	}

	if (rv_gyr || (gyr_chip_id != BMI088_REG_GYR_CHIP_VALUE)) {
		huart2_printf("I2C gyroscope not present.\r\n");
		return -1;
	}

	huart2_printf("BMI088 found: GYR=%d (%02x) ACC=%d (%02x)\r\n", rv_gyr, gci,
			rv_acc, aci);

	/* initialize BMI088 */

	/* enable accel */

	uint8_t data = BMI088_REG_ACC_PWR_CTRL_ACCELEROMETER_ON;
	rv_acc = bmi088_spi_write(BMI088_REG_ACC_PWR_CTRL, &data, sizeof(data), BMI088_ACCEL);

	HAL_Delay(50); // Datasheet PG 13

	data = BMI088_REG_ACC_PWR_CONF_ACTIVE_MODE;
	rv_acc = bmi088_spi_write(BMI088_REG_ACC_PWR_CONF, &data, sizeof(data), BMI088_ACCEL);

	data = ACC_RANGE;
	rv_acc |= bmi088_spi_write(BMI088_REG_ACC_RANGE, &data, sizeof(data), BMI088_ACCEL);

	/* configure accel filter to normal mode and output rate to 100 Hz and LPF to normal mode */

	data = BMI088_REG_ACC_CONF_BWP_NORMAL | BMI088_REG_ACC_CONF_ODR_100Hz;
	rv_acc |= bmi088_spi_write(BMI088_REG_ACC_CONF, &data, sizeof(data), BMI088_ACCEL);

	/* IO1 inactive */
	data = 0;
	rv_acc |= bmi088_spi_write(BMI088_REG_ACC_INT1_IO_CONF, &data, sizeof(data), BMI088_ACCEL);

	/* IO2 inactive */
	data = 0;
	rv_acc |= bmi088_spi_write(BMI088_REG_ACC_INT2_IO_CONF, &data, sizeof(data), BMI088_ACCEL);

	/* map_data */
	data = BMI088_REG_ACC_INT1_INT2_MAP_DATA_INT1_DRDY;
	rv_acc |= bmi088_spi_write(BMI088_REG_ACC_INT2_IO_CONF, &data, sizeof(data), BMI088_ACCEL);

	/* configure gyro range to 2000 DPS */

	data = BMI088_REG_GYR_GYRO_RANGE_2000DPS;
	rv_gyr = bmi088_spi_write(BMI088_REG_GYR_GYRO_RANGE, &data, sizeof(data), BMI088_GYRO);

	/* configure gyro bandwidth to 1000 Hz (116 Hz filter bandwidth) */

	data = BMI088_REG_GYR_GYRO_BANDWIDTH_47Hz;
	rv_gyr |= bmi088_spi_write(BMI088_REG_GYR_GYRO_BANDWIDTH, &data, sizeof(data), BMI088_GYRO);

	/* IO3 inactive */
	data = 0;
	rv_gyr |= bmi088_spi_write(BMI088_REG_GYR_INT3_INT4_IO_CONF, &data, sizeof(data), BMI088_GYRO);

	/* IO3 map */
	data = BMI088_REG_GYR_INT3_INT4_IO_MAP_INT3;
	rv_gyr |= bmi088_spi_write(BMI088_REG_GYR_INT3_INT4_IO_MAP, &data, sizeof(data), BMI088_GYRO);

	/* int off */
	data = 0;
	rv_gyr |= bmi088_spi_write(BMI088_REG_GYR_INT_CTRL, &data, sizeof(data), BMI088_GYRO);

	if (rv_acc) {
		huart2_printf("I2C accelerometer could not be configured.\r\n");
		return -1;
	}

	if (rv_gyr) {
		huart2_printf("I2C accelerometer could not be configured.\r\n");
		return -1;
	}
#if 0
	do {
		huart2_printf("Start reading BMI088 [yN]?\r\n");
		int rv = huart2_fgets(inbuf, 1024);
		if (rv)
			continue;
	} while (strcmp(inbuf, "y\r"));
#endif

	static struct imu_measurement imu_value = { 0 };

	/* vectors for sensor fusion */
	float theta_ax;
	float theta_ay;

	float theta_gx = 0;
	float theta_gy = 0;
	float theta_gz = 0;

	float theta_x = 0;
	float theta_y = 0;
	float theta_z = 0;

	/* data buffer for low pass filter */
	float _ax[4];
	float _ay[4];
	float _az[4];

	/* parameters for sensor fusion */
	float dt;   /* sample rate (roughly) */
	float alpha = 0.25 /* i.e., 75% accelerometer -- 25% gyro */;

	while (1) {

		int16_t gyro_raw[3];
		int16_t acc_raw[3];

		/* get measured raw data */

		// 00010010
		// 01001000

		int rv_gyr = bmi088_spi_read(BMI088_REG_GYR_DATA0, (uint8_t *)gyro_raw, sizeof(gyro_raw), BMI088_GYRO);
		int rv_acc = bmi088_spi_read(BMI088_REG_ACC_DATA0, (uint8_t *)acc_raw, sizeof(acc_raw), BMI088_ACCEL);

		/* scale data according to settings */
		for (int i = 0; i < 3; i++) {
			if (!rv_acc) {
				imu_value.acc[i] = acc_raw[i] / 32768.0 * 1.5
						* (1 << (ACC_RANGE + 1));
			}
			if (!rv_gyr) {
				imu_value.gyr[i] = 2000.0 * gyro_raw[i] / 32767.0;
			}
		}

		/* calculate if we have measurements */
		if (!rv_acc && !rv_gyr) {

		    /* Implement sensor fusion using a complementary filter.
		     *
		     * This means:
		     * (1) low-pass filter the accelerometer data
		     * (2) high-pass filter the gyro data (not implemented)
		     * (3) calculate the accelerometer angle via trigonometry
		     * (4) integrate the gyro data to get the orientation estimate from the gyro
		     * (5) weighted average of the two estimates according to alpha factor
		     *
		     * See https://ahrs.readthedocs.io/en/latest/filters/complementary.html */

			/* get time delay of current and last measurement */
			dt = get_dt();

		    /*
		     * (1) Implement moving average LPF for accelerometer; potential for future FIR filter
		     */

		    /* shift the buffer down */
		    memmove(_ax+1,_ax,sizeof(_ax)-sizeof(_ax[0]));
		    memmove(_ay+1,_ay,sizeof(_ay)-sizeof(_ay[0]));
		    memmove(_az+1,_az,sizeof(_az)-sizeof(_az[0]));

		    /* shift in the current measurment */
		    _ax[0] = imu_value.acc[0], _ay[0] = imu_value.acc[1], _az[0] = imu_value.acc[2];

		    /* calculate the dot product of the buffer and the coefficient vector
		     * as the measurements */

		    /* just use a 1/n coefficient for now (moving average) */
		    float coeff = 1.0/((float)(sizeof(_ax))/sizeof(_ax[0]));

		    float ax = 0;
		    float ay = 0;
		    float az = 0;

		    for (int i = 0; i < sizeof(_ax)/sizeof(_ax[0]); i++) {
		    	ax += _ax[i] * coeff;
		    }
		    for (int i = 0; i < sizeof(_ay)/sizeof(_ay[0]); i++) {
		    	ay += _ay[i] * coeff;
		    }
		    for (int i = 0; i < sizeof(_az)/sizeof(_az[0]); i++) {
		    	az += _az[i] * coeff;
		    }

		    /*
		     * (2) scale the Gyro measurements from DPS to rad/s
		     */
		    float gx = imu_value.gyr[0]/180.0*M_PI;
		    float gy = imu_value.gyr[1]/180.0*M_PI;
		    float gz = imu_value.gyr[2]/180.0*M_PI;


		    /*
		     * (3) calculate the accelerometer angle via trigonometry
		     */
		    theta_ax  = atan2f(ay, az);
		    theta_ay  = atan2f(-ax,sqrtf(ay*ay+az*az));

		    /*
		     * (4) calculate the gyroscope angle via integration of the angular velocity
		     */

		    theta_gx += gx * dt;
		    theta_gy += gy * dt;
		    theta_gz += gz * dt;

		    /* if the drift gets too bad, reset the gyro contribution
		     * can only do this for roll and pitch, not for yaw as we have no mag. */
		    if (fabs(theta_gx-theta_ax) > M_PI/32) {
		    	theta_gx = theta_ax;
		    }
		    if (fabs(theta_gy-theta_ay) > M_PI/32) {
		    	theta_gy = theta_ay;
		    }

		    /*
		     * (5) average the two measurments
		     */
		    theta_x = alpha * theta_gx + (1-alpha) * theta_ax;
		    theta_y = alpha * theta_gy + (1-alpha) * theta_ay;
		    theta_z = theta_gz;

		    /* align to orientation of click shield (USB connector points backwards) */
		    float roll = -theta_y;
		    float pitch = theta_x;
		    float yaw   = theta_z;

		    /* output the measurments */
		    huart2_printf("ROLL=%5.2f PITCH=%5.2f YAW=%5.2f\r\n",roll*180.0/M_PI,pitch*180.0/M_PI,yaw*180.0/M_PI);
		    HAL_Delay(20);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
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
	while (1) {
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

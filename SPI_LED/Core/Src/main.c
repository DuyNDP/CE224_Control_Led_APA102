/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "arm_math.h" // Thư viện toán học DSP
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUM_LEDS 30           // How many LEDs are in your strip
#define LED_FRAME_SIZE 4      // 4 bytes per LED (B, G, R, Brightness)
#define START_FRAME_SIZE 4    // 4 bytes for the start frame
#define END_FRAME_SIZE 4      // 4 bytes for the end frame
#define BUFFER_SIZE (START_FRAME_SIZE + (NUM_LEDS * LED_FRAME_SIZE) + END_FRAME_SIZE)

// --- CẤU HÌNH XỬ LÝ ÂM THANH (FFT) ---
#define FFT_SAMPLES 512              // Số mẫu FFT (phải là lũy thừa của 2: 512, 1024, 2048...)
#define SAMPLING_RATE 64565          // Clock 84MHz / 1301 (ARR=1300) = ~64565 Hz

// --- CẤU HÌNH HIỂN THỊ (SCALING & LIMIT) ---
#define TARGET_MAX_VAL  10000.0f  // Giới hạn hiển thị cường độ (0 - 10k)
#define TARGET_MAX_HZ   30000.0f // Giới hạn hiển thị tần số (0 - 30k)

// Hệ số chia để thu nhỏ giá trị FFT khổng lồ xuống thang 10k.
#define MAG_SCALE_FACTOR 35.0f

// --- CẤU HÌNH LED ---
#define LED_UPDATE_INTERVAL 1000  // Thời gian update LED: 1000ms = 1 giây
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim2;

USART_HandleTypeDef husart6;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */
uint8_t spi_led_buffer[BUFFER_SIZE];
uint8_t usart_led_buffer[BUFFER_SIZE];

// --- Biến cho xử lý âm thanh ---
uint16_t adc_buffer[FFT_SAMPLES];       // Buffer chứa dữ liệu thô từ Mic (DMA nạp vào đây)
float32_t fft_in_buf[FFT_SAMPLES];      // Buffer đầu vào cho hàm FFT (Float)
float32_t fft_out_buf[FFT_SAMPLES];     // Buffer đầu ra (số phức)
float32_t fft_mag_buf[FFT_SAMPLES / 2]; // Buffer biên độ tần số (Kết quả cuối cùng)

volatile uint8_t fft_process_flag = 0;  // Cờ báo hiệu đã thu đủ mẫu
arm_rfft_fast_instance_f32 fft_handler; // Bộ quản lý thư viện FFT

// --- BIẾN KẾT QUẢ OUTPUT ---
float audio_peak_val = 0.0f; // Cường độ âm thanh lớn nhất hiện tại (Volume)
float audio_peak_hz  = 0.0f; // Tần số của âm thanh đó (Pitch/Tone)

// các biến dùng để debug
int32_t debug_peak_val = 0;
int32_t debug_peak_hz = 0;

// --- Biến quản lý thời gian LED ---
uint32_t last_led_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART6_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void spi_update(void)
{
	extern SPI_HandleTypeDef hspi3;
    // Start the DMA transfer
    HAL_SPI_Transmit_DMA(&hspi3, spi_led_buffer, BUFFER_SIZE);
}

void spi_set_led(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
    if (index < 0 || index >= NUM_LEDS) {
        return; // Out of bounds
    }

    // Calculate the starting position for this LED in the buffer
    // (skip the 4-byte start frame)
    uint32_t pos = START_FRAME_SIZE + (index * LED_FRAME_SIZE);

    // APA102 LED Frame:
    // Byte 0: 0xE0 (Global Brightness, 5 bits)
    // Byte 1: Blue (0-255)
    // Byte 2: Green (0-255)
    // Byte 3: Red (0-255)

    spi_led_buffer[pos + 0] = 0xE0 | (brightness & 0x1F); // Brightness (masked to 5 bits)
    spi_led_buffer[pos + 1] = b;
    spi_led_buffer[pos + 2] = g;
    spi_led_buffer[pos + 3] = r;
}

void spi_chase_effect()
{
	for (int i = 0; i < NUM_LEDS; i++) {
	   // Set all to a dim blue
		for(int j=0; j < NUM_LEDS; j++) {
			 spi_set_led(j, 0, 0, 50, 2); // R, G, B, Brightness
		}

		// Set the current "chaser" LED to bright white
		spi_set_led(i, 255, 0, 0, 5);

		// Send the data to the strip
		spi_update();

		HAL_Delay(25); // Animation speed
	}
}

void usart_set_led(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t brightness)
{
    if (index >= NUM_LEDS) {
        return;
    }

    uint32_t pos = START_FRAME_SIZE + (index * LED_FRAME_SIZE);

    usart_led_buffer[pos + 0] = 0b11100000 | (brightness & 0x1F);
    usart_led_buffer[pos + 1] = b;
    usart_led_buffer[pos + 2] = g;
    usart_led_buffer[pos + 3] = r;
}

void usart_update(void)
{
    // Wait for any previous DMA transfer to finish
    // Note: We check HAL_UART_STATE_BUSY_TX
    while (HAL_USART_GetState(&husart6) == HAL_USART_STATE_BUSY_TX);

    // Start the DMA transfer using the UART HAL function
    HAL_USART_Transmit_DMA(&husart6, usart_led_buffer, BUFFER_SIZE);
}

void usart_chase_effect()
{
	for (int i = 0; i < NUM_LEDS; i++) {
	   // Set all to a dim blue
		for(int j=0; j < NUM_LEDS; j++) {
			 usart_set_led(j, 0, 0, 50, 2); // R, G, B, Brightness
		}

		// Set the current "chaser" LED to bright white
		usart_set_led(i, 255, 0, 0, 5);

		// Send the data to the strip
		usart_update();

		HAL_Delay(25); // Animation speed
	}
}

void led_init()
{
	// 1. Set Start Frame (4 bytes of 0x00)
	for (int i = 0; i < START_FRAME_SIZE; i++) {
		spi_led_buffer[i] = 0x00;
		usart_led_buffer[i] = 0x00;
	}

	// 2. Set End Frame (4 bytes of 0xFF)
	// Calculate end frame position
	int end_frame_pos = START_FRAME_SIZE + (NUM_LEDS * LED_FRAME_SIZE);
	for (int i = 0; i < END_FRAME_SIZE; i++) {
		spi_led_buffer[end_frame_pos + i] = 0xFF;
		usart_led_buffer[end_frame_pos + i] = 0xFF;
	}

	// 3. Set all LEDs to "off" initially
	for (int i = 0; i < NUM_LEDS; i++) {
	  	 spi_set_led(i, 0, 0, 0, 0); // R, G, B, Brightness
	  	 usart_set_led(i, 0, 0, 0, 0);
	 }
	 spi_update(); // Send the "off" state
	 usart_update();
}

void process_audio_data(void) {
    if (fft_process_flag) {
        // 1. Lọc nhiễu DC (Zero centering)
        // Đưa tín hiệu từ 0-4095 về dao động quanh 0
        float32_t avg = 0;
        for (uint16_t i = 0; i < FFT_SAMPLES; i++) {
            avg += (float32_t)adc_buffer[i];
        }
        avg /= (float32_t)FFT_SAMPLES;

        for (uint16_t i = 0; i < FFT_SAMPLES; i++) {
            fft_in_buf[i] = (float32_t)adc_buffer[i] - avg;
        }

        // 2. Thực hiện FFT (Time Domain -> Frequency Domain)
        arm_rfft_fast_f32(&fft_handler, fft_in_buf, fft_out_buf, 0);

        // 3. Tính độ lớn (Magnitude)
        // Hàm này thay thế cho đoạn code: mag = sqrt(re*re + im*im)
        // Kết quả lưu vào fft_mag_buf. Chỉ có FFT_SAMPLES/2 phần tử hợp lệ.
        arm_cmplx_mag_f32(fft_out_buf, fft_mag_buf, FFT_SAMPLES / 2);

        // 4. PHÂN TÍCH TÌM PEAK (Giống cấu trúc bạn yêu cầu)
        // Tìm xem tần số nào đang chiếm ưu thế nhất (To nhất)
        float max_mag = 0.0f;
        uint16_t max_index = 0;

        // Bắt đầu từ k=1 hoặc k=2 để bỏ qua thành phần DC (k=0) thường rất lớn nhưng vô nghĩa
        for (uint16_t k = 2; k < (FFT_SAMPLES / 2); k++) {
            float current_mag = fft_mag_buf[k];

            // Tìm giá trị lớn nhất (Peak Finding)
            if (current_mag > max_mag) {
                max_mag = current_mag;
                max_index = k;
            }
        }

        // 5. Cập nhật kết quả ra biến toàn cục

        // Xử lý Cường độ (VAL): Chia nhỏ xuống và cắt trần (Clamp)
        float final_val = max_mag / MAG_SCALE_FACTOR;
        if (final_val > TARGET_MAX_VAL) final_val = TARGET_MAX_VAL; // Cắt nếu vượt 10k
        if (final_val < 0.0f) final_val = 0.0f;

        // Xử lý Tần số (HZ): Tính theo công thức chuẩn và cắt trần
        float final_hz = (float)max_index * (SAMPLING_RATE / (float)FFT_SAMPLES);
        if (final_hz > TARGET_MAX_HZ) final_hz = TARGET_MAX_HZ; // Cắt nếu vượt 100k
        audio_peak_val = final_val;
        audio_peak_hz  = final_hz;

        // Ép kiểu sang số nguyên để SWV vẽ cho đẹp
        debug_peak_val = (int32_t)audio_peak_val;
        debug_peak_hz  = (int32_t)audio_peak_hz;

        // 6. Reset cờ
        fft_process_flag = 0;
        extern ADC_HandleTypeDef hadc1;
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, FFT_SAMPLES);
    }
}

// Hàm xử lý LED thông minh: Tách biệt độ sáng và màu sắc
void update_led_smart(float vol, float hz) {

	if (HAL_GetTick() - last_led_tick >= LED_UPDATE_INTERVAL) {

	        // Quyết định màu dựa trên Tần số (hz)
	        uint8_t r=0, g=0, b=0;

	        // Nếu im lặng (Volume quá bé) -> Tắt LED
	        if (vol < 1000.0f) {
	            r = 0; g = 0; b = 0;
	        }
	        else {
	            // Logic chọn màu đơn giản
	            if (hz < 500.0f)       { r=255; g=0;   b=0; }   // Bass: Đỏ
	            else if (hz < 1500.0f) { r=0;   g=255; b=0; }   // Mid:  Xanh Lá
	            else { r=0;   g=0;   b=255; } // Treble: Xanh Dương
	        }

	        // Set toàn bộ 30 LED cùng 1 màu
	        // Độ sáng fix cứng mức trung bình (10) hoặc theo Volume tùy bạn.
	        // Ở đây tôi để mức 15 cho rõ, volume chỉ quyết định On/Off.
	        uint8_t brightness = (r+g+b > 0) ? 15 : 0;

	        for (int i = 0; i < NUM_LEDS; i++) {
	            spi_set_led(i, r, g, b, brightness);
	        }

	        spi_update(); // Gửi tín hiệu ra dây LED

	        // Cập nhật thời gian
	        last_led_tick = HAL_GetTick();
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
  MX_SPI3_Init();
  MX_USART6_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  led_init();
  spi_update();
  // Khởi tạo cấu trúc FFT (512 mẫu)
  arm_rfft_fast_init_f32(&fft_handler, FFT_SAMPLES);

  // Bắt đầu Timer 2 (để tạo nhịp)
  HAL_TIM_Base_Start(&htim2);

  // Bắt đầu ADC với DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, FFT_SAMPLES);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  process_audio_data();

	  // Gọi hàm LED mới với 2 biến kết quả từ FFT
	  // audio_peak_val: Cường độ
	  // audio_peak_hz:  Tần số
	  update_led_smart(audio_peak_val, audio_peak_hz);
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1300;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  husart6.Instance = USART6;
  husart6.Init.BaudRate = 5250000;
  husart6.Init.WordLength = USART_WORDLENGTH_8B;
  husart6.Init.StopBits = USART_STOPBITS_1;
  husart6.Init.Parity = USART_PARITY_NONE;
  husart6.Init.Mode = USART_MODE_TX;
  husart6.Init.CLKPolarity = USART_POLARITY_LOW;
  husart6.Init.CLKPhase = USART_PHASE_1EDGE;
  husart6.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    // This is where you would set a flag, e.g.
    // g_transfer_complete = 1;
    // Your main loop can then check this flag before
    // filling the buffer with new data and calling show_leds() again.
}

// Hàm này được gọi tự động khi DMA điền đầy buffer
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if (hadc->Instance == ADC1) {
		// 1. Dừng ADC/DMA ngay lập tức để dữ liệu không bị ghi đè
	    HAL_ADC_Stop_DMA(&hadc1);

	    // 2. Báo cờ
	    fft_process_flag = 1;
	}
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
#ifdef USE_FULL_ASSERT
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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "si5351.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* SI5351 硬件控制引脚 (OEB/SSEN)
 * OEB  = PE0, 输出使能 (LOW 有效)
 * SSEN = PE1, 扩频使能 (LOW 禁用扩频) */
#define SI5351_OEB_PORT   GPIOE
#define SI5351_OEB_PIN    GPIO_PIN_0
#define SI5351_SSEN_PORT  GPIOE
#define SI5351_SSEN_PIN   GPIO_PIN_1

#define SI5351_OEB_LOW()   HAL_GPIO_WritePin(SI5351_OEB_PORT,  SI5351_OEB_PIN,  GPIO_PIN_RESET)
#define SI5351_SSEN_LOW()  HAL_GPIO_WritePin(SI5351_SSEN_PORT, SI5351_SSEN_PIN, GPIO_PIN_RESET)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* I2C 调试变量 — 在此处设断点查看 */
static uint8_t  i2c_found_devices[16];
static uint8_t  i2c_found_count;
static HAL_StatusTypeDef i2c_last_error;
static uint8_t  i2c_tested_addr;

/* SI5351 状态调试变量 */
static uint8_t  si5351_reg00;   /* reg 0x00: [7]=SYS_INIT, [5]=LOL_A, [3:0]=REVID */
static uint8_t  si5351_reg03;   /* reg 0x03: OE 回读 */
static uint8_t  si5351_pll_locked;  /* 1=锁定, 0=失锁 */

/* 寄存器回读验证 (确认 I2C 写入是否真的生效) */
static uint8_t  si5351_r16;     /* CLK0 控制寄存器 (期望值见 _write_clk_ctrl) */
static uint8_t  si5351_r42;     /* MS0_P3[15:8] (期望 0x00) */
static uint8_t  si5351_r43;     /* MS0_P3[7:0]  (期望 0x01) */
static uint8_t  si5351_r44;     /* MS0: R-div|P1[17:16] (期望 0x00) */
static uint8_t  si5351_r45;     /* MS0_P1[15:8] (期望 0x01) */
static uint8_t  si5351_r46;     /* MS0_P1[7:0]  (期望 0x00) */
static uint8_t  si5351_r26;     /* PLLA_P3[15:8] (期望 0x00) */
static uint8_t  si5351_r29;     /* PLLA_P1[15:8] (期望 0x0A) */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
static void SI5351_HW_Init(void);
static uint8_t I2C_ScanBus(void);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* ── SI5351 硬件引脚初始化 ── */
  SI5351_HW_Init();

  /* ── I2C 总线扫描, 查找 SI5351 ──
   * 调试: 若在此处设断点, 查看 i2c_found_count / i2c_found_devices[]
   *       以及 i2c_last_error / i2c_tested_addr 排查 I2C 通信问题 */
  uint8_t si5351_dev_addr = I2C_ScanBus();
  if (si5351_dev_addr == 0) {
      /* 扫描未找到设备 — 可能原因:
       * 1. I2C 总线 SCL/SDA 没有上拉电阻 (需 4.7kΩ 到 3.3V)
       * 2. SI5351 模块未供电或 OEB/SSEN 引脚接错
       * 3. SCL/SDA 引脚接反
       * 4. I2C 时序配置有问题
       *
       * 先用默认地址 0x60 尝试, 同时在此处设断点查看调试变量 */
      si5351_dev_addr = 0x64;
  }

  /* ── SI5351 I2C 驱动初始化 ── */
  si5351_set_addr(si5351_dev_addr);
  si5351_init(&hi2c1);

  /* ── 配置 PLL VCO = 600MHz ──
   * SI5351B 只有一个 PLL (PLLA), 所有 8 路输出共用此 VCO
   */
  si5351_pll_config(600000000);

  /* 等待 PLL 锁定 (至少 10ms) */
  HAL_Delay(50);

  /* 回读 reg 0x00 检查 PLL 锁定和设备 ID:
   * [7]=SYS_INIT(1=初始化中), [5]=LOL_A(1=失锁), [3:0]=REVID */
  HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(si5351_dev_addr << 1), 0x00,
                   I2C_MEMADD_SIZE_8BIT, &si5351_reg00, 1, HAL_MAX_DELAY);
  si5351_pll_locked = ((si5351_reg00 & 0x20) == 0) ? 1 : 0;

  /* ── 配置 8 路时钟输出 ──
   * CLK0~5: 支持分数 MultiSynth, 范围 2.5kHz~200MHz
   * CLK6~7: 仅整数分频, 范围 28kHz~200MHz
   *
   * VCO=600MHz 时各通道频率配置:
   */
  si5351_clk_config(0, 2000000, SI5351_DRIVE_8MA);  /* 100 MHz */
  si5351_clk_config(1,  50000000, SI5351_DRIVE_8MA);  /*  50 MHz */
  si5351_clk_config(2,  25000000, SI5351_DRIVE_8MA);  /*  25 MHz */
  si5351_clk_config(3,  10000000, SI5351_DRIVE_8MA);  /*  10 MHz */
  si5351_clk_config(4,   5000000, SI5351_DRIVE_8MA);  /*   5 MHz */
  si5351_clk_config(5,   1000000, SI5351_DRIVE_8MA);  /*   1 MHz */
  si5351_clk_config(6,  10000000, SI5351_DRIVE_8MA);  /*  10 MHz (整数分频) */
  si5351_clk_config(7,     28000, SI5351_DRIVE_4MA);  /*  28 kHz (整数分频) */

  /* 输出使能由 OEB 硬件引脚控制, 不写 reg 0x03 (与参考驱动一致) */

  /* 等待 PLL 稳定 */
  HAL_Delay(50);

  /* 回读验证 */
  HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(si5351_dev_addr << 1), 0x03,
                   I2C_MEMADD_SIZE_8BIT, &si5351_reg03, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(si5351_dev_addr << 1), 0x00,
                   I2C_MEMADD_SIZE_8BIT, &si5351_reg00, 1, HAL_MAX_DELAY);
  si5351_pll_locked = ((si5351_reg00 & 0x20) == 0) ? 1 : 0;

  /* 回读关键寄存器, 确认 I2C 写入确实生效 */
  HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(si5351_dev_addr << 1), 16,
                   I2C_MEMADD_SIZE_8BIT, &si5351_r16, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(si5351_dev_addr << 1), 42,
                   I2C_MEMADD_SIZE_8BIT, &si5351_r42, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(si5351_dev_addr << 1), 43,
                   I2C_MEMADD_SIZE_8BIT, &si5351_r43, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(si5351_dev_addr << 1), 44,
                   I2C_MEMADD_SIZE_8BIT, &si5351_r44, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(si5351_dev_addr << 1), 45,
                   I2C_MEMADD_SIZE_8BIT, &si5351_r45, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(si5351_dev_addr << 1), 46,
                   I2C_MEMADD_SIZE_8BIT, &si5351_r46, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(si5351_dev_addr << 1), 26,
                   I2C_MEMADD_SIZE_8BIT, &si5351_r26, 1, HAL_MAX_DELAY);
  HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(si5351_dev_addr << 1), 29,
                   I2C_MEMADD_SIZE_8BIT, &si5351_r29, 1, HAL_MAX_DELAY);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* ── SI5351 硬件控制引脚初始化 ──
 * OEB (Output Enable Bar): 拉低使能全部输出
 * SSEN (Spread Spectrum Enable): 拉低禁用扩频
 *
 * 注意: 这两个引脚必须在 si5351_init() 之前初始化,
 *       否则即使 I2C 配置正确也没有时钟信号输出 */
static void SI5351_HW_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* 使能 GPIOE 时钟 (如果 CubeMX 尚未使能) */
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* OEB + SSEN: 推挽输出, 拉低 */
    GPIO_InitStruct.Pin   = SI5351_OEB_PIN | SI5351_SSEN_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(SI5351_OEB_PORT, &GPIO_InitStruct);

    /* 拉低: OEB=0 使能输出, SSEN=0 禁用扩频 */
    SI5351_OEB_LOW();
    SI5351_SSEN_LOW();

    /* 等待芯片上电稳定 (>300ms, 参考驱动) */
    HAL_Delay(300);
}

/* ── I2C 总线扫描 ──
 * 先尝试 SI5351 已知地址 (0x60/0x62), 再全量扫描 1~127
 * 返回 SI5351 地址, 未发现则返回 0
 * 调试变量 i2c_last_error / i2c_tested_addr 记录最后一次尝试的 HAL 返回值 */
static uint8_t I2C_ScanBus(void) {
    i2c_found_count = 0;

    /* 首先快速测试 SI5351 已知地址 */
    uint8_t known_addrs[] = {0x60, 0x62, 0x64};
    for (uint8_t i = 0; i < 3; i++) {
        i2c_tested_addr = known_addrs[i];
        i2c_last_error = HAL_I2C_IsDeviceReady(&hi2c1,
            (uint16_t)(known_addrs[i] << 1), 3, 10);
        if (i2c_last_error == HAL_OK) {
            i2c_found_devices[0] = known_addrs[i];
            i2c_found_count = 1;
            return known_addrs[i];
        }
    }

    /* 未找到已知地址, 全量扫描 */
    for (uint8_t addr = 1; addr < 128; addr++) {
        i2c_tested_addr = addr;
        i2c_last_error = HAL_I2C_IsDeviceReady(&hi2c1,
            (uint16_t)(addr << 1), 2, 5);
        if (i2c_last_error == HAL_OK) {
            if (i2c_found_count < 16) {
                i2c_found_devices[i2c_found_count++] = addr;
            }
        }
    }

    if (i2c_found_count > 0) return i2c_found_devices[0];
    return 0;
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

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

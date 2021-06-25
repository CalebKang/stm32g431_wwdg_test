/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32g4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
	uint32_t *stackpoint;
	stackpoint	= (uint32_t *)__current_sp();
	
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR0, 0x00000002);	//0:WWDG, 1:Hardfault, 2:NMI, 3:BusFault
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR1, __current_sp());	//SP
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR2, __return_address());	//LR
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR3, __current_pc());	//PC
	
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR4, *(stackpoint + 0));	//r0
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR5, *(stackpoint + 1));	//r1
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR6, *(stackpoint + 2));	//r2
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR7, *(stackpoint + 3));	//r3
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR8, *(stackpoint + 4));	//r12
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR9, *(stackpoint + 5));	//lr
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR10, *(stackpoint + 6));	//pc
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR11, *(stackpoint + 7));	//psr
	
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  uint32_t *stackpoint;
	stackpoint	= (uint32_t *)__current_sp();
	
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR0, 0x00000001);	//0:WWDG, 1:Hardfault, 2:NMI, 3:BusFault
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR1, __current_sp());	//SP
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR2, __return_address());	//LR
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR3, __current_pc());	//PC
	
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR4, *(stackpoint + 0));	//r0
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR5, *(stackpoint + 1));	//r1
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR6, *(stackpoint + 2));	//r2
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR7, *(stackpoint + 3));	//r3
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR8, *(stackpoint + 4));	//r12
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR9, *(stackpoint + 5));	//lr
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR10, *(stackpoint + 6));	//pc
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR11, *(stackpoint + 7));	//psr

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
	uint32_t *stackpoint;
	stackpoint	= (uint32_t *)__current_sp();
	
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR0, 0x00000003);	//0:WWDG, 1:Hardfault, 2:NMI, 3:BusFault
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR1, __current_sp());	//SP
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR2, __return_address());	//LR
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR3, __current_pc());	//PC
	
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR4, *(stackpoint + 0));	//r0
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR5, *(stackpoint + 1));	//r1
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR6, *(stackpoint + 2));	//r2
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR7, *(stackpoint + 3));	//r3
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR8, *(stackpoint + 4));	//r12
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR9, *(stackpoint + 5));	//lr
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR10, *(stackpoint + 6));	//pc
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR11, *(stackpoint + 7));	//psr

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */

  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles Window watchdog interrupt.
  */
void WWDG_IRQHandler(void)
{
  /* USER CODE BEGIN WWDG_IRQn 0 */
	uint32_t *stackpoint;
	stackpoint	= (uint32_t *)__current_sp();
	
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR0, 0x00000001);	//0:WWDG, 1:Hardfault, 2:NMI, 3:BusFault
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR1, __current_sp());	//SP
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR2, __return_address());	//LR
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR3, __current_pc());	//PC
	
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR4, *(stackpoint + 0));	//r0
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR5, *(stackpoint + 1));	//r1
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR6, *(stackpoint + 2));	//r2
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR7, *(stackpoint + 3));	//r3
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR8, *(stackpoint + 4));	//r12
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR9, *(stackpoint + 5));	//lr
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR10, *(stackpoint + 6));	//pc
	LL_RTC_BKP_SetRegister(RTC, LL_RTC_BKP_DR11, *(stackpoint + 7));	//psr

	while(1)
	{
	}
  /* USER CODE END WWDG_IRQn 0 */
  /* USER CODE BEGIN WWDG_IRQn 1 */

  /* USER CODE END WWDG_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

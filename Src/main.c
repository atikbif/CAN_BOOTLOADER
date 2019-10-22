/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "can.h"
#include "iwdg.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "can_tx_stack.h"
#include "can_cmd.h"
#include "crc.h"
#include "eeprom.h"
#include "flash_interface.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define FLASH_USER_START_ADDR   ADDR_FLASH_PAGE_256   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_PAGE_511 + FLASH_PAGE_SIZE - 1   /* End @ of user Flash area */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

typedef  void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t JumpAddress;

tx_stack can1_tx_stack;
tx_stack can2_tx_stack;
uint8_t group_id = 1;
uint8_t pos_in_group = 1;

static CAN_RxHeaderTypeDef   RxHeader;
static uint8_t               RxData[8];
static CAN_TxHeaderTypeDef   TxHeader;
static uint32_t              TxMailbox1=0;
static uint32_t              TxMailbox2=0;
static uint8_t               TxData[8];

extern uint8_t search_next_try;

struct {
	uint8_t id;
	uint8_t state;
	uint8_t data[8*7];
	uint8_t length;
	uint8_t cnt;
	uint8_t addr1;
	uint8_t addr2;
	uint8_t addr3;
	uint8_t addr4;
}boot_packet;

uint32_t FirstPage = 0, NbOfPages = 0, BankNumber = 0;
uint32_t Address = 0, PAGEError = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;

__IO uint16_t real_crc;
__IO uint16_t wr_crc;
__IO uint32_t pr_length;

uint8_t led_cnt = 0;

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static uint32_t getProjectLength() {
	uint32_t res = *(__IO uint8_t *)(FLASH_USER_START_ADDR+4);
	res <<= 8;res |=  *(__IO uint8_t *)(FLASH_USER_START_ADDR+5);
	res <<= 8;res |=  *(__IO uint8_t *)(FLASH_USER_START_ADDR+6);
	res <<= 8;res |=  *(__IO uint8_t *)(FLASH_USER_START_ADDR+7);
	uint32_t max_value = FLASH_USER_END_ADDR - FLASH_USER_START_ADDR-FLASH_PAGE_SIZE;
	if(res>max_value) res=max_value;
	return res;
}

static uint16_t getWrittenCRC() {
	uint16_t res = *(__IO uint8_t *)(FLASH_USER_START_ADDR+1);
	res <<= 8;res |=  *(__IO uint8_t *)(FLASH_USER_START_ADDR+0);
	return res;
}

static uint16_t getRealCRC() {
	uint16_t res = GetCRC16((uint8_t *)(FLASH_USER_START_ADDR+FLASH_PAGE_SIZE),pr_length);
	return res;
}

uint64_t conv_doubleword(uint64_t inp) {
	uint64_t res = 0;
	res |= inp&0xFF;
	res = res << 8; res |= (inp>>8)&0xFF;
	res = res << 8; res |= (inp>>16)&0xFF;
	res = res << 8; res |= (inp>>24)&0xFF;
	res = res << 8; res |= (inp>>32)&0xFF;
	res = res << 8; res |= (inp>>40)&0xFF;
	res = res << 8; res |= (inp>>48)&0xFF;
	res = res << 8; res |= (inp>>56)&0xFF;
	return res;
}

static void initCANFilter() {
	CAN_FilterTypeDef  sFilterConfig;

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;

	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

	sFilterConfig.FilterBank = 14;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 14;
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
}

static void check_can_rx(uint8_t can_num) {
	uint32_t addr = 0;
	uint8_t offset = 0;
	uint64_t data;
	CAN_HandleTypeDef *hcan;
	uint8_t i = 0;
	tx_stack_data packet;
	id_field *p_id;
	if(can_num==1) hcan = &hcan1; else hcan = &hcan2;
	if(HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0)) {
		if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
			//HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
			p_id = (id_field*)(&(RxHeader.ExtId));
			if(p_id->cmd==LAST_POINT) {
				if(p_id->group_addr == group_id) {
					search_next_try = 0;	// ответ от следующей в цепочке точки
				}
			}else if(p_id->cmd==FIND_NEXT_POINT) {
				if(p_id->param==FIND_REQUEST) {	// запрос от предыдущей в цепочке точки
					pos_in_group = RxData[0] + 1;
					next_point(FIND_ANSWER);	// отправить ответ
				}else if(p_id->param==FIND_ANSWER) {
					search_next_try = 0;	// ответ от следующей в цепочке точки
				}
			}else if(p_id->cmd==GET_POINTS_STATE) {
				group_id = p_id ->group_addr;
				send_point_state(1);
			}else if(p_id->cmd==GATE_STATE) {
				if(can_num==1) {
					group_id = p_id ->group_addr;
				}
			}
			if(p_id->point_addr == pos_in_group && p_id->group_addr == group_id && p_id->cmd==BOOT) {
				//HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
				if(p_id->type==BOOT_WRITE_HEADER) {
					boot_packet.state = 0x00;
					boot_packet.length = RxData[0];
					boot_packet.id = RxData[1];
					boot_packet.addr1 = RxData[2];
					boot_packet.addr2 = RxData[3];
					boot_packet.addr3 = RxData[4];
					boot_packet.addr4 = RxData[5];
					boot_packet.cnt = 0;

				}else if(p_id->type==BOOT_WRITE_DATA) {

					if(RxData[0]==boot_packet.id) {

						if(p_id->param>=1 && p_id->param<=8) {
							boot_packet.state |= 1<<(p_id->param-1);
							for(i=0;i<7;i++) {
								boot_packet.data[(p_id->param-1)*7+i] = RxData[i+1];
								boot_packet.cnt++;
							}
							if(boot_packet.cnt>=boot_packet.length) {

								// send answer and write data to flash
								send_write_ack(boot_packet.id);
								addr = boot_packet.addr1;
								addr = (addr << 8) | boot_packet.addr2;
								addr = (addr << 8) | boot_packet.addr3;
								addr = (addr << 8) | boot_packet.addr4;
								addr += FLASH_USER_START_ADDR;
								offset=0;
								while(offset<boot_packet.length && offset<56) {
									data = boot_packet.data[offset++];
									data=data<<8;if(offset<56) data|=boot_packet.data[offset++];
									data=data<<8;if(offset<56) data|=boot_packet.data[offset++];
									data=data<<8;if(offset<56) data|=boot_packet.data[offset++];
									data=data<<8;if(offset<56) data|=boot_packet.data[offset++];
									data=data<<8;if(offset<56) data|=boot_packet.data[offset++];
									data=data<<8;if(offset<56) data|=boot_packet.data[offset++];
									data=data<<8;if(offset<56) data|=boot_packet.data[offset++];

									data = conv_doubleword(data);
									HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, data);
									addr+=8;
								}
							}
						}
					}
				}else if(p_id->type==BOOT_ERASE_PAGE_REQ) {
					HAL_IWDG_Refresh(&hiwdg);
					addr = FLASH_USER_START_ADDR + FLASH_PAGE_SIZE*(p_id->param);
					EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
					EraseInitStruct.Banks       = GetBank(addr);
					EraseInitStruct.Page        = GetPage(addr);
					EraseInitStruct.NbPages     = 1;
					if(HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError)==HAL_OK) {
						send_erase_ack(p_id->param);
					}

				}else if(p_id->type==BOOT_PROG_FINISHED) {
					write_var(0);
					HAL_NVIC_SystemReset();
				}
			}

			if(p_id->group_addr == group_id) {
				if(p_id->cmd==FIND_NEXT_POINT) return;
			}

			if(!(p_id->point_addr == pos_in_group && p_id->group_addr == group_id)) {
				packet.id = RxHeader.ExtId;
				packet.length = RxHeader.DLC;
				for(i=0;i<packet.length;++i) packet.data[i] = RxData[i];
				if(can_num==1)	{
					add_tx_can_packet(&can2_tx_stack,&packet);
				}
				else {
					add_tx_can_packet(&can1_tx_stack,&packet);
				}
			}
		}
	}
}

void can_write_from_stack() {
	tx_stack_data packet;
	uint8_t i = 0;
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)!=0) {
		if(get_tx_can_packet(&can1_tx_stack,&packet)) {
			TxHeader.StdId = 0;
			TxHeader.ExtId = packet.id;
			TxHeader.RTR = CAN_RTR_DATA;
			TxHeader.IDE = CAN_ID_EXT;
			TxHeader.TransmitGlobalTime = DISABLE;
			TxHeader.DLC = packet.length;
			for(i=0;i<packet.length;++i) TxData[i] = packet.data[i];
			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox1);
		}else break;
	}
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan2)!=0) {
		if(get_tx_can_packet(&can2_tx_stack,&packet)) {
			TxHeader.IDE = CAN_ID_EXT;
			TxHeader.StdId = 0;
			TxHeader.ExtId = packet.id;
			TxHeader.RTR = CAN_RTR_DATA;
			TxHeader.IDE = CAN_ID_EXT;
			TxHeader.TransmitGlobalTime = DISABLE;
			TxHeader.DLC = packet.length;
			for(i=0;i<packet.length;++i) TxData[i] = packet.data[i];
			HAL_CAN_AddTxMessage(&hcan2, &TxHeader, TxData, &TxMailbox2);
		}else break;
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

  HAL_Delay(400);
  HAL_FLASH_Unlock();

    /* Clear OPTVERR bit set on virgin samples */
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);

  init_eeprom();
  pr_length = getProjectLength();
  real_crc = getRealCRC();
  wr_crc = getWrittenCRC();
  if(real_crc==wr_crc) {
	  uint64_t v = read_var();
	  if(v<5) {
		  write_var(++v);
		  HAL_Delay(100);
		  JumpAddress = *(__IO uint32_t*) (FLASH_USER_START_ADDR + FLASH_PAGE_SIZE + 4);
		  Jump_To_Application = (pFunction) JumpAddress;
		  __set_MSP(*(__IO uint32_t*) FLASH_USER_START_ADDR + FLASH_PAGE_SIZE);
		  Jump_To_Application();
	  }

  }

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM6_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  HAL_GPIO_WritePin(SDZ_GPIO_Port,SDZ_Pin,GPIO_PIN_RESET);




  /* Get the 1st page to erase */
  FirstPage = GetPage(FLASH_USER_START_ADDR);
  /* Get the number of pages to erase from 1st page */
  NbOfPages = GetPage(FLASH_USER_END_ADDR) - FirstPage + 1;
  /* Get the bank */
  BankNumber = GetBank(FLASH_USER_START_ADDR);
  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.Banks       = BankNumber;
  EraseInitStruct.Page        = FirstPage;
  EraseInitStruct.NbPages     = NbOfPages;

/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
   you have to make sure that these data are rewritten before they are accessed during code
   execution. If this cannot be done safely, it is recommended to flush the caches by setting the
   DCRST and ICRST bits in the FLASH_CR register. */
  //if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK) {

  //}

  initCANFilter();
  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {Error_Handler();}
  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {Error_Handler();}

  init_can_tx_stack(&can1_tx_stack);
  init_can_tx_stack(&can2_tx_stack);

  send_point_state(1);

  while (1)
  {
	  HAL_Delay(100);
	  HAL_IWDG_Refresh(&hiwdg);
	  if(led_cnt==0) HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
	  else if(led_cnt==1) HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
	  led_cnt++;if(led_cnt>=3) led_cnt=0;
	  //HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 80;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	check_can_rx(1);
	check_can_rx(2);
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

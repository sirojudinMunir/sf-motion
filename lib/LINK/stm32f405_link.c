#include "stm32f405_link.h"

// low level implementation on stm32f405rgt6

static uint32_t get_timer_clock(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1 || htim->Instance == TIM8 || 
      htim->Instance == TIM9 || htim->Instance == TIM10 || 
      htim->Instance == TIM11) {
    // Timer in APB2
    uint32_t pclk = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1)
      return pclk * 2;  // Timer clock doubled if APB prescaler != 1
    else
      return pclk;
  } else {
    // Timer in APB1
    uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1)
      return pclk * 2;
    else
      return pclk;
  }
}

void link_set_pwm_freq(TIM_HandleTypeDef *htim, uint32_t freq) {
  const uint32_t timer_clock = get_timer_clock(htim);

  // prescaler and period for center-aligned mode
  uint32_t prescaler = 0;
  uint32_t period = (timer_clock / (2 * freq)) - 1;

  if (period > 0xFFFF) {
    prescaler = period / 0xFFFF;
    period = (timer_clock / (2 * freq * (prescaler + 1))) - 1;
  }

  htim->Instance->PSC = prescaler;
  htim->Instance->ARR = period;
}

uint32_t link_get_max_pwm(TIM_HandleTypeDef *htim) {
  return htim->Instance->ARR;
}

int link_enable_pwm(TIM_HandleTypeDef *htim) {
  if (HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1) != HAL_OK) return -1;
  if (HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2) != HAL_OK) return -1;
  if (HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3) != HAL_OK) return -1;
  if (HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1) != HAL_OK) return -1;
  if (HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2) != HAL_OK) return -1;
  if (HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_3) != HAL_OK) return -1;
  // channel 4 is used to trigger the ADC
  if (HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4) != HAL_OK) return -1;
  htim->Instance->CCR4 = 10;
  return 0;
}

int link_disable_pwm(TIM_HandleTypeDef *htim) {
  htim->Instance->CCR1 = 0;
  htim->Instance->CCR2 = 0;
  htim->Instance->CCR3 = 0;
  htim->Instance->CCR4 = 0;
  if (HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_1) != HAL_OK) return -1;
  if (HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_2) != HAL_OK) return -1;
  if (HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_3) != HAL_OK) return -1;
  if (HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_1) != HAL_OK) return -1;
  if (HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_2) != HAL_OK) return -1;
  if (HAL_TIMEx_PWMN_Stop(htim, TIM_CHANNEL_3) != HAL_OK) return -1;
  if (HAL_TIM_PWM_Stop(htim, TIM_CHANNEL_4) != HAL_OK) return -1;
  return 0;
}

int link_enable_adc(ADC_HandleTypeDef *hadc) {
  if (HAL_ADCEx_InjectedStart_IT(hadc) != HAL_OK) return -1;
  return 0;
}

int link_disable_adc(ADC_HandleTypeDef *hadc) {
  if (HAL_ADCEx_InjectedStop_IT(hadc) != HAL_OK) return -1;
  return 0;
}

int link_write_flash(void *data, uint32_t len) {
  HAL_StatusTypeDef status;
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  
  HAL_FLASH_Unlock();

  EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector       = FLASH_SECTOR_NUM;
  EraseInitStruct.NbSectors    = 1;

  status = HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);
  if (status != HAL_OK) {
    HAL_FLASH_Lock();
    return -1;
  }

  uint32_t address = FLASH_SECTOR_ADDR;
  uint8_t *src = (uint8_t *)data;

  for (uint32_t i = 0; i < len; i += 4) {
    uint32_t word = *(uint32_t*)(src + i);
    status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, word);
    if (status != HAL_OK) {
      HAL_FLASH_Lock();
      return -1;
    }
    address += 4;
  }

  HAL_FLASH_Lock();
  return 0;
}

/******************************************************************************* */

extern CAN_HandleTypeDef hcan1;
static uint32_t TxMailbox;

// Filter
static void link_can_filter_config(CAN_HandleTypeDef *hcan) {
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0;
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x0000;
  canfilterconfig.FilterIdLow = 0x0000;
  canfilterconfig.FilterMaskIdHigh = 0x0000;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(hcan, &canfilterconfig);
}

int link_can_init(void) {
  link_can_filter_config(&hcan1);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  return 0;
}

int link_can_send_data(uint32_t id, uint8_t *data, uint32_t len) {
  CAN_TxHeaderTypeDef TxHeader;

  TxHeader.DLC = len;
  TxHeader.StdId = id;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;

  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, data, &TxMailbox) != HAL_OK) {
    return -1;
  }
  return 0;
}

int link_can_recv_data(uint32_t *id, uint8_t *data, uint32_t *len) {
  CAN_RxHeaderTypeDef rx_header;
  if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rx_header, data) == HAL_OK) {
    *id = rx_header.StdId;
    *len = rx_header.DLC;
    return 0;
  }
  return -1;
}

_Bool link_can_is_mailboxes_free(void) {
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
    return 1;
  }
  return 0;
}


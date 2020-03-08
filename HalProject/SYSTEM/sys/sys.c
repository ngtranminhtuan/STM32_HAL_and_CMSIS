#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//m?t s? ch?c nang
void GPIO_Set(GPIO_TypeDef* GPIOx,uint32_t  BITx,uint32_t  MODE,uint32_t PULL,uint32_t  OSPEED)
{  
		GPIO_InitTypeDef GPIO_InitStruct;
      
	
	if (GPIOx == GPIOA) {
		// Enable clock for GPIOA
		__HAL_RCC_GPIOA_CLK_ENABLE();

	} else if (GPIOx == GPIOB) {
		// Enable clock for GPIOB
		__HAL_RCC_GPIOB_CLK_ENABLE();

	} else if (GPIOx == GPIOC) {
		// Enable clock for GPIOC
		__HAL_RCC_GPIOC_CLK_ENABLE();

	}else if (GPIOx == GPIOD) {
		// Enable clock for GPIOD
		__HAL_RCC_GPIOD_CLK_ENABLE();

	}
		//else if (GPIOx == GPIOE) {
//		// Enable clock for GPIOE
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

//	}else if (GPIOx == GPIOF) {
//		// Enable clock for GPIOF
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);

//	}else if (GPIOx == GPIOG) {
//		// Enable clock for GPIOG
//		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);

//	}
	 
  GPIO_InitStruct.Pin = BITx;
  GPIO_InitStruct.Mode = MODE;
  GPIO_InitStruct.Speed = OSPEED;
	GPIO_InitStruct.Pull = PULL;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
        
} 

//********************************************************************************  


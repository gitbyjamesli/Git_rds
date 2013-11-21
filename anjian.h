/**********************************************************************







*/

#include "stm32f10x.h"

// 各按键io端口声明
#define KEY_DOWN_PIN      GPIO_Pin_0
#define KEY_DOWN_PORT     GPIOB
#define KEY_DOWN_CLK      RCC_APB2Periph_GPIOB

#define KEY_OK_PIN        GPIO_Pin_1
#define KEY_OK_PORT       GPIOB
#define KEY_OK_CLK        RCC_APB2Periph_GPIOB

#define KEY_UP_PIN        GPIO_Pin_2
#define KEY_UP_PORT       GPIOB
#define KEY_UP_CLK        RCC_APB2Periph_GPIOB

#define KEY_LEFT_PIN      GPIO_Pin_12
#define KEY_LEFT_PORT     GPIOB
#define KEY_LEFT_CLK      RCC_APB2Periph_GPIOB

#define KEY_RIGHT_PIN     GPIO_Pin_5
#define KEY_RIGHT_PORT    GPIOC
#define KEY_RIGHT_CLK     RCC_APB2Periph_GPIOC

#define KEY_MENU_PIN      GPIO_Pin_4
#define KEY_MENU_PORT     GPIOC
#define KEY_MENU_CLK      RCC_APB2Periph_GPIOC
 
#define KEY_RDS_PIN       GPIO_Pin_7
#define KEY_RDS_PORT      GPIOA
#define KEY_RDS_CLK       RCC_APB2Periph_GPIOA


#define KEY_DOWN_CLEAR()        EXTI_ClearFlag(EXTI_Line0)			 // 清下降沿标志
#define KEY_OK_CLEAR()          EXTI_ClearFlag(EXTI_Line1)
#define KEY_UP_CLEAR()          EXTI_ClearFlag(EXTI_Line2)
#define KEY_LEFT_CLEAR()        EXTI_ClearFlag(EXTI_Line12)
#define KEY_RIGHT_CLEAR()       EXTI_ClearFlag(EXTI_Line5)
#define KEY_MENU_CLEAR()        EXTI_ClearFlag(EXTI_Line4)
#define KEY_RDS_CLEAR()         EXTI_ClearFlag(EXTI_Line7)

extern FlagStatus UP_LockFlag;
extern FlagStatus UP_DelayEnd;
extern uint16_t   UP_StartDelayct;
extern uint16_t   UP_Delayct;

extern FlagStatus DOWN_DelayEnd;
extern FlagStatus DOWN_LockFlag;
extern uint16_t   DOWN_StartDelayct;
extern uint16_t   DOWN_Delayct;

extern FlagStatus LEFT_DelayEnd;
extern FlagStatus LEFT_LockFlag;
extern uint16_t   LEFT_StartDelayct;
extern uint16_t   LEFT_Delayct;

extern FlagStatus RIGHT_DelayEnd;
extern FlagStatus RIGHT_LockFlag;
extern uint16_t   RIGHT_StartDelayct;
extern uint16_t   RIGHT_Delayct;



extern FlagStatus KEY_DOWN_STATUS(void);// 返回SET表示按键被按下过，反之为RESET
extern FlagStatus KEY_OK_STATUS(void);
extern FlagStatus KEY_UP_STATUS(void);
extern FlagStatus KEY_LEFT_STATUS(void);
extern FlagStatus KEY_RIGHT_STATUS(void);
extern FlagStatus KEY_MENU_STATUS(void);
extern FlagStatus KEY_RDS_STATUS(void);
extern FlagStatus AUX_INPUT_STATUS(void);
extern FlagStatus MP3_INPUT_STATUS(void);
extern FlagStatus DTU_INPUT_STATUS(void);

extern void KEY_UP_delayct(void);// 长按持续计数, 在定时器函数里调用，间隔10ms
extern void KEY_DOWN_delayct(void);
extern void KEY_LEFT_delayct(void);
extern void KEY_RIGHT_delayct(void);

// FILE END------------------------------------------------------------------

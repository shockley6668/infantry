#ifndef BSP_LED_H
#define BSP_LED_H
#include "struct_typedef.h"

/**
  * @brief          aRGB show
  * @param[in]      aRGB: 0xaaRRGGBB, 'aa' is alpha, 'RR' is red, 'GG' is green, 'BB' is blue
  * @retval         none
  */
/**
  * @brief          显示RGB
  * @param[in]      aRGB:0xaaRRGGBB,'aa' 是透明度,'RR'是红色,'GG'是绿色,'BB'是蓝色
  * @retval         none
  */
extern void aRGB_led_show(uint32_t aRGB);
extern void led_init();
extern void led_flow();
extern void led_off_flow();
extern void led_task(void const *pvParameters);
extern void led_dashboard();
extern void led_light(uint8_t led_1,uint8_t led_2,uint8_t led_3,uint8_t led_4,uint8_t led_5,uint8_t led_6,uint8_t led_7);

#define DELAY_TIME 100
#define DELAY_LONG_TIME 200

#define CHASSIS_TASK_INIT_TIME 357
//PWM端的 io口
#define LED1_PORT GPIOE
#define LED1_PIN GPIO_PIN_9
#define LED2_PORT  GPIOE
#define LED2_PIN GPIO_PIN_11
#define LED3_PORT  GPIOE
#define LED3_PIN  GPIO_PIN_13
#define LED4_PORT GPIOC
#define LED4_PIN GPIO_PIN_6
#define LED5_PORT GPIOI
#define LED5_PIN GPIO_PIN_6
#define LED6_PORT GPIOI
#define LED6_PIN GPIO_PIN_7

//用户接口端的
#define LED7_PORT GPIOB
#define LED7_PIN GPIO_PIN_14

typedef enum{
    ON_FLOW=0,
    OFF_FLOW,
    AUTO_AIM,
    SHOOT,
    CAP,
    SPIN,

    MODE_NUM
}led_mode_e;

typedef struct {
    led_mode_e mode;


}led_t;



#endif

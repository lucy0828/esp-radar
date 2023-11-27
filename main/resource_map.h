#include "driver/gpio.h"

/* sensor SPI interface */
#define PIN_XENSIV_BGT60TRXX_SPI_CSN        GPIO_NUM_4
#define PIN_XENSIV_BGT60TRXX_SPI_SCLK       GPIO_NUM_5
#define PIN_XENSIV_BGT60TRXX_SPI_MOSI       GPIO_NUM_19
#define PIN_XENSIV_BGT60TRXX_SPI_MISO       GPIO_NUM_21
/* sensor interrupt output pin */
#define PIN_XENSIV_BGT60TRXX_IRQ            GPIO_NUM_33
/* sensor HW reset pin */
#define PIN_XENSIV_BGT60TRXX_RSTN           GPIO_NUM_27
/* enable 1V8 LDO on radar wingboard*/
#define PIN_XENSIV_BGT60TRXX_LDO_EN         GPIO_NUM_14
#define PIN_LED_RED                         GPIO_NUM_26
#define PIN_LED_GREEN                       GPIO_NUM_25
#define PIN_LED_BLUE                        GPIO_NUM_34
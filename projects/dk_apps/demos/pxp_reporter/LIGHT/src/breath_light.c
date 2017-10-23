#include <hw_breath.h>
#include "hw_led.h"



// level
// 1 2 初始化 并 开启 light1
// 其他 就是关闭 light1
void breath_light_init_start_stop(uint8_t level)
{

        breath_config config = {
                .dc_min = 0,
                .dc_max = 255,
                .freq_div = 255,
                .polarity = HW_BREATH_PWM_POL_POS
        };

        switch (level) {
        case 1:
                config.dc_step = 96;
                break;
        case 2:
                config.dc_step = 32;
                break;
        default:
                /* Simply disable breath timer and return */
                hw_breath_disable();
                return;
        }

        /* Configure and enable breath timer */
        hw_breath_init(&config);
        hw_led_set_led1_src(HW_LED_SRC1_BREATH);
        hw_led_enable_led1(true);
        hw_breath_enable();
}

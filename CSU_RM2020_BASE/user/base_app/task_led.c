#include "task_led.h"
#include "headfile.h"
#include "task_Ano.h"
#include "imu_packet.h"
#include "imu_data_decode.h"
#include "oled.h"

#if defined USE_NEW_OLED
MLED_DEFINE(led1, D, 10, 0);
MLED_DEFINE(led2, D, 11, 0);
MLED_DEFINE(beep, D, 7, 0);
#else
MLED_DEFINE(led1, B, 3, 0);
MLED_DEFINE(led2, B, 1, 0);
MLED_DEFINE(led3, E, 2, 0);
#endif
void	task_led_sys(void* param)
{
	#if defined USE_NEW_OLED
	mled_init(&led1);
    mled_init(&led2);
	mled_init(&beep);
	while(1)
    {
		mled_on(&led1);
        mled_off(&led2);
		mled_on(&beep);
        task_delay_ms(1000);
		
		mled_off(&led1);
        mled_on(&led2);
		mled_off(&beep);
        task_delay_ms(1000);
		
		mled_on(&led1);
        mled_on(&led2);
		mled_on(&beep);
        task_delay_ms(1000);
		
		mled_off(&led1);
        mled_on(&led2);
		mled_off(&beep);
        task_delay_ms(1000);
		
		mled_on(&led1);
        mled_off(&led2);
		mled_on(&beep);
        task_delay_ms(1000);
		
		mled_off(&led1);
        mled_off(&led2);
		mled_off(&beep);
        task_delay_ms(1000);
	}
	
	#else
    mled_init(&led1);
    mled_init(&led2);
    mled_init(&led3);
//    task_insert(task_led_user, NULL, 4);

    while(1)
    {
        mled_off(&led1);
        mled_on(&led2);
        mled_on(&led3);
        task_delay_ms(1000);

        mled_on(&led1);
        mled_off(&led2);
        mled_on(&led3);
        task_delay_ms(1000);

        mled_on(&led1);
        mled_on(&led2);
        mled_off(&led3);
        task_delay_ms(1000);

        mled_on(&led1);
        mled_off(&led2);
        mled_off(&led3);
        task_delay_ms(1000);

        mled_off(&led1);
        mled_on(&led2);
        mled_off(&led3);
        task_delay_ms(1000);

        mled_off(&led1);
        mled_off(&led2);
        mled_on(&led3);
        task_delay_ms(1000);

        mled_off(&led1);
        mled_off(&led2);
        mled_off(&led3);
		task_delay_ms(1000);
    }
	#endif
}

U32	led_user_period	= 1200;

void	led_user_control(U32 ms)
{
    led_user_period = ms;
}

void app_key_test(void * p_param)
{
    printf("app_key1 has been down\r\n");
}

void app_key_test2(void * p_param)
{
    printf("app_key2 has been down\r\n");
}

void app_bar_test(void * p_param)
{
    U32 data;

    data = TO_U32(p_param);

    printf("the bar is %d\r\n", data);
}

void	task_led_user(void* param)
{
    p_mled	led_user = param;
    resolve_add((u8*)"led_user", (resolve_f)led_user_control);
    resolve_add(APP_KEY1, app_key_test);
    resolve_add(APP_KEY2, app_key_test2);
    resolve_add(APP_BAR1, app_bar_test);

    while(1)
    {
        if((led_user_period > 50) && (led_user_period < 10000))
        {
            mled_toggle(led_user);
            task_delay_ms(led_user_period);
        }
        else if(led_user_period == 0)
        {
            mled_off(led_user);
            task_delay_ms(1000);
        }
        else
        {
            PRINT("user led period should bewteen 50 and 10000(ms).\r\n");
            led_user_period	= 0;
        }
		task_delay_ms(10);
    }
}


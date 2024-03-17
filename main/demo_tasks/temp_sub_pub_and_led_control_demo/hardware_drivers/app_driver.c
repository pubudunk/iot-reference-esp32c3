#include <stdio.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "app_driver.h"
#include "esp_idf_version.h"
#include "dht11.h"
#include "driver/gpio.h"

static const char * TAG = "app_driver";

#define GRI_LED_GPIO    CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_GPIO_NUMBER

#ifdef CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_RMT
    #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL( 5, 0, 0 )
        static led_strip_handle_t led_strip;
    #else
        static led_strip_t * led_strip;
    #endif
#endif

#if ESP_IDF_VERSION == ESP_IDF_VERSION_VAL( 4, 3, 0 )
    #ifndef CONFIG_IDF_TARGET_ESP32
        #define APP_SOC_TEMP_SENSOR_SUPPORTED
    #else
        #define APP_SOC_TEMP_SENSOR_SUPPORTED    SOC_TEMP_SENSOR_SUPPORTED
    #endif
#endif

static esp_err_t temperature_sensor_init()
{
    #ifdef APP_SOC_TEMP_SENSOR_SUPPORTED
        /* Initialize touch pad peripheral, it will start a timer to run a filter */
        ESP_LOGI( TAG, "Initializing Temperature sensor" );
        temp_sensor_config_t temp_sensor = TSENS_CONFIG_DEFAULT();
        temp_sensor_get_config( &temp_sensor );
        ESP_LOGD( TAG, "default dac %d, clk_div %d", temp_sensor.dac_offset, temp_sensor.clk_div );
        temp_sensor.dac_offset = TSENS_DAC_DEFAULT; /* DEFAULT: range:-10℃ ~  80℃, error < 1℃. */
        temp_sensor_set_config( temp_sensor );
        return( temp_sensor_start() );
    #else
        /* For the SoCs that do not have a temperature sensor (like ESP32) we report a dummy value. */
        return ESP_OK;
    #endif /* ifdef APP_SOC_TEMP_SENSOR_SUPPORTED */
}

static esp_err_t led_init()
{
    esp_err_t ret = ESP_FAIL;

    #ifdef CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_RMT
        #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL( 5, 0, 0 )
            led_strip_config_t strip_config =
            {
                .strip_gpio_num = GRI_LED_GPIO,
                .max_leds       = 1, /* at least one LED on board */
            };
            led_strip_rmt_config_t rmt_config =
            {
                .resolution_hz = 10 * 1000 * 1000, /* 10MHz */
            };
            ret = led_strip_new_rmt_device( &strip_config, &rmt_config, &led_strip );
        #else /* if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL( 5, 0, 0 ) */
            led_strip = led_strip_init( 0, GRI_LED_GPIO, 1 );
        #endif /* if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL( 5, 0, 0 ) */
    #elif CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_GPIO
        ret = gpio_reset_pin( GRI_LED_GPIO );
        ret |= gpio_set_direction( GRI_LED_GPIO, GPIO_MODE_OUTPUT );
    #endif /* ifdef CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_RMT */
    ret |= app_driver_led_on();
    return ret;
}

esp_err_t app_driver_init()
{
    esp_err_t temp_sensor_ret, led_ret;

    temp_sensor_ret = temperature_sensor_init();
    led_ret = led_init();

    if( temp_sensor_ret && led_ret )
    {
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}

esp_err_t app_driver_led_on()
{
    esp_err_t ret = ESP_FAIL;

    #ifdef CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_RMT
        #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL( 5, 0, 0 )
            ret = led_strip_set_pixel( led_strip, 0, 0, 25, 0 );
            /* Refresh the strip to send data */
            ret |= led_strip_refresh( led_strip );
        #else
            led_strip->set_pixel( led_strip, 0, 0, 25, 0 );
            led_strip->refresh( led_strip, 100 );
        #endif
    #elif CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_GPIO
        ret = gpio_set_level( GRI_LED_GPIO, 1 );
    #endif /* ifdef CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_RMT */
    return ret;
}

esp_err_t app_driver_led_off()
{
    esp_err_t ret = ESP_FAIL;

    #ifdef CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_RMT
        #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL( 5, 0, 0 )
            ret = led_strip_clear( led_strip );
        #else
            led_strip->clear( led_strip, 50 );
        #endif
    #elif CONFIG_GRI_TEMPERATURE_PUB_SUB_AND_LED_CONTROL_DEMO_LED_GPIO
        ret = gpio_set_level( GRI_LED_GPIO, 0 );
    #endif
    return ret;
}

/* PK: Added */

extern uint8_t ucButtonStatus;

#define PUSH_BTN_GPIO_PIN     19
#define GPIO_INPUT_PIN_SEL  ( 0x01 << PUSH_BTN_GPIO_PIN )

enum debounce_state {
    STATE_IDLE,
    STATE_DEBOUNCE,
    STATE_REPORT
};

/* Task to read button status and debounce */
void vPushButtonRead( void * pvParameters )
{
    enum debounce_state eState = STATE_IDLE;
    uint8_t ucVal = 0;
    uint8_t ucCount = 0, ucPressed = 0, ucReleased = 0;

    for( ;; )
    {
        switch(eState) {
            case STATE_IDLE:
                ucVal = gpio_get_level(PUSH_BTN_GPIO_PIN);
                
                /* if button is pressed move to debounce state */
                if(1 == ucVal) {
                    eState = STATE_DEBOUNCE;
                    ucCount = 0;
                    ucPressed = 0;
                    ucReleased = 0;
                    //ESP_LOGI( TAG, "STATE_IDLE: button pressed" );
                }
            break;
            case STATE_DEBOUNCE:
                /* Read value for 6 iterations and record pushed vs released */
                ucVal = gpio_get_level(PUSH_BTN_GPIO_PIN);
                if(1 == ucVal)
                    ucPressed++;
                else
                    ucReleased++;

                ucCount++;
                if(ucCount == 10) 
                    eState = STATE_REPORT;

                //ESP_LOGI( TAG, "STATE_DEBOUNCE" );
            break;
            case STATE_REPORT:
                /* report as button pressed if more than 50% of iteration are button pressed */
                if(ucPressed >= 5) {
                    if(ucButtonStatus)
                        ucButtonStatus = 0;
                    else
                        ucButtonStatus = 1;
                }
                eState = STATE_IDLE;
                ESP_LOGI( TAG, "STATE_REPORT: button state %d", ucButtonStatus );
            break;
            default:
            break;
        }
        vTaskDelay(1);
    }
}


#if 0
static void IRAM_ATTR push_btn_isr_handler(void* arg)
{
    ESP_DRAM_LOGI( TAG, "button pressed" );

    /* not worrying about race condition for now */
    if(ucButtonStatus)
        ucButtonStatus = 0;
    else
        ucButtonStatus = 1;
}
#endif

esp_err_t app_driver_hw_init(void)
{
    /* configure interrupt on GPIO 19 - button */

    gpio_config_t io_conf = {};
    BaseType_t xReturned;
#if 0
    /* interrupt of rising edge */
    io_conf.intr_type = GPIO_INTR_POSEDGE;
#endif
    /* bit mask of the pins  */
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    /* set as input mode */
    io_conf.mode = GPIO_MODE_INPUT;
    /* enable pull-down mode */
    io_conf.pull_down_en = 1;
    gpio_config(&io_conf);

    xReturned = xTaskCreate(vPushButtonRead, "PushButtonTask", 2048, NULL, 0, NULL);
    
    configASSERT( pdPASS == xReturned);
    if( pdPASS != xReturned ) {
        ESP_LOGE( TAG, "task creation failed" );
    }

#if 0
    /* install gpio isr service */
    gpio_install_isr_service(0);
    /* hook isr handler for specific gpio pin */
    gpio_isr_handler_add(PUSH_BTN_GPIO_PIN, push_btn_isr_handler, NULL);
#endif

    return ESP_OK;
}

float app_driver_temp_sensor_read_celsius()
{
    #ifdef APP_SOC_TEMP_SENSOR_SUPPORTED
        float tsens_out;
        temp_sensor_read_celsius( &tsens_out );
        return tsens_out;
    #else
        /* For the SoCs that do not have a temperature sensor (like ESP32) we report a dummy value. */
        return 0.0f;
    #endif
}

static float fTemp = 0.0f;
static uint8_t ucHumidity = 0;

void app_driver_temp_sensor_read_values(void)
{
    /* PK: Added DH11 support*/
    getData();
    fTemp = getFtemp();
    ucHumidity = getHumidity();
}

uint8_t app_driver_temp_sensor_get_temperature(void)
{
    /* PK: Added DH11 support*/
   return fTemp;
}

uint8_t app_driver_temp_sensor_get_humidity(void)
{
    /* PK: Added DH11 support*/
   return ucHumidity;
}
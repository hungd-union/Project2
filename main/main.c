#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <sdkconfig.h>
#include <stdbool.h>
#include "esp_adc/adc_oneshot.h"
#include "math.h"



#define greenLED_PIN    16        
#define redLED_PIN      15      
#define ignitionButton  4
#define driveSeatBelt   5
#define passengerSeatBelt   6
#define driveSeatSense  1
#define passengerSeatSense 2
#define Alarm 17
#define leftLamp 18
#define rightLamp 8


bool dSense = false;
bool dsbelt = false;
bool pSense = false;
bool psbelt = false;
bool engine = false;
bool hold = false;

int delayMS = 10; //ms
int off = 0; //mV
int middle = 3100 //mv Anything over 3000
int middle2 = 1620 //
//Off is anything less than 1000
//1000 to 2000 is on
//2000 is a auto
//1300 and 500
//1300 is daylight
//500 is dusk 
/**
 * returns a boolean determining whether all of the car alarms systems have been satisifed ie:
 * driver seat belt, driver is seated etc.
 */
bool enable(void){

    bool dslvl = gpio_get_level(driveSeatSense);
    bool dsbeltlvl = gpio_get_level(driveSeatBelt);
    bool pslvl = gpio_get_level(passengerSeatSense);
    bool psbltlvl = gpio_get_level(passengerSeatBelt);

    if (!dslvl){
        dSense = true; //Driver sensor
    }
    else{dSense = false;}

    if (!dsbeltlvl){
        dsbelt = true; // driver seatbelt sensor
    }
    else{dsbelt = false;}

    if (!pslvl){
        pSense = true; // passenger seat level
    }
    else{pSense = false;}

    if (!psbltlvl){
        psbelt = true; // passenger seatbelt level
    }
    else{psbelt = false;}

    bool IgnitReady = dSense && dsbelt && pSense && psbelt;
    return IgnitReady;
    }

    /**
     * Will configure all of the pins within the design resetting all of the pins within the design to a known state
     * setting the direction to either input or output 
     * enabling pullup resistors within the ESP
     * And finally setting all of the output pins to zero
     */

bool ignitionPressed (void) {
    bool igniteHold = gpio_get_level(ignitionButton) == 0;
    if (igniteHold) {
        hold = true;
    }
    if (hold && !igniteHold)
    {
        hold = false;
        return true;
    }
    return false;
}


void pinConfig(void){
    gpio_reset_pin(greenLED_PIN);
    gpio_reset_pin(redLED_PIN);
    gpio_reset_pin(leftLamp);
    gpio_reset_pin(rightLamp);
    gpio_reset_pin(ignitionButton);
    gpio_reset_pin(driveSeatBelt);
    gpio_reset_pin(passengerSeatBelt);
    gpio_reset_pin(driveSeatSense);
    gpio_reset_pin(passengerSeatSense);
    gpio_reset_pin(Alarm);

    gpio_set_direction(greenLED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(redLED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(leftLamp, GPIO_MODE_OUTPUT);
    gpio_set_direction(rightLamp, GPIO_MODE_OUTPUT);
    gpio_set_direction(Alarm, GPIO_MODE_OUTPUT);
    gpio_set_direction(ignitionButton, GPIO_MODE_INPUT);
    gpio_set_direction(driveSeatBelt, GPIO_MODE_INPUT);
    gpio_set_direction(passengerSeatBelt, GPIO_MODE_INPUT);
    gpio_set_direction(driveSeatSense, GPIO_MODE_INPUT);
    gpio_set_direction(passengerSeatSense, GPIO_MODE_INPUT);

    gpio_pullup_en(ignitionButton);
    gpio_pullup_en(driveSeatBelt);
    gpio_pullup_en(driveSeatSense);
    gpio_pullup_en(passengerSeatBelt);
    gpio_pullup_en(passengerSeatSense);

    gpio_set_level(greenLED_PIN, 0);
    gpio_set_level(redLED_PIN, 0);
    gpio_set_level(Alarm, 0);

}


void adcConfig(void) {
    //ADC Config LightSensor
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };                                                  // Unit configuration
    adc_oneshot_unit_handle_t adc1_handle;              // Unit handle
    adc_oneshot_new_unit(&init_config1, &adc1_handle);  // Populate unit handle
   
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };                                                  // Channel config
    adc_oneshot_config_channel                          // Configure the chan
    (adc1_handle, LightSensorAdcChannel, &config);

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = LightSensorAdcChannel,
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };                                                  // Calibration config
    adc_cali_handle_t adc1_cali_chan_handle;            // Calibration handle
    adc_cali_create_scheme_curve_fitting                // Populate cal handle
    (&cali_config, &adc1_cali_chan_handle);


    //ADC Config Potentiometer
    adc_oneshot_unit_init_cfg_t init_config1 = {        
        .unit_id = ADC_UNIT_1,
    };                                                  // Unit configuration
    adc_oneshot_unit_handle_t adc1_handle;              // Unit handle
    adc_oneshot_new_unit(&init_config1, &adc1_handle);  // Populate unit handle
   
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };                                                  // Channel config
    adc_oneshot_config_channel                          // Configure the chan
    (adc1_handle, ModeAdcChannel, &config);
    
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ModeAdcChannel,
        .atten = ADC_ATTEN,
        .bitwidth = BITWIDTH
    };                                                  // Calibration config
    adc_cali_handle_t adc1_cali_chan_handle;            // Calibration handle
    adc_cali_create_scheme_curve_fitting                // Populate cal handle
    (&cali_config, &adc1_cali_chan_handle);

}

void app_main(void) {
    pinConfig();
    adcConfig();
    bool initial_message = true;
    while(1){
        bool ignitEn = ignitionPressed();
        if (!engine) {
            bool ready = enable();

            if (dSense && initial_message){
                printf("Welcome to enhanced Alarm system model 218 -W25\n");
                initial_message = false;

            }

            if(ready){
                gpio_set_level(greenLED_PIN, 1);
            }
            else {
                gpio_set_level(greenLED_PIN, 0);
            }

            if(ignitEn){
                if (ready) {
                    printf("Starting the engine.\n");
                    gpio_set_level(greenLED_PIN, 0);
                    gpio_set_level(redLED_PIN, 1);
                    engine = true;
                }

                else {
                    gpio_set_level (Alarm, 1);

                    if (!dSense){
                    printf("Driver seat not occupied\n");
                    }
                
                    if (!dsbelt){
                    printf("Driver seatbelt not fastened\n");
                    }

                    if (!pSense){
                    printf("Passenger seat not occupied\n");
                    }

                    if (!psbelt){
                    printf("Passenger seatbelt not fastened\n");
                    }
                    vTaskDelay (3000/ portTICK_PERIOD_MS);
                }
            }
            else {
                gpio_set_level (Alarm, 0);
            }
        }

        else {
            if (ignitEn) {
                gpio_set_level (redLED_PIN, 0);
                printf("Stopping the engine.\n");
                engine = false;
            }

            else {
                
            }

        }

    vTaskDelay(delayMS / portTICK_PERIOD_MS);
    }
}
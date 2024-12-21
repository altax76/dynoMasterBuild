#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <string.h>
#include <stdlib.h>
#include "driver/adc.h"
#include "driver/dac.h"
#include "esp_timer.h"
#include "driver/ledc.h"
#include "driver/timer.h"
#include <math.h>




#define LED_GPIO_PIN GPIO_NUM_2

//#define REFRESH_RATE_MS 100                 // this should not be used in the final program

//static const char *TAG = "PulseCounter";    //this should not be used in the final program

#define PULSE_PIN 4                 // pulse pin 4
#define PCNT_UNIT PCNT_UNIT_0       //using this channel
#define PCNT_H_LIM_VAL 60           //pulse count high limit
#define PCNT_L_LIM_VAL -1           //this should never be hit
//#define FILTER_LENGTH 10            // Filter length in clock cycles (1ms debounce)
#define FILTER_LENGTH 100

#define HX711_DOUT_PIN  GPIO_NUM_33 
#define HX711_SCK_PIN   GPIO_NUM_32 

int64_t current_time;
double delta_t;
double current_rpm = 10000;
int64_t current_rpm_time_stamp;
int64_t previous_rpm_time_stamp;

double servo_update_rate = 10;

char serial_input[100] = "";

volatile int strain_value = 0;

int brake_voltage = 60; //voltage set to 0 until calibrated

bool test_state = false;
bool measure_test_state = false;
bool tach_test_state = false;
bool pau_test_state = false;
bool rpm_state;
bool apply_brake_state = false;
bool pau_voltage_test_state = false;

int target_rpm = 1000;

//Coefficients for z-domain backward euler descrete time PID Algorithm
volatile double a0 = 0;
volatile double a1 = 0;
volatile double a2 = 0;
volatile double b0 = 0;
volatile double b1 = 0;
volatile double b2 = 0;

//Current and past error
volatile double Ik = 0;
volatile double Ik1 = 0;
volatile double Ik2 = 0;

//Current and past outputs
volatile double Ok = 0;
volatile double Ok1 = 0;
volatile double Ok2 = 0;

//global PAU OUTPUT for PID Algorithm
double pau_output = 0.0;

//Ramp for PAU output
double ramp = 0;

int pau_test_voltage = 0;

//emergency voltage to PAU
float eOutput = 79; //set to 79 to apply 1.02v to pau

//for configuration
volatile double pau_emergency_voltage = 0;

bool requested_data = false;


//this was just made to simplify code, it really doesnt need to be in here.
int64_t getTime() {
    int64_t completion_time = esp_timer_get_time();
    return completion_time;
}


// Interrupt handler function for PCNT
static void IRAM_ATTR pcnt_isr_handler(void *arg) {
    uint32_t status = 0;

    pcnt_get_event_status(PCNT_UNIT, &status);                                      // Get the event status

    if (status & PCNT_EVT_H_LIM) {                                                  // Check if high limit event occurred
        int64_t previous_time = current_time;                                       //record the new time
        current_time = esp_timer_get_time();                                        //get the current time
        int64_t time_between_interupts = current_time - previous_time;              //check the time between 60 pulses
        delta_t = (double)time_between_interupts/1000000;                           //convert to seconds
        current_rpm = PCNT_H_LIM_VAL/delta_t;                                                   //converting delta_t to rpm, there are 60 counts per pulse
        current_rpm_time_stamp = getTime();                                         //record the current time stamp
        pcnt_counter_clear(PCNT_UNIT);                                              // Clear the counter after 60 pulses        
    }
}


void pcnt_example_init(void) {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = PULSE_PIN,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_UNIT,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
    };

    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(PCNT_UNIT, FILTER_LENGTH);
    pcnt_filter_enable(PCNT_UNIT);
    
    // Enable interrupt on high limit
    pcnt_event_enable(PCNT_UNIT, PCNT_EVT_H_LIM);

    // Install the ISR service
    pcnt_isr_service_install(0);  // 0 is the interrupt priority (default)
    pcnt_isr_handler_add(PCNT_UNIT, pcnt_isr_handler, NULL);  // Attach the ISR handler

    pcnt_counter_pause(PCNT_UNIT);
    pcnt_counter_clear(PCNT_UNIT);
    pcnt_counter_resume(PCNT_UNIT);
}

bool check_string_start(const char *str, const char *prefix) {
    return strncmp(str, prefix, strlen(prefix)) == 0;
}

void remove_prefix(char *str, const char *prefix) {
    size_t prefix_len = strlen(prefix);
    if (strncmp(str, prefix, prefix_len) == 0) {
        memmove(str, str + prefix_len, strlen(str) - prefix_len + 1);
    }
}

void serial_output_task(void *arg) {
    while (1) {
        
        //This sends continuous raw strain value to test the strain gauge in configuration
        if (measure_test_state) {
            printf("w%d\n",strain_value);
        }

        //This sents continuous RPM value to test the tachometer in configuration
        if (tach_test_state) {
            printf("w%f\n", current_rpm);
        }

        //This is turned on in steady state tuning and sends, the current time, rpm, raw strain, unfiltered PID output, and filtered PID output
        if (requested_data) {
            printf("s%f %d %d %d %d %d\n", ((double)current_rpm_time_stamp/1000000), (int)current_rpm, strain_value, (int)Ok, (int)pau_output, (int)target_rpm);
            requested_data = false;
        }
        if (rpm_state) {
            if (previous_rpm_time_stamp != current_rpm_time_stamp) {                                //check if there has been a new rpm value sent, so that repeated values are not sent.
                printf("s%f %f\n", (double)current_rpm, ((double)current_rpm_time_stamp/1`00000));     // send the current rpm, and time stamp
            }  
            previous_rpm_time_stamp = current_rpm_time_stamp;                                       // reset the check for repeats
        }


        vTaskDelay(pdMS_TO_TICKS(10)); // all this data is send every 10ms
    }
}


//All of the comunication between the ESP32 and the high end software is done through the serial monitor
void serial_input_task(void *pvParameters) {
    char serial_input[100]; // Adjust size as needed
    while (1) {

        //First check if the high end software user has input an new line through the serial monitor, if there is a new input, continue in the if statement to check what it might have been
        if (fgets(serial_input, sizeof(serial_input), stdin) != NULL) {
            strcat(serial_input,"\n");

            // this was made for debugging of the serial monitor, by entering "test", everything after will be printed back through the serial monitor
            if(check_string_start(serial_input,"test")){
                remove_prefix(serial_input,"test");
                printf("%s", serial_input); 
            }

            //This statement is used in the calibration to get a live reading of the straing gauge
            if(check_string_start(serial_input,"strain_measure_start")){
                measure_test_state = true;
            }
            if (check_string_start(serial_input,"strain_measure_stop")){
                measure_test_state = false;
            }
            
            //This statement is used to start the Inertia Pull test
            if(check_string_start(serial_input,"rpm_start")){
                rpm_state = true;
            }
            if (check_string_start(serial_input,"rpm_stop")){
                rpm_state = false;
            }

            //This statement is used in the calibration to get the one reading of the current strain to find the calibration equation
            if (check_string_start(serial_input,"single_strain_value")) {
                char str[10];
                sprintf(str, "%d", strain_value); // Converts integer to string
                printf("w%s\n", str);
            }

            //This statement is used in the calibration to get a live reading of the RPM
            if (check_string_start(serial_input,"tach_test_start")){
                tach_test_state = true;
            }
            if (check_string_start(serial_input,"tach_test_stop")){
                tach_test_state = false;
            }


            //This statement is used to start the Steady State tuning test
            if (check_string_start(serial_input,"pau_test_start")){
                pau_test_state = true;
                ramp = 0;                                                   //this resets the ramp factor to zero every time the test is initiated
                Ok = 0;
                Ok1 = 0;
                Ok2 = 0;
                Ik = 0;
                Ik1 = 0;
                Ik2 = 0;
            }

            if (check_string_start(serial_input,"request_data")){
                requested_data = true;
            }
            if (check_string_start(serial_input,"pau_test_stop")){
                pau_test_state = false;
            }

            if (check_string_start(serial_input,"pau_voltage_test_start")){
                pau_voltage_test_state = true;
            }
            if (check_string_start(serial_input,"pau_voltage_test_stop")){
                pau_voltage_test_state = false;
            }

            //This check what the new target rpm is for steady state tuning
            if (check_string_start(serial_input,"target_rpm")){
                remove_prefix(serial_input,"target_rpm");
                target_rpm = atoi(serial_input);
            }

            if (check_string_start(serial_input,"servo_update_rate")){
                remove_prefix(serial_input,"servo_update_rate");
                servo_update_rate = atoi(serial_input);
            }

            //This checks what the new coefficient are for the backward Euler z-domain function are. The coefficient need to be sent to the ESP32, no the gains directly
            if (check_string_start(serial_input,"rkPID")){
                remove_prefix(serial_input,"rkPID");
                    // Tokenize the input string
                char *token = strtok(serial_input, " ");
                if (token != NULL) {
                    a0 = atof(token);
                    token = strtok(NULL, " ");
                }
                if (token != NULL) {
                    a1 = atof(token);
                    token = strtok(NULL, " ");
                }
                if (token != NULL) {
                    a2 = atof(token);
                    token = strtok(NULL, " ");
                }
                if (token != NULL) {
                    b0 = atof(token);
                    token = strtok(NULL, " ");
                }
                if (token != NULL) {
                    b1 = atof(token);
                    token = strtok(NULL, " ");
                }
                if (token != NULL) {
                    b2 = atof(token);
                }

                //This resets the error and output of the PID algorithm every time a new gain is sent to the ESP32
                Ok = 0;
                Ok1 = 0;
                Ok2 = 0;
                Ik = 0;
                Ik1 = 0;
                Ik2 = 0;
            }

            //This is used to change test the appopriate voltage sent to the PAU in an emergency situation. This is set by the user in the configuration software.
            if (check_string_start(serial_input,"ePAU")){
                remove_prefix(serial_input,"ePAU");
                char *endptr;
                pau_emergency_voltage = strtod(serial_input, &endptr);
                pau_test_voltage = pau_emergency_voltage;
            }

            //This checks if the high end software user has press the brake button on the computer
            if (check_string_start(serial_input,"applyBrakeOn")){
                apply_brake_state = true;
            }
            if (check_string_start(serial_input,"applyBrakeOff")){
                apply_brake_state = false;
            }

        }
        vTaskDelay(pdMS_TO_TICKS(100)); //this tas is run every 100ms
    }
}



// The following PID algorithm is based on the discrete-time implementation of the backward Euler method, where the integral and derivative terms are approximated using numerical techniques.
// This approach discretizes continuous control signals, allowing for stable and accurate performance in digital systems by estimating future values based on current and previous steps.
void pidTask(void *pvParameters) {

    bool led_state1 = false;

gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO_PIN), // Pin mask for GPIO2
        .mode = GPIO_MODE_OUTPUT,              // Set as output
        .pull_up_en = GPIO_PULLUP_DISABLE,     // Disable pull-up
        .pull_down_en = GPIO_PULLDOWN_DISABLE, // Disable pull-down
        .intr_type = GPIO_INTR_DISABLE         // Disable interrupts
    };

    gpio_config(&io_conf);

    while (1) {
        // Read input (e.g., from ADC)
        if(pau_test_state) {
            //gpio_set_level(LED_GPIO_PIN, 1);
            double target_ramp = current_rpm-(current_rpm-target_rpm)*ramp; //target_ramp is the target set by the user time a ramp factor so that when a new target is set, the pid algorithm has a second of ramping before applying the full load to the car

            

            Ik2 = Ik1;                                                      //The error at t-2 is equal to the error at t-1
            Ik1 = Ik;                                                       //The error at t-1 is equal to the current error

            Ik=current_rpm-target_ramp;  
            //Ik = 233;
                                        //The current error is the current rpm minus the target rpm
    
            Ok2=Ok1;                                                        //The output at t-2 is equal to the output at t-1
            Ok1=Ok;                                                         //The output at t-1 is equal to the current output
            Ok = (double)1/a0*(b0*Ik+b1*Ik1+b2*Ik2-a1*Ok1-a2*Ok2);          //Recalculate the output
            
            if (ramp < 0.90) {                                              //If the ramp is still not equal to 1
                ramp = ramp + 0.1;                                          //add 0.1 to get it closer to one
            }
            
            // the following if, else if, else statement is a saturation filter because the DAC output of the esp32 is only an output of 0 to 255;
            if (Ok > 255) {                                 
                pau_output = 255;
            } 
            else if (Ok < 0 || isnan(Ok)) {
                pau_output = 0;
            }
            else {
                pau_output = Ok;
            }
            
        }
        gpio_set_level(LED_GPIO_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(servo_update_rate));                                     //This task is run every 100ms
    }
}

// There are multiple scenarios where we might want to turn on the PAU:
    //eStop presses is the first and most important case
    //then check if the steady state tuning test is turned on
    //then check if the high end soft ware user has turned on the brake manually
void PAUOutput(void *pvParameters) {
    // Configure ADC for pin 34
    adc1_config_width(ADC_WIDTH_BIT_12);

    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_0);          // This is the E_Stop Reading

    // Initialize DAC for pin 25
    dac_output_enable(DAC_CHANNEL_1);                                   // DAC_CHANNEL_1 corresponds to GPIO25, which is the output pin for the PAU Voltage to the motor controller

    while(1) {

        int adc_reading = adc1_get_raw(ADC1_CHANNEL_6);                 //Check what the voltage is on the eStop button

        if (adc_reading == 4095) {                                          // if the E_Stop is Pressed
            dac_output_voltage(DAC_CHANNEL_1, (uint8_t)eOutput);            // Turn on the PAU to the predetermined value of "eOutput" to slow down the dyno wheels at an apporpirately safe speed
        } else if (pau_voltage_test_state) {                                // From the calibraion page if the user wants to output specific value to the pau
            dac_output_voltage(DAC_CHANNEL_1, (uint8_t)pau_test_voltage);   // set value from the high end software
        } else if (pau_test_state) {                                        // If the PID Test is active
            dac_output_voltage(DAC_CHANNEL_1, (uint8_t)pau_output);         // Turn on the PAU to the value determined in the pidTask task/function
        } else if (apply_brake_state) {                                     // if the brake is turned on from the high end software
            dac_output_voltage(DAC_CHANNEL_1, (uint8_t)brake_voltage);      // Turn on the PAU to the predetermined value of "brakeVoltage" to slow down the dyno wheels at an apporpirately safe speed
        } else {                                                            // If none of the previous states are active
            dac_output_voltage(DAC_CHANNEL_1, (uint8_t)0);                  // Turn off the PAU
        }

        vTaskDelay(pdMS_TO_TICKS(servo_update_rate));                                 // This loop runs every 100 ms
    }
}


void hx711_task(void *pvParameter) {
    int32_t reading = 0;
    
    // Configure DOUT as input and SCK as output
    gpio_set_direction(HX711_DOUT_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(HX711_SCK_PIN, GPIO_MODE_OUTPUT);

    while (1) {
        // Wait for the HX711 to be ready (DOUT pin goes low)
        while (gpio_get_level(HX711_DOUT_PIN) == 1) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        // Read 24-bit data from the HX711
        reading = 0;
        for (int i = 0; i < 24; i++) {
            gpio_set_level(HX711_SCK_PIN, 1);
            esp_rom_delay_us(1);  // small delay to ensure timing
            reading = (reading << 1) | gpio_get_level(HX711_DOUT_PIN);
            gpio_set_level(HX711_SCK_PIN, 0);
            esp_rom_delay_us(1);
        }

        // Set the clock pin high for one more cycle to complete the reading
        gpio_set_level(HX711_SCK_PIN, 1);
        esp_rom_delay_us(1);
        gpio_set_level(HX711_SCK_PIN, 0);

        // Convert the 24-bit reading (two's complement)
        int sum = 0;
        for (int i = 0; i < 10; i++) 
            {
            if (reading & 0x800000) {
            reading |= ~0xFFFFFF;  // Extend sign bit if negative
            sum += reading;
            vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
        strain_value = sum/10;
    }
}




void app_main(void) {
    pcnt_example_init();

    xTaskCreate(serial_output_task, "serial_output_task", 2048, NULL, 5, NULL); //output selected data

    xTaskCreate(serial_input_task, "serial_input_task", 2048, NULL, 5, NULL);   // read serial data

    xTaskCreate(pidTask, "pidTask", 2048, NULL, 5, NULL);
    xTaskCreate(PAUOutput, "PAUOutput", 2048, NULL, 5, NULL);

    xTaskCreate(&hx711_task, "hx711_task", 2048, NULL, 5, NULL);
}

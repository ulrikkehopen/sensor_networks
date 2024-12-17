/**
 * Copyright (c) 2017, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <cstdint>
#include <cstdio>
#include <cstring>


#include "mbed_version.h"

#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"

// Application helpers
#include "DummySensor.h"
#include "trace_helper.h"
#include "lora_radio_helper.h"

//Sensor folder
#include "acc.hpp"
#include "tmphum.hpp"
#include "rgb.hpp"
#include "gps.hpp"
#include "soil.hpp"
#include "led.hpp"

#define WAITING_TIME        1000ms

using namespace events;
using namespace std::chrono_literals;

// Max payload size can be LORAMAC_PHY_MAXPAYLOAD.
// This example only communicates with much shorter messages (<30 bytes).
// If longer messages are used, these buffers must be changed accordingly.
uint8_t tx_buffer[30];
uint8_t rx_buffer[30];
constexpr size_t TX_BUFFER_SIZE = 30;

/*
 * Sets up an application dependent transmission timer in ms. Used only when Duty Cycling is off for testing
 */
#define TX_TIMER                        10s

/**
 * Maximum number of events for the event queue.
 * 10 is the safe number for the stack events, however, if application
 * also uses the queue for whatever purposes, this number should be increased.
 */
#define MAX_NUMBER_OF_EVENTS            10

/**
 * Maximum number of retries for CONFIRMED messages before giving up
 */
#define CONFIRMED_MSG_RETRY_COUNTER     3

/**
 * Waiting time for collecting sensor data
 */
#define WAITING_TIME 1000ms

/**
 * Define sensor variables
 */

static I2C i2c(PB_9, PB_8);
float *accelerations;
uint16_t *colours;
uint16_t temperature,humidity,soil;
float latitude, longitude;
int baudrate = 9600;

/**
 * Dummy pin for dummy sensor
 */
#define PC_9                            0

/**
 * Pins for LED
 */
DigitalOut red_led(PB_13);
DigitalOut green_led(PB_14);



/**
 * Dummy sensor class object
 */
DS1820  ds1820(PC_9);




/**
* This event queue is the global event queue for both the
* application and stack. To conserve memory, the stack is designed to run
* in the same thread as the application and the application is responsible for
* providing an event queue to the stack that will be used for ISR deferment as
* well as application information event queuing.
*/
static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS *EVENTS_EVENT_SIZE);

/**
 * Event handler.
 *
 * This will be passed to the LoRaWAN stack to queue events for the
 * application which in turn drive the application.
 */
static void lora_event_handler(lorawan_event_t event);

/**
 * Constructing Mbed LoRaWANInterface and passing it the radio object from lora_radio_helper.
 */
static LoRaWANInterface lorawan(radio);

/**
 * Application specific callbacks
 */
static lorawan_app_callbacks_t callbacks;

/**
 * Default and configured device EUI, application EUI and application key
 */
static const uint8_t DEFAULT_DEV_EUI[] = {0x40, 0x39, 0x32, 0x35, 0x59, 0x37, 0x91, 0x94};
static uint8_t DEV_EUI[] = {0x7b, 0x39, 0x32, 0x35, 0x59, 0x37, 0x91, 0x94};
static uint8_t APP_EUI[] = {0x70, 0xb3, 0xd5, 0x7e, 0xd0, 0x00, 0xfc, 0x4d};
static uint8_t APP_KEY[] = {0xf3, 0x1c, 0x2e, 0x8b, 0xc6, 0x71, 0x28, 0x1d,
                            0x51, 0x16, 0xf0, 0x8f, 0xf0, 0xb7, 0x92, 0x8f};

/**
 * Entry point for application
 */
int main(void)
{


    printf("\r\n*** Sensor Networks @ ETSIST, UPM ***\r\n"
           "   Mbed (v%d.%d.%d) LoRaWAN example\r\n",
           MBED_MAJOR_VERSION, MBED_MINOR_VERSION, MBED_PATCH_VERSION);


    
    printf("\r\n DEV_EUI: ");
    for (int i = 0; i < sizeof(DEV_EUI); ++i) printf("%02x", DEV_EUI[i]);
    printf("\r\n APP_EUI: ");
    for (int i = 0; i < sizeof(APP_EUI); ++i) printf("%02x", APP_EUI[i]);
    printf("\r\n APP_KEY: ");
    for (int i = 0; i < sizeof(APP_KEY); ++i) printf("%02x", APP_KEY[i]);
    printf("\r\n");

    if (!memcmp(DEV_EUI, DEFAULT_DEV_EUI, sizeof(DEV_EUI))) {
        printf("\r\n *** You are using the default device EUI value!!! *** \r\n");
        printf("Please, change it to ensure that the device EUI is unique \r\n");
        return -1;
    }
    // Sensor initialization
    initMMA8451Q(&i2c);
    initSi7021(&i2c);
    initTCS34725(&i2c, PA_8);
    initGPS(PA_9,PA_10,baudrate);

    // setup tracing
    setup_trace();

    // stores the status of a call to LoRaWAN protocol
    lorawan_status_t retcode;

    // Initialize LoRaWAN stack
    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }

    printf("\r\n Mbed LoRaWANStack initialized \r\n");

    // prepare application callbacks
    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Set number of retries in case of CONFIRMED messages
    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER)
            != LORAWAN_STATUS_OK) {
        printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

    printf("\r\n CONFIRMED message retries : %d \r\n",
           CONFIRMED_MSG_RETRY_COUNTER);

    // Enable adaptive data rate
    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\r\n enable_adaptive_datarate failed! \r\n");
        return -1;
    }

    printf("\r\n Adaptive data  rate (ADR) - Enabled \r\n");

    lorawan_connect_t connect_params;
    connect_params.connect_type = LORAWAN_CONNECTION_OTAA;
    connect_params.connection_u.otaa.dev_eui = DEV_EUI;
    connect_params.connection_u.otaa.app_eui = APP_EUI;
    connect_params.connection_u.otaa.app_key = APP_KEY;
    connect_params.connection_u.otaa.nb_trials = 3;

    retcode = lorawan.connect(connect_params);

    if (retcode == LORAWAN_STATUS_OK ||
            retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

    printf("\r\n Connection - In Progress ...\r\n");

    // make your event queue dispatching events forever
    ev_queue.dispatch_forever();

    return 0;
}

/**
 * Test sensors
 */

static void test_sensors()
{
    startSi7021Measurement();
        ThisThread::sleep_for(MEASUREMENT_TIME); // Si7021 needs some time to complete measurements

        accelerations = getAcceleration();

        temperature = getTemperature();
        humidity = getHumidity();

        colours = getColours();

        //led_on(PB_13);

        //soil = measureSoil();

        //printf("GPS: #Sats: %d Lat(UTC): %.6f %c Long(UTC): %.6f %c Altitude: %.1f m GPS time: %s\n", get_num_satellites(), get_latitude(), get_lat_dir(), get_longitude(), get_long_dir(), get_altitude(), get_time());
        
        printf("\n\r------------------------\n\r");

        ThisThread::sleep_for(WAITING_TIME - MEASUREMENT_TIME);
}

/**
 * Sends sensorvalues to the Network Server
 */
static void send_message()
{
    uint16_t packet_len;
    int16_t retcode;
    int32_t sensor_value;
    size_t pos = 0;
    
    //test_sensors();
    startSi7021Measurement();
        ThisThread::sleep_for(MEASUREMENT_TIME); // Si7021 needs some time to complete measurements
        // Collect sensor data
        accelerations = getAcceleration();
        temperature = getTemperature();
        humidity = getHumidity();
        colours = getColours();
        soil = measureSoil();
        //latitude =  get_latitude();
        //longitude = get_longitude();
        latitude = 63.25732;
        longitude = 10.24105;
        printf("GPS: Lat(UTC): %.6f Long(UTC): %.6f\n", get_latitude(),  get_longitude());
        //testing colours
        printf("Clear: %i; Red = %i; Green = %i; Blue = %i\n\r", colours[0], colours[1], colours[2], colours[3]);
        
        
        ThisThread::sleep_for(WAITING_TIME - MEASUREMENT_TIME);
   
    
    
    
    tx_buffer[pos] = humidity & 0x00ff;//payload[0]
    pos++;
    tx_buffer[pos] = (humidity >> 8) & 0x00ff;//payload[1]
    pos++;
    //Temperature
    tx_buffer[pos] = temperature & 0x00ff;//payload[2]
    pos++;
    tx_buffer[pos] = (temperature >> 8) & 0x00ff;//payload[3]
    pos++;
    //Soil
    tx_buffer[pos] = soil & 0x00ff;//payload[4]
    pos++;
    tx_buffer[pos] = (soil >> 8) & 0x00ff;//payload[5]
    pos++;

    //latitude
    tx_buffer[pos] = (*(uint32_t *) &latitude) & 0x00ff;//payload[6]
    pos++;
    tx_buffer[pos] = ((*(uint32_t *) &latitude) >> 8) & 0x00ff;//payload[7]
    pos++;
    tx_buffer[pos] = ((*(uint32_t *) &latitude) >> 16) & 0x00ff;//payload[8]
    pos++;
    tx_buffer[pos] = ((*(uint32_t *) &latitude) >> 24) & 0x00ff;//payload[9]
    pos++;
    //longitude
    tx_buffer[pos] = (*(uint32_t *) &longitude) & 0x00ff;//payload[10]
    pos++;
    tx_buffer[pos] = ((*(uint32_t *) &longitude) >> 8) & 0x00ff;//payload[11]
    pos++;
    tx_buffer[pos] = ((*(uint32_t *) &longitude) >> 16) & 0x00ff;//payload[12]
    pos++;
    tx_buffer[pos] = ((*(uint32_t *) &longitude) >> 24) & 0x00ff;//payload[13]
    pos++;

    //Colours
    //Clear
    tx_buffer[pos] = colours[0] & 0x00ff;//payload[14]
    pos++;
    tx_buffer[pos] = (colours[0] >> 8) & 0x00ff;//payload[15]
    pos++;
     //Red
    tx_buffer[pos] = colours[1] & 0x00ff;//payload[16]
    pos++;
    tx_buffer[pos] = (colours[1] >> 8) & 0x00ff;//payload[17]
    pos++;
     //Green
    tx_buffer[pos] = colours[2] & 0x00ff;//payload[18]
    pos++;
    tx_buffer[pos] = (colours[2] >> 8) & 0x00ff;//payload[19]
    pos++;
     //Blue
    tx_buffer[pos] = colours[3] & 0x00ff;//payload[20]
    pos++;
    tx_buffer[pos] = (colours[3] >> 8) & 0x00ff;//payload[21]
    pos++;



    
    // Debug: Print the payload
    printf("Payload: ");
    for (int i = 0; i < pos; i++) {
        printf("%02X ", tx_buffer[i]);
    }
    printf("\r\n");

    retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, pos,
                           MSG_UNCONFIRMED_FLAG);

    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - WOULD BLOCK\r\n")
        : printf("\r\n send() - Error code %d \r\n", retcode);

        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
            //retry in 3 seconds
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                ev_queue.call_in(3s, send_message);
            }
        }
        return;
    }

    printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(tx_buffer, 0, sizeof(tx_buffer));

}
/**
 * Sends a dummy message to the Network Server
 */
static void send_dummy_message()
{
    uint16_t packet_len;
    int16_t retcode;
    int32_t sensor_value;

    if (ds1820.begin()) {
        ds1820.startConversion();
        sensor_value = ds1820.read();
        printf("\r\n Dummy Sensor Value = %d \r\n", sensor_value);
        ds1820.startConversion();
    } else {
        printf("\r\n No sensor found \r\n");
        return;
    }

    packet_len = snprintf((char *) tx_buffer, sizeof(tx_buffer),
                          "Dummy Sensor Value is %d", sensor_value);

    retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, packet_len,
                           MSG_UNCONFIRMED_FLAG);

    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - WOULD BLOCK\r\n")
        : printf("\r\n send() - Error code %d \r\n", retcode);

        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
            //retry in 3 seconds
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                ev_queue.call_in(3s, send_dummy_message);
            }
        }
        return;
    }

    printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(tx_buffer, 0, sizeof(tx_buffer));
}

/**
 * Receive a message from the Network Server
 */
static void receive_message()
{
    uint8_t port;
    int flags;
    int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);

    if (retcode < 0) {
        printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }

    printf(" RX Data on port %u (%d bytes): ", port, retcode);
    for (uint8_t i = 0; i < retcode; i++) {
        printf("%02x ", rx_buffer[i]);
    }
    printf("\r\n");

    
    const char expected_off[]="OFF";

    if (strcmp((char *)rx_buffer, "Red")== 0) {
        red_led = 0;    // Turn ON Red LED
        green_led = 1;  // Turn OFF Green LED
        printf("Command received: RED LED ON\n");
    } else if (strcmp((char *)rx_buffer, "Green") == 0) {
        red_led = 1;    // Turn OFF Red LED
        green_led = 0;  // Turn ON Green LED
        printf("Command received: GREEN LED ON\n");
    } else if (strcmp((char *)rx_buffer, "OFF") == 0) {
        red_led = 1;    // Turn OFF Red LED
        green_led = 1;  // Turn OFF Green LED
        printf("Command received: LEDs OFF\n");
    } else {
        printf("Unknown command: %s\n", rx_buffer);
    }
    
    memset(rx_buffer, 0, sizeof(rx_buffer));
}





/**
 * Event handler
 */
static void lora_event_handler(lorawan_event_t event)
{
    switch (event) {
        case CONNECTED:
            printf("\r\n Connection - Successful \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            } else {
                ev_queue.call_every(TX_TIMER, send_message);
            }

            break;
        case DISCONNECTED:
            ev_queue.break_dispatch();
            printf("\r\n Disconnected Successfully \r\n");
            break;
        case TX_DONE:
            printf("\r\n Message Sent to Network Server \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("\r\n Transmission Error - EventCode = %d \r\n", event);
            // try again
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        case RX_DONE:
            printf("\r\n Received message from Network Server \r\n");
            receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            printf("\r\n Error in reception - Code = %d \r\n", event);
            break;
        case JOIN_FAILURE:
            printf("\r\n OTAA Failed - Check Keys \r\n");
            break;
        case UPLINK_REQUIRED:
            printf("\r\n Uplink required by NS \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;
        default:
            MBED_ASSERT("Unknown Event");
    }
}

// EOF

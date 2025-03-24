
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "dsps_dotprod.h"
#include "esp_timer.h"


#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23 // not used but full duplex is more synchronised
#define PIN_NUM_CLK  18

#define BUFFER_SIZE 4000 // buffer can not exceed 4092
#define REDUCED_BUFFER_SIZE 1000

#define PI 3.14159265358979323846
#define SAMPLE_FREQUENCY 390000
#define TARGET_FREQUENCY 39000

#define REF_TABLE_SIZE 10


#define secondIndex 16
#define thirdIndex 32

static const char *TAG = "TDOA_DSP";

// Data is read as continuous stream of bits. FPGA buffer sends data as stream of 16 bit samples where first 4 bits are ID.
// Function makes sure that input and output buffers data transfers are aligned
bool align_data_acquisition(void) {
    // Configure the CLK pin as a GPIO output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_CLK),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    io_conf.pin_bit_mask = (1ULL << PIN_NUM_MISO); // Configure the MISO pin as a GPIO input
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);


    bool data_buffer[64];
    // Read one full buffer
    for (int i = 0; i < 64; i++) {
        gpio_set_level(PIN_NUM_CLK, 1);
        esp_rom_delay_us(250);
        data_buffer[i] = gpio_get_level(PIN_NUM_MISO); // Store the current bit on MISO
        gpio_set_level(PIN_NUM_CLK, 0);
        esp_rom_delay_us(250);
    }
    char bitString[65];  // 64 bits plus null terminator
    for (int i = 0; i < 64; i++) {
        bitString[i] = data_buffer[i] ? '1' : '0';
    }
    bitString[64] = '\0';
    ESP_LOGI(TAG, "Data buffer: %s", bitString);


    for(int i = 0; i < 19; i++) { // Guaranteed to have full first 4 bits of ID from one 16 bit sample if we read 19 bits
        if((data_buffer[i] == 0 && data_buffer[i+secondIndex] == 0) && (data_buffer[i+1] == 0 && data_buffer[i+secondIndex+1] == 0)) {
           uint8_t ID1, ID2, ID3; // Two bit values to store IDs

           ID1 = (data_buffer[i+2] << 1) | data_buffer[i+3];

           ID2 = (data_buffer[i+secondIndex+2] << 1) | data_buffer[i+secondIndex+3];

           if((ID2 != ID1 +1) && !(ID1 == 3 && ID2 == 0)) continue;

           if(data_buffer[i+thirdIndex] == 0 && data_buffer[i+thirdIndex+1] == 0 ) {
            ID3 = (data_buffer[i+thirdIndex+2] << 1) | data_buffer[i+thirdIndex+3];

            if((ID3 != ID2 +1) && !(ID2 == 3 && ID3 == 0)) continue;

            int8_t start_index = ID1*16 - i;
            // Clock out bits until the next bit is the start of our 64 bit word
            for(int j = 0; j < 64 - start_index; j++) {
                gpio_set_level(PIN_NUM_CLK, 1);
                esp_rom_delay_us(25);
                gpio_set_level(PIN_NUM_CLK, 0);
                esp_rom_delay_us(25);
                }
            //gpio_reset_pin(PIN_NUM_CLK); // Reset pins for reconfiguration by SPI driver
            //gpio_reset_pin(PIN_NUM_MISO);
            return 1; // Buffers are aligned
            }
        }
    }
    for (int i = 0; i < 64; i++) {
        bitString[i] = data_buffer[i] ? '1' : '0';
    }
    bitString[64] = '\0';
    ESP_LOGI(TAG, "Data buffer: %s", bitString);

    return 0; // Failed to read correct data from buffer
}



int16_t ref_sine_table_Q15[REF_TABLE_SIZE] __attribute__((aligned(16))); // global lookuptable for target frequency sine. Format: Q15
int16_t ref_cos_table_Q15[REF_TABLE_SIZE] __attribute__((aligned(16))); 

void initialize_sine_table_Q15(void) {
    double angleStep = 2*PI*TARGET_FREQUENCY/SAMPLE_FREQUENCY;
    for (int i = 0; i < REF_TABLE_SIZE; i++) {
        double value = sin(i*angleStep)/REF_TABLE_SIZE; // divide by tablesize to avoid overflow in dotproduct calculation
        ref_sine_table_Q15[i] = (int16_t)(value * 32767.0); // converting to Q15 format. 32767 = 2^15 -1 
    }
}

void initialize_cos_table_Q15(void) {
    double angleStep = 2*PI*TARGET_FREQUENCY/SAMPLE_FREQUENCY;
    for (int i = 0; i < REF_TABLE_SIZE; i++) {
        double value = cos(i*angleStep)/REF_TABLE_SIZE;
        ref_cos_table_Q15[i] = (int16_t)(value * 32767.0);
    }
}



uint32_t calculate_signal_strength(const int16_t *cosArr, const int16_t *sineArr, const int16_t *sigArr, uint32_t len) {
    esp_err_t ret;
    int16_t R; // real component of signal
    int16_t I; // imaginary component of signal
    uint32_t result = 0; 
    ret = dsps_dotprod_s16_ae32(cosArr, sigArr, &R, len, 0); //const int16_t *src1, const int16_t *src2, int16_t *dest, int len, int8_t shift
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calculate dotproduct real: %s", esp_err_to_name(ret));
        return 0;
    }
    ret = dsps_dotprod_s16_ae32(sineArr, sigArr, &I, len, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calculate dotproduct imaginary: %s", esp_err_to_name(ret));
        return 0;
    }
    result = R*R + I*I;
    return result;
}


void normalize_signal_arr_Q15(int16_t *signal_arr, uint8_t len) { // normalizes array of Q15 values between -32768 32767 and divide by tablesize
    int16_t abs_max = 0;
    for (size_t i = 0; i < len; i++) {
        int16_t abs_val = (signal_arr[i] < 0) ? -signal_arr[i] : signal_arr[i];
        if (abs_val > abs_max) abs_max = abs_val;
    }

    int32_t scale_factor = ((int32_t)32767 << 15) / abs_max;

    for (int i = 0; i < len; i++) {
        int32_t temp = (int32_t)signal_arr[i] * scale_factor;
        signal_arr[i] = (int16_t)(temp >> 15);
        signal_arr[i] /= REF_TABLE_SIZE; // divide by tablesize to avoid overflow in dotproduct calculation
    }
}


uint16_t buffer_ping[BUFFER_SIZE] __attribute__((aligned(16)));
uint16_t buffer_pong[BUFFER_SIZE] __attribute__((aligned(16)));

uint16_t tx_buffer[BUFFER_SIZE] __attribute__((aligned(16))); // dummy data in order to use full duplex

QueueHandle_t dmaQueue;



typedef enum {
    BUFFER_PING = 0,
    BUFFER_PONG = 1,
} buffer_id_t;

typedef struct {
    buffer_id_t buf_id;
    uint64_t end_time_us;
} dma_event_t;

spi_device_handle_t spi;
spi_transaction_t trans_ping, trans_pong;

// Acquisition task: continuously collects SPI DMA transactions.
// This task runs with minimal CPU intervention because DMA fills the buffers.
void acquisition_task(void *pvParameters) {
    spi_transaction_t *rtrans;
    dma_event_t event;
    while (1) {
        ESP_ERROR_CHECK(spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY)); // Waits until dma is finished
        event.buf_id = (buffer_id_t) rtrans->user;
        event.end_time_us = esp_timer_get_time();
        xQueueSend(dmaQueue, &event, portMAX_DELAY); // Notify the processing task that this buffer is ready.
    }
}

void processing_task(void *pvParameters) {
    dma_event_t event;
    static uint8_t printCount = 0;
    while(1) {
        // waits for notification that a DMA buffer is finished
        if(xQueueReceive(dmaQueue, &event, portMAX_DELAY) == pdTRUE) {
            if (event.buf_id == BUFFER_PING) {
                ESP_ERROR_CHECK(spi_device_queue_trans(spi, &trans_ping, portMAX_DELAY)); // requeue the ping transaction
                //TODO: process ping data 
                if(printCount % 8 == 0) {
                    ESP_LOGI(TAG, "Ping Buffer first 4 samples: HEX: %04X %04X %04X %04X %04X %04X %04X %04X, DEC: %d %d %d %d %d %d %d %d", 
                        buffer_ping[0], buffer_ping[1], buffer_ping[2], buffer_ping[3], buffer_ping[4], buffer_ping[5], buffer_ping[6], buffer_ping[7],
                        buffer_ping[0], buffer_ping[1], buffer_ping[2], buffer_ping[3], buffer_ping[4], buffer_ping[5], buffer_ping[6], buffer_ping[7]);
                }
            } else {
                ESP_ERROR_CHECK(spi_device_queue_trans(spi, &trans_pong, portMAX_DELAY)); 
                //TODO: process pong data
                if(printCount % 7 == 0) {
                    ESP_LOGI(TAG, "Pong Buffer first 4 samples: HEX: %04X %04X %04X %04X, DEC: %d %d %d %d", 
                        buffer_pong[0], buffer_pong[1], buffer_pong[2], buffer_pong[3],
                        buffer_pong[0], buffer_pong[1], buffer_pong[2], buffer_pong[3]);
                }
            }
            printCount ++;
        }
    }
}





void app_main(void) {
    //initialize_sine_table();
    esp_err_t ret;


    
    while(!align_data_acquisition()) { 
        ESP_LOGI(TAG, "Failed to align buffers");
        return;
    }
    
    
    
    ESP_LOGI(TAG, "Buffers aligned successfully");

    memset(tx_buffer, 0, sizeof(tx_buffer)); // fill tx buffer with dummy data for full duplex

    memset(buffer_ping, 1, sizeof(buffer_ping));
    memset(buffer_pong, 0, sizeof(buffer_pong));

    dmaQueue = xQueueCreate(10, sizeof(buffer_id_t));

    if (dmaQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create DMA queue");
        return;
    }


    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BUFFER_SIZE * sizeof(uint16_t),
        .flags = 0,
        .intr_flags = 0,
    };
    ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return;
    }

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 24900000, // 24.9MHz 64 bits per sample period
        .mode = 0,
        .spics_io_num = -1, // Chip-select not used
        .queue_size = 2, // Allows queuing up another device for double buffering (ping pong buffering)
        .flags = 0, // Full duplex for more consistent transmission, output is not used
    };
    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failted to add SPI device: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "SPI device added successfully");

    memset(&trans_ping, 0, sizeof(trans_ping));
    trans_ping.length = BUFFER_SIZE * 16;
    trans_ping.tx_buffer = tx_buffer;
    trans_ping.rx_buffer = buffer_ping;
    trans_ping.user = (void *) BUFFER_PING;

    memset(&trans_pong, 0, sizeof(trans_pong));
    trans_pong.length = BUFFER_SIZE * 16;
    trans_pong.tx_buffer = tx_buffer;
    trans_pong.rx_buffer = buffer_pong;
    trans_pong.user = (void *) BUFFER_PONG;

    ret = spi_device_queue_trans(spi, &trans_ping, portMAX_DELAY);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Queue trans ping failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = spi_device_queue_trans(spi, &trans_pong, portMAX_DELAY);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Queue trans pong failed: %s", esp_err_to_name(ret));
        return;
    }
    // create freeRTOS tasks that will run continuously
    xTaskCreate(acquisition_task, "acquisition_task", 4096, NULL, 5, NULL);
    xTaskCreate(processing_task, "processing_task", 4096, NULL, 5, NULL);
}

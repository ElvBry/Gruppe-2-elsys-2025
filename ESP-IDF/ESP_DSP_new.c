
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
#include "esp_system.h"
#include "esp_cpu.h"
#include <inttypes.h>


#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23 // not used but full duplex is more synchronised
#define PIN_NUM_CLK  18

#define BUFFER_SIZE 240 // buffer can not exceed 255
#define REDUCED_BUFFER_SIZE BUFFER_SIZE/4

#define PI 3.14159265358979323846
#define SAMPLE_FREQUENCY 390000

#define TARGET_FREQUENCY_1 39000
#define SIGNAL_THRESHOLD_39kHz 15000000000ULL
#define NORMALIZED_THRESHOLD_39kHz 650000000000000ULL
#define BINARY_SEARCH_STEPS_39kHz 6
#define REF_TABLE_SIZE_39kHz 10

#define MAX_SIGNAL_DIFFERENCE_US 2000
#define MAX_SIGNAL_DURATION_US 3000

#define NUM_FREQUENCIES 1
#define NUM_RECEIVERS 4




#define secondIndex 16
#define thirdIndex 32

#define extraShift 0

static const char *TAG = "TDOA_DSP";


typedef struct {
    uint32_t frequency;
    uint64_t last_ping_time_us;
    int16_t *ref_sine;
    int16_t *ref_cos;
    uint8_t ref_table_len;
    uint64_t signal_threshold;
    uint64_t normalized_threshold;
    uint8_t binary_search_steps;
    bool valid_flags[NUM_RECEIVERS]; // Valid after signal is detected in 2 subsequent buffers
    bool all_valid;
    uint64_t arrival_times[NUM_RECEIVERS]; // Arrival timestamps for each receiver
} frequency_state_t;



//
//  SIGNAL PROCESSING
//

int16_t ref_sine_table_Q15_39kHz[REF_TABLE_SIZE_39kHz] __attribute__((aligned(16))); // global lookuptable for target frequency sine. Format: Q15
int16_t ref_cos_table_Q15_39kHz[REF_TABLE_SIZE_39kHz] __attribute__((aligned(16))); 


void initialize_sine_table_Q15(int16_t *src,const uint32_t target_frequency, uint8_t len) {
    double angleStep = 2*PI*target_frequency/SAMPLE_FREQUENCY;
    for (int i = 0; i < len; i++) {
        double value = sin(i*angleStep)/len; // divide by tablesize to avoid overflow in dotproduct calculation
        src[i] = (int16_t)(value * 32767.0); // converting to Q15 format. 32767 = 2^15 -1 
    }
}

void initialize_cos_table_Q15(int16_t *src,const uint32_t target_frequency, uint8_t len) {
    double angleStep = 2*PI*target_frequency/SAMPLE_FREQUENCY;
    for (int i = 0; i < len; i++) {
        double value = cos(i*angleStep)/len;
        src[i] = (int16_t)(value * 32767.0);
    }
}

static inline uint64_t calculate_arrival_time_us(uint64_t end_time_us, uint8_t start_index) {
    double sample_period_us = 1000000.0 / SAMPLE_FREQUENCY;
    double delay_us = start_index * sample_period_us; // Calculate the delay as a floating point value for better precision
   
    uint64_t arrival_time_us = end_time_us - (uint64_t)(delay_us + 0.5);
    return arrival_time_us;
}


static inline void remove_dc_offset(int16_t* buffer, uint16_t len) {
    int32_t sum = 0;
    for (int i = 0; i < 10; i++) sum += buffer[i];

    int16_t offset = (int16_t)(sum / 10);

    for (int i = 0; i < len; i++) buffer[i] -= offset;
}


static inline uint64_t calculate_signal_strength(const int16_t *ref_cos, const int16_t *ref_sine, const int16_t *signal_arr, uint32_t len) {
    esp_err_t ret;
    int16_t R; // real component of signal
    int16_t I; // imaginary component of signal
    uint64_t result = 0; 
    // Computes dotproduct with optimized dsps function
    ret = dsps_dotprod_s16_ae32(ref_cos, signal_arr, &R, len, 0); //const int16_t *src1, const int16_t *src2, int16_t *dest, int len, int8_t shift
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calculate dotproduct real: %s", esp_err_to_name(ret));
        return 0;
    }
    ret = dsps_dotprod_s16_ae32(ref_sine, signal_arr, &I, len, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to calculate dotproduct imaginary: %s", esp_err_to_name(ret));
        return 0;
    }
    result = R*R + I*I;
    return result;
}

// normalizes array of Q15 values between -32768 32767
static inline void normalize_signal_arr_Q15(int16_t *signal_arr, uint8_t len) {
    int16_t abs_max = 0;
    for (size_t i = 0; i < len; i++) {
        int16_t abs_val = (signal_arr[i] < 0) ? -signal_arr[i] : signal_arr[i];
        if (abs_val > abs_max) abs_max = abs_val;
    }
    if(abs_max == 0) abs_max = 1;
    int32_t scale_factor = ((int32_t)32767 << 15) / abs_max;

    for (int i = 0; i < len; i++) {
        int32_t temp = (int32_t)signal_arr[i] * scale_factor;
        signal_arr[i] = (int16_t)(temp >> 15);
        signal_arr[i] /= 10; // avoid overflow in dotproduct calculation. May cause problems for signals with high tablesize
    }
}


static inline bool signalPresent(const int16_t *src, const frequency_state_t *freq_state) {
    return calculate_signal_strength(freq_state->ref_cos, freq_state->ref_sine, &src[REDUCED_BUFFER_SIZE-freq_state->ref_table_len], freq_state->ref_table_len) > freq_state->signal_threshold;
}

// Finds index where signal starts in array. Returns 0xFF if signal is not present at the end 
static inline uint8_t binary_search_find_start_index(const int16_t *src, const frequency_state_t *freq_state) {
    // Search from index 0 to last analyzable index
    uint8_t low = 0;
    uint8_t high = REDUCED_BUFFER_SIZE - freq_state->ref_table_len;
    uint8_t startIndex = high;
    uint8_t steps = 0;
    
    uint64_t strength_low = calculate_signal_strength(freq_state->ref_cos, freq_state->ref_sine, &src[low], freq_state->ref_table_len);
    if (strength_low >= freq_state->normalized_threshold) return 0;
   
    uint64_t strength_high = calculate_signal_strength(freq_state->ref_cos, freq_state->ref_sine, &src[high], freq_state->ref_table_len);
    if (strength_high < freq_state->normalized_threshold) return 0xFF; // Error: the signal in the upper window is below threshold
   
    while (low <= high && steps < freq_state->binary_search_steps) {
        uint8_t mid = low + ((high - low) >> 1);
        uint64_t strength_mid = calculate_signal_strength(freq_state->ref_cos, freq_state->ref_sine, &src[mid], freq_state->ref_table_len);
        if (strength_mid >= freq_state->normalized_threshold) {
            startIndex = mid;
            if (mid == 0) break; // Found the earliest possible index
            high = mid - 1;      // Search left
        } else low = mid + 1;    // Search right
        steps++;
    }
    return startIndex;
}



uint16_t align_data[4] __attribute__((aligned(16)));
uint16_t buffer_ping[BUFFER_SIZE] __attribute__((aligned(16)));
uint16_t buffer_pong[BUFFER_SIZE] __attribute__((aligned(16)));

uint16_t tx_buffer[BUFFER_SIZE] __attribute__((aligned(16))); // dummy data in order to use full duplex

int16_t rec1_arr[REDUCED_BUFFER_SIZE+1] __attribute__((aligned(16))); // last bit used to indicate if array is normalized
int16_t rec2_arr[REDUCED_BUFFER_SIZE+1] __attribute__((aligned(16)));
int16_t rec3_arr[REDUCED_BUFFER_SIZE+1] __attribute__((aligned(16)));
int16_t rec4_arr[REDUCED_BUFFER_SIZE+1] __attribute__((aligned(16)));


static inline uint16_t swap_bytes(uint16_t val) {
    return (val >> 8) | (val << 8);
}

void swap_buffer_bytes(uint16_t *buffer, size_t len) {
    for (size_t i = 0; i < len; i++) {
        buffer[i] = swap_bytes(buffer[i]);
    }
}


// Splits interlieved buffer into 4 seperate arrays and clears upper 4 bits
static inline void deinterleave_and_clear_id(const uint16_t *src, size_t num_samples, int16_t *dst1, int16_t *dst2, int16_t *dst3, int16_t *dst4) {
    const uint16_t *end = src + num_samples;
    while (src < end) {
        *dst1++ = *src++ & 0x0FFF;  // Clear the upper 4 ID-bits
        *dst2++ = *src++ & 0x0FFF;
        *dst3++ = *src++ & 0x0FFF;
        *dst4++ = *src++ & 0x0FFF;
    }
}

//
//  PROGRAM SETUP
//

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
    
    int bit_index = 0;
    // Loop over the first 4 uint16_t values (4 * 16 = 64 bits)
    for (int word = 0; word < 4; word++) {
        // Process bits MSB-first (bit 15 to bit 0)
        for (int bit = 15; bit >= 0; bit--) {
            data_buffer[bit_index++] = ((align_data[word] >> bit) & 0x1) ? true : false; // Shift the current word right by 'bit' and mask out the lowest bit.
        }
    }

    for(int i = 0; i < 19; i++) { // Guaranteed to have full first 4 bits of ID from one 16 bit sample if we read 19 bits
        uint8_t ID1, ID2, ID3; // Two bit values to store IDs

        ID1 = (data_buffer[i] << 3) | (data_buffer[i+1] << 2) | (data_buffer[i+2] << 1) | data_buffer[i+3];
        ID2 = (data_buffer[i+secondIndex] << 3) | (data_buffer[i+secondIndex+1] << 2) | (data_buffer[i+secondIndex+2] << 1) | data_buffer[i+secondIndex+3];

        if(((ID2 != ID1 +1) && !(ID1 == 3 && ID2 == 0) ) || (ID1 > 3 || ID2 > 3)) continue;

        ID3 = (data_buffer[i+thirdIndex] << 3) | (data_buffer[i+thirdIndex+1] << 2) | (data_buffer[i+thirdIndex+2] << 1) | (data_buffer[i+thirdIndex+3]);

        if( ((ID3 != ID2 +1) && !(ID2 == 3 && ID3 == 0)) || ID3 > 3) continue;


        int8_t start_index = (ID1 * 16) - i;
        if (start_index < 0) start_index += 64;
        ESP_LOGI(TAG, "%d", 64 - start_index);

        // Clock out bits until the next bit is the start of our 64 bit word
        for(int j = 0; j < 64 - start_index + extraShift; j++) {
            gpio_set_level(PIN_NUM_CLK, 1);
            esp_rom_delay_us(25);
            gpio_set_level(PIN_NUM_CLK, 0);
            esp_rom_delay_us(25);
        }
        char bitString[65];
        uint64_t hexVal = 0;
        for (int i = 0; i < 64; i++) {
            bool bit = data_buffer[(i + 64 - start_index) % 64];
            bitString[i] = bit ? '1' : '0';
            hexVal = (hexVal << 1) | (bit ? 1ULL : 0ULL);
        }
        bitString[64] = '\0';
        ESP_LOGI(TAG, "Data buffer (binary): %s", bitString);
        ESP_LOGI(TAG, "Data buffer (hex): 0x%016llX", hexVal);
        return 1; // Buffers are aligned
    }
    char bitString[65];
    for (int i = 0; i < 64; i++) {
        bitString[i] = data_buffer[i] ? '1' : '0';
    }
    bitString[64] = '\0';
    ESP_LOGI(TAG, "Data buffer: %s", bitString);

    return 0; // Failed to read correct data from buffer
}


// holds clk pin high for over 1 ms in order to communicate to FPGA that it should reset its output buffer
void reset_FPGA_output() {
    // Configure the CLK pin as a GPIO output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_NUM_CLK),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(PIN_NUM_CLK, 1);
    esp_rom_delay_us(1000000);
    gpio_set_level(PIN_NUM_CLK, 0);
    esp_rom_delay_us(1000);
    /* debugging timing
    for(int i = 0; i < 63; i++) {
        gpio_set_level(PIN_NUM_CLK, 1);
        esp_rom_delay_us(25);
        gpio_set_level(PIN_NUM_CLK, 0);
        esp_rom_delay_us(25);
    }
    */
}


//
//  TASKS
//

void print_buffer_csv_inline(const int16_t *buffer, size_t len) {
    char line[2048] = {0};
    char temp[16];

    for (size_t i = 0; i < len; i++) {
        snprintf(temp, sizeof(temp), "%d", buffer[i]);
        strcat(line, temp);
        if (i < len - 1) strcat(line, ",");
    }

    ESP_LOGI(TAG, "%s", line);
}

// Prints receiver timestamps in format
void print_receiver_values_csv(const frequency_state_t *freq_state) {
    char line[256] = {0};
    char temp[64];
    
    snprintf(line, sizeof(line), "%" PRIu32, freq_state->frequency);
    for (int i = 0; i < NUM_RECEIVERS; i++) {
        snprintf(temp, sizeof(temp), "R%d: %" PRIu64, i + 1, freq_state->arrival_times[i]);
        strcat(line, temp);
        if (i < NUM_RECEIVERS - 1) strcat(line, ", ");
    }
    
    ESP_LOGI(TAG, "%s", line);
}

// Returns cycle count for debugging and testing
static inline uint32_t get_ccount(void) {
    uint32_t ccount;
    __asm__ __volatile__("rsr.ccount %0" : "=a" (ccount));
    return ccount;
}

static inline void update_frequency_state(int16_t *src, frequency_state_t *freq_state, const uint8_t index, const uint64_t end_time_us) {
    if(freq_state->valid_flags[index] == true) return; // Avoid looking for already found signal
    if(signalPresent(src, freq_state)) { // Initial check for signal in order to avoid normalizing noise which breaks binary search
        if(src[REDUCED_BUFFER_SIZE] == false) {
            normalize_signal_arr_Q15(src, REDUCED_BUFFER_SIZE);
            src[REDUCED_BUFFER_SIZE] = (int16_t)true;
        }
        uint8_t estimated_start_index = binary_search_find_start_index(src, freq_state);
        if(estimated_start_index != 0xFF) {
            uint64_t estimated_arival_time_us = calculate_arrival_time_us(end_time_us,estimated_start_index);
            
            if(freq_state->arrival_times[index] == 0) { // First successful measurement of signal
                freq_state->arrival_times[index] = estimated_arival_time_us;
                return;
            }
            // Second successful measurement of signal
            freq_state->valid_flags[index] = true;
            for (int i = 0; i < NUM_RECEIVERS; i++) {
                if(!freq_state->valid_flags[i]) break;
                if(i == NUM_RECEIVERS -1) freq_state->all_valid = true;
            }
        }
    }
}

// Allow frequency to be measured again by setting its updatable values to 0
static inline void reset_frequency_state(frequency_state_t *freq_state) {
    freq_state->all_valid = false;
    freq_state->last_ping_time_us = 0;
    for(int i = 0; i < NUM_RECEIVERS; i++) {
        freq_state->valid_flags[i] = false;
        freq_state->arrival_times[i] = 0;
    }
}

frequency_state_t frequency_states[NUM_FREQUENCIES] = {
        {
        .valid_flags = {false,false,false,false},
        .all_valid = false,
        .last_ping_time_us = 0,
        .arrival_times = {0,0,0,0},
    }
};


static inline void prosess_buffer(uint16_t *src, uint64_t end_time_us) {
    swap_buffer_bytes(src,BUFFER_SIZE);
    deinterleave_and_clear_id(src, BUFFER_SIZE, rec1_arr, rec2_arr,rec3_arr,rec4_arr);
    rec1_arr[0] = (rec1_arr[0] + rec1_arr[1])/2; // first value is often malformed
    rec2_arr[0] = (rec2_arr[0] + rec2_arr[1])/2;
    rec3_arr[0] = (rec3_arr[0] + rec3_arr[1])/2;
    rec4_arr[0] = (rec4_arr[0] + rec4_arr[1])/2;

    remove_dc_offset(rec1_arr, REDUCED_BUFFER_SIZE);
    remove_dc_offset(rec2_arr, REDUCED_BUFFER_SIZE);
    remove_dc_offset(rec3_arr, REDUCED_BUFFER_SIZE);
    remove_dc_offset(rec4_arr, REDUCED_BUFFER_SIZE);
    for (int i = 0; i < NUM_FREQUENCIES; i++) {
       
        if(frequency_states[i].all_valid) {
            if((end_time_us > frequency_states[i].last_ping_time_us + MAX_SIGNAL_DURATION_US)) reset_frequency_state(&frequency_states[i]);  // Avoid searching for frequency if it has been found. After a duration it can be measured again
            continue;
        }
        // Targeted frequency is reset if not enough receivers noticed it within a realistic timeframe
        if((frequency_states[i].last_ping_time_us > 0) && (end_time_us > frequency_states[i].last_ping_time_us + MAX_SIGNAL_DIFFERENCE_US)) {
            reset_frequency_state(&frequency_states[i]);
            continue;
        }
        update_frequency_state(rec1_arr, &frequency_states[i], 0, end_time_us);
        update_frequency_state(rec2_arr, &frequency_states[i], 1, end_time_us);
        update_frequency_state(rec3_arr, &frequency_states[i], 2, end_time_us);
        update_frequency_state(rec4_arr, &frequency_states[i], 3, end_time_us);
        if(frequency_states[i].all_valid == true)  print_receiver_values_csv(&frequency_states[i]);
    }
    rec1_arr[REDUCED_BUFFER_SIZE] = 0;
    rec2_arr[REDUCED_BUFFER_SIZE] = 0;
    rec3_arr[REDUCED_BUFFER_SIZE] = 0;
    rec4_arr[REDUCED_BUFFER_SIZE] = 0;
}




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
    static uint64_t printCount = 0;
    static uint8_t misalignedSigCount = 0;
    int64_t startCycleCount;
    int64_t endCycleCount;
    while(1) {
        // waits for notification that a DMA buffer is finished
        if(xQueueReceive(dmaQueue, &event, portMAX_DELAY) == pdTRUE) {
            
            if (event.buf_id == BUFFER_PING) {
                startCycleCount = get_ccount();
                prosess_buffer(buffer_ping, event.end_time_us);
                endCycleCount = get_ccount();
                misalignedSigCount = (((buffer_ping[0] &0xF000) != 0x0000) ||
                                      ((buffer_ping[1] &0xF000) != 0x1000) ||
                                      ((buffer_ping[2] &0xF000) != 0x2000) ||
                                      ((buffer_ping[3] &0xF000) != 0x3000)) ? misalignedSigCount +1 : 0;
                                      
                ESP_ERROR_CHECK(spi_device_queue_trans(spi, &trans_ping, portMAX_DELAY)); // requeue the ping transaction
                if(printCount % 2000 == 0) {                
                    ESP_LOGI(TAG, "Ping Buffer first 8 samples: HEX: %04X %04X %04X %04X %04X %04X %04X %04X, CYCLES: %lld, DEC: %d %d %d %d %d %d %d %d", 
                        buffer_ping[0], buffer_ping[1], buffer_ping[2], buffer_ping[3], buffer_ping[4], buffer_ping[5], buffer_ping[6], buffer_ping[7], endCycleCount-startCycleCount,
                        buffer_ping[0]&0x0FFF, buffer_ping[1]&0x0FFF, buffer_ping[2]&0x0FFF, buffer_ping[3]&0x0FFF, buffer_ping[4]&0x0FFF, buffer_ping[5]&0x0FFF, buffer_ping[6]&0x0FFF, buffer_ping[7]&0x0FFF);
                    
                } 
                if(printCount % 20000 == 0) {
                    print_buffer_csv_inline(rec1_arr, REDUCED_BUFFER_SIZE);
                }
            } else {
                startCycleCount = get_ccount();
                prosess_buffer(buffer_pong, event.end_time_us);
                endCycleCount = get_ccount();
                ESP_ERROR_CHECK(spi_device_queue_trans(spi, &trans_pong, portMAX_DELAY)); 
                if(printCount % 2000 == 0) {
                    ESP_LOGI(TAG, "Pong Buffer first 8 samples: HEX: %04X %04X %04X %04X %04X %04X %04X %04X, CYCLES: %lld, DEC: %d %d %d %d %d %d %d %d", 
                        buffer_pong[0], buffer_pong[1], buffer_pong[2], buffer_pong[3], buffer_pong[4], buffer_pong[5], buffer_pong[6], buffer_pong[7], endCycleCount-startCycleCount,
                        buffer_pong[0]&0x0FFF, buffer_pong[1]&0x0FFF, buffer_pong[2]&0x0FFF, buffer_pong[3]&0x0FFF, buffer_pong[4]&0x0FFF, buffer_pong[5]&0x0FFF, buffer_pong[6]&0x0FFF, buffer_pong[7]&0x0FFF);
                }
                if(printCount % 20000 == 0) {
                    print_buffer_csv_inline(rec1_arr, REDUCED_BUFFER_SIZE);
                }
                printCount++;
                if(misalignedSigCount > 10) {
                    ESP_LOGE(TAG,"MisalignedSigCount greater than 10 RESTARTING...");
                    esp_restart();
                }
            }
        }
    }
}


void app_main(void) {
    initialize_sine_table_Q15(ref_sine_table_Q15_39kHz,TARGET_FREQUENCY_1, REF_TABLE_SIZE_39kHz);
    initialize_cos_table_Q15(ref_cos_table_Q15_39kHz, TARGET_FREQUENCY_1, REF_TABLE_SIZE_39kHz);

    frequency_states[0] = (frequency_state_t){
        .frequency = TARGET_FREQUENCY_1,
        .ref_sine = ref_sine_table_Q15_39kHz,
        .ref_cos = ref_cos_table_Q15_39kHz,
        .ref_table_len = REF_TABLE_SIZE_39kHz,
        .signal_threshold = SIGNAL_THRESHOLD_39kHz,
        .normalized_threshold = NORMALIZED_THRESHOLD_39kHz,
        .binary_search_steps = BINARY_SEARCH_STEPS_39kHz,
    };

    esp_err_t ret;

    memset(tx_buffer, 0, sizeof(tx_buffer)); // Fill tx buffer with dummy data for full duplex

    memset(buffer_ping, 0, sizeof(buffer_ping));
    memset(buffer_pong, 0, sizeof(buffer_pong));

    dmaQueue = xQueueCreate(10, sizeof(buffer_id_t));

    if(dmaQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create DMA queue");
        return;
    }

    reset_FPGA_output(); // holds clk pin high for over 1 ms in order to communicate to FPGA that it should reset its output buffer


    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4 * sizeof(uint16_t), // First reading, gets changed to BUFFER_SIZE afterwards
        .flags = 0,
        .intr_flags = 0,
    };
    /*
    ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return;
    }
    */
   

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 24900000, // 24.9MHz 64 bits per sample period
        .mode = 0,
        .spics_io_num = -1, // Chip-select not used
        .queue_size = 2, // Allows queuing up another device for double buffering (ping pong buffering)
        .flags = 0, // Full duplex for more consistent transmission, output is not used
    };
    /*
    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add alignment SPI device: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Alignment SPI device added successfully");
    // Set up a transaction that reads 4 16-bit words (64 bits)
    spi_transaction_t trans_align = {0};
    trans_align.length = 4 * 16;
    trans_align.tx_buffer = tx_buffer;
    trans_align.rx_buffer = align_data; 
    ret = spi_device_transmit(spi, &trans_align);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Alignment SPI transaction failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Alignment data read: %04X %04X %04X %04X",align_data[0], align_data[1], align_data[2], align_data[3]);
    
    while(!align_data_acquisition()) { 
        ESP_LOGI(TAG, "Failed to align buffers");
        return;
    }
    ESP_LOGI(TAG, "Buffers aligned successfully");
    spi_bus_remove_device(spi);
    spi_bus_free(VSPI_HOST);
    */


    // Reinitialize the SPI bus with the full buffer size for continuous transfers.
    buscfg.max_transfer_sz = BUFFER_SIZE * 2;
    ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize continuous failed: %s", esp_err_to_name(ret));
        return;
    }
    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device continuous failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "SPI device re-added for continuous operation");


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

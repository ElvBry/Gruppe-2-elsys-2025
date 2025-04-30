
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
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include "dsps_dotprod.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_cpu.h"
#include <inttypes.h>


// Program that reads ADC data from an FPGA and calculates the time difference between starting points of a target sine frequency
// Log level is set to INFO meaning only LOGE/LOGW/LOGI will be output through serial in order to avoid overcrowding and triggering of the watchdog

#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23 // not used but full duplex is more synchronised
#define PIN_NUM_CLK  18

#define BUFFER_SIZE 3600
#define REDUCED_BUFFER_SIZE BUFFER_SIZE/4

#define PI 3.14159265358979323846
#define SAMPLE_FREQUENCY 390000

#define TARGET_FREQUENCY_1 39000
#define SIGNAL_THRESHOLD_39kHz 600000
#define NORMALIZED_THRESHOLD_39kHz 600000
#define BINARY_SEARCH_STEPS_39kHz 15
#define REF_TABLE_SIZE_39kHz 10

#define MAX_SIGNAL_DIFFERENCE_US 800
#define MAX_SIGNAL_DURATION_US 15000
#define SIGNAL_Q15_MAX 8192

#define MAX_CYCLE_COUNT 2*240*REDUCED_BUFFER_SIZE // estimate of CPU_FREQUENCY*REDUCED_BUFFER_SIZE/SAMPLE_FREQYENCY

#define NUM_FREQUENCIES 1
#define NUM_RECEIVERS 4



static const char *TAG = "TDOA_DSP";


typedef struct {
    uint32_t frequency;
    uint64_t last_ping_time_us;
    uint64_t cooldown_until_us;
    int16_t *ref_sine;
    int16_t *ref_cos;
    uint16_t ref_table_len;
    uint64_t signal_threshold;
    uint64_t normalized_threshold;
    uint8_t binary_search_steps;
    bool valid_flags[NUM_RECEIVERS]; // Valid after signal is detected in 2 subsequent buffers
    bool all_valid;
    uint64_t arrival_times[NUM_RECEIVERS]; // Arrival timestamps for each receiver
} frequency_state_t;

void static inline print_buffer_csv_inline(const int16_t *buffer, const size_t len) {
    char line[2048] = {0};
    char temp[16];

    for (size_t i = 0; i < len; i++) {
        snprintf(temp, sizeof(temp), "%d", buffer[i]);
        strcat(line, temp);
        if (i < len - 1) strcat(line, ",");
    }

    ESP_LOGI(TAG, "%s", line);
}

//
//  SIGNAL PROCESSING
//

int16_t ref_sine_table_Q15_39kHz[REF_TABLE_SIZE_39kHz] __attribute__((aligned(16))); // global lookuptable for target frequency sine. Format: Q15
int16_t ref_cos_table_Q15_39kHz[REF_TABLE_SIZE_39kHz] __attribute__((aligned(16))); 


void initialize_sine_table_Q15(int16_t *src,const uint32_t target_frequency, const uint8_t len) {
    double angleStep = 2*PI*target_frequency/SAMPLE_FREQUENCY;
    for (int i = 0; i < len; i++) {
        double value = sin(i*angleStep)/len; // divide by a value larger than tablesize to avoid overflow in dotproduct calculation
        src[i] = (int16_t)(value * 32767.0); // converting to Q15 format. 32767 = 2^15 -1 
    }
}

void initialize_cos_table_Q15(int16_t *src,const uint32_t target_frequency, const uint8_t len) {
    double angleStep = 2*PI*target_frequency/SAMPLE_FREQUENCY;
    for (int i = 0; i < len; i++) {
        double value = cos(i*angleStep)/len;
        src[i] = (int16_t)(value * 32767.0);
    }
}

static inline uint64_t calculate_arrival_time_us(uint64_t end_time_us, const uint16_t start_index, const uint16_t len) {
    double sample_period_us = 1000000.0 / SAMPLE_FREQUENCY;
    double delay_us = (len-start_index) * sample_period_us; // Calculate the delay as a floating point value for better precision
    uint64_t arrival_time_us = end_time_us - (uint64_t)(delay_us + 0.5);
    return arrival_time_us;
}


static inline void remove_dc_offset(int16_t* buffer, const uint16_t len) {
    int64_t sum = 0;
    for (int i = 0; i < len; i++) sum += buffer[i];

    int16_t offset = (int16_t)(sum / len);

    for (int i = 0; i < len; i++) buffer[i] -= offset;
}


static inline uint64_t calculate_signal_strength(const int16_t *ref_cos, const int16_t *ref_sine, const int16_t *signal_arr, const uint32_t len) {
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

// normalizes array to Q15 values withing safemax in order avoid problems in dotproduct function
static inline void normalize_signal_arr_Q15(int16_t *signal_arr, const uint16_t len, const uint8_t table_len) {
    int16_t abs_max = 0;
    for (size_t i = 0; i < len; i++) {
        int16_t abs_val = (signal_arr[i] < 0) ? -signal_arr[i] : signal_arr[i];
        if (abs_val > abs_max) abs_max = abs_val;
    }
    if (abs_max == 0) {
        ESP_LOGE(TAG, "Tried to normalize all 0 array");
        abs_max = 1;
    }

    // Limit scale to Q15_SAFE_MAX to prevent dot product overflow
    int32_t scale_factor = ((int32_t)SIGNAL_Q15_MAX << 15) / abs_max;

    for (int i = 0; i < len; i++) {
        int32_t temp = (int32_t)signal_arr[i] * scale_factor;
        signal_arr[i] = (int16_t)(temp >> 15);
    }
}

static inline bool signal_present(const int16_t *src, const frequency_state_t *freq_state, const uint16_t len) { 

    uint64_t strength = calculate_signal_strength(
        freq_state->ref_cos, 
        freq_state->ref_sine, 
        &src[len - freq_state->ref_table_len], 
        freq_state->ref_table_len
    );
    ESP_LOGD(TAG, "Calculated signal strength: %" PRIu64, strength);
    return freq_state->signal_threshold < strength;
}

// Finds index where signal starts in array. Returns 0xFF if signal is not present at the end 
static inline uint16_t binary_search_find_start_index(const int16_t *src, const frequency_state_t *freq_state) {
    // Search from index 0 to last analyzable index
    uint16_t low = 0;
    uint16_t high = REDUCED_BUFFER_SIZE - freq_state->ref_table_len;
    uint16_t startIndex = high;
    uint16_t steps = 0;
    
    uint64_t strength_low = calculate_signal_strength(freq_state->ref_cos, freq_state->ref_sine, &src[low], freq_state->ref_table_len);
    ESP_LOGD(TAG, "Calculated normalized strength low: %" PRIu64, strength_low);
    if (strength_low >= freq_state->normalized_threshold) {
        return 0;
    }
    uint64_t strength_high = calculate_signal_strength(freq_state->ref_cos, freq_state->ref_sine, &src[high], freq_state->ref_table_len);
    ESP_LOGD(TAG, "Calculated normalized strength high: %" PRIu64, strength_high);
    if (strength_high < freq_state->normalized_threshold) {
        ESP_LOGE(TAG, "strength high: %lld", strength_high);
        return 0xFF; // Error: the signal in the upper window is below threshold
    }

    while (low <= high && steps < freq_state->binary_search_steps) {
        uint16_t mid = low + ((high - low) >> 1);
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
static inline void deinterleave_and_clear_id(const uint16_t *src, size_t num_samples, int16_t **dst) {
    const uint16_t *end = src + num_samples;

    int16_t *local_dst[NUM_RECEIVERS];
    for (int i = 0; i < NUM_RECEIVERS; ++i) {
        local_dst[i] = dst[i]; // Copy original pointers
    }

    while (src < end) {
        for (int i = 0; i < NUM_RECEIVERS; ++i) {
            *local_dst[i]++ = *src++ & 0x0FFF;
        }
    }
}

//
//  PROGRAM SETUP
//

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
    /* debugging timing by clocking out extra bits on restart */
    for(int i = 0; i < 0; i++) {
        gpio_set_level(PIN_NUM_CLK, 1);
        esp_rom_delay_us(25);
        gpio_set_level(PIN_NUM_CLK, 0);
        esp_rom_delay_us(25);
    }    
}

//
//  TASKS
//



// Prints timestamps in microseconds relative to the receiver that received ping first
void print_receiver_values_csv(const frequency_state_t *freq_state) {
    char line[256] = {0};
    char temp[64];

    snprintf(line, sizeof(line), "%" PRIu32, freq_state->frequency);

    // Find the minimum timestamp
    uint64_t min_time = freq_state->arrival_times[0];
    for (int i = 1; i < NUM_RECEIVERS; i++) {
        if (freq_state->arrival_times[i] < min_time) {
            min_time = freq_state->arrival_times[i];
        }
    }

    // Append each offset value (always positive)
    for (int i = 0; i < NUM_RECEIVERS; i++) {
        uint64_t delta = freq_state->arrival_times[i] - min_time;
        snprintf(temp, sizeof(temp), "R%d: %" PRIu64, i + 1, delta);
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

// Returns true if the processing has used more than its maximum allowable cycles
static inline bool error_check_cycle(const int64_t maxCycleCount, const uint64_t startCycle, const uint8_t index) {
    int64_t currentCycle = get_ccount();
    if(startCycle + maxCycleCount < currentCycle) {
        ESP_LOGE(TAG,"ERROR: cycleCount too high %u, cycle: %lld", index, currentCycle-startCycle);
        return true;
    }
    return false;
}




// Searches through buffer to find signal of targeted frequency, if signal is found the arrival time will be logged.
// If signal was found in previous buffer the receiver will mark the signal as valid. If the flag was the last one the all_valid flag will be set to true
static inline void find_signal_present(int16_t *src, frequency_state_t *freq_state, const uint8_t index, const uint64_t end_time_us, const uint64_t startCycle) {
    if(freq_state->valid_flags[index] == true) {
        ESP_LOGD(TAG, "Skip processing receiver: %u: ", index);
        return; // Avoid looking for already found signal
    }
    
    // Second measurement of signal
    if(freq_state->arrival_times[index] != 0) {
        if(signal_present(src, freq_state, freq_state->ref_table_len)) {
            freq_state->valid_flags[index] = true;
            for (int i = 0; i < NUM_RECEIVERS; i++) {
                if(!freq_state->valid_flags[i]) break;
                if(i == NUM_RECEIVERS -1) freq_state->all_valid = true;
            }
        } else freq_state->arrival_times[index] = 0;
        return;
    }
    
    
   
    //if(src[1] != 0) print_buffer_csv_inline(src, REDUCED_BUFFER_SIZE);
    ESP_LOGD(TAG, "Processing signal");
    if(!signal_present(src, freq_state, REDUCED_BUFFER_SIZE)) {
        ESP_LOGD(TAG, "Signal processing failed");
        return; // Initial check for signal in order to avoid normalizing noise which breaks binary search
    }
    //print_buffer_csv_inline(src, REDUCED_BUFFER_SIZE);
    if(src[REDUCED_BUFFER_SIZE] == false) {
        normalize_signal_arr_Q15(src, REDUCED_BUFFER_SIZE, freq_state->ref_table_len);
        src[REDUCED_BUFFER_SIZE] = (int16_t)true;
       
    }
    uint16_t estimated_start_index = binary_search_find_start_index(src, freq_state);
    if(estimated_start_index == 0xFF) {
        //print_buffer_csv_inline(src+REDUCED_BUFFER_SIZE/4, REDUCED_BUFFER_SIZE/4);
        ESP_LOGD(TAG, "could not find signal after normalization");
        return;
    }
    
    ESP_LOGD(TAG,"Found signal receiver: %u, cycles: %lld", index, get_ccount()-startCycle);
    //print_buffer_csv_inline(src+estimated_start_index, REDUCED_BUFFER_SIZE-estimated_start_index);
    /*
    print_buffer_csv_inline(rec1_arr, REDUCED_BUFFER_SIZE); // debug
    print_buffer_csv_inline(rec2_arr, REDUCED_BUFFER_SIZE);
    print_buffer_csv_inline(rec3_arr, REDUCED_BUFFER_SIZE);
    print_buffer_csv_inline(rec4_arr, REDUCED_BUFFER_SIZE);
    */
    uint64_t estimated_arrival_time_us = calculate_arrival_time_us(end_time_us,estimated_start_index, REDUCED_BUFFER_SIZE);
    if(estimated_arrival_time_us > freq_state->last_ping_time_us)
        freq_state->last_ping_time_us = estimated_arrival_time_us;
    freq_state->arrival_times[index] = estimated_arrival_time_us; // First successful measurement of signal
}

// Opposite of find_signal_present checks if the signal has dissapeared in two buffers in a row
// First signal not detected leads to removed valid flag, second to removed arrival time. When all are gone valid_flags is removed
// If a signal is detected both arrival time and valid flag are reset
static inline void check_signal_gone(int16_t *src, frequency_state_t *freq_state, uint8_t index, uint64_t end_time_us) {
    
    if (signal_present(src, freq_state, REDUCED_BUFFER_SIZE)) {  // If signal is present at end of buffer Signal present, return
        freq_state->valid_flags[index] = true;
        freq_state->arrival_times[index] = end_time_us; // reset timer
        return;
    }

    if (freq_state->valid_flags[index]) { // First removal
        freq_state->valid_flags[index] = 0;
        return;
    }
    
    freq_state->arrival_times[index] = 0; // Second removal

    // If all receivers have lost the signal, reset the overall valid flag.
    bool allZero = true;
    for (int i = 0; i < NUM_RECEIVERS; i++) {
        if (freq_state->arrival_times[i] != 0) {
            allZero = false;
            break;
        }
    }
    if (allZero) {
        freq_state->last_ping_time_us = 0;
        freq_state->all_valid = 0;
    }
}

// Allow frequency to be measured again by setting its updatable values to 0
static inline void reset_frequency_state(frequency_state_t *freq_state) {
    freq_state->cooldown_until_us = esp_timer_get_time() + MAX_SIGNAL_DURATION_US;
    freq_state->all_valid = false;
    freq_state->last_ping_time_us = 0;
    for(int i = 0; i < NUM_RECEIVERS; i++) {
        freq_state->valid_flags[i] = false;
        freq_state->arrival_times[i] = 0;
    }
}

frequency_state_t frequency_states[NUM_FREQUENCIES] = {{
    .cooldown_until_us = 0,
    .valid_flags = {false,false,false,false},
    .all_valid = false,
    .last_ping_time_us = 0,
    .arrival_times = {0,0,0,0},
}};

void overwrite_receiver_with_ref_cos(int16_t *receiver, const int16_t *ref_cos, const uint8_t ref_len, const uint8_t num_copies) {
    for (int copy = 0; copy < num_copies; copy++) {
        for (int i = 0; i < ref_len; i++) {
            receiver[copy * ref_len + i] = ref_cos[i];
        }
    }
}


int16_t* receiver_arrays[NUM_RECEIVERS] = {rec1_arr, rec2_arr, rec3_arr, rec4_arr};

static inline void prepare_buffers(uint16_t *src) {
    swap_buffer_bytes(src, BUFFER_SIZE);
    deinterleave_and_clear_id(src, BUFFER_SIZE, receiver_arrays);

    for (int i = 0; i < NUM_RECEIVERS; ++i) {
        receiver_arrays[i][0] = receiver_arrays[i][1]; // First value often malformed
        remove_dc_offset(receiver_arrays[i], REDUCED_BUFFER_SIZE);
        //overwrite_receiver_with_ref_cos(receiver_arrays[i], frequency_states[0].ref_cos, REF_TABLE_SIZE_39kHz, 6); debug
        
    }
    //ESP_LOGI(TAG, "Finished overwrite");
}

static inline void process_receivers(uint64_t end_time_us, const uint64_t startCycle) {
    uint64_t start_time_us = calculate_arrival_time_us(end_time_us, 0, REDUCED_BUFFER_SIZE);
    for (int i = 0; i < NUM_FREQUENCIES; i++) {
        if(frequency_states[i].all_valid) {
            
            for(int j = 0; j < NUM_RECEIVERS; j++) {
                check_signal_gone(receiver_arrays[j], &frequency_states[i], j, end_time_us);
            }
            
            // Old approach:
            /*
            ESP_LOGD(TAG, "ALL FOUND: %lld", end_time_us);
            if((start_time_us > frequency_states[i].cooldown_until_us)) {
                reset_frequency_state(&frequency_states[i]);  // Avoid searching for frequency if it has been found. After a duration it can be measured again
                ESP_LOGD(TAG, "reset frequency max_duration");
            }
            */
            
            continue;
        }

        // Targeted frequency is reset if not enough receivers noticed it within a realistic timeframe
        if((frequency_states[i].last_ping_time_us > 0) && (start_time_us > frequency_states[i].last_ping_time_us + MAX_SIGNAL_DIFFERENCE_US)) {
            reset_frequency_state(&frequency_states[i]);
            ESP_LOGD(TAG, "reset frequency max_difference");
            continue;
        }

        // If the arrival time for two receivers are the same a problem has almost certainly occured and we should ignore the reading
        bool any_equal = false;
        for (int a = 0; a < NUM_RECEIVERS; a++) {
            for (int b = a + 1; b < NUM_RECEIVERS; b++) {
                if (frequency_states[i].arrival_times[a] == 0) break;
                if (frequency_states[i].arrival_times[a] == frequency_states[i].arrival_times[b]) {
                    any_equal = true;
                    break;
                }
            }
            if (any_equal) break;
        }

        if (any_equal) {
            ESP_LOGD(TAG, "Duplicate arrival times detected, resetting.");
            reset_frequency_state(&frequency_states[i]);
            continue;
        }

        if(error_check_cycle(MAX_CYCLE_COUNT, startCycle, 100)) break;
        ESP_LOGD(TAG, "Before update");

        for(int j = 0; j < NUM_RECEIVERS; j++) {
            find_signal_present(receiver_arrays[j], &frequency_states[i], j, end_time_us, startCycle);
            if(error_check_cycle(MAX_CYCLE_COUNT, startCycle, j)) break;
            ESP_LOGD(TAG, "finished update %d", j + 1);
        }
        
        if(frequency_states[i].all_valid == true)  print_receiver_values_csv(&frequency_states[i]);
    }
    for(int j = 0; j < NUM_RECEIVERS; j++) {
        receiver_arrays[j][REDUCED_BUFFER_SIZE] = 0;
    }
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
    static uint32_t misalignedSigCount = 0;
    int64_t startCycleCount;
    int64_t endCycleCount;
    while(1) {
        // waits for notification that a DMA buffer is finished
        if(xQueueReceive(dmaQueue, &event, portMAX_DELAY) == pdTRUE) {
            
            if (event.buf_id == BUFFER_PING) {
                misalignedSigCount = (((buffer_ping[0] &0xF000) != 0x0000) ||
                                      ((buffer_ping[1] &0xF000) != 0x1000) ||
                                      ((buffer_ping[2] &0xF000) != 0x2000) ||
                                      ((buffer_ping[3] &0xF000) != 0x3000)) ? misalignedSigCount +1 : 0;
                prepare_buffers(buffer_ping);
                startCycleCount = get_ccount();
                process_receivers(event.end_time_us,startCycleCount);
                endCycleCount = get_ccount();
                ESP_LOGD(TAG, "TIME: %lld, CYCLES: %lld", event.end_time_us, endCycleCount-startCycleCount);
                ESP_ERROR_CHECK(spi_device_queue_trans(spi, &trans_ping, portMAX_DELAY)); // requeue the ping transaction

                if(printCount % 2000 == 0) {
                    ESP_LOGD(TAG, "Ping Buffer first 8 samples: HEX: %04X %04X %04X %04X %04X %04X %04X %04X, CYCLES: %lld, DEC: %d %d %d %d %d %d %d %d", 
                        buffer_ping[0], buffer_ping[1], buffer_ping[2], buffer_ping[3], buffer_ping[4], buffer_ping[5], buffer_ping[6], buffer_ping[7], endCycleCount-startCycleCount,
                        buffer_ping[0]&0x0FFF, buffer_ping[1]&0x0FFF, buffer_ping[2]&0x0FFF, buffer_ping[3]&0x0FFF, buffer_ping[4]&0x0FFF, buffer_ping[5]&0x0FFF, buffer_ping[6]&0x0FFF, buffer_ping[7]&0x0FFF);   
                } 
                /*
                if(printCount % 2000000 == 0) {
                    print_buffer_csv_inline(rec1_arr, REDUCED_BUFFER_SIZE);
                }
                */
                
                //print_buffer_csv_inline(rec1_arr, REDUCED_BUFFER_SIZE);
            } else {
                prepare_buffers(buffer_pong);
                startCycleCount = get_ccount();
                process_receivers(event.end_time_us,startCycleCount);
                endCycleCount = get_ccount();
                ESP_ERROR_CHECK(spi_device_queue_trans(spi, &trans_pong, portMAX_DELAY)); 
                if(printCount % 2000 == 0) {
                    ESP_LOGD(TAG, "Pong Buffer first 8 samples: HEX: %04X %04X %04X %04X %04X %04X %04X %04X, CYCLES: %lld, DEC: %d %d %d %d %d %d %d %d", 
                        buffer_pong[0], buffer_pong[1], buffer_pong[2], buffer_pong[3], buffer_pong[4], buffer_pong[5], buffer_pong[6], buffer_pong[7], endCycleCount-startCycleCount,
                        buffer_pong[0]&0x0FFF, buffer_pong[1]&0x0FFF, buffer_pong[2]&0x0FFF, buffer_pong[3]&0x0FFF, buffer_pong[4]&0x0FFF, buffer_pong[5]&0x0FFF, buffer_pong[6]&0x0FFF, buffer_pong[7]&0x0FFF);
                }
                if(printCount % 20000 == 0) {
                    //print_buffer_csv_inline(rec2_arr, REDUCED_BUFFER_SIZE);
                }
                printCount++;
                if(misalignedSigCount > 10000) {
                ESP_LOGE(TAG,"MisalignedSigCount greater than 10000 RESTARTING...");
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

    dmaQueue = xQueueCreate(10, sizeof(dma_event_t));

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
        .max_transfer_sz =  BUFFER_SIZE * sizeof(uint16_t), // amount of bytes in each buffer
        .flags = 0,
        .intr_flags = 0,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SAMPLE_FREQUENCY*NUM_RECEIVERS*16, // 24.9 MHz reads data at the sampling rate
        .mode = 0,
        .spics_io_num = -1, // Chip-select not used
        .queue_size = 2, // Allows queuing up another device for double buffering (ping pong buffering)
        .flags = 0, // Full duplex for more consistent transmission, output is not used
    };
    
    ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return;
    }
    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
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
    ESP_LOGI(TAG, "Trans ping queued successfully");


    ret = spi_device_queue_trans(spi, &trans_pong, portMAX_DELAY);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Queue trans pong failed: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Trans pong queued successfully");

    // create freeRTOS tasks that will run continuously
    xTaskCreate(acquisition_task, "acquisition_task", 4096, NULL, 5, NULL);
    xTaskCreate(processing_task, "processing_task", 4096, NULL, 5, NULL);
}

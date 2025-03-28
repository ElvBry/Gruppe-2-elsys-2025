// This code represents the old approach that was used to align buffers.
// It remains unused because we are able to synch the data transfer with the FPGA at the very start of the program.
// This approach still needs extra shifts for unexptected startups and restarting SPI transactions multiple times
// instead of restarting proved to be too cumbersome.



#define secondIndex 16
#define thirdIndex 32

#define extraShift 0

uint16_t align_data[4] __attribute__((aligned(16)));

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


void app_main() {
    
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz =  4 * sizeof(uint16_t), // 4 full samples
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
    ret = spi_device_transmit(spi, &trans_align); // Transmits a single transaction
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
    spi_bus_free(VSPI_HOST); // Back to new program
}

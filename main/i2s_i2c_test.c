#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <string.h>
#include <esp_timer.h>
#include <esp_vfs_fat.h>
#include <freertos/ringbuf.h>

// Include I2S driver
#include <driver/i2s.h>

// Incluse SD card driver
#include <sdcard.h>
#include <mhz14a.h>
#include <bme.h>
// Connections to INMP441 I2S microphone
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32

// Use I2S Processor 0
#define I2S_PORT I2S_NUM_0

#define bufferCount 8
#define bufferLen 64
int16_t sBuffer16[bufferLen];
int8_t sBuffer8[bufferLen * 2];


int16_t buffer16[bufferLen] = {0};
uint8_t buffer32[bufferLen * 4] = {0};

uint16_t bufferTemp[16] = {0};

// Buffer for data to save to SD card
RingbufHandle_t buf_handle_max;
RingbufHandle_t buf_handle_inm;

static char data_max[400] = "";
static char data_inm[bufferLen * 8] = "";

// config for protocol uart
uart_config_t uartMHZ14a ={
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0, 
    .source_clk = UART_SCLK_DEFAULT
};
// Config pin of protocol i2c for bme
#define SDA_PIN 21                     
#define SCL_PIN 22  





// Set up I2S Processor configuration
void i2s_install() {
  // Set up I2S Processor configuration
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 4000, // or 44100 if you like
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Ground the L/R pin on the INMP441.
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S |I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = bufferCount,
    .dma_buf_len = bufferLen,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0,
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
}

// Set I2S pin configuration
void i2s_setpin() {
    const i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = -1,
        .data_in_num = I2S_SD
    };

    i2s_set_pin(I2S_PORT, &pin_config);
}

/**
 * @brief Read data from INMP441 and send to ring buffer
 * 
 * @param pvParameters 
 */
void readData(void* parameter) {

    // Set up I2C
    i2s_install();
    i2s_setpin();
    i2s_start(I2S_PORT);

    size_t bytesRead = 0;
    char data_temp[8] = ""; 
    uint32_t co2_ppm=0; 
/*
    while (1) {
        vTaskDelay(1); // Feed for watchdog, if not watchdog timer will be triggered!

        esp_err_t result = i2s_read(I2S_PORT, &sBuffer16, sizeof(sBuffer16), &bytesRead, portMAX_DELAY);
        //printf("%d\n", bytesIn);
        int sampleRead = bytesRead / 2;
        if (result == ESP_OK) {
            for (uint8_t i = 0; i < sampleRead; i += 1) {
                // memset(data_temp, 0, sizeof(data_temp));
                // sprintf(data_temp, "\n%d", sBuffer16[i]);
                // strcat(data_inm, data_temp);
                printf("%d %d %d\n", 3000, -3000, sBuffer16[i]);
            }
        }
        // xRingbufferSend(buf_handle_inm, data_inm, sizeof(data_inm), pdMS_TO_TICKS(5));
        // memset(data_inm, 0, sizeof(data_inm));
    }
*/

    while (1)
    {
        vTaskDelay(1); // Feed for watchdog, if not watchdog timer will be triggered!

        i2s_read(I2S_PORT, &buffer32, sizeof(buffer32), &bytesRead, 100);
        int samplesRead = bytesRead / 4;
        ESP_ERROR_CHECK_WITHOUT_ABORT(mhz14a_getDataFromSensorViaUART(&co2_ppm));
        ESP_LOGI("MHZ14A", "CO2: %lu", co2_ppm);
        for (uint8_t i = 0; i < samplesRead; i++) {
            uint8_t mid = buffer32[i * 4 + 2];
            uint8_t msb = buffer32[i * 4 + 3];
            uint16_t raw = (((uint32_t)msb) << 8) + ((uint32_t)mid);
            memcpy(&buffer16[i], &raw, sizeof(raw)); // Copy so sign bits aren't interfered with somehow.
            // printf("%d %d %d\n", 3000, -3000, buffer16[i]);
            
            memset(data_temp, 0, sizeof(data_temp));
            sprintf(data_temp, "\n%d", buffer16[i]);
            strcat(data_inm, data_temp);
            
        }
        
        xRingbufferSend(buf_handle_inm, data_inm, sizeof(data_inm), pdMS_TO_TICKS(5));
        memset(data_inm, 0, sizeof(data_inm));
        
    }
    

    // while(1) 
    // {
    //     vTaskDelay(1);

    //     i2s_read(I2S_PORT, &bufferTemp, sizeof(bufferTemp), &bytesRead, 100);

    //     printf("----%d----\n", bufferTemp[3]);
    // }
}

/**
 * @brief Receive data from 2 ring buffers and save them to SD card
 * 
 * @param parameter 
 */
void saveDataToSD(void *parameter) {
  while(1) {
    size_t item_size1;
    size_t item_size2;

    //Receive an item from no-split INMP441 ring buffer
    char *item1 = (char *)xRingbufferReceive(buf_handle_inm, &item_size1, 1);

    //Check received item
    if (item1 != NULL) {
      //Return Item
      // Serial.println("r");
      vRingbufferReturnItem(buf_handle_inm, (void *)item1);
      sdcard_writeDataToFile("test", item1);
    } 

    //Receive an item from no-split MAX30102 ring buffer
    char *item2 = (char *)xRingbufferReceive(buf_handle_max, &item_size2, 1);

    //Check received item
    if (item2 != NULL) {
      //Return Item
      // Serial.println("rev");
      vRingbufferReturnItem(buf_handle_max, (void *)item2);
      sdcard_writeDataToFile("hello", item2);
    } 
  }
}

void app_main(void)
{
    // Initialize SPI Bus
    ESP_LOGI(__func__, "Initialize SD card with SPI interface.");
    esp_vfs_fat_mount_config_t mount_config_t = MOUNT_CONFIG_DEFAULT();
    spi_bus_config_t spi_bus_config_t = SPI_BUS_CONFIG_DEFAULT();
    sdmmc_host_t host_t = SDSPI_HOST_DEFAULT();
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = CONFIG_PIN_NUM_CS;
    slot_config.host_id = host_t.slot;

    sdmmc_card_t SDCARD;
    ESP_ERROR_CHECK_WITHOUT_ABORT(sdcard_initialize(&mount_config_t, &SDCARD, &host_t, &spi_bus_config_t, &slot_config));

    // Initialise ring buffers
    buf_handle_max = xRingbufferCreate(1028 * 3, RINGBUF_TYPE_NOSPLIT);
    buf_handle_inm = xRingbufferCreate(1028 * 5, RINGBUF_TYPE_NOSPLIT);
    // Initialise mhz14
    ESP_LOGI("MHZ14A","hhhh");
    ESP_ERROR_CHECK_WITHOUT_ABORT(mhz14a_initUART(&uartMHZ14a));
    ESP_LOGI("MHZ14A","hhhh");
    // Initialise i2c for bme
    i2c_master_bme_init(SDA_PIN, SCL_PIN);


    // Set up I2C
    // ESP_ERROR_CHECK(i2cdev_init()); 
    
    // Create tasks
    // xTaskCreatePinnedToCore(max30102_test, "max30102_test", 1024 * 5, NULL, 6, NULL, 0);
    bme280_reader_task(void *ignore);
    xTaskCreatePinnedToCore(readData, "readData", 1024 * 10, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(saveDataToSD, "saveToSD", 1024 * 5, NULL, 5, NULL, 1);
}

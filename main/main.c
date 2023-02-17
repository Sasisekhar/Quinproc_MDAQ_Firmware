#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"
#include <math.h>
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_system.h"
#include <math.h>
#include "esp_dsp.h"
#include "WiFiHelper.h"

i2s_chan_handle_t rx_handle;

QueueHandle_t buffQueue;

#define N_SAMPLES 2048

void LPF(int16_t*);

static void readTask (void *args) {
    int32_t raw_buf[N_SAMPLES];
    int16_t output_buf[N_SAMPLES];
    size_t r_bytes = 0;
    int samples_read = 0;

    buffQueue = xQueueCreate(5, sizeof(output_buf));

    while(1) {
        if(i2s_channel_read(rx_handle, raw_buf, sizeof(int32_t) * N_SAMPLES, &r_bytes, portMAX_DELAY) == ESP_OK) {
            samples_read = r_bytes/sizeof(int32_t);
            int sample_index = 0;
            for (int i = 0; i < samples_read; i++) {
                output_buf[sample_index++] = (raw_buf[i] & 0xFFFFFFF0) >> 11;
            }
            xQueueSend(buffQueue, (void*)output_buf, (TickType_t) 0);
        }
    }
}

static void printTask (void *args) {
    int16_t rxBuffer[N_SAMPLES];
    int k = 0;
    while(1) {
        if(buffQueue != 0) {
            if(xQueueReceive(buffQueue, &(rxBuffer), (TickType_t)10)){
                // ESP_LOGI("printTask", "%d", rxBuffer[0]);

                LPF(rxBuffer);

                // if(k == 100) {
                //     LPF(rxBuffer);
                //     k = 0;
                // } else {
                //     k++;
                // }
            }
        }
    }
}

void LPF(int16_t *inputBuffer) {

    float freq = 0.0625;
    float qFactor = 1;

    esp_err_t ret = ESP_OK;
    float coeffs_lpf[5];
    float w_lpf[5] = {0,0};

    float leftInBuffer[N_SAMPLES/2];
    float rightInBuffer[N_SAMPLES/2];

    float leftOutBuffer[N_SAMPLES/2];
    float rightOutBuffer[N_SAMPLES/2];

    int k = 0;
    //Preprocessing input signal
    for(int i = 0; i < N_SAMPLES; i += 2) {
        // leftInBuffer[k] = ((float)((inputBuffer[i] >> 8)*2)/8388608) - 1;
        // rightInBuffer[k++] = ((float)((inputBuffer[i + 1] >> 8)*2)/8388608) - 1;

        leftInBuffer[k] = (float)(inputBuffer[i]);
        rightInBuffer[k++] = (float)(inputBuffer[i + 1]);
    }
    
    // Calculate iir filter coefficients
    ret = dsps_biquad_gen_lpf_f32(coeffs_lpf, freq, qFactor);
    if (ret  != ESP_OK) {
        ESP_LOGE("LPF", "Operation error = %i", ret);
        return;
    }

    // Process input signals
    //Left Channel
    ret = dsps_biquad_f32_ae32(leftInBuffer, leftOutBuffer, N_SAMPLES/2, coeffs_lpf, w_lpf);
    if (ret  != ESP_OK){
        ESP_LOGE("LPF", "Operation error = %i", ret);
        return;
    }
    //Right Channel
    ret = dsps_biquad_f32_ae32(rightInBuffer, rightOutBuffer, N_SAMPLES/2, coeffs_lpf, w_lpf);
    if (ret  != ESP_OK){
        ESP_LOGE("LPF", "Operation error = %i", ret);
        return;
    }

    for(int i = 0; i < 50; i++) {
        printf("%f\n", leftOutBuffer[i]);
    }
}

void app_main(void) {

    initNVS();

    WiFiConnectHelper();

    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    i2s_new_channel(&rx_chan_cfg, NULL, &rx_handle);

    i2s_std_config_t rx_std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = 8000,
            .mclk_multiple = 600,
        },
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t) 25,
            .ws = (gpio_num_t) 26,
            .dout = I2S_GPIO_UNUSED,
            .din = (gpio_num_t) 32,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    i2s_channel_init_std_mode(rx_handle, &rx_std_cfg);
    i2s_channel_enable(rx_handle);
    // xTaskCreate(readTask, "readTask", 65536, NULL, 5, NULL);
    // xTaskCreate(printTask, "printTask", 65536, NULL, 5, NULL);
}

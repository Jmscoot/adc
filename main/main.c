#include "esp_adc/adc_continuous.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <string.h>

#define _EXAMPLE_ADC_UNIT_STR(unit) #unit
#define EXAMPLE_ADC_UNIT_STR(unit) _EXAMPLE_ADC_UNIT_STR(unit)

#define GET_CHANNEL(p_data) ((p_data)->type1.channel)
#define GET_DATA(p_data) ((p_data)->type1.data)

#define READ_LEN 32

static adc_channel_t channel[1] = {ADC_CHANNEL_7};

static TaskHandle_t s_task_handle;
static const char *TAG = "ADC";

float adc_to_voltage(uint32_t adc_val) {
  return (adc_val * 3.3) / 4095.0; // 2^12=4096, 12bits
}
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle,
                                     const adc_continuous_evt_data_t *edata,
                                     void *user_data) {
  BaseType_t mustYield = pdFALSE;
  // Notify that ADC continuous driver has done enough number of conversions
  vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

  return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num,
                                adc_continuous_handle_t *out_handle) {
  adc_continuous_handle_t handle = NULL;

  adc_continuous_handle_cfg_t adc_config = {
      .max_store_buf_size = 1024,
      .conv_frame_size = READ_LEN,
  };
  ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

  adc_continuous_config_t dig_cfg = {
      .sample_freq_hz =
          20 * 1000, // 20,000(sample/sec)*2(bytes/sample)=40,000(bytes/sec)
      .conv_mode = ADC_CONV_SINGLE_UNIT_1,
      .format = ADC_DIGI_OUTPUT_FORMAT_TYPE1,
  };

  adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
  dig_cfg.pattern_num = channel_num;
  for (int i = 0; i < channel_num; i++) {
    adc_pattern[i].atten = ADC_ATTEN_DB_12;
    adc_pattern[i].channel = channel[i] & 0x7;
    adc_pattern[i].unit = ADC_UNIT_1;
    adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

    ESP_LOGI(TAG, "adc_pattern[%d].atten is :%" PRIu8, i, adc_pattern[i].atten);
    ESP_LOGI(TAG, "adc_pattern[%d].channel is :%" PRIu8, i,
             adc_pattern[i].channel);
    ESP_LOGI(TAG, "adc_pattern[%d].unit is :%" PRIu8, i, adc_pattern[i].unit);
  }
  dig_cfg.adc_pattern = adc_pattern;
  ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

  *out_handle = handle;
}

void adc_task(void *param) {
  while (1) {
  }
}

void app_main(void) {
  esp_err_t ret;
  uint32_t ret_num = 0;
  uint8_t result[READ_LEN] = {0};
  memset(result, 0xcc, READ_LEN);

  s_task_handle = xTaskGetCurrentTaskHandle();

  adc_continuous_handle_t handle = NULL;
  continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t),
                      &handle);

  adc_continuous_evt_cbs_t cbs = {
      .on_conv_done = s_conv_done_cb,
  };
  ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));
  ESP_ERROR_CHECK(adc_continuous_start(handle));

  //   bool calibrated = adc_calibration_init();

  while (1) {

    /**
     * This is to show you the way to use the ADC continuous mode driver
     event
     * callback. This `ulTaskNotifyTake` will block when the data processing
     in
     * the task is fast. However in this example, the data processing (print)
     is
     * slow, so you barely block here.
     *
     * Without using this event callback (to notify this task), you can still
     * just call `adc_continuous_read()` here in a loop, with/without a
     certain
     * block timeout.
     */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // char unit[] = EXAMPLE_ADC_UNIT_STR(ADC_UNIT_1);
    float avg_vol = 0;
    while (1) {
      ret = adc_continuous_read(handle, result, READ_LEN, &ret_num, 0);
      if (ret == ESP_OK) {
        ESP_LOGI("TASK", "ret is %x, ret_num is %" PRIu32 " bytes", ret,
                 ret_num);
        for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {

          // sample하나당 16bit(2bytes) 12(data)+4(ch)
          adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
          uint32_t chan_num = GET_CHANNEL(p);
          uint32_t data = GET_DATA(p);
          float voltage = adc_to_voltage(data);
          /* Check the channel number validation, the data is invalid if the
           * channel num exceed the maximum channel */
          //   if (chan_num < SOC_ADC_CHANNEL_NUM(ADC_UNIT_1)) {
          //     ESP_LOGI(TAG,
          //              "Unit: %s, Channel: %" PRIu32 ", Value: %" PRIu32
          //              ", Voltage: %.3fV",
          //              unit, chan_num, data, voltage);
          //   } else {
          //     ESP_LOGW(TAG, "Invalid data [%s_%" PRIu32 "_%" PRIu32 "]",
          //     unit,
          //              chan_num, data);
          //   }
          avg_vol += voltage;
        }
        avg_vol = avg_vol / 32;
        if (avg_vol < 0.1) {
          ESP_LOGI(TAG, "0");
        } else if (avg_vol >= 0.1 && avg_vol < 0.3) {
          ESP_LOGI(TAG, "1");
        } else if (avg_vol >= 0.3 && avg_vol < 0.5) {
          ESP_LOGI(TAG, "2");
        } else if (avg_vol >= 0.5 && avg_vol < 0.6) {
          ESP_LOGI(TAG, "3");
        } else if (avg_vol >= 0.6) {
          ESP_LOGI(TAG, "4");
        }
        /**
         * Because printing is slow, so every time you call
         `ulTaskNotifyTake`,
         * it will immediately return. To avoid a task watchdog timeout, add
         a
         * delay here. When you replace the way you process the data,
         usually
         * you don't need this delay (as this task will block for a while).
         */
        vTaskDelay(1);
      } else if (ret == ESP_ERR_TIMEOUT) {
        // We try to read `EXAMPLE_READ_LEN` until API returns timeout, which
        // means there's no available data
        break;
      }
    }
  }

  ESP_ERROR_CHECK(adc_continuous_stop(handle));
  ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}

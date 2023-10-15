//
// Created by Lorenzo on 14/10/2023.
//

#ifndef ESPTESTING_MOISTURESENSOR_H
#define ESPTESTING_MOISTURESENSOR_H
#define MAX_MOISTURE 1500
#include "common_sensor_interface.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/adc.h"

class MoistureSensor : public FixedRateSensorInterface<uint32_t>{
private:
    //config
    adc1_channel_t _channel;
    uint8_t _filter_samples_count;
    uint64_t _sampling_rate;
    //values
    uint32_t _final_reading;
    esp_err_t _error;
    TaskHandle_t _handle;
    //locks
    SemaphoreHandle_t _reading_lock;
    SemaphoreHandle_t _sample_rate_lock;

    static void task_code(void* param);
public:
    /*
     * Create the moisture sensor object
     * in the sampling task a tick is waited for every sample in the filter
     */
    MoistureSensor(adc1_channel_t channel, uint8_t filter_samples_count);

    ~MoistureSensor();

    esp_err_t start_sampling(const char* task_name) override;

    esp_err_t stop_sampling() override;

    esp_err_t set_sampling_rate(uint64_t millis) override;

    esp_err_t get_sampling_rate(uint64_t &millis) override;

    esp_err_t get_sampling_status(bool &sampling_status) override;

    esp_err_t get_readings(uint32_t &store) override;

    esp_err_t get_last_error() override;

};


#endif //ESPTESTING_MOISTURESENSOR_H

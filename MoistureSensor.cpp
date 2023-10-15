//
// Created by Lorenzo on 14/10/2023.
//

#include "MoistureSensor.h"

#define lock(mtx) {if(xSemaphoreTake(mtx, portMAX_DELAY) != pdTRUE){_error = ESP_ERR_INVALID_STATE;return ESP_ERR_INVALID_STATE;}}
#define unlock(mtx) {if (xSemaphoreGive(mtx) != pdTRUE){_error = ESP_ERR_INVALID_STATE;return ESP_ERR_INVALID_STATE;}}


MoistureSensor::MoistureSensor(adc1_channel_t channel, uint8_t filter_samples_count){
    esp_err_t tmp;
    _filter_samples_count = filter_samples_count;
    _error = ESP_OK;
    _final_reading = 0;
    _sampling_rate = 1000;
    _handle = NULL;
    _channel = channel;
    _error = ((_reading_lock = xSemaphoreCreateMutex()) != NULL)? _error: ESP_ERR_INVALID_STATE;
    _error = ((_sample_rate_lock = xSemaphoreCreateMutex()) != NULL)? _error: ESP_ERR_INVALID_STATE;
    _error = ((tmp = adc1_config_width(ADC_WIDTH_BIT_12)) == ESP_OK)? _error : tmp;
    _error = ((tmp = adc1_config_channel_atten(channel, ADC_ATTEN_DB_11)) == ESP_OK)? _error : tmp;
}

MoistureSensor::~MoistureSensor(){
    if(_handle != NULL){
        vTaskDelete(_handle);
        _handle = NULL;
    }
    vSemaphoreDelete(_reading_lock);
    vSemaphoreDelete(_sample_rate_lock);
}

esp_err_t MoistureSensor::start_sampling(const char* task_name) {
    stop_sampling();
    BaseType_t code = xTaskCreate(
            &MoistureSensor::task_code,
            task_name,
            1000,
            (void*)this,
            tskIDLE_PRIORITY,
            &_handle
    );
    if(code == pdPASS){
        return ESP_OK;
    }else{
        _error = ESP_FAIL;
        return  _error;
    }
}

esp_err_t MoistureSensor::stop_sampling() {
    if(_handle != NULL){
        lock(_reading_lock);
        vTaskDelete(_handle);
        unlock(_reading_lock);
        _handle = NULL;
    }
    return ESP_OK;
}

esp_err_t MoistureSensor::set_sampling_rate(uint64_t millis) {
    lock(_sample_rate_lock);
    _sampling_rate = millis;
    unlock(_sample_rate_lock);
    return ESP_OK;
}

esp_err_t MoistureSensor::get_sampling_rate(uint64_t &millis) {
    lock(_sample_rate_lock);
    millis = _sampling_rate;
    unlock(_sample_rate_lock);
    return ESP_OK;
}

esp_err_t MoistureSensor::get_sampling_status(bool &sampling_status) {
    sampling_status = _handle != NULL;
    return ESP_OK;
}

esp_err_t MoistureSensor::get_readings(uint32_t &store) {
    lock(_reading_lock);
    store = _final_reading;
    unlock(_reading_lock);
    return ESP_OK;
}

esp_err_t MoistureSensor::get_last_error(){
    return _error;
}

void MoistureSensor::task_code(void *param) {
    auto* self = (MoistureSensor*)param;
    uint64_t increment;
    uint32_t filtered;
    TickType_t prev_wake = xTaskGetTickCount();
    for(;;){
        if (xSemaphoreTake(self->_reading_lock, portMAX_DELAY) != pdTRUE){
            self->_handle = NULL;
            vTaskDelete(NULL);
            return;
        }
        filtered = 0;
        for (int i = 0; i < self->_filter_samples_count; ++i) {
            filtered += adc1_get_raw(self->_channel);
            xTaskDelayUntil(&prev_wake, 1);
        }
        filtered /= self->_filter_samples_count;
        self->_final_reading = 1000 - (filtered*1000/MAX_MOISTURE);
        if (xSemaphoreGive(self->_reading_lock) != pdTRUE){
            self->_handle = NULL;
            vTaskDelete(NULL);
            return;
        }
        self->get_sampling_rate(increment);
        increment -= pdTICKS_TO_MS(self->_filter_samples_count);
        xTaskDelayUntil(&prev_wake, pdMS_TO_TICKS(increment));
    }
}
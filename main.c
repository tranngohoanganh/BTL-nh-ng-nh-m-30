
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "ssd1306.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_http_client.h"

#define WIFI_SSID "Internet Photo H2"
#define WIFI_PASSWORD "hoidelamgi"
#define SERVER_URL "https://script.google.com/macros/s/AKfycbwmrmEyB9YCGmGB9exB4-NzqBHdx9YVXBeWGA5VKhyKm8brUU1Or_lAizQN4l-w-W-xDw/exec"

static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// GPIO Pin Definitions
#define I2C_SCL_PIN     GPIO_NUM_22   // I2C Clock
#define I2C_SDA_PIN     GPIO_NUM_21   // I2C Data
#define IR_SENSOR_PIN   GPIO_NUM_3   // IR Sensor 
#define RED_LED_PIN     GPIO_NUM_4   // Temperature Warning Red LED
#define GREEN_LED_PIN   GPIO_NUM_5   // Object Detection Green LED
#define SW420_PIN       GPIO_NUM_18  // Vibration Sensor
#define BUZZER_PIN      GPIO_NUM_19  // Buzzer


#define OLED_ADDR       0x3C         // OLED I2C Address

// Sensor Definitions
#define AHT30_SENSOR_ADDR   0x38
#define I2C_MASTER_FREQ_HZ  100000

#define ACK_CHECK_EN 0x1    

static const char *TAG = "MULTI_TASK_PROJECT";

// Global Variables
static ssd1306_handle_t oled_device;
static float current_temperature = 0.0;
static float current_humidity = 0.0;
static bool object_detected = false;
static bool vibration_state = false;

// I2C Initialization
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
    return ESP_OK;
}

// AHT30 Initialization
static esp_err_t aht30_init(void) {
    uint8_t init_cmd[] = {0xBE, 0x08, 0x00};
     vTaskDelay(pdMS_TO_TICKS(300));
    i2c_master_write_to_device(I2C_NUM_0, AHT30_SENSOR_ADDR, init_cmd, sizeof(init_cmd), pdTRUE);
    return ESP_OK;
}



static esp_err_t aht30_read_data(uint8_t *data, size_t len)
{
    uint8_t measure_cmd[] = {0xAC, 0x33, 0x00};
    i2c_master_write_to_device(I2C_NUM_0, AHT30_SENSOR_ADDR, measure_cmd, sizeof(measure_cmd), pdTRUE);
    vTaskDelay(pdMS_TO_TICKS(80)); // Thời gian chờ AHT30 xử lý đo đạc

    return i2c_master_read_from_device(I2C_NUM_0, AHT30_SENSOR_ADDR, data, sizeof(data), pdTRUE);
}




// OLED Initialization
static void oled_init() {
    oled_device = ssd1306_create(I2C_NUM_0, OLED_ADDR);
    ssd1306_clear_screen(oled_device, 0x00);
}

// Vibration Sensor Task
void vibration_sensor_task(void *pvParameters) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << SW420_PIN),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);

    while (1) {
        // Read vibration sensor
        vibration_state = gpio_get_level(SW420_PIN) == 1;   // Vibrating = True

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


// IR Sensor Task
void ir_sensor_task(void *pvParameters) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << IR_SENSOR_PIN),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);

    char oled_buffer[64];
    while (1) {
        object_detected = !gpio_get_level(IR_SENSOR_PIN);
        
        // Điều khiển LED xanh dựa trên trạng thái cảm biến
        // gpio_set_level(GREEN_LED_PIN, object_detected);
        
        snprintf(oled_buffer, sizeof(oled_buffer), 
                 "IR Sensor: %s", object_detected ? "OBJECT" : "CLEAR");
        
        ESP_LOGI(TAG, "%s", oled_buffer);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Temperature and Humidity Task
void aht30_task(void *pvParameters) {
    ESP_ERROR_CHECK(aht30_init());
    char oled_buffer[64];

    while (1) {
        uint8_t data[6];
        esp_err_t ret = aht30_read_data(data, sizeof(data));
        //esp_err_t ret = aht30_read_data(&current_temperature, &current_humidity);
        if (ret == ESP_OK) {
            
            // snprintf(oled_buffer, sizeof(oled_buffer), 
            //          "Temp: %.1fC\nHumid: %.1f%%", 
            //          current_temperature, current_humidity);
            

           uint32_t humidity = ((data[1] << 12) | (data[2] << 4) | (data[3] >> 4));
            uint32_t temperature = (((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]);

            float hum_percent = (humidity * 100.0) / 1048576.0;
            float temp_celsius = (temperature * 200.0 / 1048576.0) - 50;

            ESP_LOGI(TAG, "Humidity: %.2f %%", hum_percent);
            ESP_LOGI(TAG, "Temperature: %.2f °C", temp_celsius);
        } else {
            ESP_LOGE(TAG, "Failed to read data from AHT30 sensor");
            //ESP_LOGI(TAG, "%s", oled_buffer);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}



// Temperature Warning Task
void temperature_warning_task(void *pvParameters) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << RED_LED_PIN),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);

    while (1) {
        gpio_set_level(RED_LED_PIN, current_temperature > 60.0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// Buzzer Task
void buzzer_task(void *pvParameters) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << BUZZER_PIN),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);

    while (1) {
        // Buzzer is triggered if sensors dectect vibration or temperature exceeds 60 Celsius Degree
        if (vibration_state || current_temperature > 60.0) {
            for (int i = 0; i < 5; i++) {
                gpio_set_level(BUZZER_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(1000)); 
                gpio_set_level(BUZZER_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(1000));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


// OLED Update Task
void oled_update_task(void *pvParameters) {
    oled_init();
    char oled_buffer[64];

    while (1) {
        ssd1306_clear_screen(oled_device, 0x00);
        
        // snprintf(oled_buffer, sizeof(oled_buffer), 
        //          "Temp: %.1fC\nHumid: %.1f%%\nIR: %s", 
        //          current_temperature, 
        //          current_humidity, 
        //          object_detected ? "OBJECT" : "CLEAR");


        snprintf(oled_buffer, sizeof(oled_buffer),
         "T:%.1fC H:%.1f%% IR:%s",
         current_temperature,
         current_humidity,
         object_detected ? "OBJ" : "CLR");


        
        
        ESP_LOGI(TAG, "oled: %s", oled_buffer);
        ssd1306_draw_string(oled_device, 0, 0, (const uint8_t *)oled_buffer, 12, 1);
        // ssd1306_draw_string(oled_device, 0, 0, (const uint8_t *)"Hello", 16, 1); 
        ssd1306_refresh_gram(oled_device);
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//****************************************************************//

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Wi-Fi connecting...");
    }
    else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        ESP_LOGI(TAG, "Wi-Fi disconnected, retrying...");
        esp_wifi_connect();
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ESP_LOGI(TAG, "Wi-Fi connected, got IP");
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta()
{
    wifi_event_group = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Wi-Fi initialized successfully");
}

esp_err_t http_event_handler(esp_http_client_event_handle_t evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ON_DATA:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_DATA: %.*s", evt->data_len, (char *)evt->data);
        break;
    default:
        break;
    }
    return ESP_OK;
}

// Task to Post Data
void post_data_task(void *pvParameters) {
    while (1) {
        char full_url[256];
        snprintf(full_url, sizeof(full_url), "%s?temp=%.2f&hum=%.2f&osb=%d&vib=%d", 
                 SERVER_URL, 
                 current_temperature, 
                 current_humidity, 
                 object_detected ? 1 : 0, 
                 vibration_state ? 1 : 0);

        esp_http_client_config_t config = {
            .url = full_url,
            .method = HTTP_METHOD_GET,
            .event_handler = http_event_handler,
            .disable_auto_redirect = true,
        };

        esp_http_client_handle_t client = esp_http_client_init(&config);
        esp_err_t err = esp_http_client_perform(client);

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "HTTP GET request successful, status: %d", 
                     esp_http_client_get_status_code(client));
        } else {
            ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
        }

        esp_http_client_cleanup(client);

        // Post data every 5 seconds
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main (void) {
    // Initialize I2C
    ESP_ERROR_CHECK(i2c_master_init());

    nvs_flash_init();
    wifi_init_sta();

    // Create tasks with specified priorities (5>4>1=2>3)
    xTaskCreate(temperature_warning_task, "temp_warning", 2048, NULL, 5, NULL);
    xTaskCreate(buzzer_task, "buzzer_trigger", 2048, NULL, 4, NULL);
    xTaskCreate(post_data_task, "post_data", 4096, NULL, 3, NULL);
    xTaskCreate(vibration_sensor_task, "vibration_sensor", 2048, NULL, 2, NULL);
    xTaskCreate(ir_sensor_task, "ir_sensor", 2048, NULL, 2, NULL);
    xTaskCreate(aht30_task, "aht30_read", 2048, NULL, 2, NULL);
    xTaskCreate(oled_update_task, "oled_update", 2048, NULL, 1, NULL);
}
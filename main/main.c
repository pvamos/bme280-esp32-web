#include "bme280.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#define WIFI_SSID "YourSSID"
#define WIFI_PASS "YourPassword"

static const char *TAG = "Main";

// HTML content to display both raw and compensated values
static const char *html_page =
    "<html><body>"
    "<h1>BME280 Sensor Readings</h1>"
    "<p><b>Raw Values:</b></p>"
    "<ul>"
    "<li>Temperature (raw): %" PRIi32 "</li>"
    "<li>Pressure (raw): %" PRIi32 "</li>"
    "<li>Humidity (raw): %" PRIi32 "</li>"
    "</ul>"
    "<p><b>Compensated Values:</b></p>"
    "<ul>"
    "<li>Temperature: %.2f &deg;C</li>"
    "<li>Pressure: %.2f hPa</li>"
    "<li>Humidity: %.2f %%</li>"
    "</ul>"
    "</body></html>";

// HTTP request handler
static esp_err_t root_handler(httpd_req_t *req) {
    int32_t temp_raw, press_raw, hum_raw;
    float temp_final, press_final, hum_final;

    // Read raw and compensated data
    bme280_read_raw(&temp_raw, &press_raw, &hum_raw);
    bme280_calculate(temp_raw, press_raw, hum_raw, &temp_final, &press_final, &hum_final);

    // Prepare the response
    char response[1024];
    snprintf(response, sizeof(response), html_page,
             temp_raw, press_raw, hum_raw, 
             temp_final, press_final, hum_final);
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

    // Log the values
    ESP_LOGI(TAG, "Raw Values - Temp: %" PRIi32 ", Press: %" PRIi32 ", Hum: %" PRIi32,
             temp_raw, press_raw, hum_raw);
    ESP_LOGI(TAG, "Compensated Values - Temp: %.2f °C, Press: %.2f hPa, Hum: %.2f %%",
             temp_final, press_final, hum_final);

    return ESP_OK;
}

// HTTP request handler for invalid routes (does nothing)
static esp_err_t default_handler(httpd_req_t *req) {
    // Just close the connection without sending a response
    return ESP_OK;
}

// Start the webserver
void start_webserver() {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.max_open_sockets = 3;           // Set concurrent connections
    config.lru_purge_enable = true;         // Free least recently used connections
    config.recv_wait_timeout = 5;           // Set socket receive timeout
    config.send_wait_timeout = 5;           // Set socket send timeout
    config.stack_size = 16384;               // Set stack size
    config.uri_match_fn = httpd_uri_match_wildcard; // Set a default handler for unregistered routes

    httpd_uri_t root_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = root_handler,
        .user_ctx = NULL,
    };

    httpd_uri_t catch_all_uri = {
        .uri = "/*", // Matches all other routes
        .method = HTTP_GET,
        .handler = default_handler,
        .user_ctx = NULL,
    };

    ESP_ERROR_CHECK(httpd_start(&server, &config));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &root_uri));
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &catch_all_uri));
    ESP_LOGI(TAG, "Webserver started");
}

// Initialize Wi-Fi
void wifi_init() {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
    ESP_LOGI(TAG, "WiFi initialized and connected");
}

void app_main() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init();
    i2c_master_init();
    bme280_init();

    start_webserver();
}

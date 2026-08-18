#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_http_client.h"
#include "esp_tls.h"
#include "esp_timer.h"
#include "esp_sntp.h"
#include "cJSON.h"
#include "led_strip.h"
#include "secrets.h"

// Pin Configuration
#define PIR_PIN GPIO_NUM_4
#define PIR_PIN2 GPIO_NUM_3
#define ALARM_CONTROL_PIN GPIO_NUM_10
#define LED_PIN GPIO_NUM_8

// Timing Configuration
#define ALARM_DURATION_MS 30000  // 30 seconds
#define PIR_DEBOUNCE_MS 5000     // 5 seconds

// False Alarm Reduction Configuration
#define MOTION_CONFIRMATION_WINDOW_MS 10000  // 10 second window to confirm motion
#define MOTION_CONFIRMATION_COUNT 3          // Require 3 motion events to trigger (raised from 2 to reject regular low-amplitude noise)
#define PIR_STABILIZATION_TIME_MS 15000     // 15 seconds for PIR to stabilize
#define STARTUP_GRACE_PERIOD_MS 45000       // 45 seconds grace period after startup
#define MIN_MOTION_DURATION_MS 500          // Minimum time motion must be present
#define SENSOR_TRIGGER_COOLDOWN_MS 120000   // Minimum time between a sensor's confirmed triggers, prevents runaway re-alarming from a noisy sensor

static const char *TAG = "ALARM_SYSTEM";

// WiFi event group
static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

// Alarm state (shared: either sensor confirming motion drives the physical alarm)
static bool alarm_active = false;
static int64_t alarm_start_time = 0;

// Per-sensor motion confirmation tracking, so each sensor reports independently to its own webhook
typedef struct {
    int64_t last_motion_time;
    int motion_event_count;
    int64_t first_motion_time;
    int64_t motion_start_time;
    bool continuous_motion;
    int64_t last_trigger_time;
} motion_tracker_t;

static motion_tracker_t pir1_tracker = {0};
static motion_tracker_t pir2_tracker = {0};

// Diagnostic: interrupt-driven edge counters, cannot miss a pulse regardless of poll timing
static volatile uint32_t pir1_edge_count = 0;
static volatile uint32_t pir2_edge_count = 0;

static void IRAM_ATTR pir1_isr_handler(void* arg)
{
    pir1_edge_count++;
}

static void IRAM_ATTR pir2_isr_handler(void* arg)
{
    pir2_edge_count++;
}

// Message queue for webhook retries
static QueueHandle_t webhook_queue;
#define WEBHOOK_QUEUE_SIZE 10

typedef struct {
    char event_type[32];
    char timestamp[64];
    int sensor; // 1 = PIR1 (GPIO4/AM312), 2 = PIR2 (GPIO3)
} webhook_message_t;

static int wifi_retry_count = 0;
#define MAX_WIFI_RETRY 10

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                              int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "WiFi station started, connecting to %s", WIFI_SSID);
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*) event_data;
        ESP_LOGI(TAG, "Disconnected from WiFi. Reason: %d", event->reason);
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        
        if (wifi_retry_count < MAX_WIFI_RETRY) {
            wifi_retry_count++;
            ESP_LOGI(TAG, "Retrying WiFi connection... (attempt %d/%d)", wifi_retry_count, MAX_WIFI_RETRY);
            vTaskDelay(5000 / portTICK_PERIOD_MS);  // Wait longer for repeater
            esp_wifi_connect();
        } else {
            ESP_LOGW(TAG, "WiFi connection failed after %d attempts, will retry via monitor task", MAX_WIFI_RETRY);
            wifi_retry_count = 0; // Reset for monitor task to retry
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "WiFi connected, explicitly starting DHCP client...");
        esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
        if (netif) {
            esp_err_t ret = esp_netif_dhcpc_start(netif);
            ESP_LOGI(TAG, "DHCP client start result: %s", esp_err_to_name(ret));
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        wifi_retry_count = 0; // Reset retry count on successful connection
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "WiFi connected bit set, ready for webhooks");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_LOST_IP) {
        ESP_LOGW(TAG, "Lost IP address");
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));
    esp_event_handler_instance_t instance_lost_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_LOST_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_lost_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,  // Match router's auth
            .threshold.rssi = -100,  // More reasonable threshold 
            .scan_method = WIFI_FAST_SCAN,  // Faster connection
            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,  // Connect to strongest
            .listen_interval = 1,  // Default for stability
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    // Set compatible protocols for better connectivity 
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // Disable power save mode for maximum range and stability
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    
    // Set maximum transmit power for ESP32-C3 (20 dBm)
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80));  // 80 = 20dBm (0.25dBm units)
    
    // Configure for long range mode
    wifi_country_t country = {
        .cc = "US",
        .schan = 1,
        .nchan = 11,
        .max_tx_power = 80,  // Maximum power
        .policy = WIFI_COUNTRY_POLICY_MANUAL,
    };
    ESP_ERROR_CHECK(esp_wifi_set_country(&country));

    ESP_LOGI(TAG, "WiFi initialization finished.");
}

static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.nist.gov");
    esp_sntp_setservername(2, "time.google.com");
    esp_sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
    esp_sntp_init();
}

static void get_current_time_string(char* time_str, size_t max_len)
{
    time_t now;
    struct tm timeinfo;
    
    time(&now);
    localtime_r(&now, &timeinfo);
    
    // If time is not set yet, use uptime as fallback
    if (timeinfo.tm_year < (2020 - 1900)) {
        int64_t uptime_ms = esp_timer_get_time() / 1000;
        int uptime_sec = uptime_ms / 1000;
        int hours = uptime_sec / 3600;
        int minutes = (uptime_sec % 3600) / 60;
        int seconds = uptime_sec % 60;
        snprintf(time_str, max_len, "System uptime: %02d:%02d:%02d", hours, minutes, seconds);
    } else {
        strftime(time_str, max_len, "%Y-%m-%d %H:%M:%S UTC", &timeinfo);
    }
}

static void queue_webhook_message_for_sensor(const char* event_type, int sensor)
{
    webhook_message_t msg;
    strncpy(msg.event_type, event_type, sizeof(msg.event_type) - 1);
    msg.event_type[sizeof(msg.event_type) - 1] = '\0';
    msg.sensor = sensor;

    get_current_time_string(msg.timestamp, sizeof(msg.timestamp));

    if (xQueueSend(webhook_queue, &msg, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Webhook queue full, dropping message: %s", event_type);
    } else {
        ESP_LOGI(TAG, "Queued webhook message: %s (sensor %d)", event_type, sensor);
    }
}

static void queue_webhook_message(const char* event_type)
{
    // Default/system-level events (no specific sensor) go to PIR1's webhook
    queue_webhook_message_for_sensor(event_type, 1);
}

static bool send_discord_webhook_with_timestamp(const char* event_type, const char* time_str, int sensor)
{
    char message[256];
    const char* sensor_label = (sensor == 2) ? "PIR2" : "PIR1 (AM312)";

    if (strcmp(event_type, "alarm_triggered") == 0) {
        snprintf(message, sizeof(message),
                 "🚨 **ALARM TRIGGERED!** 🚨\n"
                 "Motion detected by %s!\n"
                 "Time: %s", sensor_label, time_str);
    } else if (strcmp(event_type, "alarm_stopped") == 0) {
        snprintf(message, sizeof(message),
                 "✅ Alarm has been deactivated\n"
                 "Time: %s", time_str);
    } else if (strcmp(event_type, "system_started") == 0) {
        snprintf(message, sizeof(message),
                 "🔌 ESP32 Alarm System Started\n"
                 "System online and monitoring\n"
                 "Time: %s", time_str);
    } else {
        snprintf(message, sizeof(message),
                 "Alarm event: %s\n"
                 "Time: %s", event_type, time_str);
    }

    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "content", message);
    cJSON_AddStringToObject(root, "username", "ESP32 Alarm System");

    char *post_data = cJSON_PrintUnformatted(root);

    esp_http_client_config_t config = {
        .url = (sensor == 2) ? WEBHOOK_URL_PIR2 : WEBHOOK_URL_PIR1,
        .event_handler = http_event_handler,
        .method = HTTP_METHOD_POST,
        .timeout_ms = 15000,  // Longer timeout for weak signals
        .buffer_size = 2048,  // Larger buffer
        .skip_cert_common_name_check = true,
        .keep_alive_enable = false,  // Don't keep connections alive
        .disable_auto_redirect = false,
    };
    
    esp_http_client_handle_t client = esp_http_client_init(&config);
    
    esp_http_client_set_header(client, "Content-Type", "application/json");
    esp_http_client_set_post_field(client, post_data, strlen(post_data));
    
    esp_err_t err = esp_http_client_perform(client);
    bool success = (err == ESP_OK);
    
    if (success) {
        ESP_LOGI(TAG, "Discord webhook sent successfully");
    } else {
        ESP_LOGE(TAG, "Discord webhook failed: %s", esp_err_to_name(err));
    }
    
    esp_http_client_cleanup(client);
    cJSON_Delete(root);
    free(post_data);
    
    return success;
}

static bool is_wifi_connected(void)
{
    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

static void wifi_monitor_task(void* arg)
{
    int no_ip_count = 0;
    
    while (1) {
        // Check WiFi status every 30 seconds (less interference)
        vTaskDelay(30000 / portTICK_PERIOD_MS);
        
        wifi_ap_record_t ap_info;
        esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
        
        if (ret != ESP_OK) {
            // Not connected, try to reconnect
            ESP_LOGW(TAG, "WiFi disconnected, attempting reconnection...");
            esp_wifi_connect();
            no_ip_count = 0;
        } else {
            // Connected, check signal strength and IP status
            ESP_LOGI(TAG, "WiFi connected to %s, RSSI: %d dBm", ap_info.ssid, ap_info.rssi);
            
            // Check if we have IP - give DHCP more time before restarting
            if (!is_wifi_connected()) {
                no_ip_count++;
                ESP_LOGW(TAG, "WiFi connected but no IP for %d checks (%d minutes)", no_ip_count, no_ip_count/2);
                
                // Only restart DHCP after 3+ minutes (6 checks) to allow slower repeater DHCP
                if (no_ip_count >= 6) {
                    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
                    if (netif) {
                        ESP_LOGW(TAG, "Restarting DHCP after 3+ minutes...");
                        esp_netif_dhcpc_stop(netif);
                        vTaskDelay(5000 / portTICK_PERIOD_MS); // Longer wait
                        esp_err_t dhcp_ret = esp_netif_dhcpc_start(netif);
                        ESP_LOGI(TAG, "DHCP restart result: %s", esp_err_to_name(dhcp_ret));
                    }
                }
                
                // If no IP for a very long time, reconnect WiFi entirely
                if (no_ip_count >= 12) { // 6 minutes of no IP
                    ESP_LOGW(TAG, "No IP for 6+ minutes, full WiFi reconnect...");
                    esp_wifi_disconnect();
                    vTaskDelay(5000 / portTICK_PERIOD_MS);
                    esp_wifi_connect();
                    no_ip_count = 0;
                }
            } else {
                no_ip_count = 0; // Reset on successful IP
            }
            
            // Only reconnect if signal is at the absolute limit
            if (ap_info.rssi < -105) {
                ESP_LOGW(TAG, "WiFi signal at limit (%d dBm), reconnecting...", ap_info.rssi);
                esp_wifi_disconnect();
                vTaskDelay(5000 / portTICK_PERIOD_MS);  // Longer wait
                esp_wifi_connect();
            } else if (ap_info.rssi < -90) {
                ESP_LOGI(TAG, "WiFi signal weak but usable (%d dBm)", ap_info.rssi);
            }
        }
    }
}

static void webhook_sender_task(void* arg)
{
    webhook_message_t msg;
    
    while (1) {
        // Wait for a message in the queue
        if (xQueueReceive(webhook_queue, &msg, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI(TAG, "Processing queued webhook: %s", msg.event_type);
            
            // Keep trying to send until successful
            while (1) {
                // Wait for WiFi connection
                if (!is_wifi_connected()) {
                    ESP_LOGI(TAG, "Waiting for WiFi to send webhook... (WiFi bit not set)");
                    vTaskDelay(5000 / portTICK_PERIOD_MS);
                    continue;
                }
                
                ESP_LOGI(TAG, "WiFi connected, attempting to send webhook...");
                
                // Try to send the webhook
                if (send_discord_webhook_with_timestamp(msg.event_type, msg.timestamp, msg.sensor)) {
                    ESP_LOGI(TAG, "Successfully sent queued webhook: %s", msg.event_type);
                    break; // Success, move to next message
                } else {
                    ESP_LOGW(TAG, "Failed to send webhook, retrying in 5 seconds...");
                    vTaskDelay(5000 / portTICK_PERIOD_MS);
                }
            }
        }
    }
}

static void process_sensor_motion(motion_tracker_t* t, bool motion_now, int64_t current_time,
        bool startup_grace_period, int sensor)
{
    if (motion_now) {
        if (!t->continuous_motion) {
            t->continuous_motion = true;
            t->motion_start_time = current_time;
        }

        // Check if motion has been continuous for minimum duration
        if ((current_time - t->motion_start_time) >= MIN_MOTION_DURATION_MS) {
            if (!startup_grace_period &&
                (current_time - t->last_motion_time > PIR_DEBOUNCE_MS)) {

                // First motion event or new motion after debounce
                if (t->motion_event_count == 0 ||
                    (current_time - t->first_motion_time > MOTION_CONFIRMATION_WINDOW_MS)) {
                    // Start new confirmation window
                    t->motion_event_count = 1;
                    t->first_motion_time = current_time;
                    ESP_LOGI(TAG, "Sensor %d: motion event 1/%d detected, waiting for confirmation...",
                            sensor, MOTION_CONFIRMATION_COUNT);
                } else {
                    // Additional motion within confirmation window
                    t->motion_event_count++;
                    ESP_LOGI(TAG, "Sensor %d: motion event %d/%d detected",
                            sensor, t->motion_event_count, MOTION_CONFIRMATION_COUNT);

                    // Check if we have enough confirmations
                    if (t->motion_event_count >= MOTION_CONFIRMATION_COUNT) {
                        t->motion_event_count = 0;  // Reset for next detection

                        if (current_time - t->last_trigger_time < SENSOR_TRIGGER_COOLDOWN_MS) {
                            ESP_LOGI(TAG, "Sensor %d: motion confirmed but within cooldown, suppressing repeat trigger", sensor);
                        } else {
                            t->last_trigger_time = current_time;
                            ESP_LOGI(TAG, "Sensor %d: motion confirmed after %d events", sensor, MOTION_CONFIRMATION_COUNT);
                            queue_webhook_message_for_sensor("alarm_triggered", sensor);

                            if (!alarm_active) {
                                alarm_active = true;
                                alarm_start_time = current_time;
                                gpio_set_level(ALARM_CONTROL_PIN, 1);
                                ESP_LOGI(TAG, "ALARM ACTIVATED! (triggered by sensor %d)", sensor);
                            }
                        }
                    }
                }

                t->last_motion_time = current_time;
            }
        }
    } else {
        t->continuous_motion = false;

        // Reset motion count if confirmation window expired
        if (t->motion_event_count > 0 &&
            (current_time - t->first_motion_time > MOTION_CONFIRMATION_WINDOW_MS)) {
            ESP_LOGI(TAG, "Sensor %d: motion confirmation window expired, resetting count", sensor);
            t->motion_event_count = 0;
        }
    }
}

static void alarm_task(void* arg)
{
    // Configure PIR sensor input with pull-down
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIR_PIN) | (1ULL << PIR_PIN2),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,  // Enable pull-down to prevent floating
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIR_PIN, pir1_isr_handler, NULL);
    gpio_isr_handler_add(PIR_PIN2, pir2_isr_handler, NULL);

    // Configure alarm output
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << ALARM_CONTROL_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    
    // Force a hard reset pulse on the LED data line before handing it to the RMT
    // driver: hold it LOW for 100ms as a plain GPIO to guarantee the WS2812's
    // reset threshold is met, in case a prior stuck color needs an unambiguous reset.
    {
        gpio_config_t led_reset_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << LED_PIN),
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
        };
        gpio_config(&led_reset_conf);
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    // Configure WS2812 RGB LED and send a single explicit off frame at boot.
    // Nothing else in the firmware writes to it afterward.
    led_strip_config_t strip_config = {
        .strip_gpio_num = LED_PIN,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000,
    };
    led_strip_handle_t led_strip;
    led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip);
    led_strip_clear(led_strip);

    gpio_set_level(ALARM_CONTROL_PIN, 0);
    
    // Let PIR sensor stabilize after power-up
    ESP_LOGI(TAG, "Waiting for PIR sensor to stabilize (%d seconds)...", 
            PIR_STABILIZATION_TIME_MS / 1000);
    
    // Read PIR during stabilization to debug
    int stabilization_steps = PIR_STABILIZATION_TIME_MS / 1000;
    for (int i = 0; i < stabilization_steps; i++) {
        int pir_level = gpio_get_level(PIR_PIN);
        int pir_level2 = gpio_get_level(PIR_PIN2);
        ESP_LOGI(TAG, "PIR stabilization %d/%d: GPIO%d = %d, GPIO%d = %d",
                i+1, stabilization_steps, PIR_PIN, pir_level, PIR_PIN2, pir_level2);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "PIR stabilization complete");

    // Start monitoring PIR even before WiFi connects
    ESP_LOGI(TAG, "Starting PIR monitoring. PIR on GPIO%d and GPIO%d, Alarm on GPIO%d", PIR_PIN, PIR_PIN2, ALARM_CONTROL_PIN);
    
    // Wait for initial WiFi connection
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, false, true, 60000 / portTICK_PERIOD_MS);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connected!");
        // Initialize SNTP for time synchronization
        initialize_sntp();
        
        // Queue a test webhook
        ESP_LOGI(TAG, "Queueing system started webhook...");
        queue_webhook_message("system_started");
    } else {
        ESP_LOGW(TAG, "Initial WiFi connection timeout (1 min), continuing without network features");
    }
    
    int last_pir_state = -1;
    int last_pir_level1 = -1;
    int last_pir_level2 = -1;
    int64_t startup_time = esp_timer_get_time() / 1000;

    while (1) {
        int pir_level1 = gpio_get_level(PIR_PIN);
        int pir_level2 = gpio_get_level(PIR_PIN2);
        int pir_state = (pir_level1 == 1 || pir_level2 == 1) ? 0 : 1;  // motion if either PIR is HIGH (standard PIR polarity); pir_state 0 = motion, 1 = idle
        int64_t current_time = esp_timer_get_time() / 1000; // Convert to ms

        // Log individual PIR pin changes for debugging which sensor triggered
        if (pir_level1 != last_pir_level1) {
            ESP_LOGI(TAG, "GPIO%d (PIR1) changed: %d -> %d", PIR_PIN, last_pir_level1, pir_level1);
            last_pir_level1 = pir_level1;
        }
        if (pir_level2 != last_pir_level2) {
            ESP_LOGI(TAG, "GPIO%d (PIR2) changed: %d -> %d", PIR_PIN2, last_pir_level2, pir_level2);
            last_pir_level2 = pir_level2;
        }

        if (pir_state != last_pir_state) {
            ESP_LOGI(TAG, "PIR state changed: %d -> %d", last_pir_state, pir_state);
            last_pir_state = pir_state;
        }

        {
            static int64_t last_edge_log = 0;
            if (current_time - last_edge_log > 3000) {
                last_edge_log = current_time;
                ESP_LOGI(TAG, "EDGE_COUNT: PIR1(GPIO%d)=%lu PIR2(GPIO%d)=%lu",
                        PIR_PIN, (unsigned long)pir1_edge_count, PIR_PIN2, (unsigned long)pir2_edge_count);
            }
        }

        // Check for startup grace period
        bool startup_grace_period = (current_time - startup_time < STARTUP_GRACE_PERIOD_MS);

        // Motion detection with confirmation logic, tracked independently per sensor
        process_sensor_motion(&pir1_tracker, pir_level1 == 1, current_time, startup_grace_period, 1);
        process_sensor_motion(&pir2_tracker, pir_level2 == 1, current_time, startup_grace_period, 2);

        // Log ignored motion during grace period
        if (pir_state == 0 && startup_grace_period) {
            static int64_t last_grace_log = 0;
            if (current_time - last_grace_log > 5000) {  // Log every 5 seconds max
                ESP_LOGI(TAG, "Motion ignored during startup grace period (%lld/%d ms remaining)", 
                        STARTUP_GRACE_PERIOD_MS - (current_time - startup_time), 
                        STARTUP_GRACE_PERIOD_MS);
                last_grace_log = current_time;
            }
        }
        
        // Check if alarm should be turned off
        if (alarm_active && (current_time - alarm_start_time > ALARM_DURATION_MS)) {
            alarm_active = false;
            gpio_set_level(ALARM_CONTROL_PIN, 0);
            ESP_LOGI(TAG, "Alarm deactivated");
            
            queue_webhook_message("alarm_stopped");
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    ESP_LOGI(TAG, "ESP32-C3 Alarm System Starting...");

    // Create webhook queue
    webhook_queue = xQueueCreate(WEBHOOK_QUEUE_SIZE, sizeof(webhook_message_t));
    if (webhook_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create webhook queue");
        return;
    }
    
    // Initialize WiFi
    wifi_init_sta();
    
    // Create WiFi monitor task
    xTaskCreate(wifi_monitor_task, "wifi_monitor", 2048, NULL, 3, NULL);
    
    // Create webhook sender task
    xTaskCreate(webhook_sender_task, "webhook_sender", 4096, NULL, 5, NULL);
    
    // Create alarm task
    xTaskCreate(alarm_task, "alarm_task", 4096, NULL, 5, NULL);
}
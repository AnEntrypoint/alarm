#ifndef CONFIG_H
#define CONFIG_H

// WiFi Configuration
#define WIFI_SSID "Fort20Dodge"
#define WIFI_PASSWORD "F1n477y_73g47"

// Webhook Configuration
#define WEBHOOK_HOST "discord.com"
#define WEBHOOK_PATH "/api/webhooks/1385912261025202266/JFabHnUkcx_8ZgGtocqQupntdYbffftMFgKLCKg4N0ZUyKqvf3AbXKiba-m5OtC0Ui28"
#define WEBHOOK_PORT 443

// Pin Configuration
#define PIR_PIN 4
#define ALARM_CONTROL_PIN 10

// Timing Configuration
#define ALARM_DURATION 30000  // 30 seconds
#define PIR_DEBOUNCE_TIME 5000  // 5 seconds

// False Alarm Reduction Configuration
#define MOTION_CONFIRMATION_WINDOW 10000  // 10 second window to confirm motion
#define MOTION_CONFIRMATION_COUNT 2       // Require 2 motion events to trigger
#define PIR_STABILIZATION_TIME 15000     // 15 seconds for PIR to stabilize
#define STARTUP_GRACE_PERIOD 45000       // 45 seconds grace period after startup
#define MIN_MOTION_DURATION 500          // Minimum time motion must be present

#endif
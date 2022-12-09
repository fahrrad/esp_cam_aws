/* Read Camera and publish to AWS IoT 

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "protocol_examples_common.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"
#include "mbedtls/base64.h"
#include "mqtt_client.h"

#include "esp_camera.h"

// Established as the size of a grayscale bmp of 96x96
#define B64_BUFFER_SIZE 13729

// Topic name for the communication to AWS
#define TOPIC_NAME_AWS_MESSSAGES "dt/mons/messages"

// Topic name for receiving command to take a picture
#define TOPIC_NAME_AWS_CMD_SNAP "cmd/mons/snap"

// Topic name for sending pictures from the device to AWS IoT
#define TOPIC_NAME_AWS_PHOTOS "cmd/mons/snap/photo"

// Topic name for sending heartbeat signal
#define TOPIC_NAME_HEARTHBEAT "dt/mons/heartbeat"

// TAG is used for logging
static const char *TAG = "MONS";

// Heartbeats BPM
#define HEARTBEAT_BPM 15
#define HEARTBEAT_INTERVAL (60000 / HEARTBEAT_BPM)

// Pin mapping between ESP32 and the camera on the board
// ESP pin to camera pin mapping
#define CAM_PIN_PWDN    32 
#define CAM_PIN_RESET   -1 //software reset will be performed
#define CAM_PIN_XCLK    0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27

#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0       5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22


static camera_config_t camera_config = {
    .pin_pwdn  = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = 20000000,//EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_GRAYSCALE,//YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_96X96,//QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .fb_location = CAMERA_FB_IN_DRAM, // No PSRAM

    .jpeg_quality = 12, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 1, //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY//CAMERA_GRAB_LATEST. Sets when buffers should be filled
};

// Certificates, see CMakeList.txt file to see how these get filled.
// the client_crt file is called {certificate id}-certificate.pem.crt when downloading from IoT core console
extern const uint8_t client_cert_pem_start[] asm("_binary_client_crt_start");
extern const uint8_t client_cert_pem_end[] asm("_binary_client_crt_end");

// the client_key file is called {certificate id}-private.pem.key when downloading from IoT core console
extern const uint8_t client_key_pem_start[] asm("_binary_client_key_start");
extern const uint8_t client_key_pem_end[] asm("_binary_client_key_end");

// The amazon root ca is called AmazonRootCA1.pem when downloading from the IoT core console
extern const uint8_t server_cert_pem_start[] asm("_binary_AmazonRootCA1_pem_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_AmazonRootCA1_pem_end");

// buffer for base64-encoded version of the BMP that will be send to AWS IoT core
unsigned char *img_b64;

// Client object for MQTT connection
esp_mqtt_client_handle_t client;

// Hearthbeat task handler
TaskHandle_t heartBeatTaskHandle = NULL;

static void log_error_if_nonzero(const char *message, int error_code)
{
    if (error_code != 0) {
        ESP_LOGE(TAG, "Last error %s: 0x%x", message, error_code);
    }
}

/// @brief Takes a camera framebuffer object, and converts the buffer into a BMP. Then, b64 encode that BMP and send it to an AWS IoT Topic
/// @param fb camera framebuffer, than contains a foto
void upload_image_bmp(camera_fb_t * fb){
    ESP_LOGI(TAG, "About to upload image of len: %d", fb->len);

    uint8_t * buf = NULL;
    size_t buf_len = 0;
    bool converted = frame2bmp(fb, &buf, &buf_len);

    if(converted){
        size_t outlen;
        int encoded = mbedtls_base64_encode(img_b64, B64_BUFFER_SIZE, &outlen, (unsigned char *) buf, buf_len);
        
        if(encoded == 0) {
            ESP_LOGI(TAG, "BMP encoded. Size: %d", outlen);
            esp_mqtt_client_publish(client, TOPIC_NAME_AWS_PHOTOS, (char *) img_b64, outlen, 0, 0);
        } else {
            ESP_LOGE(TAG, "BMP not encoded! %d", encoded);
        }

        // free memory allocated by frame2bmp
        if(buf != NULL){
            free(buf);
        } 
    } else {
        ESP_LOGE(TAG, "conversion failed!");
    }
}

/// @brief Starts the camera
/// @return 
esp_err_t camera_init(){

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera Init Failed");
        return err;
    }
    return ESP_OK;
}

/// @brief Take a picture and uploads it
/// @return 
esp_err_t camera_capture(){
    //acquire a frame
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera Capture Failed");
        return ESP_FAIL;
    }
    //replace this with your own function
    upload_image_bmp(fb);
  
    //return the frame buffer back to the driver for reuse
    esp_camera_fb_return(fb);
    return ESP_OK;
}

/// @brief sends a heartbeat message every 5 seconds
/// @param pvParameters 
void heartBeatTask(void *pvParameters)
{
    while (1)
    {
        // TODO make sure this ends up in the frontend
        esp_mqtt_client_publish(client, TOPIC_NAME_HEARTHBEAT, "{\"message\": \"ping\"}", 0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(HEARTBEAT_INTERVAL));
    }
}

/*
 * @brief Event handler registered to receive MQTT events
 *
 *  This function is called by the MQTT client event loop.
 * 
 *  TODO 
 *
 * @param handler_args user data registered to the event.
 * @param base Event base for the handler(always MQTT Base in this example).
 * @param event_id The id for the received event.
 * @param event_data The data for the event, esp_mqtt_event_handle_t.
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%lu", base, event_id);
    esp_mqtt_event_handle_t event = event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        esp_mqtt_client_publish(client, TOPIC_NAME_AWS_MESSSAGES, "Hello, Mons!", 0, 0, 0);
        
        esp_mqtt_client_subscribe(client, TOPIC_NAME_AWS_CMD_SNAP, 0);
        
        // This is the heartbeat task that will run in the background
        // It can only be started once
        // 
        if (heartBeatTaskHandle == NULL){
            xTaskCreate(heartBeatTask, "heartbeat", configMINIMAL_STACK_SIZE * 8, NULL, 5, &heartBeatTaskHandle);
            ESP_LOGI(TAG, "Task created!");
        }else{
            ESP_LOGI(TAG, "Task was already created, probably a reconnect");
        }
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        if(strcmp(event->topic, TOPIC_NAME_AWS_CMD_SNAP)){
            printf("Snap!\n");
        }
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}

static void mqtt_app_start(void)
{
  const esp_mqtt_client_config_t mqtt_cfg = {
    .broker.address.uri = CONFIG_EXAMPLE_AWS_IOT_URI,
    .broker.verification.certificate = (const char *)server_cert_pem_start,
    .credentials = {
      .authentication = {
        .certificate = (const char *)client_cert_pem_start,
        .key = (const char *)client_key_pem_start,
      },
    }
  };

    ESP_LOGI(TAG, "[APP] Free memory: %lu bytes", esp_get_free_heap_size());
    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

void app_main(void)
{
    img_b64 = malloc (B64_BUFFER_SIZE * sizeof(char));
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(camera_init());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    mqtt_app_start();
}

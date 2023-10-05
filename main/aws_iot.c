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
#include "cJSON.h"

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

#include "esp_tls.h"
#include "esp_http_client.h"

// Buffer for MQTT messages
#define MAX_MSG_BUFFER_SIZE 6000
char mqtt_message_buffer[MAX_MSG_BUFFER_SIZE] = {0};

// HTTP response buffer
#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 2048

// Struct representing the url parameters needed for a presigned URL Post
struct presigned_url_post_parameters {
    char url[2000];
    char key[41];
} parameters;

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

// Topic name for the communication to AWS
#define TOPIC_NAME_AWS_MESSSAGES "dt/snap/messages"

// Topic name for receiving command to take a picture
#define TOPIC_NAME_AWS_CMD_SNAP "cmd/snap/start"

// Topic that will receive a presigned URL to publish an image to 
#define TOPIC_NAME_AWS_PRESIGNED_URL "cmd/snap/get-url/response"

#define TOPIC_NAME_AWS_CMD_REQUEST_PRESIGNED_URL "cmd/snap/get-url/request"

#define TOPIC_NAME_REKOGNIZE_REQUEST "cmd/snap/rekognize/request"

// Topic name for sending heartbeat signal
#define TOPIC_NAME_HEARTHBEAT "dt/snap/heartbeat"

// TAG is used for logging
static const char *TAG = "SNAP";

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

    .pixel_format = PIXFORMAT_JPEG,//YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_UXGA,//QQVGA-UXGA, For ESP32, do not use sizes above QVGA when not JPEG. The performance of the ESP32-S series has improved a lot, but JPEG mode always gives better frame rates.

    .jpeg_quality = 12, //0-63, for OV series camera sensors, lower number means higher quality
    .fb_count = 2, //When jpeg mode is used, if fb_count more than one, the driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_LATEST//CAMERA_GRAB_LATEST. Sets when buffers should be filled
};

// Certificates, see CMakeList.txt file to see how these get filled.
// the client_crt file is called {certificate id}-certificate.pem.crt when downloading from IoT core console
extern const uint8_t client_cert_pem_start[] asm("_binary_client_crt_start");
extern const uint8_t client_cert_pem_end[] asm("_binary_client_crt_end");

// the client_key file is called {certificate id}-private.pem.key when downloading from IoT core console.
// THERE ARE 2 .key files! Make sure to select the -private one here
extern const uint8_t client_key_pem_start[] asm("_binary_client_key_start");
extern const uint8_t client_key_pem_end[] asm("_binary_client_key_end");

// The amazon root ca is called AmazonRootCA1.pem when downloading from the IoT core console
extern const char server_cert_pem_start[] asm("_binary_AmazonRootCA1_pem_start");
extern const char server_cert_pem_end[] asm("_binary_AmazonRootCA1_pem_end");


// The amazon  ca is for presigned URLs from s3
// extern const char server_s3_cert_pem_start[] asm("_binary_AmazonPresignedS3CA_pem_start");
// extern const char server_s3_cert_pem_end[] asm("_binary_AmazonPresignedS3CA_pem_end");

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

void request_presigned_url() { 
    ESP_LOGI(TAG, "[MQTT] Requesting a presigned URL for S3");
    esp_mqtt_client_publish(client, TOPIC_NAME_AWS_CMD_REQUEST_PRESIGNED_URL, "", 0, 0, 0);
} 


// gets the fields out of a json and populate the global var 'parameters'
void parse_json_and_populate_struct(cJSON * json){
    // URL
    const cJSON * url = cJSON_GetObjectItemCaseSensitive(json, "url");
    if (cJSON_IsString(url) && (url->valuestring != NULL))
    {
        ESP_LOGD(TAG, "Url:  \"%s\" size: %d", url->valuestring, strlen(url->valuestring));
        strcpy(parameters.url,  url->valuestring);
    }
    // KEY
    const cJSON * key = cJSON_GetObjectItemCaseSensitive(json, "key");
    if(cJSON_IsString(key) && (key->valuestring != NULL)){
        ESP_LOGD(TAG, "Key:  \"%s\" size: %d", key->valuestring, strlen(key->valuestring));
        strcpy(parameters.key,  key->valuestring);
    } 
}

/// @brief initialize the camera
/// @return 
esp_err_t camera_init(){
    
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return err;
    }

    return ESP_OK;
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    static char *output_buffer;  // Buffer to store response of http request from event handler
    static int output_len;       // Stores number of bytes read
    switch(evt->event_id) {
        case HTTP_EVENT_ERROR:
            ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
            break;
        case HTTP_EVENT_ON_CONNECTED:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
            break;
        case HTTP_EVENT_HEADER_SENT:
            ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
            break;
        case HTTP_EVENT_ON_HEADER:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
            break;
        case HTTP_EVENT_ON_DATA:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
            ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, data=%.s", (char *) evt->data);
            /*
             *  Check for chunked encoding is added as the URL for chunked encoding used in this example returns binary data.
             *  However, event handler can also be used in case chunked encoding is used.
             */
            if (!esp_http_client_is_chunked_response(evt->client)) {
                // If user_data buffer is configured, copy the response into the buffer
                int copy_len = 0;
                if (evt->user_data) {
                    copy_len = MIN(evt->data_len, (MAX_HTTP_OUTPUT_BUFFER - output_len));
                    if (copy_len) {
                        memcpy(evt->user_data + output_len, evt->data, copy_len);
                    }
                } else {
                    const int buffer_len = esp_http_client_get_content_length(evt->client);
                    if (output_buffer == NULL) {
                        output_buffer = (char *) malloc(buffer_len);
                        output_len = 0;
                        if (output_buffer == NULL) {
                            ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                            return ESP_FAIL;
                        }
                    }
                    copy_len = MIN(evt->data_len, (buffer_len - output_len));
                    if (copy_len) {
                        memcpy(output_buffer + output_len, evt->data, copy_len);
                    }
                }
                output_len += copy_len;
            }

            break;
        case HTTP_EVENT_ON_FINISH:
            ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
            if (output_buffer != NULL) {
                // Response is accumulated in output_buffer. Uncomment the below line to print the accumulated response
                ESP_LOGD(TAG, "%s", output_buffer);
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "[HTTP] Transfer completed.");
            int mbedtls_err = 0;
            esp_err_t err = esp_tls_get_and_clear_last_error((esp_tls_error_handle_t)evt->data, &mbedtls_err, NULL);
            if (err != 0) {
                ESP_LOGI(TAG, "Last esp error code: 0x%x", err);
                ESP_LOGI(TAG, "Last mbedtls failure: 0x%x", mbedtls_err);
            }
            if (output_buffer != NULL) {
                free(output_buffer);
                output_buffer = NULL;
            }
            output_len = 0;
            break;
        case HTTP_EVENT_REDIRECT:
            ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
            esp_http_client_set_header(evt->client, "From", "user@example.com");
            esp_http_client_set_header(evt->client, "Accept", "text/html");
            esp_http_client_set_redirection(evt->client);
            break;
    }
    return ESP_OK;
}

esp_err_t post_image(camera_fb_t * fb){
   char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};

    ESP_LOGI(TAG, "[CAM] Sending image using signed S3 url");
    ESP_LOGD(TAG, "[CAM] url: %s", parameters.url);

    // esp_http_client_config_t config = {
    //     .url = parameters.url,
    //     .event_handler = _http_event_handler,
    //     .transport_type = HTTP_TRANSPORT_OVER_SSL,
    //     .cert_pem = server_s3_cert_pem_start,
    //     // not ideal, working around a certificate error
    //     // .skip_cert_common_name_check = true,
    // };
    
    
    esp_http_client_config_t config = {
        .url = parameters.url,
        .event_handler = _http_event_handler,
         .skip_cert_common_name_check = true,
        .buffer_size_tx = 2000
    };


    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_method(client, HTTP_METHOD_PUT); 


    ESP_LOGI(TAG, "before HTTPS Request,  Free memory: %lu bytes", esp_get_free_heap_size());
    esp_err_t err = esp_http_client_open(client, fb->len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "[HTTP] =====> request_method: PUT, content_length: %d", fb->len);
        int wlen = esp_http_client_write(client, (char *) fb->buf, fb->len);
        if (wlen < 0) {
            ESP_LOGE(TAG, "Write failed");
        } else{
            ESP_LOGD(TAG, "Wrote %d bytes", wlen);
        }

        int content_length = esp_http_client_fetch_headers(client);
        if (content_length < 0) {
            ESP_LOGE(TAG, "HTTP client fetch headers failed");
        } else {
            int data_read = esp_http_client_read_response(client, local_response_buffer, MAX_HTTP_OUTPUT_BUFFER);
            if (data_read >= 0) {
                ESP_LOGI(TAG, "[HTTP] <===== response_status: %d, content_length: %"PRIu64,
                    esp_http_client_get_status_code(client),
                    esp_http_client_get_content_length(client)
                );
             } else {
                ESP_LOGE(TAG, "Failed to read response");
            }
        }
    }
    esp_http_client_cleanup(client);
    return ESP_OK;
}

/// @brief Take a picture and uploads it
/// @return 
esp_err_t camera_capture(){
    
    //acquire a frame
    camera_fb_t * last_image = esp_camera_fb_get();
    if (!last_image) {
        ESP_LOGE(TAG, "[CAM] Camera Capture Failed");
        return ESP_FAIL;
    }else{
        ESP_LOGI(TAG, "[CAM] Picture taken! Its size was: %zu bytes", last_image->len);
    }

    post_image(last_image);
  
    //return the frame buffer back to the driver for reuse
    esp_camera_fb_return(last_image);
    return ESP_OK;
}


esp_err_t process_response_presigned_url(const char* buffer){
    // Parse JSON, get URL
    cJSON * signedUrlResponse = cJSON_Parse(buffer);
    if (signedUrlResponse == NULL)
    {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL)
        {
            ESP_LOGE(TAG, "Error before: %s\n", error_ptr);
        }
        return ESP_FAIL;
    }

    parse_json_and_populate_struct(signedUrlResponse);

    camera_capture();
    char message[160] = {0};
    sprintf(message, "{\"s3Key\": \"%s\"}", parameters.key); 
    esp_mqtt_client_publish(client, TOPIC_NAME_REKOGNIZE_REQUEST, message, 0, 0, 0);

    // clean up 
    cJSON_Delete(signedUrlResponse);
    return ESP_OK;
}

/// @brief sends a heartbeat message every 5 seconds
/// @param pvParameters 
void heartBeatTask(void *pvParameters)
{
    while (1)
    {
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
        ESP_LOGI(TAG, "[MQTT] MQTT_EVENT_CONNECTED");
        esp_mqtt_client_publish(client, TOPIC_NAME_AWS_MESSSAGES, "Hello, World!", 0, 0, 0);
        
        esp_mqtt_client_subscribe(client, TOPIC_NAME_AWS_CMD_SNAP, 0);
        esp_mqtt_client_subscribe(client, TOPIC_NAME_AWS_PRESIGNED_URL, 0);
        
        // This is the heartbeat task that will run in the background
        // It can only be started once
        // 
        if (heartBeatTaskHandle == NULL){
            xTaskCreate(heartBeatTask, "heartbeat", configMINIMAL_STACK_SIZE * 8, NULL, 5, &heartBeatTaskHandle);
            ESP_LOGI(TAG, "[MQTT] Heartbeat task created");
        }else{
            ESP_LOGI(TAG, "[MQTT] Hearbeat task was already created, probably a reconnect");
        }
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "[MQTT] MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "[MQTT] MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "[MQTT] MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "[MQTT] MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        // messages can arrive in chunks. If a follow-up chunk arrives, the topic == NULL 
        if (event->topic == NULL && (event->current_data_offset != 0 &&  event->total_data_len > event->data_len)) {
                ESP_LOGD(TAG, "received following parts of the presigned url");
                strncpy(mqtt_message_buffer + event->current_data_offset, event->data, event->data_len);
                mqtt_message_buffer[event->current_data_offset + event->data_len] = '\0';

                // Last part has been received;
                if(strlen(mqtt_message_buffer) >= event->total_data_len){
                    ESP_LOGI(TAG, "[MQTT] Received presigned URL");
                    ESP_ERROR_CHECK(process_response_presigned_url(mqtt_message_buffer));
            }
            
        }else if (strncmp(event->topic, TOPIC_NAME_AWS_PRESIGNED_URL, strlen(TOPIC_NAME_AWS_PRESIGNED_URL)) == 0){
            ESP_LOGD(TAG, "[MQTT] Receiving presigned URL");
            // If current_data_offset > 0, the message needs to be concatenated to the buffer. Only if the buffer's final
            // size => total_data_len of the MQTT message, shoudl the processing start
            if( event->current_data_offset == 0 &&  event->total_data_len > event->data_len) {
                ESP_LOGD(TAG, "[MQTT] received first part of the presigned url");
                strncpy(mqtt_message_buffer, event->data, event->data_len);
                mqtt_message_buffer[event->data_len] = '\0';
            }
            
        } else if (strncmp(event->topic, TOPIC_NAME_AWS_CMD_SNAP, strlen(TOPIC_NAME_AWS_CMD_SNAP)) == 0){
            ESP_LOGI(TAG, "[MQTT] Received Snap Command");
            request_presigned_url();
        } else {
            ESP_LOGI(TAG, "[MQTT] EVENT_DATA could not be handled");
        }
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "[MQTT] MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            log_error_if_nonzero("reported from esp-tls", event->error_handle->esp_tls_last_esp_err);
            log_error_if_nonzero("reported from tls stack", event->error_handle->esp_tls_stack_err);
            log_error_if_nonzero("captured as transport's socket errno",  event->error_handle->esp_transport_sock_errno);
            ESP_LOGI(TAG, "[MQTT] Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "[MQTT] Other event id:%d", event->event_id);
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

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_BASE", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(camera_init());

    ESP_ERROR_CHECK(example_connect());

    mqtt_app_start();
}

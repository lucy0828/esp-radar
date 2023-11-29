#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <lwip/netdb.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"

#include "cli_task.h"
#include "resource_map.h"
#include "xensiv_bgt60trxx_esp.h"
#include "xensiv_radar_presence.h"

#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#define PORT CONFIG_EXAMPLE_PORT

#define XENSIV_BGT60TRXX_CONF_IMPL
#include "radar_settings_tr13c.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define XENSIV_BGT60TRXX_SPI_FREQUENCY      (20000000UL)

#define NUM_SAMPLES_PER_FRAME               (XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP *\
                                             XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME *\
                                             XENSIV_BGT60TRXX_CONF_NUM_RX_ANTENNAS)
#define NUM_CHIRPS_PER_FRAME                XENSIV_BGT60TRXX_CONF_NUM_CHIRPS_PER_FRAME
#define NUM_SAMPLES_PER_CHIRP               XENSIV_BGT60TRXX_CONF_NUM_SAMPLES_PER_CHIRP

/* RTOS tasks */
#define RADAR_TASK_NAME                     "radar_task"
#define RADAR_TASK_STACK_SIZE               (configMINIMAL_STACK_SIZE * 4)
#define RADAR_TASK_PRIORITY                 (configMAX_PRIORITIES - 1)
#define PROCESSING_TASK_NAME                "processing_task"
#define PROCESSING_TASK_STACK_SIZE          (configMINIMAL_STACK_SIZE * 4)
#define PROCESSING_TASK_PRIORITY            (configMAX_PRIORITIES - 2)

#define CLI_TASK_NAME                       "cli_task"
#define CLI_TASK_STACK_SIZE                 (configMINIMAL_STACK_SIZE * 10)
#define CLI_TASK_PRIORITY                   (tskIDLE_PRIORITY)

/*******************************************************************************
* Function Prototypes
********************************************************************************/
static void udp_client_task(void *pvParameters);
static void radar_task(void *pvParameters);
static void processing_task(void *pvParameters);
static void timer_callback(TimerHandle_t xTimer);

static int32_t init_leds(void);
static int32_t init_sensor(void);
static void xensiv_bgt60trxx_interrupt_handler(void* args);

/*******************************************************************************
* Global Variables
********************************************************************************/
typedef struct {
    int count;
    float values[NUM_SAMPLES_PER_CHIRP];
} radar_data_t;

static const char *TAG = "radar_data";
static QueueHandle_t radar_data_queue;
static spi_device_handle_t spi_obj;
static xensiv_bgt60trxx_esp_t bgt60_obj;
static uint16_t bgt60_buffer[NUM_SAMPLES_PER_FRAME];
static float32_t frame[NUM_SAMPLES_PER_FRAME];
static TaskHandle_t radar_task_handler;
static TaskHandle_t processing_task_handler;
static TimerHandle_t timer_handler;
static int count;

void app_main(void)
{   
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    
    // Adjust size as needed
    radar_data_queue = xQueueCreate(10, sizeof(radar_data_t)); 
 
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);

    timer_handler = xTimerCreate("timer", pdMS_TO_TICKS(1000), pdTRUE, NULL, timer_callback);    
    if (timer_handler == NULL)
    {
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    if (xTimerStart(timer_handler, 0) != pdPASS)
    {
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    /* Create the radar task */    
    if (xTaskCreatePinnedToCore(radar_task, RADAR_TASK_NAME, RADAR_TASK_STACK_SIZE, NULL, RADAR_TASK_PRIORITY, &radar_task_handler, 0) != pdPASS)
    {
        ESP_ERROR_CHECK(ESP_FAIL);
    }    
}

static void udp_client_task(void *pvParameters) {
    char rx_buffer[128];
    int addr_family = 0;
    int ip_protocol = 0;
    struct sockaddr_storage dest_addr = { 0 }; // Generalize for IPv4/IPv6

    // Determine address family and protocol based on the configuration
#if defined(CONFIG_EXAMPLE_IPV4)
    struct sockaddr_in *dest_addr_ipv4 = (struct sockaddr_in *)&dest_addr;
    dest_addr_ipv4->sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
    dest_addr_ipv4->sin_family = AF_INET;
    dest_addr_ipv4->sin_port = htons(PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
    struct sockaddr_in6 *dest_addr_ipv6 = (struct sockaddr_in6 *)&dest_addr;
    inet6_aton(HOST_IP_ADDR, &dest_addr_ipv6->sin6_addr);
    dest_addr_ipv6->sin6_family = AF_INET6;
    dest_addr_ipv6->sin6_port = htons(PORT);
    dest_addr_ipv6->sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
    addr_family = AF_INET6;
    ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
    ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_DGRAM, &ip_protocol, &addr_family, &dest_addr));
#endif

    while (1) {
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break; // Consider delay or retry logic here
        }

        // Set timeout for receiving
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        while (1) {
            radar_data_t data;
            if (xQueueReceive(radar_data_queue, &data, portMAX_DELAY)) {
                int err = sendto(sock, &data, sizeof(data), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break; // Consider delay or retry logic here
                }
                ESP_LOGI(TAG, "Data sent");
            }

            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                // Consider whether to break or just log the error and continue
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes: %s", len, rx_buffer);
                // Handle received data
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

static void radar_task(void *pvParameters)
{
    (void)pvParameters;

    /* Create the processing task */
    if (xTaskCreatePinnedToCore(processing_task, PROCESSING_TASK_NAME, PROCESSING_TASK_STACK_SIZE, NULL, PROCESSING_TASK_PRIORITY, &processing_task_handler, 1) != pdPASS)
    {
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    if (init_sensor() != 0)
    {
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    if (init_leds () != 0)
    {
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("****************** "
           "XENSIV BGT60TR13C radar solution demo "
           "****************** \r\n\n"
           "Human presence detection using XENSIV BGT60TR13C radar and ESP32\r\n"
           );

    printf("Press ENTER to enter setup mode, press ESC to quit setup mode \r\n");

    if (xensiv_bgt60trxx_start_frame(&bgt60_obj.dev, true) != XENSIV_BGT60TRXX_STATUS_OK)
    {
        ESP_ERROR_CHECK(ESP_FAIL);
    }

    for(;;)
    {
        /* Wait for the GPIO interrupt to indicate that another slice is available */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (xensiv_bgt60trxx_get_fifo_data(&bgt60_obj.dev, 
                                           bgt60_buffer,
                                           NUM_SAMPLES_PER_FRAME) == XENSIV_BGT60TRXX_STATUS_OK)
        {
            /* Tell processing task to take over */
            xTaskNotifyGive(processing_task_handler);
        }
        else
        {
            printf(".");
        }
    }
}

static void processing_task(void *pvParameters)
{
    for(;;)
    {
        /* Wait for frame data available to process */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // /* Print ADC data */
        // uint16_t *bgt60_buffer_ptr = &bgt60_buffer[0];

        // printf("\n\n12-bit ADC values (16-bit buffer):\n");
        // for(uint32_t i = 0; i < NUM_SAMPLES_PER_FRAME; i++) {
        //     printf("%u ", *bgt60_buffer_ptr++);  // Print each sample as an unsigned integer
        //     if ((i + 1) % NUM_SAMPLES_PER_CHIRP == 0) { // Break the line every 128 samples for readability
        //         printf("\n");
        //     }
        // }

        uint16_t *bgt60_buffer_ptr = &bgt60_buffer[0];
        float32_t *frame_ptr = &frame[0];  

        for (int32_t sample = 0; sample < NUM_SAMPLES_PER_FRAME; ++sample)
        {
            *frame_ptr++ = ((float32_t)(*bgt60_buffer_ptr++) / 4096.0F);
        }

        radar_data_t data;
        data.count = count; 

        for (int sample = 0; sample < NUM_SAMPLES_PER_CHIRP; sample++)
        {
            data.values[sample] = frame[sample];
        }

        xQueueSend(radar_data_queue, &data, portMAX_DELAY);
        count++;
    }
}

static int32_t init_sensor(void)
{
#ifdef USE_FIFO_SPI_TRANSACTION
    bool use_dma = false;
#else
    bool use_dma = true;
#endif

    /* Initialize the SPI interface to BGT60. */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_XENSIV_BGT60TRXX_SPI_MOSI,
        .miso_io_num = PIN_XENSIV_BGT60TRXX_SPI_MISO,
        .sclk_io_num = PIN_XENSIV_BGT60TRXX_SPI_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = NUM_SAMPLES_PER_FRAME * 2
    };

    /* Using DMA and single transaction */
    if (spi_bus_initialize(SPI3_HOST, &bus_cfg,
                           use_dma ? SPI_DMA_CH_AUTO : SPI_DMA_DISABLED) != ESP_OK)
    {
        printf("ERROR: spi_bus_initialize failed\n");
        return -1;
    }

    /* Set SPI data rate to communicate with sensor */
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = XENSIV_BGT60TRXX_SPI_FREQUENCY,
        .mode = 0,
        .spics_io_num = PIN_XENSIV_BGT60TRXX_SPI_CSN,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL
    };

    if (spi_bus_add_device(SPI3_HOST, &dev_cfg, &spi_obj) != ESP_OK)
    {
        printf("ERROR: spi_bus_add_device failed\n");
        return -1;
    }

    /* Enable the LDO. */
    gpio_config_t ldo_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_DEF_OUTPUT,
        .pin_bit_mask = (1ULL << PIN_XENSIV_BGT60TRXX_LDO_EN),
        .pull_down_en = 0,
        .pull_up_en = 1
    };

    if (gpio_config(&ldo_cfg) != ESP_OK)
    {
        printf("ERROR: LDO gpio_config failed\n");
        return -1;
    }

    gpio_set_level(PIN_XENSIV_BGT60TRXX_LDO_EN, 1);

    /* Wait LDO stable */
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Use a DMA or FIFO for SPI transaction */
    if (xensiv_bgt60trxx_esp_init(&bgt60_obj,
                                  spi_obj,
                                  PIN_XENSIV_BGT60TRXX_SPI_CSN,
                                  PIN_XENSIV_BGT60TRXX_RSTN,
                                  use_dma,
                                  register_list,
                                  XENSIV_BGT60TRXX_CONF_NUM_REGS) != ESP_OK) 
    {
        printf("ERROR: xensiv_bgt60trxx_esp_init failed\n");
        return -1;
    }

    /* The sensor will generate an interrupt once the sensor FIFO level is
       NUM_SAMPLES_PER_FRAME */
    if (xensiv_bgt60trxx_esp_interrupt_init(&bgt60_obj,
                                            NUM_SAMPLES_PER_FRAME,
                                            PIN_XENSIV_BGT60TRXX_IRQ,
                                            xensiv_bgt60trxx_interrupt_handler,
                                            NULL) != ESP_OK)
    {
        printf("ERROR: xensiv_bgt60trxx_esp_interrupt_init failed\n");
        return -1;
    }                                                 

    return 0;
}

static void IRAM_ATTR xensiv_bgt60trxx_interrupt_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    vTaskNotifyGiveFromISR(radar_task_handler, &xHigherPriorityTaskWoken);

    /* Context switch needed? */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static int32_t init_leds(void)
{
    gpio_config_t led_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_DEF_OUTPUT,
        .pin_bit_mask = ((1ULL<<PIN_LED_RED) | (1ULL<<PIN_LED_GREEN)),
        .pull_down_en = 0,
        .pull_up_en = 1
    };

    if (gpio_config(&led_conf) != ESP_OK)
    {
        return -1;
    }
    
    gpio_set_level(PIN_LED_RED, 0);    
    gpio_set_level(PIN_LED_GREEN, 0);   
    //gpio_set_level(PIN_LED_BLUE, 0); Blue pin is input only

    return 0;
}


static void timer_callback(TimerHandle_t xTimer)
{
    (void)xTimer;
}

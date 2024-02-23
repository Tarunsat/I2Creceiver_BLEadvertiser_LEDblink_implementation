#include <stdio.h>
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "esp_timer.h"
#include "driver/gpio.h"
#include <freertos/timers.h>

static const char *TAG = "i2c-slave";
#define DEVICE_NAME "Master boi"

uint8_t ble_addr_type;
QueueHandle_t queue;
int d=0;
#define PIN 2
void ble_app_advertise(void);

#define DEVICE_INFO_SERVICE 0x180A
#define I2C_VALUE  0x2A6E



#define LED_PIN 2
#define I2C_SLAVE_SCL_IO 7               /*!< gpio number for I2C master clock */
#define I2C_SLAVE_SDA_IO 6               /*!< gpio number for I2C master data  */
#define I2C_SLAVE_FREQ_HZ 100000        /*!< I2C master clock frequency */
#define I2C_SLAVE_TX_BUF_LEN 255                        /*!< I2C master doesn't need buffer */
#define I2C_SLAVE_RX_BUF_LEN 255                           /*!< I2C master doesn't need buffer */
#define ESP_SLAVE_ADDR 0x0A
#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */
int i2c_slave_port = 0;

int mess=0;

static esp_err_t i2c_slave_init(void)
{
  
    i2c_config_t conf_slave = {
    .sda_io_num = I2C_SLAVE_SDA_IO,          // select GPIO specific to your project
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = I2C_SLAVE_SCL_IO,          // select GPIO specific to your project
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .mode = I2C_MODE_SLAVE,
    .slave.addr_10bit_en = 0,
    .slave.slave_addr = ESP_SLAVE_ADDR,      // address of your project
    .clk_flags = 0,
    };
    esp_err_t err = i2c_param_config(i2c_slave_port, &conf_slave);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
}

void ledblink(int a)
{

  gpio_set_direction(a, GPIO_MODE_OUTPUT); // Setting pin 2 as output
  uint32_t isOn = 0; // Using an unsigned int instead of boolean as the gpio.setlevel function requires the uint as a argument
  while (d==0)
  {
    isOn = !isOn;
    gpio_set_level(a, isOn); // Set the pin 2 as the value 
    vTaskDelay(200 / portTICK_PERIOD_MS); //One second delay
  }

}

void I2Creceive()
{
    uint8_t receive_int[2];
    memset(receive_int,0,sizeof(receive_int));
    uint8_t  on_command[] = "LED_ON";
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(i2c_slave_init());
    ESP_LOGI(TAG, "I2C Slave initialized successfully");
    
    while(1)
    {
        i2c_slave_read_buffer(i2c_slave_port, receive_int, sizeof(receive_int), 1000 / portTICK_PERIOD_MS);
        i2c_reset_rx_fifo(i2c_slave_port);
        int ans = (receive_int[0]>>8) | receive_int[1];
        mess = ans;
        char *g ="I2C";
         
        memset(receive_int,0,sizeof(receive_int));
        if(mess!=0)
        {
            ESP_LOGI(TAG, "Data Received = %d", ans);  
            long ok = xQueueSend(queue,g, 1000/portTICK_PERIOD_MS);
        }
        
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }

}
void callbacker(TimerHandle_t timer)
{
    printf("Timer ended\n");
    d =1;
}
void LedListener(void * params)
{
    while(true)
    {
        char h[3];
        if(xQueueReceive(queue,h,5000/portTICK_PERIOD_MS))
        {
            printf("Queue receive\n");
            // Start the timer after resetting
            xTimerStart((TimerHandle_t)params, 0);
            printf("%s\n",h);
            d=0;
            if(strcmp(h,"I2C"))
            {
                ledblink(2);
            }
            else if(strcmp(h,"BLE"))
            {
                ledblink(9);
            }
            
        }
        else
        {
            printf("Nothing recieved\n");
        }
        
    }
}
static int device_info(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    os_mbuf_append(ctxt->om,(uint8_t*)&mess, sizeof(mess));
    return 0;

}
static const struct ble_gatt_svc_def gat_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(DEVICE_INFO_SERVICE),
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = BLE_UUID16_DECLARE(I2C_VALUE),
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_info},
         {0}}},
    {0}};

static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    long ok = xQueueSend(queue,"BLE", 1000/portTICK_PERIOD_MS);
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI("GAP", "BLE_GAP_EVENT_CONNECT %s", event->connect.status == 0 ? "OK" : "Failed");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI("GAP", "BLE_GAP_EVENT_DISCONNECT");
        ble_app_advertise();
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE_GAP_EVENT_ADV_COMPLETE");
        ble_app_advertise();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI("GAP", "BLE_GAP_EVENT_SUBSCRIBE");
        break;
    default:
        break;
    }
    return 0;

}

void ble_app_advertise(void)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_DISC_LTD;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    fields.name = (uint8_t *)ble_svc_gap_device_name();
    fields.name_len = strlen(ble_svc_gap_device_name());
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type);
    ble_app_advertise();
}

void host_task(void *param)
{
    nimble_port_run();
    vTaskDelay(1000/portTICK_PERIOD_MS);
}

void app_main(void)
{
    nvs_flash_init();
    esp_nimble_hci_init();
    nimble_port_init();
    ble_svc_gap_device_name_set(DEVICE_NAME);
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(&gat_svcs);
    ble_gatts_add_svcs(&gat_svcs);
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    nimble_port_freertos_init(host_task);
    xTaskCreate(&I2Creceive,"i2c recieve",2048,NULL,2,NULL);

    TimerHandle_t timer = xTimerCreate(
        "BlinkTimer",        // Timer name
        pdMS_TO_TICKS(1000),   // Timer period in milliseconds (500 ms)
        true,              // Auto-reload enabled
        NULL,          // Timer ID, not used in this example
        callbacker        // Timer callback function
    );
    queue = xQueueCreate(10,4*sizeof(char));
    xTaskCreate(&LedListener,"task2",2048,timer,2,NULL);
}


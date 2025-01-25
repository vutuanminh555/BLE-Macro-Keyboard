#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h" 

#include "esp_hidd_prf_api.h" 
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "driver/gpio.h"
#include "hid_dev.h" 

#include "esp_sleep.h" // for sleep mode
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/soc_caps.h"
#include "driver/rtc_io.h"

#include "ulp.h"  // For ULP Coprocessor
#include "ulp_main.h"
#include "esp_adc/adc_oneshot.h"
#include "ulp/config.h"
#include "ulp_adc.h"

#include "lcd.h"
#include "button.h" 

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// for ULP processor 
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

// This function is called once after power-on reset, to load ULP program into RTC memory and configure the ADC.
void init_ulp_program()
{
    ulp_load_binary(0, ulp_main_bin_start,
            (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));

    ulp_adc_cfg_t cfg = {
        .adc_n    = ADC_UNIT,
        .channel  = ADC_CHANNEL,
        .width    = ADC_WIDTH,
        .atten    = ADC_ATTEN,
        .ulp_mode = ADC_ULP_MODE_FSM,
    };

    ulp_adc_init(&cfg);

    ulp_low_thr = ADC_LOW_TRESHOLD;
    ulp_high_thr = ADC_HIGH_TRESHOLD;

    /* Set ULP wake up period to 20ms */
    ulp_set_wakeup_period(0, 20000);

    rtc_gpio_isolate(GPIO_NUM_12);
    esp_deep_sleep_disable_rom_logging(); // suppress boot messages

}

// This function is called every time before going into deep sleep.
// It starts the ULP program and resets measurement counter.
void start_ulp_program()
{
    /* Reset sample counter */
    ulp_sample_counter = 0;

    /* Start the program */
    ulp_run(&ulp_entry - RTC_SLOW_MEM);
}


#define LCD_PIN         (1ULL << D4) | (1ULL << D5) | (1ULL << D6) | (1ULL << D7) | (1ULL << EN) | (1ULL << RS) | (1ULL << BL)
#define BTN_PIN         (1ULL << btn1) | (1ULL << btn2) | (1ULL << btn3) | (1ULL << btn4) | (1ULL << btn5) | (1ULL << btn6) | (1ULL << btn7) | (1ULL << btn8) | (1ULL << btn9) 

volatile int WORKING_MODE = 0;

QueueHandle_t btn_evt_queue = NULL; 
QueueHandle_t hid_consumer_evt_queue = NULL; 
QueueHandle_t hid_keyboard_evt_queue = NULL; 

static uint16_t hid_conn_id = 0;
static bool sec_conn = false; // detect device connected or not 
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param);

#define HIDD_DEVICE_NAME            "BLE Macro Keyboard"

static uint8_t hidd_service_uuid128[] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x12, 0x18, 0x00, 0x00,
};

static esp_ble_adv_data_t hidd_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x03c0,       //HID Generic,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(hidd_service_uuid128),
    .p_service_uuid = hidd_service_uuid128,
    .flag = 0x6,
};

static esp_ble_adv_params_t hidd_adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x30,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


static void hidd_event_callback(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    switch(event) {
        case ESP_HIDD_EVENT_REG_FINISH: {
            if (param->init_finish.state == ESP_HIDD_INIT_OK) {
                //esp_bd_addr_t rand_addr = {0x04,0x11,0x11,0x11,0x11,0x05};
                esp_ble_gap_set_device_name(HIDD_DEVICE_NAME);
                esp_ble_gap_config_adv_data(&hidd_adv_data);
            }
            break;
        }
        case ESP_BAT_EVENT_REG: {
            break;
        }
        case ESP_HIDD_EVENT_DEINIT_FINISH:
	        break;
		case ESP_HIDD_EVENT_BLE_CONNECT: { 
            hid_conn_id = param->connect.conn_id;
            break;
        }
        case ESP_HIDD_EVENT_BLE_DISCONNECT: {
            sec_conn = false; 
            esp_ble_gap_start_advertising(&hidd_adv_params);
            break;
        }
        case ESP_HIDD_EVENT_BLE_VENDOR_REPORT_WRITE_EVT: {
            break;
        }
        case ESP_HIDD_EVENT_BLE_LED_REPORT_WRITE_EVT: {
            break;
        }
        default:
            break;
    }
    return;
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        esp_ble_gap_start_advertising(&hidd_adv_params);
        break;
    case ESP_GAP_BLE_SEC_REQ_EVT:
        esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
	    break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT:
        sec_conn = true;
        esp_bd_addr_t bd_addr;
        memcpy(bd_addr, param->ble_security.auth_cmpl.bd_addr, sizeof(esp_bd_addr_t));
        break;
    default:
        break;
    }
}

void hid_consumer_task(void *pvParameters) 
{
    uint8_t hid_key;
    while(1) { 
        vTaskDelay(10);
        if (sec_conn) {
            if(xQueueReceive(hid_consumer_evt_queue, &hid_key, portMAX_DELAY)){
                esp_hidd_send_consumer_value(hid_conn_id, hid_key, true); // press 
                esp_hidd_send_consumer_value(hid_conn_id, hid_key, false); // release
            }
        }
    }
}

void hid_keyboard_task(void *pvParameters) 
{
    uint8_t key_value;
    uint8_t release = 0;
    while(1) { 
        vTaskDelay(10);
        if (sec_conn) {
            if(xQueueReceive(hid_keyboard_evt_queue, &key_value, portMAX_DELAY)){
                esp_hidd_send_keyboard_value(hid_conn_id, 1, &key_value, 1); // press
                esp_hidd_send_keyboard_value(hid_conn_id, 0, &release, 1); // release
            }
            
        }
    }
}

void IRAM_ATTR btn_isr_handler(void* arg) 
{
    uint32_t gpio_num = (uint32_t) arg; 
    xQueueSendFromISR(btn_evt_queue, &gpio_num, NULL);
}

void btn_task(void* arg) 
{
    uint32_t io_num;
    uint8_t hid_key;
    uint8_t key_value;

    uint32_t time_last_int = 0;
    uint32_t current_time;
    while (1) {
        if (xQueueReceive(btn_evt_queue, &io_num, portMAX_DELAY)) { // should add sensor for holding button 
            current_time = xTaskGetTickCount();
            if(current_time - time_last_int > pdMS_TO_TICKS(200)){ // delay 200ms for debounce
                time_last_int = current_time;
                switch (io_num)
                {
                case btn1:
                    if(WORKING_MODE == 1){
                        key_value = HID_KEY_C;
                        xQueueSend(hid_keyboard_evt_queue, &key_value, portMAX_DELAY);
                        LCD_String_xy(2, 8, "Copy        ");
                    }
                    else if(WORKING_MODE == 2){
                        hid_key = HID_CONSUMER_PLAY;
                        xQueueSend(hid_consumer_evt_queue, &hid_key, portMAX_DELAY);
                        LCD_String_xy(2, 8, "Play/Pause  ");
                    }
                    break;
                case btn2:
                    if(WORKING_MODE == 1){
                        key_value = HID_KEY_X;
                        xQueueSend(hid_keyboard_evt_queue, &key_value, portMAX_DELAY);
                        LCD_String_xy(2, 8, "Cut         ");
                    }
                    else if(WORKING_MODE == 2){
                        hid_key = HID_CONSUMER_STOP;
                        xQueueSend(hid_consumer_evt_queue, &hid_key, portMAX_DELAY);
                        LCD_String_xy(2, 8, "Stop        ");
                    }
                    break;
                case btn3:
                    if(WORKING_MODE == 1){
                        key_value = HID_KEY_V;
                        xQueueSend(hid_keyboard_evt_queue, &key_value, portMAX_DELAY);
                        LCD_String_xy(2, 8, "Paste       ");
                    }
                    else if(WORKING_MODE == 2){
                        hid_key = HID_CONSUMER_REPEAT;
                        xQueueSend(hid_consumer_evt_queue, &hid_key, portMAX_DELAY);
                        LCD_String_xy(2, 8, "Repeat      ");
                    }
                    break;
                case btn4:
                    if(WORKING_MODE == 1){
                        key_value = HID_KEY_Z;
                        xQueueSend(hid_keyboard_evt_queue, &key_value, portMAX_DELAY);
                        LCD_String_xy(2, 8, "Undo        ");
                    }
                    else if(WORKING_MODE == 2){
                        hid_key = HID_CONSUMER_VOLUME_UP;
                        xQueueSend(hid_consumer_evt_queue, &hid_key, portMAX_DELAY);
                        LCD_String_xy(2, 8, "Volume Up   ");
                    }
                    break;
                case btn5:
                    if(WORKING_MODE == 1){
                        key_value = HID_KEY_Y;
                        xQueueSend(hid_keyboard_evt_queue, &key_value, portMAX_DELAY);
                        LCD_String_xy(2, 8, "Redo        ");
                    }
                    else if(WORKING_MODE == 2){
                        hid_key = HID_CONSUMER_VOLUME_DOWN;
                        xQueueSend(hid_consumer_evt_queue, &hid_key, portMAX_DELAY);
                        LCD_String_xy(2, 8, "Volume Down ");
                    }
                    break;
                case btn6:
                    if(WORKING_MODE == 1){
                        key_value = HID_KEY_S;
                        xQueueSend(hid_keyboard_evt_queue, &key_value, portMAX_DELAY);
                        LCD_String_xy(2, 8, "Save        ");
                    }
                    else if(WORKING_MODE == 2){
                        hid_key = HID_CONSUMER_MUTE;
                        xQueueSend(hid_consumer_evt_queue, &hid_key, portMAX_DELAY);
                        LCD_String_xy(2, 8, "Mute/Unmute ");
                    }
                    break;
                case btn7:
                    if(WORKING_MODE == 1){
                        key_value = HID_KEY_F;
                        xQueueSend(hid_keyboard_evt_queue, &key_value, portMAX_DELAY);
                        LCD_String_xy(2, 8, "Find        ");
                    }
                    else if(WORKING_MODE == 2){
                        hid_key = HID_CONSUMER_SCAN_NEXT_TRK;
                        xQueueSend(hid_consumer_evt_queue, &hid_key, portMAX_DELAY);
                        LCD_String_xy(2, 8, "Next Track  ");
                    }
                    break;
                case btn8:
                    if(WORKING_MODE == 1){
                        key_value = HID_KEY_N;
                        xQueueSend(hid_keyboard_evt_queue, &key_value, portMAX_DELAY);
                        LCD_String_xy(2, 8, "New Window  ");
                    }
                    else if(WORKING_MODE == 2){
                        hid_key = HID_CONSUMER_SCAN_PREV_TRK;
                        xQueueSend(hid_consumer_evt_queue, &hid_key, portMAX_DELAY);
                        LCD_String_xy(2, 8, "Prev Track  ");
                    }
                    break;
                case btn9:
                    if(WORKING_MODE == 1){
                        key_value = HID_KEY_W;
                        xQueueSend(hid_keyboard_evt_queue, &key_value, portMAX_DELAY);
                        LCD_String_xy(2, 8, "Close Window");
                    }
                    else if(WORKING_MODE == 2){
                        hid_key = HID_CONSUMER_RANDOM_PLAY;
                        xQueueSend(hid_consumer_evt_queue, &hid_key, portMAX_DELAY);
                        LCD_String_xy(2, 8, "Random Play ");
                    }
                    break;
                default:
                        LCD_String_xy(2, 8, "Button Error");
                    break;
                }
            
            }
        }
    }
} 

void mode_task(void *pvParameters){
    uint16_t adc_value; 
    int oldMode = 0; // avoid interfering with other LCD write command 
    bool old_sec_conn = true; // different from initial value of sec_conn
    while (1) {
        if(old_sec_conn != sec_conn){ // write to LCD only when sec_conn change value
            if(sec_conn == true){
                LCD_String_xy(3, 0, "Device Connected    ");
            }
            else
                LCD_String_xy(3, 0, "Device Disconnected ");
        }
        adc_value = ulp_last_result & UINT16_MAX;
        if(adc_value  < 512){  // adc_reading from 0 to 4095
            LCD_String_xy(3, 0, "   Turning off !!   ");
            vTaskDelay(100);
            LCD_Deinit();
            esp_deep_sleep_start();
        }
        else if(adc_value > 511 && adc_value < 2303){
            WORKING_MODE = 1;
            if(oldMode != WORKING_MODE)
            LCD_String_xy(1, 6, "General   ");
            oldMode = WORKING_MODE;
        }
        else if(adc_value > 2302){
            WORKING_MODE = 2;
            if(oldMode != WORKING_MODE)
            LCD_String_xy(1, 6, "Multimedia");
            oldMode = WORKING_MODE;
        }
        old_sec_conn = sec_conn;
        vTaskDelay(1);
    }
}

void app_main(void)
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_ULP) {
        init_ulp_program();
    }
    start_ulp_program();
    esp_sleep_enable_ulp_wakeup();

    if((ulp_last_result & UINT16_MAX) < 512){ 
    esp_deep_sleep_start();
    }

    else{
    gpio_config_t lcd_pin = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = LCD_PIN,
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&lcd_pin);

    gpio_set_level(BL, 1);
    LCD_Init();

    gpio_config_t btn_pin = { 
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = BTN_PIN,
        .pull_down_en = 0,          
        .pull_up_en = 1
    };
    gpio_config(&btn_pin);

    //create a queue to handle gpio event from isr
    btn_evt_queue = xQueueCreate(20, sizeof(uint32_t));

    // queue for hid consumer value from button 
    hid_consumer_evt_queue = xQueueCreate(10, sizeof(uint8_t)); 
    // queue for hid keyboard value from button 
    hid_keyboard_evt_queue = xQueueCreate(10, sizeof(uint8_t)); 

    //start gpio task
    xTaskCreate(&btn_task, "btn_task", 2048, NULL, 10, NULL); 
    
    //install gpio isr service
    gpio_install_isr_service(0);

    //hook isr handler to all pin connected to button
    gpio_isr_handler_add(btn1, btn_isr_handler, (void*) btn1);
    gpio_isr_handler_add(btn2, btn_isr_handler, (void*) btn2);
    gpio_isr_handler_add(btn3, btn_isr_handler, (void*) btn3);
    gpio_isr_handler_add(btn4, btn_isr_handler, (void*) btn4);
    gpio_isr_handler_add(btn5, btn_isr_handler, (void*) btn5);
    gpio_isr_handler_add(btn6, btn_isr_handler, (void*) btn6);
    gpio_isr_handler_add(btn7, btn_isr_handler, (void*) btn7);
    gpio_isr_handler_add(btn8, btn_isr_handler, (void*) btn8);
    gpio_isr_handler_add(btn9, btn_isr_handler, (void*) btn9);
    
    // Concurrent task for checking working status 
    xTaskCreate(&mode_task, "adc_task", 2048, NULL, 9, NULL); 

    // Initialize NVS.
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    esp_bt_controller_init(&bt_cfg);
    esp_bt_controller_enable(ESP_BT_MODE_BLE);
    esp_bluedroid_init();
    esp_bluedroid_enable();
    esp_hidd_profile_init();

    ///register the callback function to the gap module
    esp_ble_gap_register_callback(gap_event_handler);
    esp_hidd_register_callbacks(hidd_event_callback);

    /* set the security iocap & auth_req & key size & init key response key parameters to the stack*/
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_BOND;     //bonding with peer device after authentication
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           //set the IO capability to No output No input
    uint8_t key_size = 16;      //the key size should be 7~16 bytes
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    /* If your BLE device act as a Slave, the init_key means you hope which types of key of the master should distribute to you,
    and the response key means which key you can distribute to the Master;
    If your BLE device act as a master, the response key means you hope which types of key of the slave should distribute to you,
    and the init key means which key you can distribute to the slave. */
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    xTaskCreate(&hid_consumer_task, "hid_consumer_task", 2048, NULL, 11, NULL);
    xTaskCreate(&hid_keyboard_task, "hid_keyboard_task", 2048, NULL, 12, NULL);
    }
}

#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "zigbee.hpp"
#include "esp_zigbee_core.h"

#include <thread>
#include "ld2412_component.hpp"

namespace zb
{
    constexpr uint8_t PRESENCE_EP = 1;

    static char g_Manufacturer[] = "Orlangur\0";
    static char g_Model[] = "Presence\0";
    static const char *TAG = "ESP_ZB_PRESENCE_SENSOR";

    static ld2412::Component g_ld2412;


    struct APILock
    {
        APILock() { esp_zb_lock_acquire(portMAX_DELAY); }
        ~APILock() { esp_zb_lock_release(); }
    };

    static bool setup_sensor()
    {
        g_ld2412.SetCallbackOnMovement([&](bool presence, LD2412::PresenceResult const& p){
                uint8_t val = presence;
                {
                APILock l;
                esp_zb_zcl_set_attribute_val(PRESENCE_EP,
                        ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                        ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID, &val, false);
                }
                FMT_PRINT("Presence: {}; Data: {}\n", (int)presence, p);
                });

        if (!g_ld2412.Setup(ld2412::Component::setup_args_t{.txPin=11, .rxPin=10, .presencePin=8}))
        {
            printf("Failed to configure ld2412\n");
            fflush(stdout);
            return false;
        }
        return true;
    }

    static void create_presence_ep(esp_zb_ep_list_t *ep_list, uint8_t ep_id)
    {
        esp_zb_basic_cluster_cfg_t basic_cfg =                                                                                \
            {                                                                                       
                .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,                          
                .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE,                        
            };                                                                                      
        esp_zb_identify_cluster_cfg_t identify_cfg =                                                                             
            {                                                                                       
                .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,                   
            };                                                                                      
        esp_zb_occupancy_sensing_cluster_cfg_s presence_cfg =                                                                            
            {                                                                                       
                /*uint8_t*/  .occupancy = 0,                                                               /*!<  Bit 0 specifies the sensed occupancy as follows: 1 = occupied, 0 = unoccupied. */
                /*uint32_t*/ .sensor_type = ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_ULTRASONIC, /*!<  The attribute specifies the type of the occupancy sensor */
                /*uint8_t*/  .sensor_type_bitmap = uint8_t(1) << ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_ULTRASONIC /*!<  The attribute specifies the types of the occupancy sensor */
            };                                                                                      
        esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
        esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
        ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, g_Manufacturer));
        ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, g_Model));
        ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
        ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&identify_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
        ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
        ESP_ERROR_CHECK(esp_zb_cluster_list_add_occupancy_sensing_cluster(cluster_list, esp_zb_occupancy_sensing_cluster_create(&presence_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

        esp_zb_endpoint_config_t endpoint_config = {
            .endpoint = ep_id,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);
    }

    static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
    {
        ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
                            TAG, "Failed to start Zigbee bdb commissioning");
    }

    extern "C" void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
    {
        uint32_t *p_sg_p     = signal_struct->p_app_signal;
        esp_err_t err_status = signal_struct->esp_err_status;
        esp_zb_app_signal_type_t sig_type = *(esp_zb_app_signal_type_t*)p_sg_p;
        switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "Initialize Zigbee stack");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                bool sensor_init_ok = setup_sensor();
                ESP_LOGI(TAG, "Deferred sensor initialization %s", !sensor_init_ok ? "failed" : "successful");
                ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
                if (esp_zb_bdb_is_factory_new()) {
                    ESP_LOGI(TAG, "Start network steering");
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                } else {
                    ESP_LOGI(TAG, "Device rebooted");
                }
            } else {
                /* commissioning failed */
                ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
            }
            break;
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                esp_zb_ieee_addr_t extended_pan_id;
                esp_zb_get_extended_pan_id(extended_pan_id);
                ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                         extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                         extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                         esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            } else {
                ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;
        default:
            ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                     esp_err_to_name(err_status));
            break;
        }
    }

    void zigbee_main(void *)
    {
        ESP_LOGI(TAG, "ZB main");
        fflush(stdout);
        esp_zb_cfg_t zb_nwk_cfg = {                                                               
            .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       
            .install_code_policy = false,           
            .nwk_cfg = {
                .zed_cfg = {                                        
                    .ed_timeout = ESP_ZB_ED_AGING_TIMEOUT_16MIN,                         
                    .keep_alive = 3000,                            
                }
            },                                                          
        };
        esp_zb_init(&zb_nwk_cfg);
        ESP_LOGI(TAG, "ZB after init");
        fflush(stdout);

        //config clusters here
        esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
        create_presence_ep(ep_list, PRESENCE_EP);
        ESP_LOGI(TAG, "ZB created ep");
        fflush(stdout);

        /* Register the device */
        esp_zb_device_register(ep_list);
        ESP_LOGI(TAG, "ZB registered device");
        fflush(stdout);

        /* Config the reporting info  */
        esp_zb_zcl_reporting_info_t reporting_info = {
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
            .ep = PRESENCE_EP,
            .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
            .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            .attr_id = ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
            .flags = {},
            .run_time = {},
            .u = {
                .send_info = {
                    .min_interval = 1,
                    .max_interval = 0,
                    .delta = {.u8 = 1},
                    .reported_value = {.u8 = 0},//current value?
                    .def_min_interval = 1,
                    .def_max_interval = 0,
                }
            },
            .dst = { .short_addr = {}, .endpoint = {}, .profile_id = ESP_ZB_AF_HA_PROFILE_ID},
            .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        };

        esp_zb_zcl_update_reporting_info(&reporting_info);
        ESP_LOGI(TAG, "ZB updated attribute reporting");
        fflush(stdout);

        esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
        ESP_LOGI(TAG, "ZB set channel masks");
        fflush(stdout);

        ESP_ERROR_CHECK(esp_zb_start(false));
        ESP_LOGI(TAG, "ZB started, looping...");
        esp_zb_stack_main_loop();
    }

    void setup()
    {
        esp_zb_platform_config_t config = {
            .radio_config = {.radio_mode = ZB_RADIO_MODE_NATIVE, .radio_uart_config = {}},
            .host_config = {.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, .host_uart_config = {}},
        };
        ESP_ERROR_CHECK(nvs_flash_init());
        FMT_PRINT("nvs_flash_init done\n");
        ESP_ERROR_CHECK(esp_zb_platform_config(&config));
        FMT_PRINT("esp_zb_platform_config done\n");
        
        xTaskCreate(zigbee_main, "Zigbee_main", 4096, NULL, 5, NULL);
    }
}

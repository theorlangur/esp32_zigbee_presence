#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "zigbee.hpp"
#include "esp_zigbee_core.h"

#include "zb_helpers.hpp"

#include <thread>
#include <bitset>
#include "ld2412_component.hpp"

namespace zb
{
    constexpr uint8_t PRESENCE_EP = 1;

    static auto g_Manufacturer = ZbStr("Ionx");
    static auto g_Model = ZbStr("Occup");
    static const char *TAG = "ESP_ZB_PRESENCE_SENSOR";

    static ld2412::Component g_ld2412;

    static ZclAttributeAccess<
        PRESENCE_EP
        , ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING
        , ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
        , ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID
        , esp_zb_zcl_occupancy_sensing_occupancy_t> g_OccupancyState;

    using ZclAttributeOccupiedToUnoccupiedTimeout_t = ZclAttributeAccess<
        PRESENCE_EP
        , ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING
        , ESP_ZB_ZCL_CLUSTER_SERVER_ROLE
        , ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_PIR_OCC_TO_UNOCC_DELAY_ID
        , uint16_t>;
    ZclAttributeOccupiedToUnoccupiedTimeout_t g_OccupiedToUnoccupiedTimeout;

    esp_zb_ieee_addr_t g_CoordinatorIeee;

    struct EpCluster
    {
        uint8_t ep;
        uint16_t cluster_id;
    };

    static EpCluster g_OwnClusters[] = {
        {PRESENCE_EP, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING}
    };

    static bool setup_sensor()
    {
        g_ld2412.SetCallbackOnMovement([&](bool presence, LD2412::PresenceResult const& p){
                esp_zb_zcl_occupancy_sensing_occupancy_t val = presence ? ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_OCCUPIED : ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_UNOCCUPIED;
                {
                    APILock l;
                    if (auto status = g_OccupancyState.Set(val); !status)
                    {
                        FMT_PRINT("Failed to set attribute with error {:x}\n", (int)status.error());
                    }

                    if (false)
                    {
                        FMT_PRINT("Reporting Presence: {}\n", (int)presence);
                        if (auto r = g_OccupancyState.Report(g_DestCoordinator); !r)
                        {
                            FMT_PRINT("Failed to report attribute with error {:x}\n", r.error());
                        }
                    }
                }
                FMT_PRINT("Presence: {}; Data: {}\n", (int)presence, p);
                });

        if (!g_ld2412.Setup(ld2412::Component::setup_args_t{.txPin=11, .rxPin=10, .presencePin=8}))
        {
            printf("Failed to configure ld2412\n");
            fflush(stdout);
            return false;
        }
        ESP_LOGI(TAG, "Sensor setup done");
        return true;
    }

    static void bind_cb(esp_zb_zdp_status_t zdo_status, void *user_ctx)
    {
        if (zdo_status == ESP_ZB_ZDP_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Bound to coordinator successfully!");
        }
    }

    static void bind_to_coordinator(EpCluster cluster)
    {
        esp_zb_zdo_bind_req_param_t bind_req;
        esp_zb_ieee_address_by_short(/*coordinator*/uint16_t(0), g_CoordinatorIeee);
        esp_zb_get_long_address(bind_req.src_address);
        bind_req.src_endp = cluster.ep;
        bind_req.cluster_id = cluster.cluster_id;
        bind_req.dst_addr_mode = ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED;
        memcpy(bind_req.dst_address_u.addr_long, g_CoordinatorIeee, sizeof(esp_zb_ieee_addr_t));
        bind_req.dst_endp = cluster.ep;//it this right?
        bind_req.req_dst_addr = esp_zb_get_short_address();
        ESP_LOGI(TAG, "Try to bind Occupancy");
        esp_zb_zdo_device_bind_req(&bind_req, bind_cb, nullptr);
    }

    static void check_own_binds()
    {
        esp_zb_zdo_mgmt_bind_param_t cmd_req;
        cmd_req.dst_addr = esp_zb_get_short_address();
        cmd_req.start_index = 0;
        ESP_LOGI(TAG, "Sending request to self to get our current binds");
        esp_zb_zdo_binding_table_req(&cmd_req, 
            [](const esp_zb_zdo_binding_table_info_t *table_info, void *user_ctx)
            {
                ESP_LOGI(TAG, "Got a response with %d entries", table_info->count);
                std::bitset<std::size(g_OwnClusters)> unboundClusters;
                unboundClusters.reset();
                auto *pRec = table_info->record;
                bool coordinatorBindsMissing = true;
                while(pRec)
                {
                    if (pRec->dst_addr_mode == ESP_ZB_ZDO_BIND_DST_ADDR_MODE_64_BIT_EXTENDED)
                    {
                        if (std::memcmp(pRec->dst_address.addr_long, g_CoordinatorIeee, sizeof(esp_zb_ieee_addr_t)) == 0)
                        {
                            for(int i = 0; i < std::size(g_OwnClusters); ++i)
                            {
                                if (g_OwnClusters[i].ep == pRec->src_endp && g_OwnClusters[i].cluster_id == pRec->cluster_id)
                                {
                                    unboundClusters.set(i);
                                    break;
                                }
                            }

                            if (unboundClusters.all())
                            {
                                ESP_LOGI(TAG, "Coordinator is completely bound");
                                coordinatorBindsMissing = false;
                                break;
                            }
                        }
                    }
                    pRec = pRec->next;
                }
                if (coordinatorBindsMissing)
                {
                    ESP_LOGI(TAG, "Coordinator is not fully bound. Binding...");
                    for(int i = 0; i < std::size(g_OwnClusters); ++i)
                    {
                        if (!unboundClusters.test(i))
                        {
                            ESP_LOGI(TAG, "EP %X; Cluster ID: %X;", g_OwnClusters[i].ep, g_OwnClusters[i].cluster_id);
                            bind_to_coordinator(g_OwnClusters[i]);
                        }
                    }
                }
            }
            , nullptr);
    }

    static void create_presence_config_custom_cluster(esp_zb_cluster_list_t *cluster_list)
    {
        //TODO: here
    }

    static void create_presence_ep(esp_zb_ep_list_t *ep_list, uint8_t ep_id)
    {
        static esp_zb_basic_cluster_cfg_t basic_cfg =                                                                                \
            {                                                                                       
                .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,                          
                .power_source = 0x1,//mains                        
            };                                                                                      
        static esp_zb_identify_cluster_cfg_t identify_cfg =                                                                             
            {                                                                                       
                .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,                   
            };                                                                                      
        static esp_zb_occupancy_sensing_cluster_cfg_s presence_cfg =                                                                            
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
        esp_zb_attribute_list_t *pOccupancyAttributes = esp_zb_occupancy_sensing_cluster_create(&presence_cfg);
        uint16_t delay = 10;
        ESP_ERROR_CHECK(esp_zb_occupancy_sensing_cluster_add_attr(pOccupancyAttributes, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_PIR_OCC_TO_UNOCC_DELAY_ID, &delay));
        ESP_ERROR_CHECK(esp_zb_cluster_list_add_occupancy_sensing_cluster(cluster_list, pOccupancyAttributes, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

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

    static void configure_reporting()
    {
        esp_zb_zcl_reset_all_reporting_info();
        /* Config the reporting info  */
        esp_zb_zcl_reporting_info_t reporting_info = {
            .direction = /*ESP_ZB_ZCL_REPORT_DIRECTION_RECV,*/ESP_ZB_ZCL_REPORT_DIRECTION_SEND,
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

        ESP_ERROR_CHECK(esp_zb_zcl_update_reporting_info(&reporting_info));
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

                {
                    uint16_t timeout = g_ld2412.GetTimeout();
                    if (auto status = g_OccupiedToUnoccupiedTimeout.Set(timeout); !status)
                    {
                        FMT_PRINT("Failed to set occupied to unoccupied timeout with error {:x}\n", (int)status.error());
                    }
                }
                ESP_LOGI(TAG, "Deferred sensor initialization %s", !sensor_init_ok ? "failed" : "successful");
                ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
                if (esp_zb_bdb_is_factory_new()) {
                    ESP_LOGI(TAG, "Start network steering");
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                } else {
                    ESP_LOGI(TAG, "Device rebooted");
                    esp_zb_ieee_address_by_short(/*coordinator*/uint16_t(0), g_CoordinatorIeee);
                    check_own_binds();
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

                esp_zb_ieee_address_by_short(/*coordinator*/uint16_t(0), g_CoordinatorIeee);
                check_own_binds();
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

    static SetAttributeHandler g_AttributeHandlers[] = {
        AttrDescr<ZclAttributeOccupiedToUnoccupiedTimeout_t, 
            [](auto const& to, const auto *message)->esp_err_t
            {
                FMT_PRINT("Changing timeout to {}\n", to);
                g_ld2412.ChangeTimeout(to);
                return ESP_OK;
            }
        >{}

        ,{}//last one
    };
    static SetAttributesHandlingDesc g_AttributeHandlingDesc = {
        /*default*/[](const esp_zb_zcl_set_attr_value_message_t *message)->esp_err_t
        {
            esp_err_t ret = ESP_OK;

            ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
            ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                                message->info.status);
            ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d), data type(%d)", message->info.dst_endpoint, message->info.cluster,
                     message->attribute.id, message->attribute.data.size, message->attribute.data.type);
            return ret;
        }
        ,g_AttributeHandlers
    };

    void zigbee_main(void *)
    {
        ESP_LOGI(TAG, "ZB main");
        fflush(stdout);
        {
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
        }
        ESP_LOGI(TAG, "ZB after init");
        fflush(stdout);

        //config clusters here
        esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
        create_presence_ep(ep_list, PRESENCE_EP);
        ESP_LOGI(TAG, "ZB created ep");
        fflush(stdout);
        ep_list->endpoint.rep_info_count = 8;

        /* Register the device */
        esp_zb_device_register(ep_list);
        esp_zb_core_action_handler_register(generic_zb_action_handler<&g_AttributeHandlingDesc>);
        ESP_LOGI(TAG, "ZB registered device");
        fflush(stdout);

        configure_reporting();

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
        
        xTaskCreate(zigbee_main, "Zigbee_main", 2*4096, NULL, 5, NULL);
    }
}

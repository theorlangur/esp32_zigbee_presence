#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "zb_main.hpp"
#include "esp_zigbee_core.h"

#include "../zb_helpers/zbh_helpers.hpp"

#include "../colors_def.hpp"

#include "zb_dev_def.hpp"


namespace zb
{
    /**********************************************************************/
    /* Bind/Unbind tracking                                               */
    /**********************************************************************/
    bool apsde_data_indication_callback(esp_zb_apsde_data_ind_t ind)
    {
        if (ind.dst_short_addr == esp_zb_get_short_address() && ind.status == 0)
        {
            //when I'm the target
            if (ind.asdu_length == sizeof(APSME_BindReq) + 1)
            {
                auto cmd = APSME_Commands(ind.cluster_id);//this will have a command id
                switch(cmd)
                {
                    case APSME_Commands::Bind:
                    case APSME_Commands::Unbind:
                    {
                        APSME_BindReq *pReq = (APSME_BindReq *)(ind.asdu + 1);
                        if (pReq->cluster_id == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF)
                        {
                            FMT_PRINT("Got {} request via APSDE.indication for {}\n", (cmd == APSME_Commands::Bind ? "Bind" : "UnBind"), pReq->dst);
                            //relevant
                            //trigger binds re-check
                            g_State.m_NeedBindsChecking = true;
                        }else
                        {
                            FMT_PRINT("Got {} request via APSDE.indication for {}, cluster {:x}\n", (cmd == APSME_Commands::Bind ? "Bind" : "UnBind"), pReq->dst, (int)pReq->cluster_id);
                        }
                    }
                    break;
                    default:
                        break;
                }
            }
        }
        return false;
    }

    /**********************************************************************/
    /* Common zigbee network handling                                     */
    /**********************************************************************/
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
        static int failed_counter = 0;
        using clock_t = std::chrono::system_clock;
        static auto last_failed_counter_update = clock_t::now();
        auto reset_failure = []{
            failed_counter = 0;
            last_failed_counter_update = clock_t::now();
        };
        auto inc_failure = [](const char *pInfo){
            if (++failed_counter > 6)
            {
                FMT_PRINT("Many Failures on {}\n", pInfo);
                failed_counter = 0;
                auto n = clock_t::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(n - last_failed_counter_update).count() > 60)
                {
                    failed_counter = 1;
                    last_failed_counter_update = n;
                }else
                    esp_restart();
            }else
            {
                FMT_PRINT("Registered Failure on {}\n", pInfo);
                auto n = clock_t::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(n - last_failed_counter_update).count() > 60)
                    failed_counter = 1;
                last_failed_counter_update = n;
            }
        };
        switch (sig_type) {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "Initialize Zigbee stack");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK) {
                reset_failure();
                led::blink(false, {});
                //async setup
                InitHelpers();
                thread::start_task({.pName="LD2412_Setup", .stackSize = 2*4096}, &setup_sensor).detach();
                g_State.RunService();

                ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
                if (esp_zb_bdb_is_factory_new()) {
                    ESP_LOGI(TAG, "Start network steering");
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                } else {
                    ESP_LOGI(TAG, "Device rebooted");
                    esp_zb_ieee_address_by_short(/*coordinator*/uint16_t(0), g_State.m_CoordinatorIeee);
                }
            } else {
                /* commissioning failed */
                inc_failure("commissioning");
                ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
                led::blink_pattern(colors::kBlinkPatternZStackError, colors::kZStackError, duration_ms_t(1000));
                led::blink(true, colors::kSteering);
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;
        case ESP_ZB_ZDO_SIGNAL_LEAVE:
            ESP_LOGW(TAG, "Got leave signal");
            esp_zb_factory_reset();
            esp_restart();
            break;
        case ESP_ZB_NLME_STATUS_INDICATION:
            {
                /*
                 * Copy-paste from some ZBOSS header file for reference
                       Network command status codes
                    typedef enum zb_nwk_command_status_e
                    {
                      ZB_NWK_COMMAND_STATUS_NO_ROUTE_AVAILABLE           = 0x00, /!< No route available /
                      ZB_NWK_COMMAND_STATUS_TREE_LINK_FAILURE            = 0x01, /!< Tree link failure /
                      ZB_NWK_COMMAND_STATUS_NONE_TREE_LINK_FAILURE       = 0x02, /!< None-tree link failure /
                      ZB_NWK_COMMAND_STATUS_LOW_BATTERY_LEVEL            = 0x03, /!< Low battery level /
                      ZB_NWK_COMMAND_STATUS_NO_ROUTING_CAPACITY          = 0x04, /!< No routing capacity /
                      ZB_NWK_COMMAND_STATUS_NO_INDIRECT_CAPACITY         = 0x05, /!< No indirect capacity /
                      ZB_NWK_COMMAND_STATUS_INDIRECT_TRANSACTION_EXPIRY  = 0x06, /!< Indirect transaction expiry /
                      ZB_NWK_COMMAND_STATUS_TARGET_DEVICE_UNAVAILABLE    = 0x07, /!< Target device unavailable /
                      ZB_NWK_COMMAND_STATUS_TARGET_ADDRESS_UNALLOCATED   = 0x08, /!< Target address unallocated /
                      ZB_NWK_COMMAND_STATUS_PARENT_LINK_FAILURE          = 0x09, /!< Parent link failure /
                      ZB_NWK_COMMAND_STATUS_VALIDATE_ROUTE               = 0x0a, /!< Validate route /
                      ZB_NWK_COMMAND_STATUS_SOURCE_ROUTE_FAILURE         = 0x0b, /!< Source route failure /
                      ZB_NWK_COMMAND_STATUS_MANY_TO_ONE_ROUTE_FAILURE    = 0x0c, /!< Many-to-one route failure /
                      ZB_NWK_COMMAND_STATUS_ADDRESS_CONFLICT             = 0x0d, /!< Address conflict /
                      ZB_NWK_COMMAND_STATUS_VERIFY_ADDRESS               = 0x0e, /!< Verify address /
                      ZB_NWK_COMMAND_STATUS_PAN_IDENTIFIER_UPDATE        = 0x0f, /!< Pan identifier update /
                      ZB_NWK_COMMAND_STATUS_NETWORK_ADDRESS_UPDATE       = 0x10, /!< Network address update /
                      ZB_NWK_COMMAND_STATUS_BAD_FRAME_COUNTER            = 0x11, /!< Bad frame counter  /
                      ZB_NWK_COMMAND_STATUS_BAD_KEY_SEQUENCE_NUMBER      = 0x12  /!< Bad key sequence number /
                    }
                    zb_nwk_command_status_t;
                 * */
                void *pParam = esp_zb_app_signal_get_params(p_sg_p);
                g_State.m_Internals.m_LastIndicationStatus = *(uint8_t *)pParam;
                uint16_t addr = *((uint8_t *)pParam + 1) | ((*((uint8_t *)pParam + 2)) << 8);
                ESP_LOGW(TAG, "%s, status: 0x%x; addr: 0x%x\n", esp_zb_zdo_signal_to_string(sig_type), g_State.m_Internals.m_LastIndicationStatus, addr);
            }
            break;
        case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
            ++g_State.m_Internals.m_DeviceUnavailable;
            inc_failure("dev unavailable");
            led::blink_pattern(colors::kBlinkPatternZStackError, colors::kColorBlue, duration_ms_t(1000));
            break;
        case ESP_ZB_NWK_SIGNAL_PERMIT_JOIN_STATUS:
            if (err_status == ESP_OK) {
                if (*(uint8_t *)esp_zb_app_signal_get_params(p_sg_p)) {
                    ESP_LOGI(TAG, "Network(0x%04hx) is open for %d seconds", esp_zb_get_pan_id(), *(uint8_t *)esp_zb_app_signal_get_params(p_sg_p));
                } else {
                    ESP_LOGW(TAG, "Network(0x%04hx) closed, devices joining not allowed.", esp_zb_get_pan_id());
                }
            }
            break;
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK) {
                reset_failure();
                led::blink(false, {});
                esp_zb_ieee_addr_t extended_pan_id;
                esp_zb_get_extended_pan_id(extended_pan_id);
                ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                         extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                         extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                         esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());

                esp_zb_ieee_address_by_short(/*coordinator*/uint16_t(0), g_State.m_CoordinatorIeee);
            } else {
                inc_failure("steering");
                ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
                led::blink_pattern(colors::kBlinkPatternSteeringError, colors::kSteeringError, duration_ms_t(1000));
                led::blink(true, colors::kSteering);
                esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;
        default:
            ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                     esp_err_to_name(err_status));
            break;
        }
    }

    /**********************************************************************/
    /* Registering ZigBee device with clusters and attributes             */
    /**********************************************************************/
    static void create_presence_config_custom_cluster(esp_zb_cluster_list_t *cluster_list)
    {
        esp_zb_attribute_list_t *custom_cluster = esp_zb_zcl_attr_list_create(CLUSTER_ID_LD2412);

        ESP_ERROR_CHECK(g_LD2412MoveSensitivity.AddToCluster(custom_cluster, Access::RW));
        ESP_ERROR_CHECK(g_LD2412StillSensitivity.AddToCluster(custom_cluster, Access::RW));
        ESP_ERROR_CHECK(g_LD2412State.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_LD2412MaxDistance.AddToCluster(custom_cluster, Access::RW));
        ESP_ERROR_CHECK(g_LD2412MinDistance.AddToCluster(custom_cluster, Access::RW));
        ESP_ERROR_CHECK(g_LD2412DistanceRes.AddToCluster(custom_cluster, Access::RW));
        ESP_ERROR_CHECK(g_LD2412ExState.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_LD2412Mode.AddToCluster(custom_cluster, Access::RWP));
        ESP_ERROR_CHECK(g_LD2412PIRPresence.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_LD2412EngineeringLight.AddToCluster(custom_cluster, Access::Report));

        ESP_ERROR_CHECK(g_OnOffCommandMode.AddToCluster(custom_cluster, Access::RW));
        ESP_ERROR_CHECK(g_OnOffCommandTimeout.AddToCluster(custom_cluster, Access::RW));

        ESP_ERROR_CHECK(g_PresenceDetectionIlluminanceThreshold.AddToCluster(custom_cluster, Access::RW));
        ESP_ERROR_CHECK(g_PresenceDetectionConfig.AddToCluster(custom_cluster, Access::RW, g_Config.GetPresenceDetectionMode().m_Raw));
        ESP_ERROR_CHECK(g_ExternalOnTime.AddToCluster(custom_cluster, Access::RW));
        ESP_ERROR_CHECK(g_FailureStatus.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_Internals.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_RestartsCount.AddToCluster(custom_cluster, Access::Read | Access::Report, g_Config.GetRestarts()));
        ESP_ERROR_CHECK(g_Internals2.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_ArmedForTrigger.AddToCluster(custom_cluster, Access::RWP, true));
        ESP_ERROR_CHECK(g_Internals3.AddToCluster(custom_cluster, Access::Read | Access::Report));

#if defined(ENABLE_ENGINEERING_ATTRIBUTES)
        ESP_ERROR_CHECK(g_LD2412MoveDistance.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_LD2412StillDistance.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_LD2412MoveEnergy.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_LD2412StillEnergy.AddToCluster(custom_cluster, Access::Read | Access::Report));
        ESP_ERROR_CHECK(g_LD2412EngineeringEnergyStill.AddToCluster(custom_cluster, Access::Read));
        ESP_ERROR_CHECK(g_LD2412EngineeringEnergyMove.AddToCluster(custom_cluster, Access::Read));
        ESP_ERROR_CHECK(g_LD2412EngineeringEnergyMoveMin.AddToCluster(custom_cluster, Access::Read));
        ESP_ERROR_CHECK(g_LD2412EngineeringEnergyStillMin.AddToCluster(custom_cluster, Access::Read));
        ESP_ERROR_CHECK(g_LD2412EngineeringEnergyMoveMax.AddToCluster(custom_cluster, Access::Read));
        ESP_ERROR_CHECK(g_LD2412EngineeringEnergyStillMax.AddToCluster(custom_cluster, Access::Read));
#endif

        ESP_ERROR_CHECK(esp_zb_cluster_list_add_custom_cluster(cluster_list, custom_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    }

    static void create_presence_ep(esp_zb_ep_list_t *ep_list, uint8_t ep_id)
    {

        /**********************************************************************/
        /* Boilerplate config for standard clusters: basic, identify          */
        /**********************************************************************/
        esp_zb_basic_cluster_cfg_t basic_cfg =                                                                                \
            {                                                                                       
                .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,                          
                .power_source = 0x1,//mains                        
            };                                                                                      
        esp_zb_identify_cluster_cfg_t identify_cfg =                                                                             
            {                                                                                       
                .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE,                   
            };                                                                                      
        esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
        esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&basic_cfg);
        ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, g_Manufacturer));
        ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, g_Model));
        ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_APPLICATION_VERSION_ID, &g_AppVersion));

        ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
        ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&identify_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
        ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

        /**********************************************************************/
        /* Occupancy cluster config (server)                                  */
        /**********************************************************************/
        {
            esp_zb_occupancy_sensing_cluster_cfg_s presence_cfg =                                                                            
            {                                                                                       
                /*uint8_t*/  .occupancy = 0,                                                               /*!<  Bit 0 specifies the sensed occupancy as follows: 1 = occupied, 0 = unoccupied. */
                /*uint32_t*/ .sensor_type = ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_ULTRASONIC, /*!<  The attribute specifies the type of the occupancy sensor */
                /*uint8_t*/  .sensor_type_bitmap = uint8_t(1) << ESP_ZB_ZCL_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE_ULTRASONIC /*!<  The attribute specifies the types of the occupancy sensor */
            };                                                                                      
            esp_zb_attribute_list_t *pOccupancyAttributes = esp_zb_occupancy_sensing_cluster_create(&presence_cfg);
            uint16_t delay = 10;
            ESP_ERROR_CHECK(esp_zb_occupancy_sensing_cluster_add_attr(pOccupancyAttributes, ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_ULTRASONIC_OCCUPIED_TO_UNOCCUPIED_DELAY_ID, &delay));
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_occupancy_sensing_cluster(cluster_list, pOccupancyAttributes, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
        }


        /**********************************************************************/
        /* Client occupancy cluster                                           */
        /**********************************************************************/
        {
            esp_zb_occupancy_sensing_cluster_cfg_s client_occ_cfg{};
            client_occ_cfg.occupancy=0;                                                                            
            esp_zb_attribute_list_t *pOccupancyAttributes = esp_zb_occupancy_sensing_cluster_create(&client_occ_cfg);
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_occupancy_sensing_cluster(cluster_list, pOccupancyAttributes, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
        }

        /**********************************************************************/
        /* Custom cluster                                                     */
        /**********************************************************************/
        create_presence_config_custom_cluster(cluster_list);

        {
            /**********************************************************************/
            /* Client on/off cluster for direct binding purposes                  */
            /**********************************************************************/
            esp_zb_on_off_cluster_cfg_t on_off_cfg{.on_off = false};
            esp_zb_attribute_list_t *on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(cluster_list, on_off_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
        }

        {
            /*************************************************************************/
            /* Server on/off cluster for direct binding purposes for external signal */
            /*************************************************************************/
            esp_zb_on_off_cluster_cfg_t on_off_cfg{.on_off = false};
            esp_zb_attribute_list_t *on_off_cluster = esp_zb_on_off_cluster_create(&on_off_cfg);
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(cluster_list, on_off_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
        }

        /**********************************************************************/
        /* IAS Zone client cluster                                            */
        /**********************************************************************/
        {
            esp_zb_ias_zone_cluster_cfg_t ias_client_cfg{};
            ias_client_cfg.zone_state = 0;
            //ias_client_cfg.zone_ctx.process_result_cb = ias_zone_state_change;
            esp_zb_attribute_list_t *ias_zone_cluster = esp_zb_ias_zone_cluster_create(&ias_client_cfg);
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_ias_zone_cluster(cluster_list, ias_zone_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
        }

        /**********************************************************************/
        /* Illuminance measurement cluster                                    */
        /**********************************************************************/
        {
            esp_zb_illuminance_meas_cluster_cfg_t illum_meas_cluster{};
            esp_zb_attribute_list_t *illum_meas_attrs = esp_zb_illuminance_meas_cluster_create(&illum_meas_cluster);
            ESP_ERROR_CHECK(esp_zb_cluster_list_add_illuminance_meas_cluster(cluster_list, illum_meas_attrs, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
        }

        /**********************************************************************/
        /* Endpoint configuration                                             */
        /**********************************************************************/
        esp_zb_endpoint_config_t endpoint_config = {
            .endpoint = ep_id,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(ep_list, cluster_list, endpoint_config);
    }


    /**********************************************************************/
    /* Zigbee Task Entry Point                                            */
    /**********************************************************************/
    void zigbee_main(void *)
    {
        ESP_LOGI(TAG, "ZB main");
        fflush(stdout);
        esp_zb_set_trace_level_mask(ESP_ZB_TRACE_LEVEL_CRITICAL, 0);
        {
            esp_zb_scheduler_queue_size_set(160);
            esp_zb_cfg_t zb_nwk_cfg = {                                                               
                .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,                       
                .install_code_policy = false,           
                .nwk_cfg = {
                    .zczr_cfg = {.max_children = 10}
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

        /* Register the device */
        esp_zb_device_register(ep_list);
        //here we install our handler for attributes and commands
        esp_zb_core_action_handler_register(
                generic_zb_action_handler<
                    ActionHandler{ESP_ZB_CORE_CMD_CUSTOM_CLUSTER_REQ_CB_ID, cmd_custom_cluster_req_cb<g_CommandsDesc>},
                    ActionHandler{ESP_ZB_CORE_CMD_DEFAULT_RESP_CB_ID, cmd_response_action_handler},
                    ActionHandler{ESP_ZB_CORE_CMD_READ_REPORT_CFG_RESP_CB_ID, generic_node_list_handler<ReadConfigResponseNode>},
                    ActionHandler{ESP_ZB_CORE_CMD_REPORT_CONFIG_RESP_CB_ID, generic_node_list_handler<ConfigReportResponseNode>},
                    ActionHandler{ESP_ZB_CORE_CMD_READ_ATTR_RESP_CB_ID, generic_node_list_handler<ReadAttrResponseNode>},
                    ActionHandler{ESP_ZB_CORE_REPORT_ATTR_CB_ID, report_attr_cb<g_ReportHandlingDesc>},
                    ActionHandler{ESP_ZB_CORE_CMD_IAS_ZONE_ZONE_STATUS_CHANGE_NOT_ID, ias_zone_state_change},
                    ActionHandler{ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID, set_attr_value_cb<g_AttributeHandlingDesc>}
                >
        );

        //esp_zb_ti
        esp_zb_zcl_command_send_status_handler_register(&zb::ZbCmdSend::handler);
        esp_zb_aps_data_indication_handler_register(apsde_data_indication_callback);
        ESP_LOGI(TAG, "ZB registered device");
        fflush(stdout);

        ESP_LOGI(TAG, "ZB updated attribute reporting");
        fflush(stdout);

        esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);
        ESP_LOGI(TAG, "ZB set channel masks");
        fflush(stdout);

        led::blink(true, colors::kSteering);
        ESP_ERROR_CHECK(esp_zb_start(false));
        ESP_LOGI(TAG, "ZB started, looping...");
        esp_zb_stack_main_loop();

        g_Config.on_end();
    }


    void setup()
    {
#ifdef NDEBUG
        //esp_log_level_set(ESP_LOG_NONE); 
#endif

        led::setup();
        led::blink(false, {});

        esp_zb_platform_config_t config = {
            .radio_config = {.radio_mode = ZB_RADIO_MODE_NATIVE, .radio_uart_config = {}},
            .host_config = {.host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE, .host_uart_config = {}},
        };
        ESP_ERROR_CHECK(nvs_flash_init());
        FMT_PRINT("nvs_flash_init done\n");
        ESP_ERROR_CHECK(esp_zb_platform_config(&config));
        FMT_PRINT("esp_zb_platform_config done\n");
        ESP_ERROR_CHECK(g_Config.on_start());
        xTaskCreate(zigbee_main, "Zigbee_main", 2*4096, NULL, thread::kPrioDefault, NULL);

        reset_button_loop();
    }
}

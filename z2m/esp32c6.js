const { Buffer } = require('node:buffer');
const util = require('node:util');
const {Zcl} = require('zigbee-herdsman');
const {enumLookup,numeric,deviceAddCustomCluster,onOff} = require('zigbee-herdsman-converters/lib/modernExtend');
const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const constants = require('zigbee-herdsman-converters/lib/constants');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const ota = require('zigbee-herdsman-converters/lib/ota');
const utils = require('zigbee-herdsman-converters/lib/utils');
const globalStore = require('zigbee-herdsman-converters/lib/store');
const {logger} = require('zigbee-herdsman-converters/lib/logger');
const e = exposes.presets;
const eo = exposes.options;
const ea = exposes.access;

const NS = 'zhc:orlangur';

const orlangurOccupactionExtended = {
    reset_energy_stat: () => {
        const exposes = [
            e.enum('reset_energy_stat', ea.SET, ['Reset']).withLabel('Reset Energy Statistics').withDescription('Perform reset internally gathered energy statistics'),
        ];
        const fromZigbee = [];
        const toZigbee = [
            {
                key: ['reset_energy_stat'],
                convertSet: async (entity, key, value, meta) => { await entity.command('customOccupationConfig', key, {}, {}); },
            },
        ];

        return {
            exposes,
            fromZigbee,
            toZigbee,
            isModernExtend: true,
        };
    },
    commands: () => {
        const exposes = [
            e.enum('restart', ea.SET, ['Restart']).withLabel('Restart').withDescription('Restart LD2412 module'),
            e.enum('factory_reset', ea.SET, ['Reset']).withLabel('Factory Reset').withDescription('Perform factory reset on LD2412 module'),
            e.enum('switch_bluetooth_on', ea.SET, ['On']).withLabel('Switch Bluetooth On').withDescription('Turn Bluetooth On'),
            e.enum('switch_bluetooth_off', ea.SET, ['Off']).withLabel('Switch Bluetooth Off').withDescription('Turn Bluetooth Off'),
            e.enum('recheck_binds', ea.SET, ['Recheck']).withLabel('Re-check Binds').withDescription('Run reporting capability check on existing binds again'),
        ];

        const fromZigbee = [];

        const toZigbee = [
            {
                key: ['restart','factory_reset','reset_energy_stat', 'recheck_binds'],
                convertSet: async (entity, key, value, meta) => {
                    await entity.command('customOccupationConfig', key, {}, {});
                    if (key == 'factory_reset')
                    {
                        await utils.sleep(5000);
                        const endpoint = meta.device.getEndpoint(1);
                        await endpoint.read('msOccupancySensing', ['occupancy','ultrasonicOToUDelay']);
                        await endpoint.read('customOccupationConfig', ['stillSensitivity','moveSensitivity','state']);
                        await endpoint.read('customOccupationConfig', ['min_distance', 'max_distance']);
                    }
                },
            },
            {
                key: ['switch_bluetooth_on', 'switch_bluetooth_off'],
                convertSet: async (entity, key, value, meta) => {
                    const turn_on = value == 'switch_bluetooth_on';
                    await entity.command('customOccupationConfig', 'switch_bluetooth', {on:turn_on}, {});
                },
            },
        ];

        return {
            exposes,
            fromZigbee,
            toZigbee,
            isModernExtend: true,
        };
    },
    mode: () => {
        const exposes = [
            e.enum('presence_mode', ea.ALL, ['Simple', 'Energy']).withLabel("Detection reporting mode").withCategory('config'),
        ];

        const lookup = {Simple: 2, Energy: 1};
        const fromZigbee = [
            {
                cluster: 'customOccupationConfig',
                type: ['attributeReport', 'readResponse'],
                convert: (model, msg, publish, options, meta) => {
                    const result = {};
                    const data = msg.data;
                    if (data['presence_mode'] !== undefined) 
                    {
                        for (const name in lookup) {
                            if (lookup[name] === data['presence_mode'])
                            {
                                result['presence_mode'] = name;
                                break;
                            }
                        }
                    }
                    else 
                    {
                        return;
                    }
                    return result;
                }
            }
        ];

        const toZigbee = [
            {
                key: ['presence_mode'],
                convertSet: async (entity, key, value, meta) => {
                    const payload = {[key]: lookup[value]}
                    const endpoint = meta.device.getEndpoint(1);
                    await entity.write('customOccupationConfig', payload);
                    return {state: {[key]: value}};
                },
                convertGet: async (entity, key, meta) => {
                    await entity.read('customOccupationConfig', [key]);
                },
            }
        ]

        return {
            exposes,
            fromZigbee,
            toZigbee,
            isModernExtend: true,
        };
    },
    onOff: () => {
        const exposes = [
            e.binary('external_on_off', ea.ALL, "ON", "OFF").withLabel("External On/Off state"),
        ];

        const fromZigbee = [
            {
                cluster: 'genOnOff',
                type: ['attributeReport', 'readResponse'],
                convert: (model, msg, publish, options, meta) => {
                    const result = {};
                    const data = msg.data;
                    if (data['onOff'] !== undefined) 
                    {
                        //logger.debug(`on/off report: ${util.inspect(data)};`, NS);
                        result['external_on_off'] = data['onOff'] == 1 ? "ON" : "OFF";
                    }
                    else 
                    {
                        return;
                    }
                    return result;
                }
            }
        ];

        const toZigbee = [
            {
                key: ['external_on_off'],
                convertSet: async (entity, key, value, meta) => {
                    const state = value.toLowerCase();
                    //const payload = {onOff: value == "ON"}
                    //const endpoint = meta.device.getEndpoint(1);
                    //await entity.write('genOnOff', payload);
                    await entity.command('genOnOff', state, {});
                    return {state: {[key]: value}};
                },
                convertGet: async (entity, key, meta) => {
                    await entity.read('genOnOff', ['onOff']);
                },
            }
        ]

        return {
            exposes,
            fromZigbee,
            toZigbee,
            isModernExtend: true,
        };
    },
    presenceModeDetectionConfig: () => {
        const exposes = [
            e.binary('presence_detection_edge_mm_wave'     , ea.ALL, 1, 0).withCategory('config').withDescription('Defines if presence should be detected by mmWave'),
            e.binary('presence_detection_edge_pir_internal', ea.ALL, 1, 0).withCategory('config').withDescription('Defines if presence should be detected by internal PIR'),
            e.binary('presence_detection_edge_external'    , ea.ALL, 1, 0).withCategory('config').withDescription('Defines if presence should be detected by external signal'),
            e.binary('presence_detection_keep_mm_wave'     , ea.ALL, 1, 0).withCategory('config').withDescription('Defines if presence should be kept by mmWave'),
            e.binary('presence_detection_keep_pir_internal', ea.ALL, 1, 0).withCategory('config').withDescription('Defines if presence should be kept by internal PIR'),
            e.binary('presence_detection_keep_external'    , ea.ALL, 1, 0).withCategory('config').withDescription('Defines if presence should be kept by external'),
            e.binary('illuminance_external'                , ea.ALL, 1, 0).withCategory('config').withDescription('Defines if illuminance should be taken from external bound sensor'),
        ];

        const fromZigbee = [
            {
                cluster: 'customOccupationConfig',
                type: ['attributeReport', 'readResponse'],
                convert: (model, msg, publish, options, meta) => {
                    const result = {};
                    const data = msg.data;

                    //edge
                    if (data['presence_detection_edge_mm_wave'] !== undefined) 
                        result['presence_detection_edge_mm_wave'] = data['presence_detection_edge_mm_wave'];
                    if (data['presence_detection_edge_pir_internal'] !== undefined) 
                        result['presence_detection_edge_pir_internal'] = data['presence_detection_edge_pir_internal'];
                    if (data['presence_detection_edge_external'] !== undefined) 
                        result['presence_detection_edge_external'] = data['presence_detection_edge_external'];
                    //keep
                    if (data['presence_detection_keep_mm_wave'] !== undefined) 
                        result['presence_detection_keep_mm_wave'] = data['presence_detection_keep_mm_wave'];
                    if (data['presence_detection_keep_pir_internal'] !== undefined) 
                        result['presence_detection_keep_pir_internal'] = data['presence_detection_keep_pir_internal'];
                    if (data['presence_detection_keep_external'] !== undefined) 
                        result['presence_detection_keep_external'] = data['presence_detection_keep_external'];

                    if (data['illuminance_external'] !== undefined) 
                        result['illuminance_external'] = data['illuminance_external'];

                    if (Object.keys(result).length == 0) 
                        return;

                    return result;
                }
            }
        ];

        const toZigbee = [{
            key: ['illuminance_external', 'presence_detection_edge_mm_wave', 'presence_detection_edge_pir_internal', 'presence_detection_edge_external'
                 ,'presence_detection_keep_mm_wave', 'presence_detection_keep_pir_internal', 'presence_detection_keep_external'],
            convertSet: async (entity, key, value, meta) => {
                const payload = {[key]: value}
                await entity.write('customOccupationConfig', payload);
                return {state: {[key]: value}};
            },
            convertGet: async (entity, key, meta) => {
                await entity.read('customOccupationConfig', [key]);
            },
        }];

        return {
            exposes,
            fromZigbee,
            toZigbee,
            isModernExtend: true,
        };
    },
    distanceConfig: () => {
        const exposes = [
            e.numeric('min_distance', ea.ALL).withLabel("Minimum detection distance").withUnit("m").withValueMin(1).withValueMax(12).withCategory('config'),
            e.numeric('max_distance', ea.ALL).withLabel("Maximum detection distance").withUnit("m").withValueMin(1).withValueMax(12).withCategory('config'),
        ];

        const fromZigbee = [
            {
                cluster: 'customOccupationConfig',
                type: ['attributeReport', 'readResponse'],
                convert: (model, msg, publish, options, meta) => {
                    const result = {};
                    const data = msg.data;
                    //logger.debug(`min/max distance info fZ: ${util.inspect(data)};`, NS);
                    if (data['min_distance'] !== undefined) 
                    {
                        result['min_distance'] = data['min_distance'];
                    }
                    else if (data['max_distance'] !== undefined) 
                    {
                        result['max_distance'] = data['max_distance'];
                    }
                    else 
                    {
                        return;
                    }
                    return result;
                }
            }
        ];

        const toZigbee = [
            {
                key: ['min_distance', 'max_distance'],
                convertSet: async (entity, key, value, meta) => {
                    const payload = {[key]: value}
                    await entity.write('customOccupationConfig', payload);
                    return {state: {[key]: value}};
                },
                convertGet: async (entity, key, meta) => {
                    await entity.read('customOccupationConfig', [key]);
                },
            }
        ]

        return {
            exposes,
            fromZigbee,
            toZigbee,
            isModernExtend: true,
        };
    },
    internals: () => {
        const exposes = [
            //e.composite('internals', 'internals', ea.STATE_GET)
            //.withCategory('diagnostic')
            //    .withFeature(e.numeric('bound_devices', ea.STATE))
            //    .withFeature(e.numeric('cmd_retry_failures', ea.STATE))
            //    .withFeature(e.numeric('cmd_total_failures', ea.STATE))
            //    .withFeature(e.text('configured_reports_for_binds', ea.STATE))
            e.numeric('bound_devices', ea.STATE_GET).withCategory('diagnostic').withUnit('t'),
            e.numeric('cmd_retry_failures', ea.STATE_GET).withCategory('diagnostic').withUnit('t'),
            e.numeric('cmd_total_failures', ea.STATE_GET).withCategory('diagnostic').withUnit('t'),
            e.numeric('devices_unavailable_failures', ea.STATE_GET).withCategory('diagnostic').withUnit('t'),
            e.numeric('last_indication_status', ea.STATE_GET).withCategory('diagnostic'),
            e.text('configured_reports_for_binds', ea.STATE_GET).withCategory('diagnostic')
        ];

        const fromZigbee = [
            {
                cluster: 'customOccupationConfig',
                type: ['attributeReport', 'readResponse'],
                convert: (model, msg, publish, options, meta) => {
                    const result = {};
                    const data = msg.data;
                    //logger.debug(`${attrDistance} + ${attrEnergy} presenceInfo fZ: ${util.inspect(data)};`, NS);
                    if (data['internals'] !== undefined) 
                    {
                        const buffer = Buffer.alloc(4);
                        buffer.writeUInt32LE(data['internals']);
                        const b0 = buffer.readUInt8(0);
                        result['bound_devices'] = b0 & 0x0f;
                        result['devices_unavailable_failures'] = (b0 >> 4) & 0x0f;
                        const b2 = buffer.readUInt8(2);
                        result['cmd_retry_failures'] = b2 & 0x0f;
                        result['cmd_total_failures'] = (b2 >> 4) & 0x0f;
                        const b3 = buffer.readUInt8(3);
                        result['last_indication_status'] = b3;
                        const v = buffer.readUInt8(1);
                        result['configured_reports_for_binds'] = ''
                        for(var i = 0; i < 8; ++i)
                        {
                            const b = v & (1 << i)
                            if (b)
                            {
                                if (result['configured_reports_for_binds'] == '')
                                    result['configured_reports_for_binds'] = "Bind" + i;
                                else
                                    result['configured_reports_for_binds'] += ", Bind" + i;
                            }
                        }
                        if (result['configured_reports_for_binds'] == '')
                            result['configured_reports_for_binds'] = '<no configured reports>'
                    }
                    else 
                    {
                        //logger.debug(`presenceInfo fZ nothing to process`, NS);
                        return;
                    }
                    return result;
                }
            }
        ];

        const toZigbee = [
            {
                key: ['bound_devices', 'configured_reports_for_binds', 'cmd_retry_failures', 'cmd_total_failures', 'last_indication_status', 'devices_unavailable_failures'],
                convertGet: async (entity, key, meta) => {
                    await entity.read('customOccupationConfig', ['internals']);
                },
            }
        ];

        return {
            exposes,
            fromZigbee,
            toZigbee,
            isModernExtend: true,
        };
    },
    presenceInfo: (prefix) => {
        const attrDistance = prefix + 'Distance'
        const attrEnergy = prefix + 'Energy'
        const exposes = [
            e.numeric(attrDistance, ea.STATE).withCategory('diagnostic'),
            e.numeric(attrEnergy, ea.STATE).withCategory('diagnostic'),
        ];

        const fromZigbee = [
            {
                cluster: 'customOccupationConfig',
                type: ['attributeReport', 'readResponse'],
                convert: (model, msg, publish, options, meta) => {
                    const result = {};
                    const data = msg.data;
                    //logger.debug(`${attrDistance} + ${attrEnergy} presenceInfo fZ: ${util.inspect(data)};`, NS);
                    if (data[attrDistance] !== undefined) 
                    {
                        //logger.debug(`${attrDistance} + ${attrEnergy} presenceInfo fZ: got distance: ${data[attrDistance]}`, NS);
                        result[attrDistance] = data[attrDistance];
                    }
                    else if (data[attrEnergy] !== undefined) 
                    {
                        //logger.debug(`${attrDistance} + ${attrEnergy} presenceInfo fZ: got energy: ${data[attrEnergy]}`, NS);
                        result[attrEnergy] = data[attrEnergy];
                    }
                    else 
                    {
                        //logger.debug(`presenceInfo fZ nothing to process`, NS);
                        return;
                    }
                    return result;
                }
            }
        ];

        const toZigbee = [];

        return {
            exposes,
            fromZigbee,
            toZigbee,
            isModernExtend: true,
        };
    },
    measure: (prefix, suffix, descr) => {
        const attr = prefix + '_energy_' + suffix
        const exp_entity = attr

        const exposes = e.text(exp_entity, ea.STATE_GET)
                            .withLabel(prefix + ' energy ' + suffix)
                            .withCategory('diagnostic')
                            .withDescription(descr);

        const fromZigbee = [{
            cluster: 'customOccupationConfig',
            type: ['attributeReport', 'readResponse'],
            convert: (model, msg, publish, options, meta) => {
                const result = {};
                const data = msg.data;
                //logger.debug(`fZ convert attr: ${attr}; data:${util.inspect(data)}`, NS);
                if (attr in data) 
                {
                    const buffer = Buffer.from(data[attr]);
                    if (buffer.length==14)
                    {
                        const toHexString = Array.from(buffer, byte => ('0' + (byte & 0xFF).toString(16)).slice(-2)).join(' ')
                        result[exp_entity] = toHexString
                        return result;
                    }
                }
            }
        }];

        const toZigbee = [
            {
                key: [attr],
                convertGet: async (entity, key, meta) => {
                    await entity.read('customOccupationConfig', [key]);
                },
            }
        ];

        return {
            exposes: [exposes],
            fromZigbee,
            toZigbee,
            isModernExtend: true,
        };
    },
    sensitivity: (prefix, descr) => {
        const attr = prefix + 'Sensitivity'
        const exp_entity = prefix + '_sensitivity'
        const exposes = e.composite(attr, exp_entity, ea.ALL)
                            .withLabel(prefix + ' Sensitivity').withCategory('config')
                            .withDescription('Configure sensitivity for ' + descr);
        for(var i = 0; i < 14; ++i)
            exposes.withFeature(e.numeric('gate'+i, ea.STATE_SET).withValueMin(0).withValueMax(100));

        //logger.debug(`${attr} sensitivity exposes: ${util.inspect(exposes)};`, NS);

        const fromZigbee = [{
                cluster: 'customOccupationConfig',
                type: ['attributeReport', 'readResponse'],
                convert: (model, msg, publish, options, meta) => {
                    const result = {};
                    const data = msg.data;
                    //logger.debug(`${attr} sensitivity fZ: ${util.inspect(msg.data)};`, NS);
                    //{
                    //    const obj = {}; 
                    //    Error.captureStackTrace(obj); 
                    //    logger.debug("called from:\n" + obj.stack); 
                    //}
                    if (attr in data) 
                    {
                        const buffer = Buffer.from(data[attr]);
                        if (buffer.length==14)
                        {
                            res = {}
                            for(var i = 0; i < 14; ++i)
                                res['gate'+i] = buffer[i]
                            result[exp_entity] = res
                            //logger.debug(`${exp_entity} sensitivity fZ: got data=${res}`, NS);
                            //logger.debug(`${exp_entity} fZ: ` + util.inspect(result), NS)
                            return result;
                        }else
                        {
                            //logger.debug(`${exp_entity} sensitivity fZ: buf size: ${buffer.length}`, NS);
                        }
                    }
                    //logger.debug(`${exp_entity} sensitivity fZ: no data`, NS);
                }
            }
        ];

        const toZigbee = [
            {
                key: [exp_entity],
                convertSet: async (entity, key, value, meta) => {
                    //logger.debug(`${exp_entity} sensitivity tZ: got data=` + util.inspect(value), NS);
                    const payloadValue = [];
                    for(var i = 0; i < 14; ++i)
                    {
                        payloadValue[i] = value['gate'+i]
                        //logger.debug(`${exp_entity} sensitivity tZ: gate${i}=${payloadValue[i]}`, NS);
                    }
                    const payload = {[attr]: payloadValue}
                    await entity.write('customOccupationConfig', payload);
                    return {state: {[key]: value}};
                },
                convertGet: async (entity, key, meta) => {
                    //logger.debug(`${exp_entity} sensitivity tZ: get key=${key}`, NS);
                    await entity.read('customOccupationConfig', [attr]);
                },
            },
        ];

        //logger.debug("tZ: " + util.inspect(toZigbee), NS)
        //logger.debug("fZ: " + util.inspect(fromZigbee), NS)


        return {
            exposes: [exposes],
            fromZigbee,
            toZigbee,
            isModernExtend: true,
        };
    }
}

const definition = {
    zigbeeModel: ['P-NextGen'],
    model: 'P-NextGen',
    fingerprint: [{modelID: 'P-NextGen', applicationVersion: 1, priority: -1},],
    vendor: 'Orlangur',
    description: 'ESP32C6 Occupancy Test',
    fromZigbee: [fz.occupancy],
    toZigbee: [],
    exposes: [e.occupancy()],
    extend: [
        deviceAddCustomCluster('customOccupationConfig', {
            ID: 0xfc00,
            attributes: {
                moveSensitivity: {ID: 0x0000, type: Zcl.DataType.OCTET_STR},
                stillSensitivity: {ID: 0x0001, type: Zcl.DataType.OCTET_STR},
                moveEnergy: {ID: 0x0002, type: Zcl.DataType.UINT8},
                stillEnergy: {ID: 0x0003, type: Zcl.DataType.UINT8},
                moveDistance: {ID: 0x0004, type: Zcl.DataType.UINT16},
                stillDistance: {ID: 0x0005, type: Zcl.DataType.UINT16},
                state: {ID: 0x0006, type: Zcl.DataType.ENUM8},
                min_distance: {ID: 0x0007, type: Zcl.DataType.UINT16},
                max_distance: {ID: 0x0008, type: Zcl.DataType.UINT16},
                ex_state: {ID: 0x0009, type: Zcl.DataType.ENUM8},
                presence_mode: {ID: 0x000a, type: Zcl.DataType.ENUM8},
                measured_light: {ID: 0x000b, type: Zcl.DataType.UINT8},
                move_energy_last: {ID: 0x000c, type: Zcl.DataType.OCTET_STR},
                still_energy_last: {ID: 0x000d, type: Zcl.DataType.OCTET_STR},
                move_energy_min: {ID: 0x000e, type: Zcl.DataType.OCTET_STR},
                still_energy_min: {ID: 0x000f, type: Zcl.DataType.OCTET_STR},
                move_energy_max: {ID: 0x0010, type: Zcl.DataType.OCTET_STR},
                still_energy_max: {ID: 0x0011, type: Zcl.DataType.OCTET_STR},
                pir_presence: {ID: 0x0012, type: Zcl.DataType.BOOLEAN},
                on_off_mode: {ID: 0x0013, type: Zcl.DataType.ENUM8},
                on_off_timeout: {ID: 0x0014, type: Zcl.DataType.UINT16},
                illuminance_threshold: {ID: 0x0015, type: Zcl.DataType.UINT8},
                presence_detection_edge_mm_wave: {ID: 0x0016, type: Zcl.DataType.BOOLEAN},
                presence_detection_edge_pir_internal: {ID: 0x0017, type: Zcl.DataType.BOOLEAN},
                presence_detection_edge_external: {ID: 0x0018, type: Zcl.DataType.BOOLEAN},
                presence_detection_keep_mm_wave: {ID: 0x0019, type: Zcl.DataType.BOOLEAN},
                presence_detection_keep_pir_internal: {ID: 0x001a, type: Zcl.DataType.BOOLEAN},
                presence_detection_keep_external: {ID: 0x001b, type: Zcl.DataType.BOOLEAN},
                external_on_time: {ID:0x001c, type: Zcl.DataType.UINT16},
                failure_status: {ID:0x001d, type: Zcl.DataType.UINT16},
                internals: {ID:0x001e, type: Zcl.DataType.UINT32},
                restarts_count: {ID:0x001f, type: Zcl.DataType.UINT16},
                illuminance_external: {ID:0x0020, type: Zcl.DataType.BOOLEAN},
            },
            commands: {
                restart: {
                    ID: 0x0000,
                    parameters: [],
                },
                factory_reset: {
                    ID: 0x0001,
                    parameters: [],
                },
                reset_energy_stat: {
                    ID: 0x0002,
                    parameters: [],
                },
                switch_bluetooth: {
                    ID: 0x0003,
                    parameters: [{name: 'on', type: Zcl.DataType.BOOLEAN}],
                },
                recheck_binds: {
                    ID: 0x0004,
                    parameters: [],
                },
            },
            commandsResponse: {},
        }),
        orlangurOccupactionExtended.commands(),
        numeric({
            name: 'presence_timeout',
            cluster: 'msOccupancySensing',
            attribute: 'ultrasonicOToUDelay',
            description: 'Occupied to unoccupied delay',
            valueMin: 2,
            valueMax: 120,
            access: 'ALL',
            entityCategory: 'config',
        }),
        enumLookup({
            name: 'presence_state',
            access: 'STATE_GET',
            cluster: 'customOccupationConfig',
            attribute: 'state',
            description: 'Presence state',
            lookup: {Clear: 0, Move: 1, Still: 2, MoveStill: 3, Configuring: 0x80, Failed: 0x81},
            entityCategory: 'diagnostic',
        }),
        orlangurOccupactionExtended.onOff(),
        numeric({
            name: 'external_on_time',
            cluster: 'customOccupationConfig',
            attribute: 'external_on_time',
            description: 'On Time for external signal',
            valueMin: 0,
            valueMax: 120,
            access: 'ALL',
            entityCategory: 'config',
        }),
        enumLookup({
            name: 'pir_presence',
            access: 'STATE_GET',
            cluster: 'customOccupationConfig',
            attribute: 'pir_presence',
            description: 'PIR Presence',
            lookup: {Clear: 0, Detected: 1},
            entityCategory: 'diagnostic',
        }),
        enumLookup({
            name: 'extended_state',
            access: 'STATE_GET',
            cluster: 'customOccupationConfig',
            attribute: 'ex_state',
            description: 'Extended state',
            lookup: {Normal: 0, DynamicBackgroundAnalysis: 1, Calibration: 2},
        }),
        orlangurOccupactionExtended.mode(),
        enumLookup({
            name: 'on_off_mode',
            access: 'ALL',
            cluster: 'customOccupationConfig',
            attribute: 'on_off_mode',
            description: 'On/Off Command Mode',
            lookup: {OnOff: 0, OnOnly: 1, OffOnly: 2, TimedOn: 3, TimedOnLocal: 4, Nothing: 5},
            entityCategory: 'config',
        }),
        numeric({
            name: 'on_off_timeout',
            cluster: 'customOccupationConfig',
            attribute: 'on_off_timeout',
            description: 'On/Off Timeout',
            valueMin: 0,
            valueMax: 1000,
            access: 'ALL',
            entityCategory: 'config',
        }),
        numeric({
            name: 'measured_light',
            cluster: 'customOccupationConfig',
            attribute: 'measured_light',
            description: 'Measured light level',
            valueMin: 0,
            valueMax: 255,
            access: 'STATE',
            unit: 'lx',
            entityCategory: 'diagnostic',
        }),
        numeric({
            name: 'illuminance_threshold',
            cluster: 'customOccupationConfig',
            attribute: 'illuminance_threshold',
            description: 'Illuminance level below which presence is detected',
            valueMin: 0,
            valueMax: 255,
            access: 'ALL',
            entityCategory: 'config',
        }),
        numeric({
            name: 'restarts_count',
            cluster: 'customOccupationConfig',
            attribute: 'restarts_count',
            description: 'Amount of restarts',
            valueMin: 0,
            valueMax: 65535,
            access: 'STATE',
            entityCategory: 'diagnostic',
            unit: 'times',
        }),
        enumLookup({
            name: 'failure_status',
            access: 'STATE',
            cluster: 'customOccupationConfig',
            attribute: 'failure_status',
            description: 'Last Cmd failed status',
            lookup: {
                    Ok: 0, Fail: 1, UnAuth: 0x07e, MalformedCmd: 0x080, UnsClustCmd: 0x081, UnsGenCmd: 0x082,
                    UnsManClustCmd: 0x083, UnsManGenCmd: 0x084, InvField: 0x085, UnsAttr: 0x086, InvVal: 0x087, ReadOnly: 0x088,
                    InsuffSpace: 0x089, DupeExists: 0x08a, NotFound: 0x08b, UnrepAttr: 0x08c, InvType: 0x08d, WriteOnly: 0x08f,
                    Inconsistent: 0x092, ActDenied: 0x093, Timeout: 0x094, Abort: 0x095, InvImage: 0x096, WaitForData: 0x097,
                    NoImg: 0x098, ReqMoreImg: 0x099, NotifPend: 0x09a, HwFail: 0x0c0, SwFail: 0x0c1, CalibErr: 0x0c2,
                    UnsClust: 0x0c3, Limit: 0x0c4
                },
            entityCategory: 'diagnostics',
        }),
        orlangurOccupactionExtended.presenceModeDetectionConfig(),
        orlangurOccupactionExtended.presenceInfo('move'),
        orlangurOccupactionExtended.presenceInfo('still'),
        orlangurOccupactionExtended.distanceConfig(),
        orlangurOccupactionExtended.sensitivity('move', 'Move Sensitivity'),
        orlangurOccupactionExtended.sensitivity('still', 'Still Sensitivity'),
        orlangurOccupactionExtended.reset_energy_stat(),
        orlangurOccupactionExtended.measure('still', 'last', 'Last measured still energy per gate'),
        orlangurOccupactionExtended.measure('still', 'min', 'Min measured still energy per gate'),
        orlangurOccupactionExtended.measure('still', 'max', 'Max measured still energy per gate'),
        orlangurOccupactionExtended.measure('move', 'last', 'Last measured move energy per gate'),
        orlangurOccupactionExtended.measure('move', 'min', 'Min measured move energy per gate'),
        orlangurOccupactionExtended.measure('move', 'max', 'Max measured move energy per gate'),
        orlangurOccupactionExtended.internals(),
    ],
    configure: async (device, coordinatorEndpoint) => {
        const endpoint = device.getEndpoint(1);
        await reporting.bind(endpoint, coordinatorEndpoint, ['msOccupancySensing', 'customOccupationConfig']);
        await endpoint.read('customOccupationConfig', ['presence_mode','pir_presence']);
        await endpoint.read('msOccupancySensing', ['occupancy']);
        await endpoint.read('msOccupancySensing', ['occupancy','ultrasonicOToUDelay']);
        await endpoint.read('customOccupationConfig', ['min_distance', 'max_distance', 'illuminance_threshold', 'on_off_timeout', 'on_off_mode']);
        await endpoint.read('customOccupationConfig', ['stillSensitivity','moveSensitivity','state', 'failure_status', 'internals']);
        //await endpoint.read('customOccupationConfig', ['moveDistance','stillDistance','moveEnergy','stillEnergy']);
        //await endpoint.read('customOccupationConfig', ['still_energy_last','still_energy_min','still_energy_max']);
        //await endpoint.read('customOccupationConfig', ['move_energy_last','move_energy_min','move_energy_max']);
        await endpoint.read('customOccupationConfig', [
                                                         'presence_detection_edge_mm_wave'
                                                        ,'presence_detection_edge_pir_internal'
                                                        ,'presence_detection_edge_external'
                                                        ,'presence_detection_keep_mm_wave'
                                                        ,'presence_detection_keep_pir_internal'
                                                        ,'presence_detection_keep_external'
                                                    ]);
        await endpoint.configureReporting('msOccupancySensing', [
            {
                attribute: 'occupancy',
                minimumReportInterval: 0,
                maximumReportInterval: constants.repInterval.HOUR,
                reportableChange: null,
            },
        ]);
        await endpoint.configureReporting('genOnOff', [
            {
                attribute: 'onOff',
                minimumReportInterval: 0,
                maximumReportInterval: constants.repInterval.HOUR,
                reportableChange: 1,
            },
        ]);
        await endpoint.configureReporting('customOccupationConfig', [
            //{//skip for final version
            //    attribute: 'moveDistance',
            //    minimumReportInterval: 1,
            //    maximumReportInterval: constants.repInterval.HOUR,
            //    reportableChange: null,
            //},
            //{//skip for final version
            //    attribute: 'stillDistance',
            //    minimumReportInterval: 5,
            //    maximumReportInterval: constants.repInterval.HOUR,
            //    reportableChange: null,
            //},
            //{//skip for final version
            //    attribute: 'moveEnergy',
            //    minimumReportInterval: 1,
            //    maximumReportInterval: constants.repInterval.HOUR,
            //    reportableChange: null,
            //},
            //{//skip for final version
            //    attribute: 'stillEnergy',
            //    minimumReportInterval: 5,
            //    maximumReportInterval: constants.repInterval.HOUR,
            //    reportableChange: null,
            //},
            {
                attribute: 'measured_light',
                minimumReportInterval: 30,
                maximumReportInterval: constants.repInterval.HOUR,
                reportableChange: 10,
            },
            {
                attribute: 'ex_state',
                minimumReportInterval: 0,
                maximumReportInterval: constants.repInterval.HOUR,
                reportableChange: null,
            },
            {
                attribute: 'presence_mode',
                minimumReportInterval: 0,
                maximumReportInterval: constants.repInterval.HOUR,
                reportableChange: null,
            },
            {
                attribute: 'pir_presence',
                minimumReportInterval: 0,
                maximumReportInterval: constants.repInterval.HOUR,
                reportableChange: null,
            },
            {
                attribute: 'failure_status',
                minimumReportInterval: 0,
                maximumReportInterval: constants.repInterval.HOUR,
                reportableChange: null,
            },
            {
                attribute: 'internals',
                minimumReportInterval: 0,
                maximumReportInterval: constants.repInterval.HOUR,
                reportableChange: null,
            },
        ]);

        await endpoint.read('customOccupationConfig', ['restarts_count']);
    },

};

module.exports = definition;

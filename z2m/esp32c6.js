const { Buffer } = require('node:buffer');
const util = require('node:util');
const {Zcl} = require('zigbee-herdsman');
const {enumLookup,numeric,deviceAddCustomCluster} = require('zigbee-herdsman-converters/lib/modernExtend');
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
    commands: () => {
        const exposes = [
            e.enum('restart', ea.SET, ['Restart']).withLabel('Restart').withDescription('Restart LD2412 module'),
            e.enum('factory_reset', ea.SET, ['Reset']).withLabel('Factory Reset').withDescription('Perform factory reset on LD2412 module'),
        ];

        const fromZigbee = [];

        const toZigbee = [
            {
                key: ['restart','factory_reset'],
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
        ];

        return {
            exposes,
            fromZigbee,
            toZigbee,
            isModernExtend: true,
        };
    },
    distanceConfig: () => {
        const exposes = [
            e.numeric('min_distance', ea.ALL).withLabel("Minimum detection distance").withUnit("m").withValueMin(1).withValueMax(12),
            e.numeric('max_distance', ea.ALL).withLabel("Minimum detection distance").withUnit("m").withValueMin(1).withValueMax(12),
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
    presenceInfo: (prefix) => {
        const attrDistance = prefix + 'Distance'
        const attrEnergy = prefix + 'Energy'
        const exposes = [
            e.numeric(attrDistance, ea.STATE),
            e.numeric(attrEnergy, ea.STATE),
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
    sensitivity: (prefix, descr) => {
        const attr = prefix + 'Sensitivity'
        const exp_entity = prefix + '_sensitivity'
        const exposes = e.composite(attr, exp_entity, ea.ALL)
                            .withLabel(prefix + ' Sensitivity')
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
        }),
        enumLookup({
            name: 'presence_state',
            access: 'STATE',
            cluster: 'customOccupationConfig',
            attribute: 'state',
            description: 'Presence state',
            lookup: {Clear: 0, Move: 1, Still: 2, MoveStill: 3},
        }),
        orlangurOccupactionExtended.presenceInfo('move'),
        orlangurOccupactionExtended.presenceInfo('still'),
        orlangurOccupactionExtended.distanceConfig(),
        orlangurOccupactionExtended.sensitivity('move', 'Move Sensitivity'),
        orlangurOccupactionExtended.sensitivity('still', 'Still Sensitivity'),
    ],
    configure: async (device, coordinatorEndpoint) => {
        const endpoint = device.getEndpoint(1);
        await reporting.bind(endpoint, coordinatorEndpoint, ['msOccupancySensing', 'customOccupationConfig']);
        await endpoint.read('msOccupancySensing', ['occupancy']);
        await endpoint.read('msOccupancySensing', ['occupancy','ultrasonicOToUDelay']);
        await endpoint.read('customOccupationConfig', ['min_distance', 'max_distance']);
        await endpoint.read('customOccupationConfig', ['stillSensitivity','moveSensitivity','state']);
        await endpoint.read('customOccupationConfig', ['moveDistance','stillDistance','moveEnergy','stillEnergy']);
        await endpoint.configureReporting('msOccupancySensing', [
            {
                attribute: 'occupancy',
                minimumReportInterval: 0,
                maximumReportInterval: constants.repInterval.HOUR,
                reportableChange: null,
            },
        ]);
        await endpoint.configureReporting('customOccupationConfig', [
            {
                attribute: 'moveDistance',
                minimumReportInterval: 1,
                maximumReportInterval: constants.repInterval.HOUR,
                reportableChange: null,
            },
        ]);
        await endpoint.configureReporting('customOccupationConfig', [
            {
                attribute: 'stillDistance',
                minimumReportInterval: 5,
                maximumReportInterval: constants.repInterval.HOUR,
                reportableChange: null,
            },
        ]);
    },

};

module.exports = definition;

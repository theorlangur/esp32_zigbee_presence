const { Buffer } = require('node:buffer');
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
                    logger.debug(`${attrDistance} + ${attrEnergy} presenceInfo fZ: ${data}`, NS);
                    if (data[attrDistance] !== undefined) 
                    {
                        logger.debug(`${attrDistance} + ${attrEnergy} presenceInfo fZ: got distance: ${data[attrDistance]}`, NS);
                        result[attrDistance] = data[attrDistance];
                    }
                    else if (data[attrEnergy] !== undefined) 
                    {
                        logger.debug(`${attrDistance} + ${attrEnergy} presenceInfo fZ: got energy: ${data[attrEnergy]}`, NS);
                        result[attrEnergy] = data[attrEnergy];
                    }
                    else 
                    {
                        logger.debug(`presenceInfo fZ nothing to process`, NS);
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
        const exposes = e.composite(attr, attr, ea.STATE_SET)
                            .withLabel(prefix + ' Sensitivity')
                            .withDescription('Configure sensitivity for ' + descr);
        for(var i = 0; i < 14; ++i)
            exposes.withFeature(e.numeric('gate'+i, ea.STATE_SET).withValueMin(0).withValueMax(100));

        const fromZigbee = [{
                cluster: 'customOccupationConfig',
                type: ['attributeReport', 'readResponse'],
                convert: (model, msg, publish, options, meta) => {
                    const result = {};
                    const data = msg.data;
                    logger.debug(`${attr} sensitivity fZ: ${data};`, NS);
                    if (attr in data) 
                    {
                        const buffer = Buffer.from(data[attr]);
                        if (buffer.length==14)
                        {
                            res = {}
                            for(var i = 0; i < 14; ++i)
                                res['gate'+i] = buffer[i]
                            result[attr] = res
                            logger.debug(`${attr} sensitivity fZ: got data=${res}`, NS);
                            return result;
                        }else
                        {
                            logger.debug(`${attr} sensitivity fZ: buf size: ${buffer.length}`, NS);
                        }
                    }
                    logger.debug(`${attr} sensitivity fZ: no data`, NS);
                }
            }
        ];

        const toZigbee = [
            {
                key: [attr],
                convertSet: async (entity, key, value, meta) => {
                    const payloadValue = [];
                    payloadValue[0] = 14;
                    for(var i = 0; i < 14; ++i)
                    {
                        payloadValue[1 + i] = value['gate'+i]
                    }
                    const payload = {[attr]: {value: payloadValue, type: Zcl.DataType.OCTET_STR}};
                    await entity.write('customOccupationConfig', payload);
                    return {state: {[key]: value}};
                },
                convertGet: async (entity, key, meta) => {
                    await entity.read('customOccupationConfig', [attr]);
                },
            },
        ];


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
            },
            commands: {},
            commandsResponse: {},
        }),
        numeric({
            name: 'presence_timeout',
            cluster: 0x0406,
            attribute: {ID: 'ultrasonicOToUDelay', type: 0x21},
            description: 'Occupied to unoccupied delay',
            valueMin: 2,
            valueMax: 120,
        }),
        enumLookup({
            name: 'presence_state',
            cluster: 'customOccupationConfig',
            attribute: 'state',
            description: 'Presence state',
            lookup: {Clear: 0, Move: 1, Still: 2, MoveStill: 3},
        }),
        orlangurOccupactionExtended.presenceInfo('move'),
        orlangurOccupactionExtended.presenceInfo('still'),
        orlangurOccupactionExtended.sensitivity('move', 'Move Sensitivity'),
        orlangurOccupactionExtended.sensitivity('still', 'Still Sensitivity'),
    ],
    configure: async (device, coordinatorEndpoint) => {
        const endpoint = device.getEndpoint(1);
        await reporting.bind(endpoint, coordinatorEndpoint, ['msOccupancySensing', 'customOccupationConfig']);
        await endpoint.read('msOccupancySensing', ['occupancy','ultrasonicOToUDelay']);
        await endpoint.read('customOccupationConfig', ['stillSensitivity','moveSensitivity']);
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

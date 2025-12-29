# Neva MT124 Meter Bridge for Home Assistant

This is a Home Assistant add-on that reads data from NEVA MT124 electric meters via serial port and publishes it to MQTT for integration with Home Assistant.

## Features

- Reads energy consumption data (total and tariff-based) from NEVA MT124 meters
- Retrieves current power, voltage, and current measurements (for compatible models)
- Publishes data to MQTT with Home Assistant Discovery support
- Automatic meter type detection (6102 and 7100 series)
- Configurable polling intervals and MQTT settings

## Supported Meter Models

- NEVA MT113.2106 (6102-compatible)
- NEVA MT124.7107 (7109-compatible)

## Installation

1. Copy the `neva_mt124_bridge` folder to your Home Assistant addons directory
2. Restart Home Assistant
3. Install the addon through the Supervisor → Add-ons → Neva MT124 Meter Bridge
4. Configure the addon settings
5. Start the addon

## Configuration

### Serial Settings
- **Serial Port**: Device path (e.g., `/dev/ttyUSB0`)
- **Initial Baudrate**: Usually 300 (meter identification)
- **Main Baudrate**: Usually 9600 (data polling)

### MQTT Settings
- **MQTT Host**: MQTT broker hostname or IP
- **MQTT Port**: MQTT broker port (default 1883)
- **MQTT User**: Username for MQTT authentication
- **MQTT Password**: Password for MQTT authentication
- **MQTT Topic Prefix**: Base topic for publishing data (e.g., `home/meter`)

### Polling Settings
- **Interval Seconds**: Polling interval in seconds
- **Timezone**: Timezone for logs (e.g., `Europe/Moscow`)

## MQTT Topics

The addon publishes data to the following MQTT topics (prefix configurable):

- `{prefix}/total_energy`: Total energy consumption (kWh)
- `{prefix}/tariff1`: Tariff 1 energy (kWh)
- `{prefix}/tariff2`: Tariff 2 energy (kWh)
- `{prefix}/tariff3`: Tariff 3 energy (kWh)
- `{prefix}/tariff4`: Tariff 4 energy (kWh)
- `{prefix}/power`: Current power consumption (W)
- `{prefix}/voltage`: Voltage (V) - for 6102-compatible models
- `{prefix}/current`: Current (A) - for 6102-compatible models
- `{prefix}/battery`: Battery level (%) - for 6102-compatible models
- `{prefix}/serial`: Meter serial number

Home Assistant Discovery topics are also published automatically.

## Troubleshooting

- **Meter not responding**: Check serial port permissions and connection
- **Incomplete frames**: Meter buffer issues, usually recovers automatically
- **No MQTT messages**: Verify broker connection and credentials
- **Wrong data**: Check if meter type is supported

## Acknowledgments

This project was developed based on the open-source repository: [slacky's electricity_meter_zrd](https://github.com/slacky1965/electricity_meter_zrd)

## License

MIT License

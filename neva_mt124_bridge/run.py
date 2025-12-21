import serial
import paho.mqtt.client as mqtt
import time
import json
import os
import sys
import logging

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')
logging.debug("Script started - debugging mode")

# Константы из C-кода
ACK = 0x06
SOH = 0x01
STX = 0x02
ETX = 0x03

BAUDRATE_300 = 300
BAUDRATE_9600 = 9600

MAX_VBAT_MV = 3100
MIN_VBAT_MV = 2200  # Предполагаю BATTERY_SAFETY_THRESHOLD = 2200

PKT_BUFF_MAX_LEN = 256  # Примерный размер

# Команды (как в command_array)
COMMANDS = {
    'open_channel': b'/?!\r\n',
    'ack_start': bytes([ACK, 0x30, 0x35, 0x31, 0x0D, 0x0A]),
    'password_6102': bytes([SOH, 0x50, 0x31, STX, 0x28, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x29, ETX, 0x61]),
    'password_7109': bytes([SOH, 0x50, 0x31, STX, 0x28, 0x29, ETX, 0x61]),
    'serial_number': bytes([SOH, 0x52, 0x31, STX, 0x36, 0x30, 0x30, 0x31, 0x30, 0x30, 0x46, 0x46, 0x28, 0x29, ETX, 0x64]),
    'sensors_data': bytes([SOH, 0x52, 0x31, STX, 0x36, 0x30, 0x30, 0x35, 0x30, 0x30, 0x46, 0x46, 0x28, 0x29, ETX, 0x60]),
    'tariffs_6102': bytes([SOH, 0x52, 0x31, STX, 0x30, 0x46, 0x30, 0x38, 0x38, 0x30, 0x46, 0x46, 0x28, 0x29, ETX, 0x15]),
    'tariffs_7109': bytes([SOH, 0x52, 0x31, STX, 0x30, 0x46, 0x30, 0x38, 0x38, 0x30, 0x46, 0x46, 0x28, 0x53, 0x29, ETX, 0x46]),
    'power_data': bytes([SOH, 0x52, 0x31, STX, 0x31, 0x30, 0x30, 0x37, 0x30, 0x30, 0x46, 0x46, 0x28, 0x29, ETX, 0x65]),
    'volts_data': bytes([SOH, 0x52, 0x31, STX, 0x30, 0x43, 0x30, 0x37, 0x30, 0x30, 0x46, 0x46, 0x28, 0x29, ETX, 0x17]),
    'amps_data': bytes([SOH, 0x52, 0x31, STX, 0x30, 0x42, 0x30, 0x37, 0x30, 0x30, 0x46, 0x46, 0x28, 0x29, ETX, 0x16]),
    'close_channel': bytes([SOH, 0x42, 0x30, ETX, 0x71]),
}

# Типы счётчика
NEVA_124_UNKNOWN = 0
NEVA_124_6102 = 1
NEVA_124_7109 = 2

# Функции из C (адаптированные)
def checksum(data):
    crc = 0
    for byte in data[1:-1]:  # len-1 в C, но здесь слайс
        crc ^= byte
    return crc & 0x7f

def check_even_parity(ch):
    ch ^= ch >> 4
    ch ^= ch >> 2
    ch ^= ch >> 1
    return ch & 1

def str2uint(s):
    num = 0
    for char in s:
        if '0' <= char <= '9':
            num = num * 10 + (ord(char) - ord('0'))
        else:
            break
    return num

def number_from_brackets(p_str):
    global divisor, multiplier
    divisor = 1
    multiplier = 1
    init_bracket = p_str.find(b'(')
    if init_bracket == -1:
        return None
    init_bracket += 1
    end_bracket = p_str.find(b')', init_bracket)
    if end_bracket == -1:
        return None
    value_str = p_str[init_bracket:end_bracket].decode(errors='ignore')
    point = value_str.find('.')
    if point != -1:
        integer = str2uint(value_str[:point])
        remainder = value_str[point+1:]
        rmndr_len = min(len(remainder), 4)
        rem_val = str2uint(remainder[:rmndr_len])
        for _ in range(rmndr_len):
            integer *= 10
            divisor *= 10
        value = integer + rem_val
    else:
        value = str2uint(value_str)
    return value

def number_from_tariffs(p_str):
    # Аналогично, но для тарифов с ','
    global divisor, multiplier
    divisor = 1
    multiplier = 1
    end = p_str.find(b',') if p_str.find(b',') != -1 else p_str.find(b')')
    if end == -1:
        return None
    value_str = p_str[:end].decode(errors='ignore')
    point = value_str.find('.')
    if point != -1:
        integer = str2uint(value_str[:point])
        remainder = value_str[point+1:]
        rmndr_len = min(len(remainder), 4)
        rem_val = str2uint(remainder[:rmndr_len])
        for _ in range(rmndr_len):
            integer *= 10
            divisor *= 10
        value = integer + rem_val
    else:
        value = str2uint(value_str)
    return value, end + 1  # Возвращаем значение и сдвиг

def str_from_brackets(p_str):
    init_bracket = p_str.find(b'(')
    if init_bracket == -1:
        return None
    init_bracket += 1
    end_bracket = p_str.find(b')', init_bracket)
    if end_bracket == -1:
        return None
    return p_str[init_bracket:end_bracket].decode(errors='ignore')

def send_command(ser, cmd_key):
    cmd = COMMANDS[cmd_key]
    parity_cmd = bytearray()
    for byte in cmd:
        if check_even_parity(byte):
            parity_cmd.append(byte | 0x80)
        else:
            parity_cmd.append(byte)
    ser.write(parity_cmd)
    time.sleep(0.2)
    logging.debug(f"Sent {cmd_key}: {parity_cmd.hex()}")
    return len(parity_cmd)

def response_meter(ser, cmd_key, timeout=1):
    start = time.time()
    data = bytearray()
    while time.time() - start < timeout:
        if ser.in_waiting:
            data.extend(ser.read(ser.in_waiting))
            data = bytearray(b & 0x7f for b in data)  # Маска parity
            if len(data) > 0:
                break
        time.sleep(0.01)
    if not data:
        return None, "Timeout"
    # Проверка CRC и формата (адаптировано из C)
    if cmd_key == 'open_channel':
        if data[0] != ord('/'):
            logging.debug(f"Raw data: {data.hex()}")
            return None, "Invalid response"
    elif cmd_key == 'password_6102':
        if data[0] != ACK:
            logging.debug(f"Raw data: {data.hex()}")
            return None, "Invalid response"
    else:
        crc = checksum(data)
        if crc != data[-1]:
            return None, "CRC error"
        # Дополнительные проверки по cmd
    return data, "OK"

# Основные функции get_*
def open_session(ser):
    send_command(ser, 'open_channel')
    data, err = response_meter(ser, 'open_channel')
    logging.debug("Response: %s, error: %s", data, err)
    
    if err != "OK":
        return NEVA_124_UNKNOWN
    
    time.sleep(0.1)
    
    # Если ответ равен '/', считаем что соединение установлено
    if data == bytearray(b'/'):
        logging.debug("Received '/', connection established")
        # ВАЖНО: Мы не знаем точный тип, но можно предположить MT124
        # Или вернуть NEVA_124_6102 по умолчанию для продолжения работы
        return NEVA_124_6102  # Или NEVA_124_7109, в зависимости от вашего счётчика
    
    # Старая логика парсинга (оставляем на случай расширенных ответов)
    if len(data) >= 5:
        dot_pos = data.find(b'.')
        if dot_pos != -1:
            type_str = data[dot_pos+1:dot_pos+5].decode(errors='ignore')
            logging.debug(f"Parsed type string: '{type_str}'")
            type_val = str2uint(type_str)
            logging.debug(f"Parsed type value: {type_val}")
            if type_val == 6102:
                return NEVA_124_6102
            elif type_val == 7109:
                return NEVA_124_7109
    return NEVA_124_UNKNOWN

def ack_start(ser, neva_type):
    ser.baudrate = BAUDRATE_9600
    send_command(ser, 'ack_start')
    
    data, err = response_meter(ser, 'ack_start')
    logging.debug("Response: %s, error: %s", data, err)
    if err == "OK":
        if neva_type == NEVA_124_6102:
            send_command(ser, 'password_6102')
            _, err = response_meter(ser, 'password_6102')
            logging.debug("Response: %s, error: %s", data, err)
        elif neva_type == NEVA_124_7109:
            send_command(ser, 'password_7109')
            _, err = response_meter(ser, 'password_7109')
            logging.debug("Response: %s, error: %s", data, err)
        return err == "OK"
    return False

def get_tariffs_6102(ser):
    send_command(ser, 'tariffs_6102')
    data, err = response_meter(ser, 'tariffs_6102')
    logging.debug("Response: %s, error: %s", data, err)
    if err != "OK":
        return {}
    bracket_pos = data.find(b'(')
    if bracket_pos != -1:
        p_str = data[bracket_pos:]
        tariff_summ, shift = number_from_tariffs(p_str)
        p_str = p_str[shift:]
        tariff1, shift = number_from_tariffs(p_str)
        p_str = p_str[shift:]
        tariff2, shift = number_from_tariffs(p_str)
        p_str = p_str[shift:]
        tariff3, shift = number_from_tariffs(p_str)
        p_str = p_str[shift:]
        tariff4, _ = number_from_tariffs(p_str)
        return {
            'energy_divisor': divisor,
            'tariff_summ': tariff_summ,
            'tariff1': tariff1,
            'tariff2': tariff2,
            'tariff3': tariff3,
            'tariff4': tariff4
        }
    return {}

# Аналогично для других get_* (tariffs_7109, power, volts, amps, serial, sensors для battery)
def get_tariffs_7109(ser):
    send_command(ser, 'tariffs_7109')
    data, err = response_meter(ser, 'tariffs_7109')
    logging.debug("Response: %s, error: %s", data, err)
    if err != "OK":
        return {}
    bracket_pos = data.find(b']')
    if bracket_pos != -1:
        p_str = data[bracket_pos+1:]
        tariff_summ = 0
        tariff1, shift = number_from_tariffs(p_str)
        tariff_summ += tariff1
        p_str = p_str[shift:]
        tariff2, shift = number_from_tariffs(p_str)
        tariff_summ += tariff2
        p_str = p_str[shift:]
        tariff3, shift = number_from_tariffs(p_str)
        tariff_summ += tariff3
        p_str = p_str[shift:]
        tariff4, _ = number_from_tariffs(p_str)
        tariff_summ += tariff4
        return {
            'energy_divisor': divisor,
            'tariff_summ': tariff_summ,
            'tariff1': tariff1,
            'tariff2': tariff2,
            'tariff3': tariff3,
            'tariff4': tariff4
        }
    return {}

def get_power_data(ser, neva_type):
    send_command(ser, 'power_data')
    data, err = response_meter(ser, 'power_data')
    logging.debug("Response: %s, error: %s", data, err)
    if err != "OK":
        return None
    power = number_from_brackets(data)
    if power is not None:
        global divisor, multiplier
        multiplier = 1
        if neva_type == NEVA_124_6102:
            divisor = 1000
            if power > 0xffff:
                power //= 10
                divisor //= 10
        else:
            if power != 0:
                power //= 100
                divisor = 1000
                if power > 0xffff:
                    power //= 10
                    divisor //= 10
            else:
                divisor = 1
        return power & 0xffff, divisor & 0xffff, multiplier
    return None, None, None

def get_voltage_data(ser):
    send_command(ser, 'volts_data')
    data, err = response_meter(ser, 'volts_data')
    logging.debug("Response: %s, error: %s", data, err)
    if err != "OK":
        return None
    volts = number_from_brackets(data)
    if volts is not None:
        return volts, divisor & 0xffff
    return None

def get_amps_data(ser):
    send_command(ser, 'amps_data')
    data, err = response_meter(ser, 'amps_data')
    logging.debug("Response: %s, error: %s", data, err)
    if err != "OK":
        return None
    amps = number_from_brackets(data)
    if amps is not None:
        return amps, divisor & 0xffff
    return None

def get_serial_number_data(ser):
    send_command(ser, 'serial_number')
    data, err = response_meter(ser, 'serial_number')
    logging.debug("Response: %s, error: %s", data, err)
    if err != "OK":
        return None
    return str_from_brackets(data)

def get_resbat_data(ser):
    send_command(ser, 'sensors_data')
    data, err = response_meter(ser, 'sensors_data')
    logging.debug("Response: %s, error: %s", data, err)
    if err != "OK":
        return None
    bracket_pos = data.find(b'(')
    if bracket_pos != -1:
        p_str = data[bracket_pos+1:]
        comma_pos = p_str.find(b',')
        if comma_pos != -1:
            p_str = p_str[comma_pos+1:]
            battery_mv = str2uint(p_str[:1]) * 1000 + str2uint(p_str[2:]) * 10  # Адаптировано
            if battery_mv < MIN_VBAT_MV:
                battery_mv = MIN_VBAT_MV
            battery_level = (battery_mv - MIN_VBAT_MV) // ((MAX_VBAT_MV - MIN_VBAT_MV) // 100)
            if battery_level > 100:
                battery_level = 100
            return battery_level
    return None

def close_session(ser):
    send_command(ser, 'close_channel')

# MQTT Discovery конфиги (публикуем один раз)
def publish_discovery(client, prefix, neva_type):
    device_info = {
        "identifiers": ["neva_mt124_meter"],
        "name": "Neva MT124 Meter",
        "model": "MT124" + ("-6102" if neva_type == NEVA_124_6102 else "-7109"),
        "manufacturer": "Neva"
    }
    
    # Сенсор для суммарной энергии
    config = {
        "name": "Total Energy",
        "state_topic": f"{prefix}/total_energy",
        "unit_of_measurement": "kWh",
        "device_class": "energy",
        "state_class": "total_increasing",
        "unique_id": "neva_total_energy",
        "device": device_info
    }
    client.publish(f"homeassistant/sensor/neva_mt124/total_energy/config", json.dumps(config), retain=True)
    
    # Аналогично для тарифов 1-4, power, voltage, current, battery, serial
    # Tariff 1
    config = {
        "name": "Tariff 1 Energy",
        "state_topic": f"{prefix}/tariff1",
        "unit_of_measurement": "kWh",
        "device_class": "energy",
        "state_class": "total_increasing",
        "unique_id": "neva_tariff1",
        "device": device_info
    }
    client.publish(f"homeassistant/sensor/neva_mt124/tariff1/config", json.dumps(config), retain=True)

    # Tariff 2
    config = {
        "name": "Tariff 2 Energy",
        "state_topic": f"{prefix}/tariff2",
        "unit_of_measurement": "kWh",
        "device_class": "energy",
        "state_class": "total_increasing",
        "unique_id": "neva_tariff2",
        "device": device_info
    }
    client.publish(f"homeassistant/sensor/neva_mt124/tariff2/config", json.dumps(config), retain=True)

      # Tariff 3
    config = {
        "name": "Tariff 3 Energy",
        "state_topic": f"{prefix}/tariff3",
        "unit_of_measurement": "kWh",
        "device_class": "energy",
        "state_class": "total_increasing",
        "unique_id": "neva_tariff3",
        "device": device_info
    }
    client.publish(f"homeassistant/sensor/neva_mt124/tariff3/config", json.dumps(config), retain=True)
    
      # Tariff 4
    config = {
        "name": "Tariff 4 Energy",
        "state_topic": f"{prefix}/tariff4",
        "unit_of_measurement": "kWh",
        "device_class": "energy",
        "state_class": "total_increasing",
        "unique_id": "neva_tariff4",
        "device": device_info
    }
    client.publish(f"homeassistant/sensor/neva_mt124/tariff4/config", json.dumps(config), retain=True)
    
    # Power
    config = {
        "name": "Active Power",
        "state_topic": f"{prefix}/power",
        "unit_of_measurement": "W",
        "device_class": "power",
        "state_class": "measurement",
        "unique_id": "neva_power",
        "device": device_info
    }
    client.publish(f"homeassistant/sensor/neva_mt124/power/config", json.dumps(config), retain=True)
    
    # Voltage
    config = {
        "name": "Voltage",
        "state_topic": f"{prefix}/voltage",
        "unit_of_measurement": "V",
        "device_class": "voltage",
        "state_class": "measurement",
        "unique_id": "neva_voltage",
        "device": device_info
    }
    client.publish(f"homeassistant/sensor/neva_mt124/voltage/config", json.dumps(config), retain=True)
    
    # Current
    config = {
        "name": "Current",
        "state_topic": f"{prefix}/current",
        "unit_of_measurement": "A",
        "device_class": "current",
        "state_class": "measurement",
        "unique_id": "neva_current",
        "device": device_info
    }
    client.publish(f"homeassistant/sensor/neva_mt124/current/config", json.dumps(config), retain=True)
    
    # Battery
    config = {
        "name": "Battery Level",
        "state_topic": f"{prefix}/battery",
        "unit_of_measurement": "%",
        "device_class": "battery",
        "state_class": "measurement",
        "unique_id": "neva_battery",
        "device": device_info
    }
    client.publish(f"homeassistant/sensor/neva_mt124/battery/config", json.dumps(config), retain=True)
    
    # Serial Number (string)
    config = {
        "name": "Serial Number",
        "state_topic": f"{prefix}/serial",
        "unique_id": "neva_serial",
        "device": device_info
    }
    client.publish(f"homeassistant/sensor/neva_mt124/serial/config", json.dumps(config), retain=True)

# Основной цикл
def main():
    # Чтение опций из HA (config.json в /data/options.json)
    logging.debug("Opening options.json")
    with open('/data/options.json', 'r') as f:
        options = json.load(f)
    serial_port = options['serial_port']
    initial_baudrate = options['initial_baudrate']
    main_baudrate = options['main_baudrate']
    interval = options['interval_seconds']
    prefix = options['mqtt_topic_prefix']
    
    # MQTT из HA (bashio или env)
    mqtt_host = os.environ.get('HASSIO_MQTT_HOST', 'core-mosquitto')
    mqtt_port = int(os.environ.get('HASSIO_MQTT_PORT', 1883))
    mqtt_user = os.environ.get('HASSIO_MQTT_USER', '')
    mqtt_pass = os.environ.get('HASSIO_MQTT_PASSWORD', '')
    
    client = mqtt.Client()
    if mqtt_user and mqtt_pass:
        client.username_pw_set(mqtt_user, mqtt_pass)
    client.connect(mqtt_host, mqtt_port, 60)
    logging.debug("Connected to MQTT at %s:%d", mqtt_host, mqtt_port)
    client.loop_start()
    
    discovered = False
    
    while True:
        try:
            logging.debug("Starting poll cycle")
            with serial.Serial(serial_port, initial_baudrate, timeout=10, bytesize=7, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE) as ser:  # Even parity как в C
                neva_type = open_session(ser)
                logging.debug(f"open_session returned: {neva_type}")
                if neva_type != NEVA_124_UNKNOWN:
                    if ack_start(ser, neva_type):
                        if not discovered:
                            publish_discovery(client, prefix, neva_type)
                            discovered = True
                        
                        data = {}
                        
                        serial_num = get_serial_number_data(ser)
                        if serial_num:
                            data['serial'] = serial_num
                            client.publish(f"{prefix}/serial", serial_num)
                        
                        # Date release не поддерживается, пропускаем или статично "Not supported"
                        client.publish(f"{prefix}/date_release", "Not supported")
                        
                        if neva_type == NEVA_124_6102:
                            tariffs = get_tariffs_6102(ser)
                        else:
                            tariffs = get_tariffs_7109(ser)
                            battery = get_resbat_data(ser)
                            if battery is not None:
                                data['battery'] = battery
                                client.publish(f"{prefix}/battery", battery)
                        
                        if tariffs:
                            client.publish(f"{prefix}/total_energy", tariffs['tariff_summ'] / tariffs['energy_divisor'])
                            client.publish(f"{prefix}/tariff1", tariffs['tariff1'] / tariffs['energy_divisor'])
                            client.publish(f"{prefix}/tariff2", tariffs['tariff2'] / tariffs['energy_divisor'])
                            client.publish(f"{prefix}/tariff3", tariffs['tariff3'] / tariffs['energy_divisor'])
                            client.publish(f"{prefix}/tariff4", tariffs['tariff4'] / tariffs['energy_divisor'])
                                              
                        power, power_div, mult = get_power_data(ser, neva_type)
                        if power is not None:
                            client.publish(f"{prefix}/power", (power * mult) / power_div)
                        
                        volts, volts_div = get_voltage_data(ser)
                        if volts is not None:
                            client.publish(f"{prefix}/voltage", volts / volts_div)
                        
                        amps, amps_div = get_amps_data(ser)
                        if amps is not None:
                            client.publish(f"{prefix}/current", amps / amps_div)
                        
                        print(f"Data published: {data}")
                        
                close_session(ser)
        except Exception as e:
            logging.error("Global error: %s", e)
#            print(f"Error: {e}")
        
        time.sleep(interval)

if __name__ == "__main__":
    main()

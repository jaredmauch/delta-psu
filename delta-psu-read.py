#!/usr/bin/python3

"""
Delta PSU PMBus Reader

This script provides a comprehensive interface to read and monitor Delta Electronics
Q54SG series power supply units via PMBus interface. It supports reading various
parameters including voltage, current, temperature, fan speed, and status information.

Usage Examples:
    # Basic usage (human-readable output)
    python delta-psu-read.py

    # JSON output
    python delta-psu-read.py --json

    # Save to file
    python delta-psu-read.py --json --output psu_data.json

    # Different I2C address
    python delta-psu-read.py --address 0x61

    # Different I2C bus
    python delta-psu-read.py --bus 0

    # Read from hex dump file
    python delta-psu-read.py --file ii2c-hex.txt

Features:
    - Read manufacturer information (ID, model, serial number, etc.)
    - Monitor operating parameters (voltage, current, power, efficiency)
    - Read temperature sensors and fan speeds
    - Check status registers and fault conditions
    - Read fault and warning limits
    - Monitor timing parameters
    - Output in human-readable or JSON format
    - Support for multiple I2C addresses and buses
    - Support for reading from hex dump files

Requirements:
    - Python 3.x
    - smbus module (python3-smbus package) for I2C access
    - I2C access permissions (when using I2C mode)
"""

import json
import smbus
import argparse
from datetime import datetime
import re

# Default I2C address for Delta PSU (can be changed via command line)
I2C_ADDRESS = 0x60

# PMBus page commands for multi-page devices
CMD_PAGE = 0x00
CMD_PAGE_READ = 0x01
CMD_PAGE_WRITE = 0x02
CMD_PAGE_READ_BACK = 0x03
CMD_PAGE_WRITE_BACK = 0x04

# PMBus supported commands for Delta Q54SG series
# Operation and Configuration Commands
CMD_OPERATION = 0x01          # Operation mode control
CMD_ON_OFF_CONFIG = 0x02      # On/Off configuration
CMD_WRITE_PROTECT = 0x10      # Write protection control
CMD_CAPABILITY = 0x19         # Device capability information
CMD_VOUT_MODE = 0x20         # Output voltage mode
CMD_VOUT_COMMAND = 0x21      # Output voltage command
CMD_VOUT_MAX = 0x24          # Maximum output voltage

# Frequency and Timing Commands
CMD_FREQUENCY_SWITCH = 0x33   # Switching frequency control
CMD_VIN_ON = 0x35            # Input voltage on threshold
CMD_VIN_OFF = 0x36           # Input voltage off threshold

# Fault and Warning Limit Commands
CMD_IOUT_OC_FAULT_LIMIT = 0x46   # Output overcurrent fault limit
CMD_IOUT_OC_WARN_LIMIT = 0x4A    # Output overcurrent warning limit
CMD_OT_FAULT_LIMIT = 0x4F        # Overtemperature fault limit
CMD_OT_WARN_LIMIT = 0x51         # Overtemperature warning limit
CMD_UT_FAULT_LIMIT = 0x54        # Undertemperature fault limit
CMD_UT_WARN_LIMIT = 0x56         # Undertemperature warning limit
CMD_VIN_UV_FAULT_LIMIT = 0x59    # Input undervoltage fault limit
CMD_VIN_UV_WARN_LIMIT = 0x5B     # Input undervoltage warning limit
CMD_VIN_OV_FAULT_LIMIT = 0x55    # Input overvoltage fault limit
CMD_VIN_OV_WARN_LIMIT = 0x57     # Input overvoltage warning limit
CMD_IIN_OC_FAULT_LIMIT = 0x5F    # Input overcurrent fault limit
CMD_IIN_OC_WARN_LIMIT = 0x61     # Input overcurrent warning limit

# Power Good and Timing Commands
CMD_POWER_GOOD_ON = 0x5E     # Power good on threshold
CMD_POWER_GOOD_OFF = 0x60    # Power good off threshold
CMD_TON_DELAY = 0x60         # Turn-on delay
CMD_TON_RISE = 0x61          # Turn-on rise time
CMD_TOFF_DELAY = 0x64        # Turn-off delay
CMD_TOFF_FALL = 0x65         # Turn-off fall time

# Status Commands
CMD_STATUS_WORD = 0x79           # Main status word
CMD_STATUS_VOUT = 0x7A          # Output voltage status
CMD_STATUS_IOUT = 0x7B          # Output current status
CMD_STATUS_INPUT = 0x7C         # Input status
CMD_STATUS_TEMPERATURE = 0x7D   # Temperature status
CMD_STATUS_CML = 0x7E           # Communication/memory/logic status
CMD_STATUS_OTHER = 0x7F         # Other status

# Read Commands
CMD_READ_VIN = 0x88                # Read input voltage
CMD_READ_VOUT = 0x8B               # Read output voltage
CMD_READ_IOUT = 0x8C               # Read output current
CMD_READ_TEMPERATURE_1 = 0x8D      # Read temperature sensor 1
CMD_READ_TEMPERATURE_2 = 0x8E      # Read temperature sensor 2
CMD_READ_TEMPERATURE_3 = 0x8F      # Read temperature sensor 3
CMD_READ_FAN_SPEED_1 = 0x90        # Read fan speed 1
CMD_READ_FAN_SPEED_2 = 0x91        # Read fan speed 2
CMD_READ_FAN_SPEED_3 = 0x92        # Read fan speed 3
CMD_READ_FAN_SPEED_4 = 0x93        # Read fan speed 4
CMD_READ_DUTY_CYCLE = 0x94         # Read duty cycle
CMD_READ_FREQUENCY = 0x95          # Read switching frequency
CMD_READ_POUT = 0x96               # Read output power
CMD_READ_PIN = 0x97                # Read input power

# Manufacturer Information Commands
CMD_PMBUS_REVISION = 0x98    # PMBus revision
CMD_MFR_ID = 0x99           # Manufacturer ID
CMD_MFR_MODEL = 0x9A        # Manufacturer model
CMD_MFR_REVISION = 0x9B     # Manufacturer revision
CMD_MFR_LOCATION = 0x9C     # Manufacturing location
CMD_MFR_DATE = 0x9D         # Manufacturing date
CMD_MFR_SERIAL = 0x9E       # Serial number

class DeltaPSU:
    """
    Class to interface with Delta Electronics Q54SG series power supply units.
    
    This class provides methods to read various parameters and status information
    from the PSU via PMBus interface or from a hex dump file.
    
    Attributes:
        bus (SMBus): I2C bus object (None when using file mode)
        address (int): I2C address of the PSU
        hex_data (dict): Dictionary containing hex data (256 bytes)
        debug (bool): Whether to print debug information
    """
    
    def __init__(self, bus_number=1, address=I2C_ADDRESS, hex_file=None, debug=False):
        """
        Initialize the PSU interface.
        
        Args:
            bus_number (int): I2C bus number (default: 1)
            address (int): I2C address of the PSU (default: 0x60)
            hex_file (str): Path to hex dump file (optional)
            debug (bool): Whether to print debug information
        """
        self.address = address
        self.hex_data = {}
        self.debug = debug
        
        if hex_file:
            self.bus = None
            self._load_hex_file(hex_file)
        else:
            self.bus = smbus.SMBus(bus_number)
            self._load_smbus_data()
            
    def _load_smbus_data(self):
        """
        Load first 256 bytes from SMBus device.
        """
        try:
            if self.debug:
                print(f"\nLoading SMBus data from address 0x{self.address:02X}")
            
            # Read first 256 bytes
            for addr in range(256):
                try:
                    value = self.bus.read_byte_data(self.address, addr)
                    self.hex_data[addr] = value
                except:
                    self.hex_data[addr] = 0
                    
            if self.debug:
                print(f"Loaded {len(self.hex_data)} bytes from SMBus")
                print("First few bytes:")
                for addr in sorted(self.hex_data.keys())[:10]:
                    print(f"  0x{addr:02X}: 0x{self.hex_data[addr]:02X}")
                    
        except Exception as e:
            if self.debug:
                print(f"Error reading SMBus data: {e}")
            raise
            
    def _load_hex_file(self, hex_file):
        """
        Load hex data from a file.
        
        Args:
            hex_file (str): Path to the hex dump file
        """
        try:
            if self.debug:
                print(f"\nLoading hex file: {hex_file}")
            
            with open(hex_file, 'r') as f:
                lines = f.readlines()
            
            if self.debug:
                print(f"Found {len(lines)} lines in file")
                print(f"First line (header): {lines[0].strip()}")
            
            # Skip the first line (header)
            for line_num, line in enumerate(lines[1:], 1):
                line = line.strip()
                if not line:  # Skip empty lines
                    continue
                    
                # Parse the line format: "00: 01 02 03 ..."
                match = re.match(r'(\w+):\s+((?:\w{2}\s+)*)', line)
                if match:
                    offset = int(match.group(1), 16)
                    hex_values = match.group(2).strip().split()
                    if self.debug:
                        print(f"\nLine {line_num}:")
                        print(f"  Offset: 0x{offset:02X}")
                        print(f"  Hex values: {' '.join(hex_values)}")
                    
                    for i, hex_val in enumerate(hex_values):
                        value = int(hex_val, 16)
                        self.hex_data[offset + i] = value
                        if self.debug:
                            print(f"    Address 0x{offset + i:02X}: 0x{value:02X}")
                elif self.debug:
                    print(f"\nWarning: Line {line_num} did not match expected format:")
                    print(f"  Line: {line}")
            
            if self.debug:
                print(f"\nLoaded {len(self.hex_data)} bytes from hex file")
                print("First few bytes:")
                for addr in sorted(self.hex_data.keys())[:10]:
                    print(f"  0x{addr:02X}: 0x{self.hex_data[addr]:02X}")
                    
        except Exception as e:
            if self.debug:
                print(f"Error reading hex file: {e}")
            raise
        
    def read_byte(self, command):
        """
        Read a single byte from the cached data.
        
        Args:
            command (int): PMBus command code
            
        Returns:
            int: Byte value read from the device
        """
        value = self.hex_data.get(command, 0)
        if self.debug:
            print(f"read_byte(0x{command:02X}) -> 0x{value:02X}")
        return value
        
    def write_byte(self, command, value):
        """
        Write a single byte to the PMBus device.
        
        Args:
            command (int): PMBus command code
            value (int): Byte value to write
        """
        if self.bus is None:
            raise RuntimeError("Write operations not supported in file mode")
        self.bus.write_byte_data(self.address, command, value)
        
    def read_word(self, command):
        """
        Read a word (2 bytes) from the cached data.
        Handles PMBus linear data format.
        
        Args:
            command (int): PMBus command code
            
        Returns:
            int: Word value read from the device
        """
        # Handle special cases for status registers
        if command in [CMD_STATUS_WORD, CMD_STATUS_VOUT, CMD_STATUS_IOUT,
                     CMD_STATUS_INPUT, CMD_STATUS_TEMPERATURE, CMD_STATUS_CML,
                     CMD_STATUS_OTHER]:
            # Status registers are stored at specific offsets in the hex data
            status_offsets = {
                CMD_STATUS_WORD: 0x00,      # First word in the dump
                CMD_STATUS_VOUT: 0x02,      # Second word
                CMD_STATUS_IOUT: 0x04,       # Third word
                CMD_STATUS_INPUT: 0x06,     # Fourth word
                CMD_STATUS_TEMPERATURE: 0x08, # Fifth word
                CMD_STATUS_CML: 0x0A,       # Sixth word
                CMD_STATUS_OTHER: 0x0C      # Seventh word
            }
            offset = status_offsets.get(command, command)
            byte1 = self.hex_data.get(offset, 0)
            byte2 = self.hex_data.get(offset + 1, 0)
            value = (byte2 << 8) | byte1  # Little-endian format
            if self.debug:
                print(f"read_word(0x{command:02X}) -> 0x{value:04X} (from offset 0x{offset:02X}: 0x{byte2:02X} 0x{byte1:02X})")
            return value
        
        # Special handling for VIN - read from offset 0x88 in Linear11 format
        if command == CMD_READ_VIN:
            # Read the raw value from offset 0x88
            byte1 = self.hex_data.get(0x88, 0)
            byte2 = self.hex_data.get(0x89, 0)
            raw_value = (byte2 << 8) | byte1  # Little-endian format
            
            if self.debug:
                print(f"VIN raw value from 0x88: 0x{raw_value:04X} (0x{byte2:02X} 0x{byte1:02X})")
            
            # Parse Linear11 format
            # Linear11: Y = (mX + b) * 2^R
            # where:
            # Y = real value
            # X = raw value (11 bits)
            # m = coefficient (5 bits)
            # b = offset (11 bits)
            # R = exponent (5 bits)
            
            # Extract components from raw value
            X = raw_value & 0x7FF  # Lower 11 bits
            m = (raw_value >> 11) & 0x1F  # Next 5 bits
            b = (raw_value >> 16) & 0x7FF  # Next 11 bits
            R = (raw_value >> 27) & 0x1F  # Upper 5 bits
            
            if self.debug:
                print(f"Linear11 components: X={X}, m={m}, b={b}, R={R}")
            
            # Calculate real value
            real_value = (m * X + b) * (2 ** R)
            
            if self.debug:
                print(f"VIN calculated value: {real_value}V")
            
            return real_value
            
        # Read two bytes and combine them
        byte1 = self.hex_data.get(command, 0)
        byte2 = self.hex_data.get(command + 1, 0)
        value = (byte2 << 8) | byte1  # Little-endian format
        if self.debug:
            print(f"read_word(0x{command:02X}) -> 0x{value:04X} (0x{byte2:02X} 0x{byte1:02X})")
        
        # Handle PMBus linear data format
        if command in [CMD_READ_VOUT, CMD_READ_IOUT, CMD_READ_PIN, CMD_READ_POUT]:
            # For voltage, current, and power readings
            # Y = (mX + b) * 10^R
            # where:
            # Y = real value
            # X = raw value
            # m = coefficient (stored in upper 5 bits)
            # b = offset (stored in lower 11 bits)
            # R = exponent (stored in upper 5 bits)
            
            # Get the mode byte for this command
            mode_cmd = command - 0x80  # Convert to mode command
            mode = self.hex_data.get(mode_cmd, 0)
            
            if self.debug:
                print(f"  Mode byte: 0x{mode:02X}")
            
            # Extract m, b, and R from mode byte
            m = (mode >> 3) & 0x1F
            b = mode & 0x07
            R = (mode >> 3) & 0x1F
            
            # Calculate real value
            real_value = (m * value + b) * (10 ** R)
            
            if self.debug:
                print(f"  Linear format: m={m}, b={b}, R={R}")
                print(f"  Raw value: {value}")
                print(f"  Real value: {real_value}")
            
            return int(real_value)
            
        elif command in [CMD_READ_TEMPERATURE_1, CMD_READ_TEMPERATURE_2, CMD_READ_TEMPERATURE_3]:
            # For temperature readings
            # Y = (mX + b) * 10^R
            # where m=1, b=0, R=0 for temperature
            return value
            
        elif command in [CMD_READ_FAN_SPEED_1, CMD_READ_FAN_SPEED_2, CMD_READ_FAN_SPEED_3, CMD_READ_FAN_SPEED_4]:
            # For fan speed readings
            # Y = (mX + b) * 10^R
            # where m=1, b=0, R=0 for fan speed
            return value
            
        elif command in [CMD_READ_DUTY_CYCLE]:
            # For duty cycle readings
            # Y = (mX + b) * 10^R
            # where m=1, b=0, R=0 for duty cycle
            return value
            
        elif command in [CMD_READ_FREQUENCY]:
            # For frequency readings
            # Y = (mX + b) * 10^R
            # where m=1, b=0, R=0 for frequency
            return value
            
        else:
            # For other readings, return raw value
            return value
        
    def write_word(self, command, value):
        """
        Write a word (2 bytes) to the PMBus device.
        
        Args:
            command (int): PMBus command code
            value (int): Word value to write
        """
        if self.bus is None:
            raise RuntimeError("Write operations not supported in file mode")
        self.bus.write_word_data(self.address, command, value)
        
    def read_string(self, command, length=16):
        """
        Read a string from the cached data.
        
        Args:
            command (int): PMBus command code
            length (int): Maximum length of string to read
            
        Returns:
            str: String read from the device, or "Not Available" if read fails
        """
        try:
            # Handle special cases for manufacturer data
            if command == CMD_MFR_ID:
                # Read from offset 0x0C where "DELTA" is stored
                data = [self.hex_data.get(0x0C + i, 0) for i in range(5)]  # "DELTA" is 5 characters
                if self.debug:
                    print(f"Manufacturer data from offset 0x0C: {' '.join([f'0x{x:02X}' for x in data])}")
            elif command == CMD_MFR_MODEL:
                # Read from offset 0x10 where model number is stored
                data = [self.hex_data.get(0x10 + i, 0) for i in range(11)]  # "DPS-800AB-30" is 11 characters
                if self.debug:
                    print(f"Model data from offset 0x10: {' '.join([f'0x{x:02X}' for x in data])}")
            elif command == CMD_MFR_SERIAL:
                # Read from offset 0x30 where serial number is stored
                data = [self.hex_data.get(0x30 + i, 0) for i in range(12)]  # "IBKD2022005142" is 12 characters
                if self.debug:
                    print(f"Serial data from offset 0x30: {' '.join([f'0x{x:02X}' for x in data])}")
            else:
                # For other strings, use the command address
                data = [self.hex_data.get(command + i, 0) for i in range(length)]
                if self.debug:
                    print(f"read_string(0x{command:02X}, {length}) raw data: {' '.join([f'0x{x:02X}' for x in data])}")
            
            # Filter out non-printable characters and special characters
            result = ''.join([chr(x) for x in data if x != 0 and 32 <= x <= 126])
            if self.debug:
                print(f"read_string(0x{command:02X}, {length}) -> '{result}'")
            return result
        except Exception as e:
            if self.debug:
                print(f"read_string(0x{command:02X}, {length}) -> 'Not Available' (error: {e})")
            return "Not Available"
        
    def get_status(self):
        """
        Get the status word from the device.
        
        Returns:
            int: Status word value
        """
        return self.read_word(CMD_STATUS_WORD)
        
    def get_vin(self):
        """
        Get input voltage.
        
        Returns:
            int: Input voltage in millivolts
        """
        return self.read_word(CMD_READ_VIN)
        
    def get_vout(self):
        """
        Get output voltage.
        
        Returns:
            int: Output voltage in millivolts
        """
        return self.read_word(CMD_READ_VOUT)
        
    def get_iout(self):
        """
        Get output current.
        
        Returns:
            int: Output current in milliamperes
        """
        return self.read_word(CMD_READ_IOUT)
        
    def get_temperature(self):
        """
        Get primary temperature reading.
        
        Returns:
            int: Temperature in degrees Celsius
        """
        return self.read_word(CMD_READ_TEMPERATURE_1)
        
    def get_fan_speed(self):
        """
        Get primary fan speed reading.
        
        Returns:
            int: Fan speed in RPM
        """
        return self.read_word(CMD_READ_FAN_SPEED_1)
        
    def get_power(self):
        """
        Get input and output power readings.
        
        Returns:
            tuple: (input_power, output_power) in watts
        """
        pin = self.read_word(CMD_READ_PIN)
        pout = self.read_word(CMD_READ_POUT)
        return pin, pout

    def get_manufacturer_info(self):
        """
        Get manufacturer information.
        
        Returns:
            dict: Dictionary containing manufacturer information
        """
        info = {
            'ID': self.read_string(CMD_MFR_ID),
            'Model': self.read_string(CMD_MFR_MODEL),
            'Revision': self.read_string(CMD_MFR_REVISION),
            'Location': self.read_string(CMD_MFR_LOCATION),
            'Date': self.read_string(CMD_MFR_DATE),
            'Serial': self.read_string(CMD_MFR_SERIAL),
            'PMBus Revision': self.read_byte(CMD_PMBUS_REVISION)
        }
        return info

    def get_frequency(self):
        """
        Get switching frequency.
        
        Returns:
            int: Switching frequency in Hz
        """
        return self.read_word(CMD_READ_FREQUENCY)

    def get_duty_cycle(self):
        """
        Get duty cycle.
        
        Returns:
            int: Duty cycle percentage
        """
        return self.read_word(CMD_READ_DUTY_CYCLE)

    def get_all_temperatures(self):
        """
        Get all temperature readings.
        
        Returns:
            dict: Dictionary containing all temperature sensor readings
        """
        return {
            'temp1': self.read_word(CMD_READ_TEMPERATURE_1),
            'temp2': self.read_word(CMD_READ_TEMPERATURE_2),
            'temp3': self.read_word(CMD_READ_TEMPERATURE_3)
        }

    def get_all_fan_speeds(self):
        """
        Get all fan speed readings.
        
        Returns:
            dict: Dictionary containing all fan speed readings
        """
        return {
            'fan1': self.read_word(CMD_READ_FAN_SPEED_1),
            'fan2': self.read_word(CMD_READ_FAN_SPEED_2),
            'fan3': self.read_word(CMD_READ_FAN_SPEED_3),
            'fan4': self.read_word(CMD_READ_FAN_SPEED_4)
        }

    def get_all_status(self):
        """
        Get all status registers.
        
        Returns:
            dict: Dictionary containing all status register values
        """
        return {
            'status_word': self.read_word(CMD_STATUS_WORD),
            'status_vout': self.read_word(CMD_STATUS_VOUT),
            'status_iout': self.read_word(CMD_STATUS_IOUT),
            'status_input': self.read_word(CMD_STATUS_INPUT),
            'status_temperature': self.read_word(CMD_STATUS_TEMPERATURE),
            'status_cml': self.read_word(CMD_STATUS_CML),
            'status_other': self.read_word(CMD_STATUS_OTHER)
        }

    def get_fault_limits(self):
        """
        Get all fault and warning limits.
        
        Returns:
            dict: Dictionary containing all fault and warning limit values
        """
        return {
            'iout_oc_fault_limit': self.read_word(CMD_IOUT_OC_FAULT_LIMIT),
            'iout_oc_warn_limit': self.read_word(CMD_IOUT_OC_WARN_LIMIT),
            'ot_fault_limit': self.read_word(CMD_OT_FAULT_LIMIT),
            'ot_warn_limit': self.read_word(CMD_OT_WARN_LIMIT),
            'ut_fault_limit': self.read_word(CMD_UT_FAULT_LIMIT),
            'ut_warn_limit': self.read_word(CMD_UT_WARN_LIMIT),
            'vin_uv_fault_limit': self.read_word(CMD_VIN_UV_FAULT_LIMIT),
            'vin_uv_warn_limit': self.read_word(CMD_VIN_UV_WARN_LIMIT),
            'vin_ov_fault_limit': self.read_word(CMD_VIN_OV_FAULT_LIMIT),
            'vin_ov_warn_limit': self.read_word(CMD_VIN_OV_WARN_LIMIT),
            'iin_oc_fault_limit': self.read_word(CMD_IIN_OC_FAULT_LIMIT),
            'iin_oc_warn_limit': self.read_word(CMD_IIN_OC_WARN_LIMIT)
        }

    def get_timing_parameters(self):
        """
        Get timing parameters.
        
        Returns:
            dict: Dictionary containing timing parameter values
        """
        return {
            'ton_delay': self.read_word(CMD_TON_DELAY),
            'ton_rise': self.read_word(CMD_TON_RISE),
            'toff_delay': self.read_word(CMD_TOFF_DELAY),
            'toff_fall': self.read_word(CMD_TOFF_FALL)
        }

    def get_all_info(self):
        """
        Get all PSU information in a dictionary format.
        
        Returns:
            dict: Dictionary containing all PSU information
        """
        try:
            mfr_info = self.get_manufacturer_info()
            pin, pout = self.get_power()
            
            info = {
                'timestamp': datetime.now().isoformat(),
                'i2c_address': f"0x{self.address:02X}",
                'manufacturer_info': mfr_info,
                'operating_parameters': {
                    'input_voltage': self.get_vin(),
                    'output_voltage': self.get_vout(),
                    'output_current': self.get_iout(),
                    'temperature': self.get_temperature(),
                    'fan_speed': self.get_fan_speed(),
                    'switching_frequency': self.get_frequency(),
                    'duty_cycle': self.get_duty_cycle(),
                    'input_power': pin,
                    'output_power': pout,
                    'efficiency': round((pout/pin)*100, 2) if pin > 0 else 0
                },
                'temperatures': self.get_all_temperatures(),
                'fan_speeds': self.get_all_fan_speeds(),
                'status': self.get_all_status(),
                'fault_limits': self.get_fault_limits(),
                'timing_parameters': self.get_timing_parameters()
            }
            return info
        except Exception as e:
            return {'error': str(e)}

def format_human_readable(data):
    """
    Format PSU data in human-readable format.
    
    Args:
        data (dict): Dictionary containing PSU data
        
    Returns:
        str: Formatted string
    """
    output = []
    output.append("\nDelta PSU Information:")
    output.append("-" * 50)
    
    output.append("\nManufacturer Information:")
    output.append(f"Manufacturer: {data['manufacturer']}")
    output.append(f"Model: {data['model']}")
    output.append(f"Serial: {data['serial']}")
    output.append(f"Revision: {data['revision']}")
    
    output.append("\nStatus:")
    for key, value in data['status'].items():
        output.append(f"{key}: 0x{value:04X}")
    
    output.append("\nMeasurements:")
    for key, value in data['measurements'].items():
        if key in ['vin', 'vout']:
            output.append(f"{key}: {value}V")
        elif key == 'iout':
            output.append(f"{key}: {value}A")
        elif key == 'temperature':
            output.append(f"{key}: {value}°C")
        elif key == 'fan_speed':
            output.append(f"{key}: {value} RPM")
        elif key == 'duty_cycle':
            output.append(f"{key}: {value}%")
        elif key == 'frequency':
            output.append(f"{key}: {value} Hz")
        elif key in ['pout', 'pin']:
            output.append(f"{key}: {value}W")
        else:
            output.append(f"{key}: {value}")
    
    return "\n".join(output)

def main():
    parser = argparse.ArgumentParser(description='Delta PSU PMBus Reader')
    parser.add_argument('--json', action='store_true', help='Output in JSON format')
    parser.add_argument('--output', type=str, help='Output file path')
    parser.add_argument('--address', type=str, default='0x60', help='I2C address (default: 0x60)')
    parser.add_argument('--bus', type=int, default=1, help='I2C bus number (default: 1)')
    parser.add_argument('--file', type=str, help='Read from hex dump file instead of I2C')
    parser.add_argument('--debug', action='store_true', help='Enable debug output')
    args = parser.parse_args()

    try:
        # Convert address string to integer
        address = int(args.address, 16)
        
        # Initialize PSU interface
        psu = DeltaPSU(bus_number=args.bus, address=address, hex_file=args.file, debug=args.debug)
        
        if args.debug:
            print("\nCollecting PSU information...")
        
        # Get PSU information
        data = {
            'timestamp': datetime.now().isoformat(),
            'manufacturer': psu.read_string(CMD_MFR_ID),
            'model': psu.read_string(CMD_MFR_MODEL),
            'serial': psu.read_string(CMD_MFR_SERIAL),
            'revision': psu.read_string(CMD_MFR_REVISION),
            'status': {
                'word': psu.get_status(),
                'vin': psu.read_word(CMD_STATUS_VOUT),
                'iout': psu.read_word(CMD_STATUS_IOUT),
                'temperature': psu.read_word(CMD_STATUS_TEMPERATURE),
                'fan': psu.read_word(CMD_STATUS_OTHER),
                'other': psu.read_word(CMD_STATUS_OTHER)
            },
            'measurements': {
                'vin': psu.get_vin(),
                'vout': psu.get_vout(),
                'iout': psu.get_iout(),
                'temperature': psu.get_temperature(),
                'fan_speed': psu.get_fan_speed(),
                'duty_cycle': psu.get_duty_cycle(),
                'frequency': psu.get_frequency(),
                'pout': psu.get_power()[1],
                'pin': psu.get_power()[0]
            }
        }

        if args.debug:
            print("\nCollected data:")
            print(json.dumps(data, indent=2))

        if args.json:
            output = json.dumps(data, indent=2)
        else:
            output = format_human_readable(data)

        if args.output:
            with open(args.output, 'w') as f:
                f.write(output)
            if args.debug:
                print(f"\nData saved to {args.output}")
        else:
            print(output)

    except Exception as e:
        print(f"Error: {e}")
        return 1

    return 0

if __name__ == '__main__':
    exit(main())


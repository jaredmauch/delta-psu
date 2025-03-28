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

Features:
    - Read manufacturer information (ID, model, serial number, etc.)
    - Monitor operating parameters (voltage, current, power, efficiency)
    - Read temperature sensors and fan speeds
    - Check status registers and fault conditions
    - Read fault and warning limits
    - Monitor timing parameters
    - Output in human-readable or JSON format
    - Support for multiple I2C addresses and buses

Requirements:
    - Python 3.x
    - smbus module (python3-smbus package)
    - I2C access permissions
"""

import json
import smbus
import argparse
from datetime import datetime

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
    from the PSU via PMBus interface.
    
    Attributes:
        bus (SMBus): I2C bus object
        address (int): I2C address of the PSU
    """
    
    def __init__(self, bus_number=1, address=I2C_ADDRESS):
        """
        Initialize the PSU interface.
        
        Args:
            bus_number (int): I2C bus number (default: 1)
            address (int): I2C address of the PSU (default: 0x60)
        """
        self.bus = smbus.SMBus(bus_number)
        self.address = address
        
    def read_byte(self, command):
        """
        Read a single byte from the PMBus device.
        
        Args:
            command (int): PMBus command code
            
        Returns:
            int: Byte value read from the device
        """
        return self.bus.read_byte_data(self.address, command)
        
    def write_byte(self, command, value):
        """
        Write a single byte to the PMBus device.
        
        Args:
            command (int): PMBus command code
            value (int): Byte value to write
        """
        self.bus.write_byte_data(self.address, command, value)
        
    def read_word(self, command):
        """
        Read a word (2 bytes) from the PMBus device.
        
        Args:
            command (int): PMBus command code
            
        Returns:
            int: Word value read from the device
        """
        return self.bus.read_word_data(self.address, command)
        
    def write_word(self, command, value):
        """
        Write a word (2 bytes) to the PMBus device.
        
        Args:
            command (int): PMBus command code
            value (int): Word value to write
        """
        self.bus.write_word_data(self.address, command, value)
        
    def read_string(self, command, length=16):
        """
        Read a string from the PMBus device.
        
        Args:
            command (int): PMBus command code
            length (int): Maximum length of string to read
            
        Returns:
            str: String read from the device, or "Not Available" if read fails
        """
        try:
            data = self.bus.read_i2c_block_data(self.address, command, length)
            return ''.join([chr(x) for x in data if x != 0])
        except:
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

def print_human_readable(info):
    """
    Print PSU information in human-readable format.
    
    Args:
        info (dict): Dictionary containing PSU information
    """
    print("\nDelta PSU Information:")
    print("-" * 50)
    
    print("\nManufacturer Information:")
    for key, value in info['manufacturer_info'].items():
        print(f"{key}: {value}")
    
    print("\nOperating Parameters:")
    for key, value in info['operating_parameters'].items():
        if key == 'efficiency':
            print(f"{key}: {value}%")
        elif key in ['input_voltage', 'output_voltage']:
            print(f"{key}: {value}V")
        elif key == 'output_current':
            print(f"{key}: {value}A")
        elif key == 'temperature':
            print(f"{key}: {value}°C")
        elif key == 'fan_speed':
            print(f"{key}: {value} RPM")
        elif key == 'switching_frequency':
            print(f"{key}: {value} Hz")
        elif key == 'duty_cycle':
            print(f"{key}: {value}%")
        elif key in ['input_power', 'output_power']:
            print(f"{key}: {value}W")
        else:
            print(f"{key}: {value}")
    
    print("\nTemperature Readings:")
    for key, value in info['temperatures'].items():
        print(f"{key}: {value}°C")
    
    print("\nFan Speeds:")
    for key, value in info['fan_speeds'].items():
        print(f"{key}: {value} RPM")
    
    print("\nStatus Registers:")
    for key, value in info['status'].items():
        print(f"{key}: 0x{value:04X}")
    
    print("\nFault and Warning Limits:")
    for key, value in info['fault_limits'].items():
        if 'voltage' in key:
            print(f"{key}: {value}V")
        elif 'current' in key:
            print(f"{key}: {value}A")
        elif 'temperature' in key:
            print(f"{key}: {value}°C")
        else:
            print(f"{key}: {value}")
    
    print("\nTiming Parameters:")
    for key, value in info['timing_parameters'].items():
        print(f"{key}: {value}ms")

def main():
    """
    Main function to handle command line arguments and execute the PSU reader.
    """
    parser = argparse.ArgumentParser(description='Delta PSU PMBus Reader')
    parser.add_argument('--address', '-a', type=str, default=f"0x{I2C_ADDRESS:02X}",
                      help='I2C address in hex (e.g., 0x60)')
    parser.add_argument('--json', '-j', action='store_true',
                      help='Output in JSON format')
    parser.add_argument('--output', '-o', type=str,
                      help='Save output to JSON file')
    parser.add_argument('--bus', '-b', type=int, default=1,
                      help='I2C bus number (default: 1)')
    
    args = parser.parse_args()
    
    try:
        # Convert hex address string to integer
        address = int(args.address, 16)
        psu = DeltaPSU(bus_number=args.bus, address=address)
        
        # Get all information
        info = psu.get_all_info()
        
        if args.json:
            # Output as JSON
            if args.output:
                with open(args.output, 'w') as f:
                    json.dump(info, f, indent=2)
                print(f"Data saved to {args.output}")
            else:
                print(json.dumps(info, indent=2))
        else:
            # Output in human-readable format
            print_human_readable(info)
            
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()


import serial
import struct

START_BYTE = 0xAA

# Define test case structure
class TestCase:
    def __init__(self, test_function, parameter):
        self.test_function = test_function
        self.parameter = parameter

# Function to calculate checksum
def calculate_checksum(payload_length, test_case, result):
    checksum = 0
    checksum ^= payload_length
    checksum ^= test_case.test_function
    checksum ^= test_case.parameter
    checksum ^= result
    return checksum & 0xFF  # Ensure the checksum is 8 bits

# Function to send test case and receive result
def send_and_receive_test_case(ser, test_case):
    payload_length = 2  # Length of test case data (2 bytes)
    result = 0x00  # Placeholder for result

    # Send test case frame
    ser.write(bytes([START_BYTE]))
    ser.write(bytes([payload_length]))
    ser.write(bytes([test_case.test_function, test_case.parameter]))
    checksum = calculate_checksum(payload_length, test_case, result)
    ser.write(bytes([checksum]))

    # Receive and process result frame
    start_byte = ser.read()
    if start_byte != bytes([START_BYTE]):
        print("Invalid start byte received.")
        return

    # Read payload length
    payload_length = ser.read()[0]

    # Read test function and parameter
    test_function, parameter = struct.unpack('BB', ser.read(2))

    # Read result
    result = ser.read()[0]

    # Read checksum
    received_checksum = ser.read()[0]

    # Calculate checksum
    checksum = calculate_checksum(payload_length, TestCase(test_function, parameter), result)

    # Check for checksum match
    if checksum != received_checksum:
        print("Checksum mismatch.")
        return

    # Process result
    if result == 0x01:
        print("Test case executed successfully.")
    elif result == 0x00:
        print("Test case execution failed.")
    else:
        print("Invalid result.")

# Example test cases
test_cases = [
    TestCase(0x01, 0x01),  # Test get_switch() with parameter 0x01
    TestCase(0x02, 0x02),  # Test Led_on() with parameter 0x02
    # Add more test cases as needed
]

# Serial port configuration
ser = serial.Serial('COM6', 9600, timeout=1)  # Set timeout to 1 second

try:
    # Send test cases and receive results
    for test_case in test_cases:
        send_and_receive_test_case(ser, test_case)

    # Check if no data is received
    if not ser.in_waiting:
        print("No data received from STM32.")

except serial.SerialException as e:
    print("Serial port error:", e)

finally:
    ser.close()

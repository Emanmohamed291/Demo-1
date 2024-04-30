import serial
import unittest
import time

# Constants
SERIAL_PORT = 'COM4'  # Adjust as needed
BAUD_RATE = 9600
TIMEOUT = 1  # Serial timeout
COMMAND_DELAY = 0.5  # Optional delay between sending commands
LED_STATUS_DELAY = 0.5  # Delay for LED status changes to take effect

class TestSTM32SwitchLED(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.serial = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)

    @classmethod
    def tearDownClass(cls):
        cls.serial.close()

    def send_rpc(self, command):
        # Clear buffer before sending command
        self.serial.reset_input_buffer()

        # Send command and wait for processing
        self.serial.write((command + '\n').encode())
        time.sleep(COMMAND_DELAY)  # Optional delay for processing

        # Read response
        response = self.serial.readline().decode().strip()
        return response

    def test_switch_and_led(self):
        # Step 1: Get switch status
        switch_response = self.send_rpc('GET_SWITCH_STATUS')  # Get the current switch state
        
        # Step 2: Decide what to do based on switch state
        if switch_response == 'PRESSED':
            # If switch is pressed, send command to turn on LED
            led_response = self.send_rpc('SET_LED_ON')
            self.assertEqual(led_response, 'OK')  # Expecting "OK" as confirmation
        elif switch_response == 'RELEASED':
            # If switch is released, send command to turn off LED
            led_response = self.send_rpc('SET_LED_OFF')
            self.assertEqual(led_response, 'OK')  # Expecting "OK" as confirmation
        else:
            # Handle unexpected switch status
            self.fail("Unexpected switch status: " + switch_response)

        # Step 3: Validate the LED status
        # Give some time for the LED state change
       # time.sleep(LED_STATUS_DELAY)
        
        # Get the LED status and validate
       # led_status = self.send_rpc('GET_LED_STATUS')  # Get the current LED state
       # if switch_response == 'PRESSED':
       #     self.assertEqual(led_status, 'LED_ON')  # Expecting LED to be on
       # elif switch_response == 'RELEASED':
       #     self.assertEqual(led_status, 'LED_OFF')  # Expecting LED to be off

if __name__ == '__main__':
    unittest.main()

import json
import struct
import time
import serial

class MotorProtocol:
    def __init__(self, serial_conn=None, acq_thread=None, config_file='motor_commands.json'):
        self.ser = serial_conn
        self.acq_thread = acq_thread
        
        # Load config
        with open(config_file, 'r') as f:
            config = json.load(f)
            self.commands = config.get('commands', {})
            self.plotter_dict = config.get('plotter_dict', {})
        
        # Generate methods dynamically
        self._generate_methods()
    
    def _generate_methods(self):
        """Generate methods from JSON config"""
        for cmd_name, cmd_config in self.commands.items():
            setattr(self, cmd_name, self._create_method(cmd_name, cmd_config))
    
    def _create_method(self, cmd_name, cmd_config):
        """Create method for a specific command"""
        def method(*args, **kwargs):
            # Build data packet
            data = bytes([cmd_config['code']])
            
            # Process arguments
            arg_spec = cmd_config.get('args', [])
            if len(args) != len(arg_spec):
                raise ValueError(f"Expected {len(arg_spec)} args for {cmd_name}")
            
            for i, arg_info in enumerate(arg_spec):
                arg_value = args[i]
                if arg_info['type'] == 'uint8':
                    data += bytes([arg_value])
                elif arg_info['type'] == 'float':
                    data += struct.pack('<f', arg_value)
                elif arg_info['type'] == 'uint16':
                    data += struct.pack('<H', arg_value)
            
            # Send and receive
            if cmd_config.get('response', False):
                response_type = cmd_config.get('response_type', 'uint8')
                response_size = cmd_config.get('response_size', 1)
                
                if response_type == 'uint8':
                    return self.recv_uint8_data(data, response_size)
                elif response_type == 'float':
                    return self.recv_float_data(data, response_size)
            else:
                return self.send_data(data)
        
        method.__name__ = cmd_name
        return method
    
    # ================================================================
    # Serial Connection

    def connect(self):
        self.ser = serial.Serial(
            self.port,
            self.baudrate,
            timeout=self.timeout
        )

    def reconnect(self):
        print("Reconnecting...")

        while True:
            try:
                if self.ser is not None:
                    if self.ser.is_open:
                        self.ser.close()

                time.sleep(1)

                self.ser = serial.Serial(
                    self.port,
                    self.baudrate,
                    timeout=self.timeout
                )

                print("Reconnected!")
                break

            except Exception:
                print(".", end="", flush=True)
                time.sleep(1)

    # ================================================================
    # Communication
        
    def send_data(self, data):
        self.ser.write(data)
        self.acq_thread.expect_response(1)
        payload = self.acq_thread.response_queue.get(timeout=2)
        return struct.unpack("<b", payload)[0]

    def recv_float_data(self, cmd, n=1):
        self.ser.write(cmd)
        self.acq_thread.expect_response(n*4)
        payload = self.acq_thread.response_queue.get(timeout=2)
        data = struct.unpack(
            "<" + "f"*n,
            payload
        )
        if n == 1:
            return data[0]
        return data
        
    def recv_uint8_data(self, cmd, n=1):
        self.ser.write(cmd)
        self.acq_thread.expect_response(n)
        payload = self.acq_thread.response_queue.get(timeout=2)
        data = struct.unpack(
            "<" + "B"*n,
            payload
        )
        if n == 1:
            return data[0]
        return data
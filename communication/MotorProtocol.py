import json
import struct
import time
import serial
import numpy as np

class MotorProtocol:
    def __init__(self, serial_conn=None, acq_thread=None, config_file='motor_commands.json'):
        self.ser = serial_conn
        self.acq_thread = acq_thread
        self.plotter_channels = []
        
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
            
            # Handle both positional and keyword arguments
            all_args = list(args)
            for arg_info in arg_spec:
                arg_name = arg_info['name']
                if arg_name in kwargs:
                    all_args.append(kwargs[arg_name])
            
            if len(all_args) != len(arg_spec):
                raise ValueError(f"Expected {len(arg_spec)} args for {cmd_name}, got {len(all_args)}")

            # Plotter information
            plotter_item = None
            
            for i, arg_info in enumerate(arg_spec):
                arg_value = all_args[i]
                
                # Apply converter if specified
                if 'converter' in arg_info:
                    converter = arg_info['converter']
                    if converter == 'plotter_addr':
                        # Use the plotter_dict from JSON config
                        if isinstance(arg_value, str):
                            plotter_item = arg_value
                            try:
                                arg_value = self.plotter_dict[plotter_item]
                            except KeyError:
                                raise ValueError(f"Unknown plotter item: {arg_value}")
                            print(f"{plotter_item} = 0x{arg_value:04X}")
                        else:
                            arg_value = int(arg_value)
                    # Add more converters here as needed
                
                # Pack the argument based on type
                if arg_info['type'] == 'uint8':
                    data += bytes([arg_value])
                elif arg_info['type'] == 'float':
                    data += struct.pack('<f', arg_value)
                elif arg_info['type'] == 'uint16':
                    data += struct.pack('<H', arg_value)
                elif arg_info['type'] == 'int16':
                    data += struct.pack('<h', arg_value)
                elif arg_info['type'] == 'uint32':
                    data += struct.pack('<I', arg_value)
                elif arg_info['type'] == 'int32':
                    data += struct.pack('<i', arg_value)
                else:
                    raise ValueError(f"Unsupported argument type: {arg_info['type']}")
            
            # Send and receive
            response = cmd_config.get('response', [])
            ret = self.send_recv_data(data, response)
            cmd = cmd_config['code']
            # print(f'cmd: {cmd}')
            
            # plotter_add_line (code 7)
            if cmd == 7:
                if plotter_item is not None:
                    if plotter_item not in self.plotter_channels:
                        print(f'add plotter_item {plotter_item}')
                        self.plotter_channels.append(plotter_item)
            # plotter_remove_line (code 8)
            elif cmd == 8:
                if plotter_item is not None:
                    if plotter_item in self.plotter_channels:
                        print(f'remove plotter_item {plotter_item}')
                        self.plotter_channels.remove(plotter_item)
            return ret
        
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

    def send_recv_data(self, cmd, response_spec):
        if self.acq_thread is None:
            print(f"Would receive mixed data for cmd: {cmd.hex()}")
            return {}
        
        # Calculate total bytes needed
        total_bytes = 0
        type_map = {
            'uint8': 1,
            'int8': 1,
            'uint16': 2,
            'int16': 2,
            'uint32': 4,
            'int32': 4,
            'float': 4
        }
        
        for resp_info in response_spec:
            total_bytes += type_map.get(resp_info['type'], 0)
        
        self.ser.write(cmd)
        
        if total_bytes == 0:
            return {}
        
        self.acq_thread.expect_response(total_bytes)
        payload = self.acq_thread.response_queue.get(timeout=2)
        
        results = {}
        offset = 0
        for resp_info in response_spec:
            resp_name = resp_info['name']
            resp_type = resp_info['type']
            
            if resp_type == 'uint8':
                value = struct.unpack('<B', payload[offset:offset+1])[0]
                offset += 1
            elif resp_type == 'uint16':
                value = struct.unpack('<H', payload[offset:offset+2])[0]
                offset += 2
            elif resp_type == 'uint32':
                value = struct.unpack('<I', payload[offset:offset+4])[0]
                offset += 4
            elif resp_type == 'float':
                value = struct.unpack('<f', payload[offset:offset+4])[0]
                offset += 4
            elif resp_type == 'int16':
                value = struct.unpack('<h', payload[offset:offset+2])[0]
                offset += 2
            elif resp_type == 'int32':
                value = struct.unpack('<i', payload[offset:offset+4])[0]
                offset += 4
            else:
                raise ValueError(f"Unsupported response type: {resp_type}")
            
            results[resp_name] = value
        
        return results

    def self_commissioning(self):
        self.start_measure_motor_Rs()
        time.sleep(1)
        self.start_measure_motor_Ld()
        time.sleep(1)
        self.start_measure_motor_Lq()
        time.sleep(1)
        self.get_motor_param()
        self.set_foc_bandwidth(500)
        self.start_calibrate_abs_encoder()

    def get_motor_param(self):
        pole_pairs = self.get_pole_pairs()['value']
        rs = self.get_rs()['value']
        ld = self.get_ld()['value']
        lq = self.get_lq()['value']
        print(f'pole pairs:{pole_pairs}')
        print(f'Rs:{rs}')
        print(f'Ld:{ld}')
        print(f'Lq:{lq}')

    def set_foc_bandwidth(self,bw=100):
        rs = self.get_rs()['value'] / 2
        ld = self.get_ld()['value'] / 2
        lq = self.get_lq()['value'] / 2
        omega = 2 * np.pi * bw
        id_kp = ld * omega
        id_ki = rs * omega
        iq_kp = lq * omega
        iq_ki = rs * omega
        print(f'id: kp={id_kp} ki={id_ki}')
        print(f'iq: kp={iq_kp} ki={iq_ki}')
        self.set_pid_id(id_kp, id_ki, 0)
        self.set_pid_iq(iq_kp, iq_ki, 0)
import struct
import serial
import time
import json
import numpy as np
from enum import IntEnum

class SFMComType(IntEnum):
    RESPONSE = 0x00
    READ = 0x01
    WRITE = 0x02
    ENABLE_STREAMING = 0x03
    DISABLE_STREAMING = 0x04
    STREAMING = 0x05
    EVENT = 0x06


class SFMotion:
    HEADER = b"\xA5\xA5"

    def __init__(self, serial_conn=None, acq_thread=None, node_id=1, json_path="SFMotionRegisters.json"):
        self.ser = serial_conn
        self.acq_thread = acq_thread
        self.node_id = node_id

        with open(json_path, "r") as f:
            config = json.load(f)

        self.registers = {
            item["address"]: item
            for item in config["registers"]
        }
        self.channel = IntEnum(
            "channel",
            {
                item["name"]: item["address"]
                for item in config["registers"]
                if item["streaming"]
            }
        )
        self._create_api()

    def _create_api(self):
        for register in self.registers.values():
            address = register["address"]
            name = register["name"]
            if register["read"]:
                setattr(self, f"get_{name}", 
                    lambda address=address: self.read(address)
                )
            if register["write"]:
                setattr(self, f"set_{name}",
                    lambda value=None, address=address: self.write(address, value)
                )

    def close(self):
        self.ser.close()

    # ------------------------------------------------------------
    # Low-level communication
    # ------------------------------------------------------------

    def _send_frame(self, com_type, address, data=b""):
        payload = bytes([com_type, self.node_id, address]) + data
        frame = (self.HEADER + bytes([len(payload)]) + payload)
        # print(f"TX: {frame.hex(' ')}")
        self.ser.write(frame)

    def read(self, address):
        register = self.registers.get(address)
        if register is None:
            raise ValueError(f"Unknown register address: {address}")
        if not register["read"]:
            raise ValueError(f"Address {address} ({register['name']}) is not readable")
        fmt = register["format"]
        if fmt is None:
            raise ValueError(f"Address {address} ({register['name']}) has no data format")
        size = struct.calcsize(fmt)
        self._send_frame(SFMComType.READ, address)
        response_node_id, response_address, data = (
            self.acq_thread.response_queue.get(timeout=2)
        )
        if response_node_id != self.node_id:
            raise RuntimeError(f"Unexpected node ID: {response_node_id}")
        if response_address != address:
            raise RuntimeError(f"Unexpected address: {response_address}")
        if len(data) != size:
            raise RuntimeError(f"Expected {size} bytes, received {len(data)}")
        return struct.unpack(fmt, data)[0]

    def write(self, address, value=None):
        register = self.registers.get(address)
        if register is None:
            raise ValueError(f"Unknown register address: {address}")
        if not register["write"]:
            raise ValueError(f"Address {address} ({register['name']}) is not writable")
        fmt = register["format"]
        if fmt is None:
            data = b""
        else:
            data = struct.pack(fmt, value)
        self._send_frame(SFMComType.WRITE, address, data,)
        response_node_id, response_address, response_data = (
            self.acq_thread.response_queue.get(timeout=2)
        )
        if response_node_id != self.node_id:
            raise RuntimeError(f"Unexpected node ID: {response_node_id}")
        if response_address != address:
            raise RuntimeError(f"Unexpected address: {response_address}")
        if len(response_data) != 1:
            raise RuntimeError(f"Invalid response length: {len(response_data)}")
        status = struct.unpack("<b", response_data)[0]
        if status != 0:
            raise RuntimeError(f"Device returned error: {status}")
        return True

    # ------------------------------------------------------------
    # Streaming
    # ------------------------------------------------------------

    def enable_streaming(self, address):
        self._send_frame(SFMComType.ENABLE_STREAMING, address,)
        response_node_id, response_address, response_data = self.acq_thread.response_queue.get(timeout=2)
        if response_node_id != self.node_id:
            raise RuntimeError(f"Unexpected node ID: {response_node_id}")
        if response_address != address:
            raise RuntimeError(f"Unexpected address: {response_address}")
        status = struct.unpack("<b", response_data)[0]
        if status != 0:
            raise RuntimeError(f"Device returned error: {status}")

    def disable_streaming(self, address):
        self._send_frame(SFMComType.DISABLE_STREAMING, address,)
        response_node_id, response_address, response_data = self.acq_thread.response_queue.get(timeout=2)
        if response_node_id != self.node_id:
            raise RuntimeError(f"Unexpected node ID: {response_node_id}")
        if response_address != address:
            raise RuntimeError(f"Unexpected address: {response_address}")
        status = struct.unpack("<b", response_data)[0]
        if status != 0:
            raise RuntimeError(f"Device returned error: {status}")

    # ----------------------------------------------------------------------------------
    
    def get_version(self):
        address = 1
        self._send_frame(SFMComType.READ, address)
        response_node_id, response_address, response_data = self.acq_thread.response_queue.get(timeout=2)
        if response_node_id != self.node_id:
            raise RuntimeError(f"Unexpected node ID: {response_node_id}")
        if response_address != address:
            raise RuntimeError(f"Unexpected address: {response_address}")
        major, minor, patch = struct.unpack("<3B", response_data)
        return major, minor, patch
    
    def self_commissioning(self):
        self.set_start_measure_resistance()
        time.sleep(1)
        self.set_start_measure_ld()
        time.sleep(1)
        self.set_start_measure_lq()
        time.sleep(1)
        self.get_motor_param()
        self.set_foc_bandwidth(500)
        self.set_start_calibrate_abs_encoder()
        self.set_pid_speed(0.005, 0.8, 10)
        self.set_pid_position(10, 0, 0, 1000)

    def get_motor_param(self):
        pole_pairs = self.get_pole_pairs()
        rs = self.get_rs()
        ld = self.get_ld()
        lq = self.get_lq()
        print(f'pole pairs:{pole_pairs}')
        print(f'Rs:{rs}')
        print(f'Ld:{ld}')
        print(f'Lq:{lq}')

    def set_foc_bandwidth(self,bw=100):
        rs = self.get_rs() / 2
        ld = self.get_ld() / 2
        lq = self.get_lq() / 2
        omega = 2 * np.pi * bw
        id_kp = ld * omega
        id_ki = rs * omega
        iq_kp = lq * omega
        iq_ki = rs * omega
        print(f'id: kp={id_kp} ki={id_ki}')
        print(f'iq: kp={iq_kp} ki={iq_ki}')
        self.set_id_kp(id_kp)
        self.set_id_ki(id_ki)
        self.set_id_deadband(0)
        self.set_iq_kp(iq_kp)
        self.set_iq_ki(iq_ki)
        self.set_iq_deadband(0)

    def set_pid_speed(self, kp, ki, max_out, deadband=0):
        self.set_speed_kp(kp)
        self.set_speed_ki(ki)
        self.set_speed_out_max(max_out)
        self.set_speed_deadband(deadband)

    def get_pid_speed(self):
        kp = self.get_speed_kp()
        ki = self.get_speed_ki()
        max_out = self.get_speed_out_max()
        deadband = self.get_speed_deadband()
        return {
            "kp": kp,
            "ki": ki,
            "max_out": max_out,
            "deadband": deadband
        }

    def set_pid_position(self, kp, ki, kd, max_out, deadband=0, d_filter_fc=20):
        self.set_position_kp(kp)
        self.set_position_ki(ki)
        self.set_position_kd(kd)
        self.set_position_out_max(max_out)
        self.set_position_deadband(deadband)
        self.set_position_d_filter_fc(d_filter_fc)

    def get_pid_position(self):
        kp = self.get_position_kp()
        ki = self.get_position_ki()
        kd = self.get_position_kd()
        max_out = self.get_position_out_max()
        deadband = self.get_position_deadband()
        d_filter_fc = self.get_position_d_filter_fc()
        return {
            "kp": kp,
            "ki": ki,
            "kd": kd,
            "max_out": max_out,
            "deadband": deadband,
            "d_filter_fc": d_filter_fc
        }

    
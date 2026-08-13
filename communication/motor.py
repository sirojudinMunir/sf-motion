from plotter import *
import numpy as np
import serial
import keyboard


app = QtWidgets.QApplication(sys.argv)
app.setStyle('Fusion')

pg.setConfigOptions(antialias=True, useOpenGL=True)

plotter = LivePlotter(
    max_points=1000,
    port=None,  # No auto-connect
    baudrate=115200
)

# Run ------------------------------------------
plotter.show()
sys.exit(app.exec_())

m = plotter.motor

# ================================================================
def run_svpwm(step_deg=10, rotation=1, delay=0.005):
    m.set_foc_motor_mode(5)
    pole_pairs = m.get_pole_pairs()
    for theta in np.arange(0, 2*np.pi*pole_pairs*rotation, np.deg2rad(step_deg)):
        m.set_svpwm(1, 0, theta)
        time.sleep(delay)
    m.set_svpwm(0, 0, 0)

def scene_six_step_commutation():
    run_svpwm(60, 5)
    run_svpwm(60, 5, 0.02)

def scene_openloop_control():
    run_svpwm(10, 20)

def scene_closedloop_voltage_control():
    m.set_foc_motor_mode(5)
    while not keyboard.is_pressed('esc'):
        e_rad = m.get_foc_actual_e_rad()
        m.set_svpwm(0, 1, e_rad)
        time.sleep(0.001)

    m.set_svpwm(0, 0, 0)

# ================================================================
def scene_current_control():
    m.set_foc_motor_mode(0)
    m.set_foc_current_set_point(0.5)
    time.sleep(2)
    m.set_foc_current_set_point(-0.5)
    time.sleep(4)
    m.set_foc_current_set_point(0)

def scene_speed_control():
    m.set_foc_motor_mode(1)
    m.set_foc_speed_set_point(50)
    time.sleep(1)
    m.set_foc_speed_set_point(-50)
    time.sleep(1)
    m.set_foc_speed_set_point(0)
    time.sleep(1)
    for speed in range(0, 1010, 10):
        m.set_foc_speed_set_point(speed)
        time.sleep(0.01)
    time.sleep(3)
    for speed in range(1000, -1, -10):
        m.set_foc_speed_set_point(speed)
        time.sleep(0.01)
    time.sleep(2)
    for speed in range(0, -1010, -10):
        m.set_foc_speed_set_point(speed)
        time.sleep(0.01)
    time.sleep(3)
    for speed in range(-1000, 1, 10):
        m.set_foc_speed_set_point(speed)
        time.sleep(0.01)

def scene_position_control():
    m.set_foc_motor_mode(2)

    for position in [0, 90, -90, 0, 360, 0]:
        m.set_foc_position_set_point(position)
        time.sleep(1)
        
    for position in range(0, 361, 45):
        m.set_foc_position_set_point(position)
        time.sleep(0.5)
        
    for position in range(360, -1, -45):
        m.set_foc_position_set_point(position)
        time.sleep(0.5)

# ================================================================
def get_motor_param():
    pole_pairs = m.get_pole_pairs()
    kv = m.get_kv()
    rs = m.get_rs()
    ld = m.get_ld()
    lq = m.get_lq()
    flux_linkage = m.get_flux_linkage()
    print(f'pole pairs:{pole_pairs}, kv:{kv}, rs:{rs}, ld:{ld}, lq:{lq}, flux_linkage:{flux_linkage}')

def set_foc_bandwidth(bw=100):
    rs = m.get_rs() / 2
    ld = m.get_ld() / 2
    lq = m.get_lq() / 2
    omega = 2 * np.pi * bw
    id_kp = ld * omega
    id_ki = rs * omega
    iq_kp = lq * omega
    iq_ki = rs * omega
    print(f'id: kp={id_kp} ki={id_ki}')
    print(f'iq: kp={iq_kp} ki={iq_ki}')
    m.set_pid_id(id_kp, id_ki, 0)
    m.set_pid_iq(iq_kp, iq_ki, 0)
    


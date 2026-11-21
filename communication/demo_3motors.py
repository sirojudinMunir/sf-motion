from plotter import *
import numpy as np
import serial
import keyboard
import time

scene_threads = []

def scene_demo1():
    motors = plotter.motors
    try:
        m1, m2, m3 = (motors[node_id] for node_id in (0x01, 0x02, 0x03))
    except KeyError as exc:
        raise RuntimeError("Connect devices with node IDs 1, 2, and 3 first.") from exc

    m1.move_s_curve(0, 2000, 20000, 100000)
    m2.move_s_curve(0, 2000, 20000, 100000)
    m3.move_s_curve(0, 2000, 20000, 100000)
    time.sleep(2)

    m1.move_s_curve(-360, 2000, 20000, 100000)
    time.sleep(1)
    m2.move_s_curve(-360, 2000, 20000, 100000)
    time.sleep(1)
    m3.move_s_curve(-360, 2000, 20000, 100000)
    time.sleep(1)
    m1.move_s_curve(360, 2000, 20000, 100000)
    time.sleep(1)
    m2.move_s_curve(360, 2000, 20000, 100000)
    time.sleep(1)
    m3.move_s_curve(360, 2000, 20000, 100000)
    time.sleep(1)

    m1.move_s_curve(1000, 10000, 100000, 100000)
    time.sleep(0.25)
    m2.move_s_curve(1000, 10000, 100000, 100000)
    time.sleep(0.25)
    m3.move_s_curve(1000, 10000, 100000, 100000)
    time.sleep(1)
    
    m3.move_s_curve(-1000, 10000, 100000, 100000)
    time.sleep(0.25)
    m2.move_s_curve(-1000, 10000, 100000, 100000)
    time.sleep(0.25)
    m1.move_s_curve(-1000, 10000, 100000, 100000)
    time.sleep(2)

    m1.move_s_curve(1000, 10000, 100000, 100000)
    m2.move_s_curve(0, 10000, 100000, 100000)
    time.sleep(1.7)
    m1.move_s_curve(-1000, 10000, 100000, 100000)
    m3.move_s_curve(1000, 10000, 100000, 100000)
    time.sleep(1.7)
    m1.move_s_curve(1000, 10000, 100000, 100000)
    m3.move_s_curve(-1000, 10000, 100000, 100000)
    time.sleep(1.7)

    m1.move_s_curve(0, 10000, 100000, 100000)
    m2.move_s_curve(0, 10000, 100000, 100000)
    m3.move_s_curve(0, 10000, 100000, 100000)
    time.sleep(2)

    m1.move_s_curve(-1000, 100, 20000, 100000)
    time.sleep(0.5)
    m2.move_s_curve(-1000, 100, 20000, 100000)
    time.sleep(0.5)
    m3.move_s_curve(-1000, 100, 20000, 100000)
    time.sleep(5)

    for i in range(4):
        m1.move_s_curve(1000, 5000, 10000, 500000)
        time.sleep(0.65)
        m2.move_s_curve(1000, 5000, 10000, 500000)
        time.sleep(0.65)
        m3.move_s_curve(1000, 5000, 10000, 500000)
        time.sleep(0.65)
        m1.move_s_curve(-1000, 5000, 10000, 500000)
        time.sleep(0.65)
        m2.move_s_curve(-1000, 5000, 10000, 500000)
        time.sleep(0.65)
        m3.move_s_curve(-1000, 5000, 10000, 500000)
        time.sleep(0.65)
    time.sleep(2)
    

    # sfm
    m1.move_s_curve(1000, 5000, 10000, 1000000)
    time.sleep(2)
    
    m2.move_s_curve(1000, 5000, 500000, 1000000)
    m3.move_s_curve(0, 5000, 500000, 1000000)
    time.sleep(3)


class SceneThread(QtCore.QThread):
    failed = QtCore.pyqtSignal(str)

    def __init__(self, scene):
        super().__init__()
        self.scene = scene

    def run(self):
        try:
            self.scene()
        except Exception as exc:
            self.failed.emit(str(exc))


def run_scene(scene):
    if not plotter.is_connected:
        raise RuntimeError("Connect to the devices before starting a scene.")
    if any(thread.isRunning() for thread in scene_threads):
        raise RuntimeError("A motor scene is already running.")

    worker = SceneThread(scene)
    scene_threads.append(worker)
    worker.failed.connect(print)
    worker.finished.connect(
        lambda worker=worker: scene_threads.remove(worker)
        if worker in scene_threads else None
    )
    worker.start()
    return worker


app = QtWidgets.QApplication(sys.argv)
app.setStyle('Fusion')
pg.setConfigOptions(antialias=True, useOpenGL=True)

plotter = LivePlotter(
    max_points=1024*10,
    port=None,  # No auto-connect
    baudrate=115200,
    node_id=[0x01, 0x02, 0x03]
)
plotter.console_namespace["run_scene_demo1"] = lambda: run_scene(scene_demo1)
plotter.console_namespace["run_scene"] = run_scene

plotter.show()
sys.exit(app.exec_())

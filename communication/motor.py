from plotter import *
import numpy as np
import serial
import keyboard
import time

app = QtWidgets.QApplication(sys.argv)
app.setStyle('Fusion')
pg.setConfigOptions(antialias=True, useOpenGL=True)

plotter = LivePlotter(
    max_points=1024*10,
    port=None,  # No auto-connect
    baudrate=115200,
    node_id=[0x00]
)

plotter.show()
sys.exit(app.exec_())

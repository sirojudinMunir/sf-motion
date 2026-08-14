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

    


from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
import numpy as np

app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(title="Real-Time Plot Example")
win.show()

plot = win.addPlot(title="Live Sine Wave")
curve = plot.plot()

x = np.linspace(0, 2 * np.pi, 1000)
i = 0

def update():
    global i
    y = np.sin(x + i / 10.0)
    curve.setData(x, y)
    i += 1

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)

app.exec_()

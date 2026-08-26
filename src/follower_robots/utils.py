from scipy.signal import butter, lfilter
import pyqtgraph as pg
from PyQt6 import QtWidgets
import sys
import numpy as np

class VectorRealtimeLowpass:
    def __init__(
        self,
        cutoff_hz,
        fs_hz,
        n_channels,
        order=2,
        history_seconds=10,
        debug=False,
    ):  
        self.debug = debug
        self.fs_hz = fs_hz
        self.iteration = 0

        self.b, self.a = butter(order, cutoff_hz, fs=fs_hz, btype="low")
        self.zi = np.zeros((n_channels, max(len(self.a), len(self.b)) - 1))

        if self.debug:
            # ----------------------------
            # Qt application
            # ----------------------------
            self.app = QtWidgets.QApplication.instance()
            if self.app is None:
                self.app = QtWidgets.QApplication(sys.argv)

            self.win = pg.GraphicsLayoutWidget(
                show=True,
                title="Realtime Lowpass Filter"
            )
            self.win.resize(1000, 250 * n_channels)

            # ----------------------------
            # Circular buffers
            # ----------------------------
            self.history = int(history_seconds * fs_hz)

            self.t = np.linspace(
                -history_seconds,
                0,
                self.history,
            )

            self.raw_data = np.zeros((n_channels, self.history))
            self.filtered_data = np.zeros((n_channels, self.history))

            self.raw_curves = []
            self.filtered_curves = []

            for i in range(n_channels):
                plot = self.win.addPlot(row=i, col=0)

                plot.showGrid(x=True, y=True)
                plot.setLabel("left", f"ch{i}")
                plot.setLabel("bottom", "Time [s]")

                plot.addLegend()

                raw = plot.plot(
                    self.t,
                    self.raw_data[i],
                    pen=pg.mkPen("r", width=1),
                    name="raw",
                )

                filt = plot.plot(
                    self.t,
                    self.filtered_data[i],
                    pen=pg.mkPen("g", width=2),
                    name="filtered",
                )

                self.raw_curves.append(raw)
                self.filtered_curves.append(filt)

    def step(self, x: np.ndarray) -> np.ndarray:
        self.iteration += 1

        y = np.empty_like(x)

        for i in range(len(x)):
            y_i, self.zi[i] = lfilter(
                self.b,
                self.a,
                [x[i]],
                zi=self.zi[i],
            )

            y[i] = y_i[0]

        if self.debug:
            # Shift buffers left by one sample
            self.raw_data[:, :-1] = self.raw_data[:, 1:]
            self.filtered_data[:, :-1] = self.filtered_data[:, 1:]

            # Insert newest sample
            self.raw_data[:, -1] = x
            self.filtered_data[:, -1] = y

        return y

    def update_stream(self):
        for i in range(self.raw_data.shape[0]):
            self.raw_curves[i].setData(
                self.t,
                self.raw_data[i],
            )

            self.filtered_curves[i].setData(
                self.t,
                self.filtered_data[i],
            )

        self.app.processEvents()
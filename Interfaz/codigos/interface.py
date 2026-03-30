"""
CubeSat Mission Control Dashboard
==================================
A high-fidelity telemetry dashboard for CubeSat descent monitoring.
Communicates with an ESP32 via serial (pyserial) or simulated data.

Dependencies:
    pip install PyQt6 pyserial matplotlib numpy

Usage:
    python cubesat_mission_control.py
    python cubesat_mission_control.py --port COM3       # Real serial
    python cubesat_mission_control.py --port /dev/ttyUSB0 --baud 115200
    python cubesat_mission_control.py --simulate        # Demo mode (default)
"""

import sys
import os
import math
import time
import random
import argparse
import threading
import datetime
from collections import deque
from dataclasses import dataclass, field
from typing import Optional, List, Dict, Any

# ── PyQt6 ────────────────────────────────────────────────────────────────────
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QProgressBar,
    QPushButton, QVBoxLayout, QHBoxLayout, QGridLayout, QFrame,
    QDialog, QScrollArea, QSizePolicy, QTableWidget, QTableWidgetItem,
    QHeaderView, QSplitter, QGroupBox
)
from PyQt6.QtCore import (
    Qt, QTimer, QThread, pyqtSignal, QObject, QPointF, QRectF, QSize
)
from PyQt6.QtGui import (
    QPainter, QPen, QColor, QBrush, QFont, QFontMetrics,
    QLinearGradient, QRadialGradient, QPainterPath, QPixmap,
    QPalette, QPolygonF
)

# ── Matplotlib (optional – used in analysis window) ──────────────────────────
try:
    import matplotlib
    matplotlib.use("QtAgg")
    from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
    from matplotlib.figure import Figure
    import matplotlib.dates as mdates
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

# ── NumPy (optional) ─────────────────────────────────────────────────────────
try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

# ─────────────────────────────────────────────────────────────────────────────
#  THEME & CONSTANTS
# ─────────────────────────────────────────────────────────────────────────────
CYAN       = QColor("#00E5FF")
CYAN_DIM   = QColor("#007A8C")
CYAN_GLOW  = QColor(0, 229, 255, 60)
ORANGE     = QColor("#FF6D00")
ORANGE_DIM = QColor("#8C3C00")
GREEN      = QColor("#00E676")
GREEN_DIM  = QColor("#004D1F")
RED        = QColor("#FF1744")
GRAY       = QColor("#37474F")
GRAY_DIM   = QColor("#1C2A30")
BG_DARK    = QColor("#070D11")
BG_PANEL   = QColor("#0A1520")
BG_CARD    = QColor("#0D1F2D")
TEXT_WHITE = QColor("#E0F7FA")
TEXT_DIM   = QColor("#546E7A")

FONT_MONO  = "Courier New"
FONT_SANS  = "Arial"          # fallback – overridden if system fonts differ

ALT_MAX_M  = 100.0            # maximum expected altitude in meters
TREND_LEN  = 60               # samples to keep in rolling trend chart
LAND_STABLE_SEC = 3.0         # seconds near 0 m before "landed" declared
LAND_ALT_THRESH = 1.0         # metres below which we consider "landed"

# ─────────────────────────────────────────────────────────────────────────────
#  DATA STRUCTURES
# ─────────────────────────────────────────────────────────────────────────────
@dataclass
class TelemetryFrame:
    timestamp:   float = 0.0
    altitude:    float = 80.0
    velocity:    float = -4.2      # m/s, negative = descending
    battery_v:   float = 12.8
    battery_pct: float = 88.0
    temperature: float = 18.5
    humidity:    float = 45.0
    accel_x:     float = 0.1
    accel_y:     float = -9.8
    accel_z:     float = 0.0
    rssi:        int   = -65       # dBm – used to derive link quality
    bitrate_kbps:int   = 512

    @classmethod
    def from_csv(cls, line: str) -> Optional["TelemetryFrame"]:
        """Parse comma-separated packet from ESP32.
        Expected format: ALT,VEL,BATV,BATP,TEMP,HUM,AX,AY,AZ,RSSI,BRATE
        """
        try:
            parts = [p.strip() for p in line.split(",")]
            f = cls()
            f.timestamp   = time.time()
            f.altitude    = float(parts[0])
            f.velocity    = float(parts[1])
            f.battery_v   = float(parts[2])
            f.battery_pct = float(parts[3])
            f.temperature = float(parts[4])
            f.humidity    = float(parts[5])
            f.accel_x     = float(parts[6])
            f.accel_y     = float(parts[7])
            f.accel_z     = float(parts[8])
            f.rssi        = int(parts[9])
            f.bitrate_kbps= int(parts[10])
            return f
        except Exception:
            return None


class MissionState:
    FALLING            = "FALLING"
    PARACHUTE_DEPLOYED = "PARACHUTE_DEPLOYED"
    PARACHUTE_DETACHED = "PARACHUTE_DETACHED"
    LANDED             = "LANDED"


# ─────────────────────────────────────────────────────────────────────────────
#  DATA HANDLER  (thread-safe)
# ─────────────────────────────────────────────────────────────────────────────
class DataHandler(QObject):
    """Receives raw TelemetryFrame objects, manages state machine, emits signals."""

    telemetry_updated = pyqtSignal(object)   # TelemetryFrame
    state_changed     = pyqtSignal(str)       # MissionState.*
    landing_confirmed = pyqtSignal(bool, str) # success, reason

    def __init__(self):
        super().__init__()
        self.log: List[TelemetryFrame] = []
        self._state = MissionState.FALLING
        self._land_start: Optional[float] = None
        self._lock = threading.Lock()
        self._landed = False

    @property
    def state(self) -> str:
        return self._state

    def ingest(self, frame: TelemetryFrame):
        with self._lock:
            frame.timestamp = time.time()
            self.log.append(frame)
            self._update_state(frame)
        self.telemetry_updated.emit(frame)

    def _update_state(self, f: TelemetryFrame):
        if self._landed:
            return

        # Simple state machine based on altitude + velocity
        prev = self._state

        if f.altitude <= LAND_ALT_THRESH:
            if self._land_start is None:
                self._land_start = time.time()
            elif time.time() - self._land_start >= LAND_STABLE_SEC:
                self._state = MissionState.LANDED
                self._landed = True
                self.landing_confirmed.emit(True, "NOMINAL")
        else:
            self._land_start = None
            if f.altitude < 30 and self._state == MissionState.PARACHUTE_DEPLOYED:
                self._state = MissionState.PARACHUTE_DETACHED
            elif f.altitude < 60 and self._state == MissionState.FALLING:
                self._state = MissionState.PARACHUTE_DEPLOYED

        if self._state != prev:
            self.state_changed.emit(self._state)

    def get_log_copy(self) -> List[TelemetryFrame]:
        with self._lock:
            return list(self.log)


# ─────────────────────────────────────────────────────────────────────────────
#  SERIAL WORKER
# ─────────────────────────────────────────────────────────────────────────────
class SerialWorker(QThread):
    """Reads lines from a serial port and emits parsed TelemetryFrame objects."""

    frame_received = pyqtSignal(object)
    error_occurred = pyqtSignal(str)

    def __init__(self, port: str, baud: int = 115200):
        super().__init__()
        self.port = port
        self.baud = baud
        self._running = False

    def run(self):
        try:
            import serial
            self._running = True
            ser = serial.Serial(self.port, self.baud, timeout=1)
            while self._running:
                line = ser.readline().decode("utf-8", errors="replace").strip()
                if line:
                    frame = TelemetryFrame.from_csv(line)
                    if frame:
                        self.frame_received.emit(frame)
            ser.close()
        except Exception as e:
            self.error_occurred.emit(str(e))

    def stop(self):
        self._running = False
        self.wait()


# ─────────────────────────────────────────────────────────────────────────────
#  SIMULATION WORKER  (replaces SerialWorker when --simulate)
# ─────────────────────────────────────────────────────────────────────────────
class SimulationWorker(QThread):
    """Generates realistic synthetic descent data."""

    frame_received = pyqtSignal(object)

    def __init__(self):
        super().__init__()
        self._running = False

    def run(self):
        self._running = True
        alt  = 100.0
        vel  = -3.5
        bat  = 12.8
        temp = 18.5
        hum  = 45.0
        ax, ay, az = 0.05, -9.81, 0.02

        while self._running and alt > -0.5:
            # Physics step
            dt = 0.5
            drag = 0.02 if alt > 60 else 0.15   # parachute opens below 60 m
            vel  = vel - 9.81 * dt + drag * vel * vel * dt * (-1 if vel < 0 else 1)
            vel  = max(vel, -15.0)
            alt  = max(0.0, alt + vel * dt)
            bat  = max(11.0, bat - 0.001)
            temp += random.gauss(0, 0.05)
            hum  += random.gauss(0, 0.1)
            ax    = random.gauss(0.0, 0.05)
            ay    = -9.81 + random.gauss(0, 0.1)
            az    = random.gauss(0.0, 0.05)
            rssi  = -65 + int(random.gauss(0, 3))

            f = TelemetryFrame(
                altitude    = round(alt, 1),
                velocity    = round(vel, 2),
                battery_v   = round(bat, 2),
                battery_pct = round((bat - 11.0) / (12.8 - 11.0) * 100, 1),
                temperature = round(temp, 1),
                humidity    = round(max(0, min(100, hum)), 1),
                accel_x     = round(ax, 2),
                accel_y     = round(ay, 2),
                accel_z     = round(az, 2),
                rssi        = rssi,
                bitrate_kbps= 512,
            )
            self.frame_received.emit(f)
            self.msleep(500)

        # Emit a final grounded frame a few times
        for _ in range(10):
            f.altitude = 0.0
            f.velocity = 0.0
            self.frame_received.emit(f)
            self.msleep(500)

    def stop(self):
        self._running = False
        self.wait()


# ─────────────────────────────────────────────────────────────────────────────
#  CUSTOM WIDGETS
# ─────────────────────────────────────────────────────────────────────────────

class SchematicBackground(QWidget):
    """Draws the schematic-style dark background grid."""

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Background gradient
        grad = QLinearGradient(0, 0, self.width(), self.height())
        grad.setColorAt(0, QColor("#050C12"))
        grad.setColorAt(1, QColor("#03080F"))
        p.fillRect(self.rect(), grad)

        # Subtle grid
        pen = QPen(QColor(0, 100, 130, 25), 1)
        p.setPen(pen)
        step = 40
        for x in range(0, self.width(), step):
            p.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), step):
            p.drawLine(0, y, self.width(), y)

        # Corner accent lines
        pen2 = QPen(CYAN_DIM, 1)
        p.setPen(pen2)
        sz = 60
        # top-left
        p.drawLine(10, 10, 10 + sz, 10)
        p.drawLine(10, 10, 10, 10 + sz)
        # top-right
        p.drawLine(self.width()-10, 10, self.width()-10-sz, 10)
        p.drawLine(self.width()-10, 10, self.width()-10, 10+sz)
        # bottom-left
        p.drawLine(10, self.height()-10, 10+sz, self.height()-10)
        p.drawLine(10, self.height()-10, 10, self.height()-10-sz)
        # bottom-right
        p.drawLine(self.width()-10, self.height()-10, self.width()-10-sz, self.height()-10)
        p.drawLine(self.width()-10, self.height()-10, self.width()-10, self.height()-10-sz)


class GlowLabel(QLabel):
    """QLabel with optional cyan glow effect on text."""

    def __init__(self, text="", glow_color: QColor = CYAN, parent=None):
        super().__init__(text, parent)
        self._glow = glow_color

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setFont(self.font())

        # Glow pass (blurred shadow)
        for offset in range(4, 0, -1):
            alpha = 30 + (4 - offset) * 15
            col = QColor(self._glow.red(), self._glow.green(), self._glow.blue(), alpha)
            p.setPen(col)
            r = self.rect().adjusted(-offset, -offset, offset, offset)
            p.drawText(r, self.alignment(), self.text())

        # Main text
        p.setPen(self.palette().color(QPalette.ColorRole.WindowText))
        p.drawText(self.rect(), self.alignment(), self.text())


class TrendChart(QWidget):
    """Mini rolling line chart."""

    def __init__(self, max_val=100.0, color=CYAN, parent=None):
        super().__init__(parent)
        self._data: deque = deque(maxlen=TREND_LEN)
        self._max = max_val
        self._color = color
        self.setMinimumSize(80, 40)

    def push(self, value: float):
        self._data.append(value)
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Background
        p.fillRect(self.rect(), QColor(0, 229, 255, 8))
        pen_border = QPen(QColor(0, 229, 255, 40), 1)
        p.setPen(pen_border)
        p.drawRect(self.rect().adjusted(0, 0, -1, -1))

        if len(self._data) < 2:
            return

        w, h = self.width(), self.height()
        pts = list(self._data)
        n = len(pts)

        path = QPainterPath()
        for i, v in enumerate(pts):
            x = int(i * (w - 4) / (n - 1)) + 2
            y = int(h - 4 - (v / max(self._max, 0.001)) * (h - 8)) + 2
            y = max(2, min(h - 2, y))
            if i == 0:
                path.moveTo(x, y)
            else:
                path.lineTo(x, y)

        pen = QPen(self._color, 1.5)
        p.setPen(pen)
        p.drawPath(path)

        # Glow
        pen_glow = QPen(QColor(self._color.red(), self._color.green(),
                               self._color.blue(), 50), 3)
        p.setPen(pen_glow)
        p.drawPath(path)


class AltitudeBar(QWidget):
    """Vertical altitude bar with markers and glow."""

    def __init__(self, max_alt=ALT_MAX_M, parent=None):
        super().__init__(parent)
        self._alt  = max_alt
        self._max  = max_alt
        self.setMinimumWidth(50)
        self.setMinimumHeight(300)

    def set_altitude(self, alt: float):
        self._alt = max(0.0, min(self._max, alt))
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        bar_x  = w // 2 - 10
        bar_w  = 20
        pad_t  = 20
        pad_b  = 20
        bar_h  = h - pad_t - pad_b

        # Background bar
        bg_rect = QRectF(bar_x, pad_t, bar_w, bar_h)
        p.fillRect(bg_rect.toRect(), QColor(0, 50, 70, 120))

        # Filled portion
        frac = self._alt / self._max
        fill_h = bar_h * frac
        fill_y = pad_t + bar_h - fill_h
        fill_rect = QRectF(bar_x, fill_y, bar_w, fill_h)

        grad = QLinearGradient(0, fill_y + fill_h, 0, fill_y)
        grad.setColorAt(0, QColor(0, 160, 200, 200))
        grad.setColorAt(1, CYAN)
        p.fillRect(fill_rect.toRect(), grad)

        # Glow at top of bar
        glow = QRadialGradient(bar_x + bar_w/2, fill_y, 20)
        glow.setColorAt(0, QColor(0, 229, 255, 90))
        glow.setColorAt(1, QColor(0, 0, 0, 0))
        p.fillRect(int(bar_x - 15), int(fill_y - 15), bar_w + 30, 30, glow)

        # Border
        pen = QPen(CYAN_DIM, 1)
        p.setPen(pen)
        p.drawRect(bg_rect.toRect())

        # Altitude markers
        p.setFont(QFont(FONT_MONO, 7))
        markers = [0, 10, 25, 60, 100]
        for m in markers:
            y = pad_t + bar_h - int(bar_h * m / self._max)
            col = ORANGE if m in (60,) else TEXT_DIM
            pen_m = QPen(col, 1, Qt.PenStyle.DashLine)
            p.setPen(pen_m)
            p.drawLine(bar_x - 5, y, bar_x + bar_w + 5, y)
            p.setPen(col)
            p.drawText(0, y - 5, f"{m}m")

        # THRESHOLD label at 60 m
        thresh_y = pad_t + bar_h - int(bar_h * 60 / self._max)
        p.setPen(ORANGE)
        p.setFont(QFont(FONT_MONO, 7))
        p.drawText(bar_x + bar_w + 8, thresh_y + 4, "DISTANCE TO THRESHOLD")

        # Current value marker
        cur_y = pad_t + bar_h - int(bar_h * frac)
        p.setPen(QPen(CYAN, 2))
        p.drawLine(bar_x - 8, cur_y, bar_x + bar_w + 8, cur_y)


class AntennaWidget(QWidget):
    """Draws antenna + data-link lines."""

    def __init__(self, bitrate=512, parent=None):
        super().__init__(parent)
        self._bitrate = bitrate
        self._active  = True
        self._phase   = 0.0
        self.setMinimumSize(160, 120)

        timer = QTimer(self)
        timer.timeout.connect(self._animate)
        timer.start(80)

    def _animate(self):
        self._phase = (self._phase + 0.15) % (2 * math.pi)
        self.update()

    def set_link(self, active: bool, bitrate: int):
        self._active  = active
        self._bitrate = bitrate

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        cx, cy = w // 2, h - 20

        # Ground station base
        p.setPen(QPen(CYAN_DIM, 2))
        p.setBrush(QBrush(QColor(0, 50, 70, 80)))
        p.drawEllipse(cx - 20, cy - 6, 40, 12)

        # Antenna pole
        p.setPen(QPen(CYAN, 2))
        p.drawLine(cx, cy - 6, cx, cy - 35)

        # Antenna arms
        arms = [(-25, -15), (25, -15), (-15, -25), (15, -25)]
        for ax, ay in arms:
            p.drawLine(cx, cy - 35, cx + ax, cy - 35 + ay)

        # Animated radio waves
        if self._active:
            for i in range(3):
                r  = 18 + i * 14
                off = math.sin(self._phase - i * 0.8) * 0.3 + 0.7
                alpha = int(180 * off)
                p.setPen(QPen(QColor(0, 229, 255, alpha), 1.5))
                p.drawArc(cx - r, cy - 35 - r, r * 2, r * 2, 30 * 16, 120 * 16)

        # Labels
        p.setFont(QFont(FONT_MONO, 7))
        col = CYAN if self._active else GRAY
        p.setPen(col)
        p.drawText(cx - 35, cy + 16, "GROUND STATION")
        status = "STRONG/ACTIVE" if self._active else "OFFLINE"
        p.drawText(cx - 30, 12, "DATA LINK")
        p.setPen(CYAN if self._active else RED)
        p.drawText(cx - 25, 26, status)
        p.setPen(TEXT_DIM)
        p.drawText(cx - 20, 40, f"{self._bitrate} kbps")


class StateIndicator(QWidget):
    """Rounded pill-shaped state indicator."""

    def __init__(self, label: str, active: bool = False,
                 active_color: QColor = GREEN, parent=None):
        super().__init__(parent)
        self._label        = label
        self._active       = active
        self._active_color = active_color
        self.setFixedHeight(28)
        self.setMinimumWidth(180)

    def set_active(self, active: bool):
        self._active = active
        self.update()

    def set_label(self, label: str):
        self._label = label
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        col  = self._active_color if self._active else GRAY
        bg   = QColor(col.red(), col.green(), col.blue(), 30)

        path = QPainterPath()
        path.addRoundedRect(QRectF(0, 0, w, h), 14, 14)

        # Background fill
        p.fillPath(path, bg)

        # Border
        p.setPen(QPen(col, 1.5))
        p.drawPath(path)

        # Dot indicator
        dot_r = 7
        dot_x, dot_y = 14, h // 2
        dot_col = col if self._active else QColor(80, 80, 80)
        p.setBrush(QBrush(dot_col))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawEllipse(dot_x - dot_r, dot_y - dot_r, dot_r * 2, dot_r * 2)

        # Glow dot
        if self._active:
            glow = QRadialGradient(dot_x, dot_y, dot_r * 2)
            glow.setColorAt(0, QColor(col.red(), col.green(), col.blue(), 120))
            glow.setColorAt(1, QColor(0, 0, 0, 0))
            p.setBrush(glow)
            p.drawEllipse(dot_x - dot_r * 2, dot_y - dot_r * 2,
                          dot_r * 4, dot_r * 4)

        # Label
        p.setPen(col if self._active else TEXT_DIM)
        p.setFont(QFont(FONT_MONO, 8, QFont.Weight.Bold))
        p.drawText(QRectF(30, 0, w - 34, h),
                   Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft,
                   self._label)


class DataCard(QFrame):
    """Dark card with title and value display."""

    def __init__(self, title: str, value: str = "--", unit: str = "",
                 color: QColor = CYAN, parent=None):
        super().__init__(parent)
        self._color = color
        self.setStyleSheet(f"""
            QFrame {{
                background: rgba(13,31,45,200);
                border: 1px solid {color.name()}44;
                border-radius: 6px;
            }}
        """)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 5, 8, 5)
        layout.setSpacing(1)

        self._title_lbl = QLabel(title)
        self._title_lbl.setFont(QFont(FONT_MONO, 7))
        self._title_lbl.setStyleSheet(f"color: {TEXT_DIM.name()}; background: transparent; border: none;")

        self._value_lbl = GlowLabel(value, glow_color=color)
        self._value_lbl.setFont(QFont(FONT_MONO, 14, QFont.Weight.Bold))
        self._value_lbl.setStyleSheet(f"color: {color.name()}; background: transparent; border: none;")

        if unit:
            self._unit_lbl = QLabel(unit)
            self._unit_lbl.setFont(QFont(FONT_MONO, 7))
            self._unit_lbl.setStyleSheet(f"color: {TEXT_DIM.name()}; background: transparent; border: none;")
        else:
            self._unit_lbl = None

        layout.addWidget(self._title_lbl)
        layout.addWidget(self._value_lbl)
        if self._unit_lbl:
            layout.addWidget(self._unit_lbl)

    def set_value(self, value: str):
        self._value_lbl.setText(value)


class SmallDataCard(QFrame):
    """Compact card for secondary telemetry."""

    def __init__(self, title: str, value: str = "--", parent=None):
        super().__init__(parent)
        self.setStyleSheet("""
            QFrame {
                background: rgba(10,21,32,200);
                border: 1px solid rgba(0,229,255,30);
                border-radius: 4px;
            }
        """)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(0)

        t = QLabel(title)
        t.setFont(QFont(FONT_MONO, 6))
        t.setStyleSheet("color: #546E7A; background: transparent; border: none;")

        self._val = QLabel(value)
        self._val.setFont(QFont(FONT_MONO, 10, QFont.Weight.Bold))
        self._val.setStyleSheet("color: #00E5FF; background: transparent; border: none;")

        layout.addWidget(t)
        layout.addWidget(self._val)

    def set_value(self, v: str):
        self._val.setText(v)


# ─────────────────────────────────────────────────────────────────────────────
#  POST-LANDING ANALYSIS WINDOW
# ─────────────────────────────────────────────────────────────────────────────
class AnalysisWindow(QMainWindow):
    """Shows time-series graphs and a table of recorded telemetry."""

    def __init__(self, log: List[TelemetryFrame], parent=None):
        super().__init__(parent)
        self.setWindowTitle("Post-Landing Data Analysis")
        self.resize(1100, 700)
        self._log = log
        self._build_ui()

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        central.setStyleSheet("background: #070D11; color: #E0F7FA;")

        layout = QVBoxLayout(central)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        # Title
        title = QLabel("📊  POST-LANDING TELEMETRY ANALYSIS")
        title.setFont(QFont(FONT_MONO, 14, QFont.Weight.Bold))
        title.setStyleSheet("color: #00E5FF;")
        layout.addWidget(title)

        splitter = QSplitter(Qt.Orientation.Vertical)
        layout.addWidget(splitter, 1)

        # ── Charts ──────────────────────────────────────────────────────────
        if HAS_MPL and self._log:
            ts   = [f.timestamp - self._log[0].timestamp for f in self._log]
            alts = [f.altitude    for f in self._log]
            vels = [f.velocity    for f in self._log]
            bats = [f.battery_v   for f in self._log]
            temps= [f.temperature for f in self._log]

            fig  = Figure(figsize=(10, 3.5), facecolor="#0A1520")
            axes = [fig.add_subplot(1, 4, i+1) for i in range(4)]
            datasets = [
                (alts,  "Altitude (m)",       "#00E5FF"),
                (vels,  "Velocity (m/s)",      "#FF6D00"),
                (bats,  "Battery (V)",         "#00E676"),
                (temps, "Temperature (°C)",    "#FFD740"),
            ]
            for ax, (data, ylabel, col) in zip(axes, datasets):
                ax.set_facecolor("#070D11")
                ax.plot(ts, data, color=col, linewidth=1.5)
                ax.set_xlabel("Time (s)", color="#546E7A", fontsize=7)
                ax.set_ylabel(ylabel,     color=col,       fontsize=7)
                ax.tick_params(colors="#546E7A", labelsize=6)
                for spine in ax.spines.values():
                    spine.set_edgecolor("#1C2A30")
                ax.grid(color="#1C2A30", linestyle="--", linewidth=0.5)

            fig.tight_layout(pad=1.5)
            canvas = FigureCanvas(fig)
            canvas.setStyleSheet("background: #0A1520;")
            splitter.addWidget(canvas)
        else:
            lbl = QLabel("matplotlib not installed – install it with:  pip install matplotlib")
            lbl.setStyleSheet("color: #FF6D00; padding: 20px;")
            splitter.addWidget(lbl)

        # ── Table ────────────────────────────────────────────────────────────
        cols = ["Time (s)", "Alt (m)", "Vel (m/s)", "Bat (V)", "Bat (%)",
                "Temp (°C)", "Hum (%)", "Ax (g)", "Ay (g)", "Az (g)"]
        table = QTableWidget(len(self._log), len(cols))
        table.setHorizontalHeaderLabels(cols)
        table.setStyleSheet("""
            QTableWidget { background:#0A1520; color:#90A4AE;
                           gridline-color:#1C2A30; border:none; font-family:Courier New; font-size:10px; }
            QHeaderView::section { background:#0D1F2D; color:#00E5FF;
                                   border:1px solid #1C2A30; padding:3px; font-size:10px; }
        """)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        table.verticalHeader().setDefaultSectionSize(18)
        if self._log:
            t0 = self._log[0].timestamp
            for row, f in enumerate(self._log):
                vals = [
                    f"{f.timestamp - t0:.1f}",
                    f"{f.altitude:.1f}", f"{f.velocity:.2f}",
                    f"{f.battery_v:.2f}", f"{f.battery_pct:.1f}",
                    f"{f.temperature:.1f}", f"{f.humidity:.1f}",
                    f"{f.accel_x:.2f}", f"{f.accel_y:.2f}", f"{f.accel_z:.2f}",
                ]
                for col, v in enumerate(vals):
                    item = QTableWidgetItem(v)
                    item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                    table.setItem(row, col, item)
        splitter.addWidget(table)


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN DASHBOARD WINDOW
# ─────────────────────────────────────────────────────────────────────────────
class MissionControlWindow(QMainWindow):

    def __init__(self, worker, data_handler: DataHandler):
        super().__init__()
        self._worker       = worker
        self._handler      = data_handler
        self._analysis_win = None
        self._landed       = False

        self.setWindowTitle("CubeSat Mission Control  ·  UNAL / CAN SAT TELEMETRY")
        self.resize(1280, 720)
        self.setMinimumSize(1024, 600)

        self._build_ui()
        self._connect_signals()
        self._start_clock()

    # ── UI Construction ────────────────────────────────────────────────────

    def _build_ui(self):
        root = SchematicBackground()
        self.setCentralWidget(root)
        root_layout = QHBoxLayout(root)
        root_layout.setContentsMargins(20, 20, 20, 20)
        root_layout.setSpacing(14)

        root_layout.addWidget(self._build_left_panel(),   stretch=3)
        root_layout.addWidget(self._build_center_panel(), stretch=2)
        root_layout.addWidget(self._build_right_panel(),  stretch=4)

    def _panel_base(self) -> QFrame:
        f = QFrame()
        f.setStyleSheet("""
            QFrame {
                background: qlineargradient(x1:0,y1:0,x2:0,y2:1,
                    stop:0 rgba(12,26,38,230), stop:1 rgba(7,16,24,230));
                border: 1px solid rgba(0,229,255,35);
                border-radius: 10px;
            }
        """)
        return f

    def _section_title(self, text: str, color: QColor = CYAN) -> QLabel:
        lbl = QLabel(text)
        lbl.setFont(QFont(FONT_MONO, 8, QFont.Weight.Bold))
        lbl.setStyleSheet(f"color: {color.name()}; background: transparent; border: none; "
                          f"letter-spacing: 2px;")
        return lbl

    # ── LEFT PANEL ────────────────────────────────────────────────────────
    def _build_left_panel(self) -> QFrame:
        panel = self._panel_base()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(10)

        # Title
        title = QLabel("CUBESAT MISSION CONTROL")
        title.setFont(QFont(FONT_MONO, 10, QFont.Weight.Bold))
        title.setStyleSheet("color: #E0F7FA; background: transparent; border: none; letter-spacing: 2px;")
        layout.addWidget(title)

        # CubeSat render placeholder
        self._cubesat_lbl = QLabel()
        self._cubesat_lbl.setFixedHeight(170)
        self._cubesat_lbl.setStyleSheet("""
            QLabel {
                background: rgba(0,50,70,60);
                border: 1px solid rgba(0,229,255,30);
                border-radius: 8px;
                color: #546E7A;
            }
        """)
        self._cubesat_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        pix = self._make_cubesat_placeholder()
        self._cubesat_lbl.setPixmap(pix)
        layout.addWidget(self._cubesat_lbl)

        # Mission State
        layout.addWidget(self._section_title("MISSION STATE:"))
        self._state_descent_lbl = QLabel("DESCENT INITIATED")
        self._state_descent_lbl.setFont(QFont(FONT_MONO, 13, QFont.Weight.Bold))
        self._state_descent_lbl.setStyleSheet("color: #00E5FF; background: transparent; border: none;")
        layout.addWidget(self._state_descent_lbl)

        # Indicators
        self._ind_falling  = StateIndicator("FALLING",             active=True,  active_color=GREEN)
        self._ind_chute_dep= StateIndicator("PARACHUTE DEPLOYED",  active=False, active_color=CYAN)
        self._ind_chute_det= StateIndicator("PARACHUTE DETACHED",  active=False, active_color=ORANGE)
        self._ind_landing  = StateIndicator("LANDING STATUS: —",   active=False, active_color=ORANGE)

        for ind in [self._ind_falling, self._ind_chute_dep,
                    self._ind_chute_det, self._ind_landing]:
            layout.addWidget(ind)

        layout.addStretch()

        # Clock
        self._clock_lbl = QLabel("00:00:00")
        self._clock_lbl.setFont(QFont(FONT_MONO, 11, QFont.Weight.Bold))
        self._clock_lbl.setStyleSheet("color: #546E7A; background: transparent; border: none;")
        self._clock_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self._clock_lbl)

        return panel

    def _make_cubesat_placeholder(self) -> QPixmap:
        """Draw a simple isometric CubeSat wireframe."""
        pix = QPixmap(240, 160)
        pix.fill(Qt.GlobalColor.transparent)
        p = QPainter(pix)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        cx, cy = 120, 85
        # Isometric cube faces
        def iso(x, y, z, scale=40):
            px = cx + (x - y) * scale * 0.866
            py = cy + (x + y) * scale * 0.5 - z * scale
            return QPointF(px, py)

        # Top face
        top = QPolygonF([iso(0,0,1), iso(1,0,1), iso(1,1,1), iso(0,1,1)])
        p.setBrush(QBrush(QColor(0,80,100,140)))
        p.setPen(QPen(CYAN, 1.5))
        p.drawPolygon(top)

        # Left face
        left = QPolygonF([iso(0,1,0), iso(0,1,1), iso(0,0,1), iso(0,0,0)])
        p.setBrush(QBrush(QColor(0,50,70,140)))
        p.drawPolygon(left)

        # Right face
        right = QPolygonF([iso(1,1,0), iso(1,1,1), iso(0,1,1), iso(0,1,0)])
        p.setBrush(QBrush(QColor(0,40,60,140)))
        p.drawPolygon(right)

        # Solar panel hints
        panel_pts_l = [iso(-0.6, 0.5, 0.7), iso(-0.1, 0.5, 0.7),
                       iso(-0.1, 0.5, 0.3), iso(-0.6, 0.5, 0.3)]
        panel_pts_r = [iso(1.1, 0.5, 0.7), iso(1.6, 0.5, 0.7),
                       iso(1.6, 0.5, 0.3), iso(1.1, 0.5, 0.3)]
        for pts in [panel_pts_l, panel_pts_r]:
            poly = QPolygonF(pts)
            p.setBrush(QBrush(QColor(0, 100, 160, 160)))
            p.setPen(QPen(CYAN_DIM, 1))
            p.drawPolygon(poly)

        # Antenna
        ant_base = iso(0.5, 0.5, 1)
        ant_top  = iso(0.5, 0.5, 1.6)
        p.setPen(QPen(CYAN, 2))
        p.drawLine(ant_base, ant_top)

        p.end()
        return pix

    # ── CENTER PANEL ──────────────────────────────────────────────────────
    def _build_center_panel(self) -> QFrame:
        panel = self._panel_base()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(10)

        layout.addWidget(self._section_title("ALTITUDE TRACK"))

        inner = QHBoxLayout()
        inner.setSpacing(10)

        self._alt_bar = AltitudeBar(max_alt=ALT_MAX_M)
        inner.addWidget(self._alt_bar)

        self._antenna = AntennaWidget()
        inner.addWidget(self._antenna)

        layout.addLayout(inner, 1)

        # Alt numeric under bar
        self._center_alt_lbl = QLabel("ALT: -- m")
        self._center_alt_lbl.setFont(QFont(FONT_MONO, 9, QFont.Weight.Bold))
        self._center_alt_lbl.setStyleSheet("color: #00E5FF; background: transparent; border: none;")
        self._center_alt_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self._center_alt_lbl)

        return panel

    # ── RIGHT PANEL ────────────────────────────────────────────────────────
    def _build_right_panel(self) -> QFrame:
        panel = self._panel_base()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(8)

        # Header row: title + logo
        hdr = QHBoxLayout()
        hdr.addWidget(self._section_title("TELEMETRY DASHBOARD"))
        hdr.addStretch()
        logo_lbl = self._make_logo_widget()
        hdr.addWidget(logo_lbl)
        layout.addLayout(hdr)

        # ── PRIMARY READOUT ─────────────────────────────────────────────
        layout.addWidget(self._section_title("PRIMARY READOUT", CYAN_DIM))
        prim_row = QHBoxLayout()
        prim_row.setSpacing(8)

        alt_card = QFrame()
        alt_card.setStyleSheet("""
            QFrame {
                background: rgba(0,50,80,120);
                border: 1px solid rgba(0,229,255,60);
                border-radius: 6px;
            }
        """)
        alt_inner = QVBoxLayout(alt_card)
        alt_inner.setContentsMargins(10, 6, 10, 6)
        alt_inner.setSpacing(0)

        alt_title = QLabel("ALTITUDE:")
        alt_title.setFont(QFont(FONT_MONO, 8))
        alt_title.setStyleSheet("color: #00E5FF; background: transparent; border: none;")

        self._alt_value = GlowLabel("84 m", glow_color=CYAN)
        self._alt_value.setFont(QFont(FONT_MONO, 36, QFont.Weight.Bold))
        self._alt_value.setStyleSheet("color: #00E5FF; background: transparent; border: none;")

        alt_inner.addWidget(alt_title)
        alt_inner.addWidget(self._alt_value)
        prim_row.addWidget(alt_card, 2)

        trend_frame = QFrame()
        trend_frame.setStyleSheet("background: transparent; border: none;")
        trend_v = QVBoxLayout(trend_frame)
        trend_v.setContentsMargins(0, 0, 0, 0)
        trend_label = QLabel("TREND")
        trend_label.setFont(QFont(FONT_MONO, 7))
        trend_label.setStyleSheet("color: #546E7A; background: transparent; border: none;")
        self._trend = TrendChart(max_val=ALT_MAX_M)
        self._trend.setMinimumSize(100, 55)
        trend_v.addWidget(trend_label)
        trend_v.addWidget(self._trend, 1)
        prim_row.addWidget(trend_frame, 1)

        layout.addLayout(prim_row)

        # Velocity + Battery row
        vb_row = QHBoxLayout()
        self._vel_card = DataCard("DESCENT VELOCITY", "-4.2 m/s", color=ORANGE)
        self._bat_card = DataCard("BATTERY", "12.8 V (88%)", color=GREEN)
        vb_row.addWidget(self._vel_card)
        vb_row.addWidget(self._bat_card)
        layout.addLayout(vb_row)

        # ── SECONDARY TELEMETRY ─────────────────────────────────────────
        layout.addWidget(self._section_title("SECONDARY TELEMETRY", CYAN_DIM))

        sec_row = QHBoxLayout()
        self._temp_card = SmallDataCard("TEMPERATURE", "18.5 °C")
        self._hum_card  = SmallDataCard("HUMIDITY",    "45 %")
        sec_row.addWidget(self._temp_card)
        sec_row.addWidget(self._hum_card)
        layout.addLayout(sec_row)

        # Acceleration
        accel_frame = QFrame()
        accel_frame.setStyleSheet("""
            QFrame {
                background: rgba(10,21,32,200);
                border: 1px solid rgba(0,229,255,30);
                border-radius: 4px;
            }
        """)
        accel_layout = QVBoxLayout(accel_frame)
        accel_layout.setContentsMargins(8, 6, 8, 6)
        accel_layout.setSpacing(2)

        accel_title = QLabel("3-AXIS ACCELERATION")
        accel_title.setFont(QFont(FONT_MONO, 7))
        accel_title.setStyleSheet("color: #546E7A; background: transparent; border: none;")
        accel_layout.addWidget(accel_title)

        accel_row = QHBoxLayout()
        self._ax_lbl = QLabel("X:  0.1 g")
        self._ay_lbl = QLabel("Y: -9.8 g")
        self._az_lbl = QLabel("Z:  0.0 g")
        for lbl in [self._ax_lbl, self._ay_lbl, self._az_lbl]:
            lbl.setFont(QFont(FONT_MONO, 10, QFont.Weight.Bold))
            lbl.setStyleSheet("color: #FFD740; background: transparent; border: none;")
            accel_row.addWidget(lbl)
        accel_layout.addLayout(accel_row)
        layout.addWidget(accel_frame)

        layout.addStretch()

        # ── POST-LANDING BUTTON ────────────────────────────────────────
        self._analysis_btn = QPushButton("POST-LANDING\nDATA ANALYSIS")
        self._analysis_btn.setEnabled(False)
        self._analysis_btn.setFont(QFont(FONT_MONO, 11, QFont.Weight.Bold))
        self._analysis_btn.setFixedHeight(60)
        self._analysis_btn.setStyleSheet("""
            QPushButton {
                background: rgba(0,229,255,20);
                color: rgba(0,229,255,80);
                border: 1px solid rgba(0,229,255,50);
                border-radius: 6px;
            }
            QPushButton:enabled {
                background: rgba(0,229,255,40);
                color: #00E5FF;
                border: 1px solid rgba(0,229,255,180);
            }
            QPushButton:enabled:hover {
                background: rgba(0,229,255,70);
            }
            QPushButton:enabled:pressed {
                background: rgba(0,229,255,100);
            }
        """)
        self._analysis_btn.clicked.connect(self._open_analysis)
        layout.addWidget(self._analysis_btn)

        return panel

    def _make_logo_widget(self) -> QLabel:
        """Draw a circular mission-patch logo."""
        pix = QPixmap(70, 70)
        pix.fill(Qt.GlobalColor.transparent)
        p = QPainter(pix)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Outer ring
        p.setPen(QPen(CYAN, 2))
        p.setBrush(QBrush(QColor(7, 13, 17, 200)))
        p.drawEllipse(2, 2, 66, 66)

        # Inner ring
        p.setPen(QPen(ORANGE, 1))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(8, 8, 54, 54)

        # Rocket icon (simple)
        p.setPen(QPen(CYAN, 2))
        p.drawLine(35, 15, 35, 45)
        p.drawLine(30, 35, 35, 15)
        p.drawLine(40, 35, 35, 15)
        p.drawLine(28, 40, 32, 35)
        p.drawLine(42, 40, 38, 35)

        # Text arcs
        p.setFont(QFont(FONT_MONO, 5, QFont.Weight.Bold))
        p.setPen(TEXT_DIM)
        p.drawText(QRectF(4, 46, 62, 20), Qt.AlignmentFlag.AlignCenter, "UNAL · CAN SAT")
        p.drawText(QRectF(4, 4,  62, 14), Qt.AlignmentFlag.AlignCenter, "TELEMETRY")

        p.end()
        lbl = QLabel()
        lbl.setPixmap(pix)
        lbl.setFixedSize(70, 70)
        return lbl

    # ── Signal Connections ─────────────────────────────────────────────────
    def _connect_signals(self):
        self._worker.frame_received.connect(self._handler.ingest)
        self._handler.telemetry_updated.connect(self._on_telemetry)
        self._handler.state_changed.connect(self._on_state_changed)
        self._handler.landing_confirmed.connect(self._on_landing)

    # ── Clock ───────────────────────────────────────────────────────────────
    def _start_clock(self):
        self._start_time = time.time()
        t = QTimer(self)
        t.timeout.connect(self._tick_clock)
        t.start(1000)

    def _tick_clock(self):
        elapsed = int(time.time() - self._start_time)
        h, r = divmod(elapsed, 3600)
        m, s = divmod(r, 60)
        self._clock_lbl.setText(f"{h:02d}:{m:02d}:{s:02d}")

    # ── Telemetry Update ────────────────────────────────────────────────────
    def _on_telemetry(self, f: TelemetryFrame):
        # Altitude
        self._alt_value.setText(f"{f.altitude:.0f} m")
        self._center_alt_lbl.setText(f"ALT: {f.altitude:.1f} m")
        self._alt_bar.set_altitude(f.altitude)
        self._trend.push(f.altitude)

        # Velocity + Battery
        self._vel_card.set_value(f"{f.velocity:+.1f} m/s")
        self._bat_card.set_value(f"{f.battery_v:.1f} V ({f.battery_pct:.0f}%)")

        # Secondary
        self._temp_card.set_value(f"{f.temperature:.1f} °C")
        self._hum_card.set_value(f"{f.humidity:.0f} %")

        # Acceleration
        self._ax_lbl.setText(f"X: {f.accel_x:+.1f} g")
        self._ay_lbl.setText(f"Y: {f.accel_y:+.1f} g")
        self._az_lbl.setText(f"Z: {f.accel_z:+.1f} g")

        # Antenna link quality from RSSI
        active  = f.rssi > -90
        self._antenna.set_link(active, f.bitrate_kbps)

    # ── State Machine Updates ───────────────────────────────────────────────
    def _on_state_changed(self, state: str):
        self._ind_falling.set_active(state == MissionState.FALLING)
        self._ind_chute_dep.set_active(state == MissionState.PARACHUTE_DEPLOYED)
        self._ind_chute_det.set_active(state == MissionState.PARACHUTE_DETACHED)

        labels = {
            MissionState.FALLING:            "DESCENT INITIATED",
            MissionState.PARACHUTE_DEPLOYED: "PARACHUTE DEPLOYED",
            MissionState.PARACHUTE_DETACHED: "PARACHUTE DETACHED",
            MissionState.LANDED:             "LANDING CONFIRMED",
        }
        self._state_descent_lbl.setText(labels.get(state, state))

    def _on_landing(self, success: bool, reason: str):
        self._landed = True
        label = f"LANDING STATUS: {'SUCCESSFUL' if success else 'FAILED'}"
        if not success:
            label += f" ({reason})"
        self._ind_landing.set_active(True)
        self._ind_landing.set_label(label)
        self._ind_landing._active_color = GREEN if success else RED
        self._ind_landing.update()
        self._analysis_btn.setEnabled(True)

    # ── Post-Landing Analysis ───────────────────────────────────────────────
    def _open_analysis(self):
        log = self._handler.get_log_copy()
        self._analysis_win = AnalysisWindow(log, self)
        self._analysis_win.show()

    # ── Cleanup ─────────────────────────────────────────────────────────────
    def closeEvent(self, event):
        self._worker.stop()
        super().closeEvent(event)


# ─────────────────────────────────────────────────────────────────────────────
#  ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────
def parse_args():
    parser = argparse.ArgumentParser(description="CubeSat Mission Control Dashboard")
    parser.add_argument("--port",     default=None,  help="Serial port (e.g. COM3 or /dev/ttyUSB0)")
    parser.add_argument("--baud",     default=115200, type=int, help="Baud rate")
    parser.add_argument("--simulate", action="store_true", help="Use simulated data (default if no port)")
    return parser.parse_args()


def main():
    args = parse_args()

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    # Dark palette for native elements
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window,          QColor("#070D11"))
    palette.setColor(QPalette.ColorRole.WindowText,      QColor("#E0F7FA"))
    palette.setColor(QPalette.ColorRole.Base,            QColor("#0A1520"))
    palette.setColor(QPalette.ColorRole.AlternateBase,   QColor("#0D1F2D"))
    palette.setColor(QPalette.ColorRole.Text,            QColor("#E0F7FA"))
    palette.setColor(QPalette.ColorRole.Button,          QColor("#0D1F2D"))
    palette.setColor(QPalette.ColorRole.ButtonText,      QColor("#E0F7FA"))
    palette.setColor(QPalette.ColorRole.Highlight,       QColor("#00E5FF"))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor("#070D11"))
    app.setPalette(palette)

    # Choose worker
    if args.port and not args.simulate:
        worker = SerialWorker(args.port, args.baud)
    else:
        if not args.port and not args.simulate:
            print("[INFO] No port specified – starting in simulation mode.")
        worker = SimulationWorker()

    handler = DataHandler()
    window  = MissionControlWindow(worker, handler)
    window.show()
    worker.start()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
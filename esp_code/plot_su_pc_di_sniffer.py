"""
dis_plot.py
===========
Riceve dati da Arduino via seriale e li plotta in tempo reale,
replicando il comportamento della funzione MATLAB dis().

Formato atteso da Arduino (una riga CSV per campione):
  x,y,sxu_x,sxu_y,sxl_x,sxl_y,dxu_x,dxu_y,dxl_x,dxl_y

Dipendenze:
  pip install pyserial matplotlib

Utilizzo:
  python dis_plot.py                   # porta auto-rilevata
  python dis_plot.py --port COM3       # Windows
  python dis_plot.py --port /dev/ttyUSB0  # Linux
  python dis_plot.py --port /dev/cu.usbmodem14101  # macOS
"""

import argparse
import sys
import threading
import queue
import time

import serial
import serial.tools.list_ports
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# ── Configurazione ────────────────────────────────────────────────────────────
BAUD_RATE   = 115200
SQUARE_CX   = 8.5
SQUARE_CY   = 8.5
SQUARE_L    = 17.0
AXIS_MARGIN = 1.5           # spazio extra oltre il quadrato
MAX_POINTS  = 300           # punti satellite conservati in memoria
UPDATE_MS   = 50            # intervallo refresh grafico (ms)
# ─────────────────────────────────────────────────────────────────────────────


def find_arduino_port() -> str | None:
    """Cerca automaticamente una porta Arduino."""
    arduino_keywords = ["arduino", "ch340", "cp210", "ftdi", "usbmodem", "usbserial"]
    for port in serial.tools.list_ports.comports():
        desc = (port.description + port.manufacturer if port.manufacturer else port.description).lower()
        if any(kw in desc for kw in arduino_keywords):
            return port.device
    # fallback: prima porta disponibile
    ports = serial.tools.list_ports.comports()
    return ports[0].device if ports else None


def serial_reader(port: str, baud: int, data_queue: queue.Queue, stop_event: threading.Event):
    """Thread: legge righe dalla seriale e le mette in coda."""
    try:
        ser = serial.Serial(port, baud, timeout=1)
        print(f"[OK] Connesso a {port} @ {baud} baud")
        ser.reset_input_buffer()
        while not stop_event.is_set():
            try:
                raw = ser.readline()
                if raw:
                    line = raw.decode("utf-8", errors="ignore").strip()
                    if line:
                        data_queue.put(line)
            except serial.SerialException as e:
                print(f"[ERRORE seriale] {e}")
                break
        ser.close()
    except serial.SerialException as e:
        print(f"[ERRORE apertura porta] {e}")
        stop_event.set()


def parse_line(line: str):
    """Parsa una riga CSV in (sxu, sxl, dxu, dxl) o None."""
    try:
        vals = [float(v) for v in line.split(",")]
        if len(vals) != 8:
            return None
        sxu = (vals[0], vals[1])
        sxl = (vals[2], vals[3])
        dxu = (vals[4], vals[5])
        dxl = (vals[6], vals[7])
        return sxu, sxl, dxu, dxl
    except ValueError:
        return None


class LivePlotter:
    """Gestisce il grafico matplotlib in tempo reale."""

    def __init__(self):
        matplotlib.rcParams["toolbar"] = "None"
        self.fig, self.ax = plt.subplots(figsize=(9, 8), facecolor="#0d0d0d")
        self.ax.set_facecolor("#0d0d0d")
        self.fig.canvas.manager.set_window_title("dis() – Live Plot")

        # ── Stile griglia ──
        self.ax.set_xlim(-AXIS_MARGIN, SQUARE_L + AXIS_MARGIN)
        self.ax.set_ylim(-AXIS_MARGIN, SQUARE_L + AXIS_MARGIN)
        self.ax.set_aspect("equal")
        self.ax.grid(True, color="#2a2a2a", linewidth=0.7, linestyle="--")
        self.ax.tick_params(colors="#888888")
        for spine in self.ax.spines.values():
            spine.set_edgecolor("#333333")

        # ── Quadrato blu ──
        half = SQUARE_L / 2
        rect = patches.Rectangle(
            (SQUARE_CX - half, SQUARE_CY - half),
            SQUARE_L, SQUARE_L,
            linewidth=2, edgecolor="blue", facecolor="none", zorder=2
        )
        self.ax.add_patch(rect)

        # ── Scatter artists (aggiornati in-place) ──
        # Satellliti: array di punti storici
        self.sc_sxu = self.ax.scatter([], [], c="lime",    s=10, zorder=3, label="sxu")
        self.sc_sxl = self.ax.scatter([], [], c="magenta", s=10, zorder=3, label="sxl")
        self.sc_dxu = self.ax.scatter([], [], c="cyan",    s=10, zorder=3, label="dxu")
        self.sc_dxl = self.ax.scatter([], [], c="yellow",  s=10, zorder=3, label="dxl")

        self.ax.legend(loc="upper left", facecolor="#1a1a1a",
                       edgecolor="#444", labelcolor="white", fontsize=8)

        # ── Buffer punti satellite ──
        self.buf: dict[str, list] = {k: [] for k in ("sxu", "sxl", "dxu", "dxl")}

        self.fig.tight_layout()

    def update(self, data_queue: queue.Queue):
        """Svuota la coda e aggiorna il grafico (chiamato dal timer matplotlib)."""
        updated = False
        last = None

        while not data_queue.empty():
            try:
                line = data_queue.get_nowait()
            except queue.Empty:
                break
            parsed = parse_line(line)
            if parsed:
                last = parsed
                updated = True

        if updated and last:
            sxu, sxl, dxu, dxl = last

            # Aggiorna buffers
            for key, pt in zip(("sxu", "sxl", "dxu", "dxl"), (sxu, sxl, dxu, dxl)):
                self.buf[key].append(pt)
                if len(self.buf[key]) > MAX_POINTS:
                    self.buf[key].pop(0)

            def offsets(key):
                return list(zip(*self.buf[key])) if self.buf[key] else ([], [])

            for sc, key in zip(
                (self.sc_sxu, self.sc_sxl, self.sc_dxu, self.sc_dxl),
                ("sxu", "sxl", "dxu", "dxl")
            ):
                xs, ys = offsets(key)
                sc.set_offsets(list(zip(xs, ys)) if xs else [])

            self.fig.canvas.draw_idle()

    def start_timer(self, data_queue: queue.Queue, stop_event: threading.Event):
        """Avvia il timer di refresh e mostra la finestra."""
        def _tick():
            if stop_event.is_set():
                plt.close("all")
                return
            self.update(data_queue)
            # Riavvia il timer
            self._timer = self.fig.canvas.new_timer(interval=UPDATE_MS)
            self._timer.add_callback(_tick)
            self._timer.single_shot = True
            self._timer.start()

        self._timer = self.fig.canvas.new_timer(interval=UPDATE_MS)
        self._timer.add_callback(_tick)
        self._timer.single_shot = True
        self._timer.start()

        plt.show()


# ── Entry point ───────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Plot real-time da Arduino (replica dis() MATLAB)")
    parser.add_argument("--port", default=None,
                        help="Porta seriale (es. COM3, /dev/ttyUSB0). Auto-detect se omessa.")
    parser.add_argument("--baud", type=int, default=BAUD_RATE,
                        help=f"Baud rate (default: {BAUD_RATE})")
    args = parser.parse_args()

    port = args.port or find_arduino_port()
    if not port:
        print("[ERRORE] Nessuna porta seriale trovata. Specifica --port manualmente.")
        sys.exit(1)

    data_queue  = queue.Queue(maxsize=500)
    stop_event  = threading.Event()

    # Thread seriale in background
    reader_thread = threading.Thread(
        target=serial_reader,
        args=(port, args.baud, data_queue, stop_event),
        daemon=True
    )
    reader_thread.start()

    # Attendi un attimo che la connessione sia stabilita
    time.sleep(1.5)
    if stop_event.is_set():
        print("[ERRORE] Impossibile aprire la porta. Uscita.")
        sys.exit(1)

    plotter = LivePlotter()
    try:
        plotter.start_timer(data_queue, stop_event)   # blocca fino a chiusura finestra
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        print("\n[INFO] Terminato.")


if __name__ == "__main__":
    main()

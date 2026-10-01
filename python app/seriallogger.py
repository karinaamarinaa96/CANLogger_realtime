
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import queue
import time


# ============================================================
# CONFIG
# ============================================================

DEFAULT_BAUD = 460800

HEADER1 = 0xAA
HEADER2 = 0x55

MAX_DLC = 8
MIN_PACKET_LEN = 5
MAX_PACKET_LEN = 13

BYTE_FLASH_TIME_MS = 250

BAUD_CAN_ID = 0x001


# ============================================================
# SIEMENS LIGHT THEME
# ============================================================

BG_MAIN       = "#F0F2F5"
BG_PANEL      = "#FFFFFF"
BG_HEADER     = "#E5E7E9"
BG_TABLE      = "#FFFFFF"
BG_TABLE_ALT  = "#F5F6F7"

TEXT_MAIN     = "#202428"
TEXT_SECOND   = "#5B6168"
TEXT_MUTED    = "#7A8087"

SIEMENS_BLUE  = "#009999"
SIEMENS_BLUE2 = "#007A8A"

GREEN         = "#008A00"
RED           = "#C00000"
ORANGE        = "#D88A00"

BORDER        = "#D1D5D9"
GRID          = "#D9DDE1"

FLASH_BG      = "#FFE699"
FLASH_FG      = "#202428"


# ============================================================
# CRC16 CCITT
# ============================================================

def crc16_ccitt(data):

    crc = 0xFFFF

    for byte in data:

        crc ^= byte << 8

        for _ in range(8):

            if crc & 0x8000:

                crc = (
                    (crc << 1) ^
                    0x1021
                ) & 0xFFFF

            else:

                crc = (
                    crc << 1
                ) & 0xFFFF

    return crc


# ============================================================
# CAN ENTRY
# ============================================================

class CANEntry:

    def __init__(self, can_id):

        self.can_id = can_id

        self.dlc = 0

        self.data = [0] * 8

        self.count = 0

        self.byte_flash_until = [0] * 8

        self.tree_id = None


# ============================================================
# SERIAL RECEIVER
# ============================================================

class SerialReceiver:

    def __init__(self, event_queue):

        self.event_queue = event_queue

        self.ser = None

        self.running = False

        self.thread = None

        self.buffer = bytearray()

        self.rx_bytes = 0

        self.rx_packets = 0

    # --------------------------------------------------------
    # CONNECT
    # --------------------------------------------------------

    def connect(self, port, baud):

        if self.running:
            return False

        try:

            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=0.05
            )

            self.running = True

            self.buffer.clear()

            self.rx_bytes = 0

            self.rx_packets = 0

            self.thread = threading.Thread(
                target=self._read_thread,
                daemon=True
            )

            self.thread.start()

            return True

        except Exception as e:

            self.ser = None

            self.event_queue.put(
                ("ERROR", str(e))
            )

            return False

    # --------------------------------------------------------
    # DISCONNECT
    # --------------------------------------------------------

    def disconnect(self):

        self.running = False

        if self.thread:

            self.thread.join(
                timeout=0.5
            )

        if self.ser:

            try:
                self.ser.close()

            except:
                pass

        self.ser = None

        self.thread = None

    # --------------------------------------------------------
    # READ THREAD
    # --------------------------------------------------------

    def _read_thread(self):

        while self.running:

            try:

                if self.ser is None:
                    break

                data = self.ser.read(
                    self.ser.in_waiting or 1
                )

                if data:

                    self.buffer.extend(
                        data
                    )

                    self.rx_bytes += len(
                        data
                    )

                    self._parse_buffer()

            except Exception as e:

                if self.running:

                    self.event_queue.put(
                        ("ERROR", str(e))
                    )

                break

    # --------------------------------------------------------
    # PARSER
    # --------------------------------------------------------

    def _parse_buffer(self):

        while True:

            # ------------------------------------------------
            # Find AA 55
            # ------------------------------------------------

            pos = self.buffer.find(
                bytes([
                    HEADER1,
                    HEADER2
                ])
            )

            if pos < 0:

                if (
                    self.buffer and
                    self.buffer[-1] == HEADER1
                ):

                    self.buffer = (
                        self.buffer[-1:]
                    )

                else:

                    self.buffer.clear()

                return

            # Remove garbage before header

            if pos > 0:

                del self.buffer[:pos]

            # Need AA 55 + LEN

            if len(self.buffer) < 4:

                return

            # ------------------------------------------------
            # LEN
            # ------------------------------------------------

            packet_len = (
                (self.buffer[2] << 8) |
                self.buffer[3]
            )

            if (
                packet_len < MIN_PACKET_LEN or
                packet_len > MAX_PACKET_LEN
            ):

                del self.buffer[0]

                continue

            total_length = (
                2 + packet_len
            )

            if len(self.buffer) < total_length:

                return

            # ------------------------------------------------
            # Complete packet
            # ------------------------------------------------

            packet = bytes(
                self.buffer[:total_length]
            )

            del self.buffer[
                :total_length
            ]

            # ------------------------------------------------
            # PACKET FORMAT
            #
            # [0] AA
            # [1] 55
            # [2] LEN_H
            # [3] LEN_L
            # [4] ID_H
            # [5] ID_L
            # [6] DLC
            # [7] DATA0
            # ------------------------------------------------

            can_id = (
                (packet[4] << 8) |
                packet[5]
            )

            dlc = packet[6]

            if dlc > MAX_DLC:

                continue

            # Length must match DLC

            if packet_len != dlc + 5:

                continue

            # ------------------------------------------------
            # DATA
            # ------------------------------------------------

            data_start = 7

            data_end = (
                data_start +
                dlc
            )

            data = list(
                packet[
                    data_start:data_end
                ]
            )

            # ------------------------------------------------
            # CRC
            #
            # Calculated but NOT used to reject packet.
            # ------------------------------------------------

            crc_index = data_end

            if crc_index + 1 < len(packet):

                received_crc = (
                    (packet[crc_index] << 8) |
                    packet[crc_index + 1]
                )

                calculated_crc = crc16_ccitt(
                    packet[2:data_end]
                )

                crc_ok = (
                    received_crc ==
                    calculated_crc
                )

            else:

                received_crc = 0

                calculated_crc = 0

                crc_ok = False

            self.rx_packets += 1

            self.event_queue.put(
                (
                    "CAN",
                    can_id,
                    dlc,
                    data,
                    crc_ok
                )
            )


# ============================================================
# MAIN GUI
# ============================================================

class CANMonitor:

    def __init__(self, root):

        self.root = root

        self.root.title(
            "CAN Monitor"
        )

        self.root.geometry(
            "1150x700"
        )

        self.root.minsize(
            950,
            600
        )

        self.root.configure(
            bg=BG_MAIN
        )

        # ----------------------------------------------------
        # State
        # ----------------------------------------------------

        self.events = queue.Queue()

        self.receiver = SerialReceiver(
            self.events
        )

        self.can_entries = {}

        self.total_frames = 0

        self.last_rx_time = None

        self.connected = False

        self.selected_baudrate = (
            DEFAULT_BAUD
        )

        # ----------------------------------------------------
        # Style
        # ----------------------------------------------------

        self._setup_style()

        # ----------------------------------------------------
        # GUI
        # ----------------------------------------------------

        self._build_gui()

        # ----------------------------------------------------
        # Ports
        # ----------------------------------------------------

        self.refresh_ports()

        # ----------------------------------------------------
        # Timers
        # ----------------------------------------------------

        self.root.after(
            30,
            self.process_events
        )

        self.root.after(
            100,
            self.update_gui
        )

        self.root.protocol(
            "WM_DELETE_WINDOW",
            self.on_close
        )

    # ========================================================
    # STYLE
    # ========================================================

    def _setup_style(self):

        style = ttk.Style()

        try:

            style.theme_use(
                "clam"
            )

        except:

            pass

        # ----------------------------------------------------
        # Treeview
        # ----------------------------------------------------

        style.configure(
            "Treeview",
            background=BG_TABLE,
            foreground=TEXT_MAIN,
            fieldbackground=BG_TABLE,
            rowheight=29,
            borderwidth=1,
            relief="solid",
            font=(
                "Segoe UI",
                9
            )
        )

        style.configure(
            "Treeview.Heading",
            background=BG_HEADER,
            foreground=TEXT_MAIN,
            font=(
                "Segoe UI",
                9,
                "bold"
            ),
            relief="solid",
            borderwidth=1
        )

        style.map(
            "Treeview",
            background=[
                (
                    "selected",
                    "#B8DDE2"
                )
            ],
            foreground=[
                (
                    "selected",
                    TEXT_MAIN
                )
            ]
        )

        style.map(
            "Treeview.Heading",
            background=[
                (
                    "active",
                    "#D5D9DD"
                )
            ]
        )

        # ----------------------------------------------------
        # Buttons
        # ----------------------------------------------------

        style.configure(
            "TButton",
            font=(
                "Segoe UI",
                9,
                "bold"
            ),
            padding=6,
            background="#E1E4E7",
            foreground=TEXT_MAIN,
            borderwidth=1
        )

        style.map(
            "TButton",
            background=[
                (
                    "active",
                    "#D2D6DA"
                )
            ]
        )

        # ----------------------------------------------------
        # Combobox
        # ----------------------------------------------------

        style.configure(
            "TCombobox",
            padding=5,
            fieldbackground=BG_PANEL,
            background=BG_PANEL,
            foreground=TEXT_MAIN
        )

    # ========================================================
    # GUI BUILD
    # ========================================================

    def _build_gui(self):

        # ====================================================
        # HEADER
        # ====================================================

        header = tk.Frame(
            self.root,
            bg=BG_HEADER,
            height=68,
            bd=0
        )

        header.pack(
            fill="x"
        )

        header.pack_propagate(
            False
        )

        # Siemens-style left accent

        tk.Frame(
            header,
            bg=SIEMENS_BLUE,
            width=7
        ).pack(
            side="left",
            fill="y"
        )

        title_frame = tk.Frame(
            header,
            bg=BG_HEADER
        )

        title_frame.pack(
            side="left",
            padx=18
        )

        tk.Label(
            title_frame,
            text="CAN MONITOR",
            font=(
                "Segoe UI",
                18,
                "bold"
            ),
            fg=TEXT_MAIN,
            bg=BG_HEADER
        ).pack(
            anchor="w",
            pady=(9, 0)
        )

        tk.Label(
            title_frame,
            text="Automotive CAN Bus Analyzer",
            font=(
                "Segoe UI",
                9
            ),
            fg=TEXT_SECOND,
            bg=BG_HEADER
        ).pack(
            anchor="w"
        )

        # Connection status

        self.connection_label = tk.Label(
            header,
            text="● DISCONNECTED",
            font=(
                "Segoe UI",
                9,
                "bold"
            ),
            fg=RED,
            bg=BG_HEADER
        )

        self.connection_label.pack(
            side="right",
            padx=24
        )

        # ====================================================
        # STATUS PANEL
        # ====================================================

        cards = tk.Frame(
            self.root,
            bg=BG_MAIN
        )

        cards.pack(
            fill="x",
            padx=18,
            pady=(14, 8)
        )

        # ----------------------------------------------------
        # CAN BAUDRATE
        # ----------------------------------------------------

        baud_card = self.create_card(
            cards,
            "CAN BAUDRATE"
        )

        baud_card.pack(
            side="left",
            fill="both",
            expand=True,
            padx=(0, 5)
        )

        self.baud_value = tk.Label(
            baud_card,
            text="---",
            font=(
                "Segoe UI",
                20,
                "bold"
            ),
            fg=SIEMENS_BLUE2,
            bg=BG_PANEL
        )

        self.baud_value.pack(
            anchor="w",
            padx=14,
            pady=(0, 2)
        )

        self.baud_sub = tk.Label(
            baud_card,
            text="Waiting for CAN baudrate...",
            font=(
                "Segoe UI",
                8
            ),
            fg=TEXT_MUTED,
            bg=BG_PANEL
        )

        self.baud_sub.pack(
            anchor="w",
            padx=14,
            pady=(0, 10)
        )

        # ----------------------------------------------------
        # FRAMES
        # ----------------------------------------------------

        frame_card = self.create_card(
            cards,
            "RECEIVED FRAMES"
        )

        frame_card.pack(
            side="left",
            fill="both",
            expand=True,
            padx=5
        )

        self.frames_value = tk.Label(
            frame_card,
            text="0",
            font=(
                "Segoe UI",
                20,
                "bold"
            ),
            fg=SIEMENS_BLUE2,
            bg=BG_PANEL
        )

        self.frames_value.pack(
            anchor="w",
            padx=14,
            pady=(0, 2)
        )

        tk.Label(
            frame_card,
            text="CAN frames received",
            font=(
                "Segoe UI",
                8
            ),
            fg=TEXT_MUTED,
            bg=BG_PANEL
        ).pack(
            anchor="w",
            padx=14,
            pady=(0, 10)
        )

        # ----------------------------------------------------
        # ACTIVE IDS
        # ----------------------------------------------------

        id_card = self.create_card(
            cards,
            "ACTIVE CAN IDs"
        )

        id_card.pack(
            side="left",
            fill="both",
            expand=True,
            padx=5
        )

        self.ids_value = tk.Label(
            id_card,
            text="0",
            font=(
                "Segoe UI",
                20,
                "bold"
            ),
            fg=GREEN,
            bg=BG_PANEL
        )

        self.ids_value.pack(
            anchor="w",
            padx=14,
            pady=(0, 2)
        )

        tk.Label(
            id_card,
            text="Unique identifiers",
            font=(
                "Segoe UI",
                8
            ),
            fg=TEXT_MUTED,
            bg=BG_PANEL
        ).pack(
            anchor="w",
            padx=14,
            pady=(0, 10)
        )

        # ----------------------------------------------------
        # SERIAL
        # ----------------------------------------------------

        serial_card = self.create_card(
            cards,
            "SERIAL LINK"
        )

        serial_card.pack(
            side="left",
            fill="both",
            expand=True,
            padx=(5, 0)
        )

        self.serial_value = tk.Label(
            serial_card,
            text=f"{DEFAULT_BAUD:,}",
            font=(
                "Segoe UI",
                18,
                "bold"
            ),
            fg=ORANGE,
            bg=BG_PANEL
        )

        self.serial_value.pack(
            anchor="w",
            padx=14,
            pady=(0, 2)
        )

        tk.Label(
            serial_card,
            text="UART baudrate",
            font=(
                "Segoe UI",
                8
            ),
            fg=TEXT_MUTED,
            bg=BG_PANEL
        ).pack(
            anchor="w",
            padx=14,
            pady=(0, 10)
        )

        # ====================================================
        # CONTROL BAR
        # ====================================================

        control_outer = tk.Frame(
            self.root,
            bg=BG_MAIN
        )

        control_outer.pack(
            fill="x",
            padx=18,
            pady=7
        )

        control = tk.Frame(
            control_outer,
            bg=BG_PANEL,
            bd=1,
            relief="solid"
        )

        control.pack(
            fill="x"
        )

        # ----------------------------------------------------
        # COM
        # ----------------------------------------------------

        tk.Label(
            control,
            text="COM PORT",
            fg=TEXT_SECOND,
            bg=BG_PANEL,
            font=(
                "Segoe UI",
                8,
                "bold"
            )
        ).pack(
            side="left",
            padx=(12, 5)
        )

        self.port_var = tk.StringVar()

        self.port_combo = ttk.Combobox(
            control,
            textvariable=self.port_var,
            state="readonly",
            width=12
        )

        self.port_combo.pack(
            side="left",
            padx=4,
            pady=8
        )

        ttk.Button(
            control,
            text="⟳ Refresh",
            command=self.refresh_ports
        ).pack(
            side="left",
            padx=4
        )

        # ----------------------------------------------------
        # UART BAUD
        # ----------------------------------------------------

        tk.Label(
            control,
            text="UART BAUD",
            fg=TEXT_SECOND,
            bg=BG_PANEL,
            font=(
                "Segoe UI",
                8,
                "bold"
            )
        ).pack(
            side="left",
            padx=(18, 5)
        )

        self.serial_baud_var = tk.StringVar(
            value=str(DEFAULT_BAUD)
        )

        self.serial_baud_combo = ttk.Combobox(
            control,
            textvariable=self.serial_baud_var,
            state="readonly",
            width=10,
            values=[
                "9600",
                "19200",
                "38400",
                "57600",
                "115200",
                "230400",
                "460800",
                "921600"
            ]
        )

        self.serial_baud_combo.pack(
            side="left",
            padx=4
        )

        # ----------------------------------------------------
        # CONNECT
        # ----------------------------------------------------

        self.connect_button = ttk.Button(
            control,
            text="CONNECT",
            command=self.toggle_connection
        )

        self.connect_button.pack(
            side="left",
            padx=(12, 4)
        )

        # ----------------------------------------------------
        # CLEAR
        # ----------------------------------------------------

        ttk.Button(
            control,
            text="CLEAR",
            command=self.clear_data
        ).pack(
            side="left",
            padx=4
        )

        # ----------------------------------------------------
        # LAST RX
        # ----------------------------------------------------

        self.last_rx_label = tk.Label(
            control,
            text="Last RX: ---",
            font=(
                "Segoe UI",
                8
            ),
            fg=TEXT_MUTED,
            bg=BG_PANEL
        )

        self.last_rx_label.pack(
            side="right",
            padx=12
        )

        # ====================================================
        # TABLE
        # ====================================================

        table_frame = tk.Frame(
            self.root,
            bg=BG_MAIN
        )

        table_frame.pack(
            fill="both",
            expand=True,
            padx=18,
            pady=(4, 15)
        )

        # ----------------------------------------------------
        # Table title
        # ----------------------------------------------------

        table_title = tk.Frame(
            table_frame,
            bg=BG_MAIN
        )

        table_title.pack(
            fill="x",
            pady=(0, 5)
        )

        tk.Label(
            table_title,
            text="CAN TRAFFIC",
            font=(
                "Segoe UI",
                10,
                "bold"
            ),
            fg=TEXT_MAIN,
            bg=BG_MAIN
        ).pack(
            side="left"
        )

        tk.Label(
            table_title,
            text="Changed bytes are highlighted",
            font=(
                "Segoe UI",
                8
            ),
            fg=TEXT_MUTED,
            bg=BG_MAIN
        ).pack(
            side="right"
        )

        # ----------------------------------------------------
        # Tree container
        # ----------------------------------------------------

        tree_container = tk.Frame(
            table_frame,
            bg=BG_PANEL,
            bd=1,
            relief="solid"
        )

        tree_container.pack(
            fill="both",
            expand=True
        )

        columns = (
            "ID",
            "DLC",
            "B0",
            "B1",
            "B2",
            "B3",
            "B4",
            "B5",
            "B6",
            "B7",
            "COUNT"
        )

        self.tree = ttk.Treeview(
            tree_container,
            columns=columns,
            show="headings",
            selectmode="browse"
        )

        # ----------------------------------------------------
        # Columns
        # ----------------------------------------------------

        self.tree.heading(
            "ID",
            text="CAN ID"
        )

        self.tree.column(
            "ID",
            width=100,
            anchor="center"
        )

        self.tree.heading(
            "DLC",
            text="DLC"
        )

        self.tree.column(
            "DLC",
            width=55,
            anchor="center"
        )

        for i in range(8):

            col = f"B{i}"

            self.tree.heading(
                col,
                text=col
            )

            self.tree.column(
                col,
                width=72,
                anchor="center"
            )

        self.tree.heading(
            "COUNT",
            text="COUNT"
        )

        self.tree.column(
            "COUNT",
            width=90,
            anchor="center"
        )

        # ----------------------------------------------------
        # Tags
        # ----------------------------------------------------

        self.tree.tag_configure(
            "even",
            background=BG_TABLE,
            foreground=TEXT_MAIN
        )

        self.tree.tag_configure(
            "odd",
            background=BG_TABLE_ALT,
            foreground=TEXT_MAIN
        )

        self.tree.tag_configure(
            "flash",
            background=FLASH_BG,
            foreground=FLASH_FG
        )

        self.tree.pack(
            side="left",
            fill="both",
            expand=True
        )

        scrollbar = ttk.Scrollbar(
            tree_container,
            orient="vertical",
            command=self.tree.yview
        )

        scrollbar.pack(
            side="right",
            fill="y"
        )

        self.tree.configure(
            yscrollcommand=scrollbar.set
        )

        # ====================================================
        # BOTTOM STATUS
        # ====================================================

        bottom = tk.Frame(
            self.root,
            bg=BG_HEADER,
            height=25,
            bd=1,
            relief="solid"
        )

        bottom.pack(
            fill="x",
            side="bottom"
        )

        bottom.pack_propagate(
            False
        )

        self.status_label = tk.Label(
            bottom,
            text="Ready",
            font=(
                "Segoe UI",
                8
            ),
            fg=TEXT_SECOND,
            bg=BG_HEADER
        )

        self.status_label.pack(
            side="left",
            padx=15
        )

        self.rx_bytes_label = tk.Label(
            bottom,
            text="RX: 0 bytes",
            font=(
                "Segoe UI",
                8
            ),
            fg=TEXT_MUTED,
            bg=BG_HEADER
        )

        self.rx_bytes_label.pack(
            side="right",
            padx=15
        )

    # ========================================================
    # CARD
    # ========================================================

    def create_card(
        self,
        parent,
        title
    ):

        outer = tk.Frame(
            parent,
            bg=BORDER,
            padx=1,
            pady=1
        )

        frame = tk.Frame(
            outer,
            bg=BG_PANEL,
            height=100
        )

        frame.pack(
            fill="both",
            expand=True
        )

        tk.Label(
            frame,
            text=title,
            font=(
                "Segoe UI",
                8,
                "bold"
            ),
            fg=TEXT_MUTED,
            bg=BG_PANEL
        ).pack(
            anchor="w",
            padx=14,
            pady=(10, 2)
        )

        return frame

    # ========================================================
    # PORTS
    # ========================================================

    def refresh_ports(self):

        ports = [
            port.device
            for port in serial.tools.list_ports.comports()
        ]

        self.port_combo["values"] = ports

        if ports:

            if self.port_var.get() not in ports:

                self.port_var.set(
                    ports[0]
                )

        else:

            self.port_var.set("")

    # ========================================================
    # CONNECT / DISCONNECT
    # ========================================================

    def toggle_connection(self):

        if self.connected:

            self.receiver.disconnect()

            self.connected = False

            self.connect_button.config(
                text="CONNECT"
            )

            self.connection_label.config(
                text="● DISCONNECTED",
                fg=RED
            )

            self.status_label.config(
                text="Disconnected"
            )

            return

        # ----------------------------------------------------
        # Connect
        # ----------------------------------------------------

        port = self.port_var.get()

        if not port:

            messagebox.showwarning(
                "Serial Port",
                "Please select a COM port."
            )

            return

        try:

            baud = int(
                self.serial_baud_var.get()
            )

        except:

            messagebox.showerror(
                "Baudrate",
                "Invalid UART baudrate."
            )

            return

        if self.receiver.connect(
            port,
            baud
        ):

            self.connected = True

            self.selected_baudrate = baud

            self.serial_value.config(
                text=f"{baud:,}"
            )

            self.connect_button.config(
                text="DISCONNECT"
            )

            self.connection_label.config(
                text="● CONNECTED",
                fg=GREEN
            )

            self.status_label.config(
                text=f"Connected to {port}"
            )

        else:

            messagebox.showerror(
                "Connection Error",
                f"Could not open {port}."
            )

    # ========================================================
    # EVENT PROCESSING
    # ========================================================

    def process_events(self):

        try:

            while True:

                event = self.events.get_nowait()

                event_type = event[0]

                if event_type == "CAN":

                    can_id = event[1]

                    dlc = event[2]

                    data = event[3]

                    crc_ok = event[4]

                    self.handle_can_packet(
                        can_id,
                        dlc,
                        data
                    )

                elif event_type == "ERROR":

                    self.status_label.config(
                        text=f"Serial error: {event[1]}"
                    )

                    self.connection_label.config(
                        text="● ERROR",
                        fg=RED
                    )

        except queue.Empty:

            pass

        self.root.after(
            30,
            self.process_events
        )

    # ========================================================
    # HANDLE CAN
    # ========================================================

    def handle_can_packet(
        self,
        can_id,
        dlc,
        data
    ):

        self.total_frames += 1

        self.last_rx_time = time.time()

        # ----------------------------------------------------
        # BAUDRATE PACKET
        # ----------------------------------------------------

        if can_id == BAUD_CAN_ID:

            baud_kbit = self.decode_baudrate(
                dlc,
                data
            )

            if baud_kbit is not None:

                self.baud_value.config(
                    text=f"{baud_kbit:,} kbit/s"
                )

                self.baud_sub.config(
                    text="Detected CAN bus bitrate"
                )

                self.status_label.config(
                    text=(
                        f"CAN baudrate detected: "
                        f"{baud_kbit:,} kbit/s"
                    )
                )

            return

        # ----------------------------------------------------
        # NORMAL CAN FRAME
        # ----------------------------------------------------

        if can_id not in self.can_entries:

            entry = CANEntry(
                can_id
            )

            self.can_entries[
                can_id
            ] = entry

            entry.tree_id = self.tree.insert(
                "",
                "end",
                values=self.make_row_values(
                    entry
                )
            )

        else:

            entry = self.can_entries[
                can_id
            ]

        now = time.time() * 1000

        # ----------------------------------------------------
        # DLC
        # ----------------------------------------------------

        entry.dlc = dlc

        # ----------------------------------------------------
        # DATA
        # ----------------------------------------------------

        for i in range(dlc):

            new_value = data[i]

            if entry.data[i] != new_value:

                entry.data[i] = new_value

                entry.byte_flash_until[i] = (
                    now +
                    BYTE_FLASH_TIME_MS
                )

        # Clear bytes outside DLC

        for i in range(dlc, 8):

            entry.data[i] = 0

        entry.count += 1

        self.update_tree_row(
            entry
        )

    # ========================================================
    # BAUDRATE DECODER
    # ========================================================

    def decode_baudrate(
        self,
        dlc,
        data
    ):

        # ----------------------------------------------------
        # Baudrate packet:
        #
        # DLC = 2
        #
        # data[0] = LOW BYTE
        # data[1] = HIGH BYTE
        #
        # Little Endian
        #
        # 125  -> 7D 00
        # 250  -> FA 00
        # 500  -> F4 01
        # 800  -> 20 03
        # 1000 -> E8 03
        # ----------------------------------------------------

        if dlc != 2:

            return None

        value = (
            data[0] |
            (data[1] << 8)
        )

        if 0 <= value <= 1000:

            return value

        return None

    # ========================================================
    # ROW VALUES
    # ========================================================

    def make_row_values(
        self,
        entry
    ):

        values = [
            f"{entry.can_id:03X}",
            entry.dlc
        ]

        for i in range(8):

            if i < entry.dlc:

                values.append(
                    f"{entry.data[i]:02X}"
                )

            else:

                values.append(
                    "--"
                )

        values.append(
            entry.count
        )

        return values

    # ========================================================
    # UPDATE TREE ROW
    # ========================================================

    def update_tree_row(
        self,
        entry
    ):

        if not entry.tree_id:

            return

        now = time.time() * 1000

        flashing = any(
            t > now
            for t in entry.byte_flash_until
        )

        if flashing:

            tag = "flash"

        else:

            children = (
                self.tree.get_children()
            )

            try:

                index = children.index(
                    entry.tree_id
                )

                if index % 2 == 0:

                    tag = "even"

                else:

                    tag = "odd"

            except:

                tag = "even"

        self.tree.item(
            entry.tree_id,
            values=self.make_row_values(
                entry
            ),
            tags=(tag,)
        )

    # ========================================================
    # GUI UPDATE
    # ========================================================

    def update_gui(self):

        self.frames_value.config(
            text=f"{self.total_frames:,}"
        )

        self.ids_value.config(
            text=f"{len(self.can_entries):,}"
        )

        self.rx_bytes_label.config(
            text=(
                f"RX: "
                f"{self.receiver.rx_bytes:,} bytes"
            )
        )

        # ----------------------------------------------------
        # Last RX
        # ----------------------------------------------------

        if self.last_rx_time:

            elapsed = (
                time.time() -
                self.last_rx_time
            )

            if elapsed < 1:

                self.last_rx_label.config(
                    text="Last RX: just now",
                    fg=GREEN
                )

            elif elapsed < 60:

                self.last_rx_label.config(
                    text=(
                        f"Last RX: "
                        f"{elapsed:.1f}s ago"
                    ),
                    fg=TEXT_SECOND
                )

            else:

                self.last_rx_label.config(
                    text="Last RX: no recent data",
                    fg=TEXT_MUTED
                )

        # ----------------------------------------------------
        # Refresh flashing rows
        # ----------------------------------------------------

        for entry in (
            self.can_entries.values()
        ):

            self.update_tree_row(
                entry
            )

        self.root.after(
            100,
            self.update_gui
        )

    # ========================================================
    # CLEAR
    # ========================================================

    def clear_data(self):

        self.can_entries.clear()

        self.total_frames = 0

        self.last_rx_time = None

        for item in (
            self.tree.get_children()
        ):

            self.tree.delete(
                item
            )

        self.baud_value.config(
            text="---"
        )

        self.baud_sub.config(
            text="Waiting for CAN baudrate..."
        )

        self.status_label.config(
            text="Monitor cleared"
        )

        self.last_rx_label.config(
            text="Last RX: ---",
            fg=TEXT_MUTED
        )

    # ========================================================
    # CLOSE
    # ========================================================

    def on_close(self):

        try:

            self.receiver.disconnect()

        except:

            pass

        self.root.destroy()


# ============================================================
# MAIN
# ============================================================

def main():

    root = tk.Tk()

    app = CANMonitor(
        root
    )

    root.mainloop()


if __name__ == "__main__":

    main()

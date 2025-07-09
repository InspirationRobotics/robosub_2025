"""
Simple Modem API for Inter-Sub Communication
Handles serial messaging, message parsing, acknowledgment, and threading.
"""

import time
import serial
import threading
from ...utils.deviceHelper import dataFromConfig, variables

# ----------------------------
# Config: Get serial port from sub's config
# ----------------------------
port = dataFromConfig("modem")

# ----------------------------
# (Optional) LED Class for Feedback - Commented Out
# ----------------------------
# class LED:
#     def on_send_msg(self): pass
#     def on_recv_msg(self): pass
# led = LED()

# ----------------------------
# Main Modem Class
# ----------------------------
class Modem:
    """
    Manages serial modem connection, message queuing, sending/receiving, and acknowledgments.
    """
    def __init__(self, auto_start=True):
        self.ser = serial.Serial(port, 9600, serial.PARITY_NONE, serial.STOPBITS_ONE, serial.EIGHTBITS)
        self.recv_callbacks = [self.on_receive_msg_logging, self.on_receive_ack]
        self.send_callbacks = [self.on_send_msg_logging]
        self.data_buffer = ""
        self.ACK = 0
        self.in_transit = []  # Queue of outgoing messages
        self.ack_received = []
        self.receive_active = True
        self.sending_active = True
        self.thread_recv = threading.Thread(target=self._receive_loop)
        self.thread_send = threading.Thread(target=self._send_loop)
        if auto_start:
            self.start()

    # ---- Send/Receive Core Loops ----
    def start(self):
        self.receive_active = True
        self.sending_active = True
        self.thread_recv.start()
        self.thread_send.start()

    def stop(self):
        self.receive_active = False
        self.sending_active = False
        self.thread_recv.join()
        self.thread_send.join()

    # ---- Sending Functions ----
    def send_msg(self, msg, ack=None, dest_addr=None, priority=1):
        """Queue a message to send with optional acknowledgment and address."""
        if ack is None:
            self.ACK += 1
            ack = self.ACK
        self.in_transit.append([msg, time.time(), 0, ack, dest_addr, priority])

    def _send_to_modem(self, data):
        """Low-level: send data, wait for reply."""
        self.ser.write(f"${data}".encode())
        out = b""
        time.sleep(0.1)
        while self.ser.in_waiting > 0:
            out += self.ser.read(1)
        return out.decode() if out else None

    def _send_loop(self):
        """Background thread: handles outgoing message queue and retries."""
        while self.sending_active:
            to_remove = []
            self.ack_received = []
            for it, packet in enumerate(self.in_transit):
                msg, time_sent, time_last_sent, ack, dest_addr, priority = packet
                # Retransmit if not acknowledged
                if time.time() - time_last_sent > 1.0 and ack not in self.ack_received:
                    self._transmit(msg, ack, dest_addr)
                    packet[2] = time.time()
                # Remove timed out messages
                if time.time() - time_sent > 30 and priority == 0:
                    print(f'[WARNING] Message "{msg}" timed out')
                    to_remove.append(it)
            time.sleep(0.5)
            # Remove sent/acknowledged messages
            self.in_transit = [packet for (it, packet) in enumerate(self.in_transit)
                               if it not in to_remove and packet[3] not in self.ack_received]

    def _transmit(self, msg, ack=None, dest_addr=None):
        """Transmits a message with optional ack/address, with simple chunking for long messages."""
        if ack is None:
            self.ACK += 1
            ack = self.ACK
        msg_str = f"*{msg}*@{ack}"
        # (LED feedback on send - optional)
        # led.on_send_msg()
        self._send_to_modem(msg_str)
        for cb in self.send_callbacks:
            cb(dest_addr, msg, ack)

    # ---- Receiving Functions ----
    def _receive_loop(self):
        """Background thread: listens for incoming messages and dispatches them."""
        while self.receive_active:
            raw_packet = self.ser.readline()
            if raw_packet:
                try:
                    packet = raw_packet.decode("utf-8")
                    self._dispatch(packet)
                except Exception as e:
                    print("Receive error:", e)

    def _dispatch(self, packet):
        """Parses and dispatches incoming messages."""
        # For demo, treat all packets as messages
        msg = packet.strip()
        for cb in self.recv_callbacks:
            cb(None, msg, None, None)

    # ---- Callbacks ----
    def on_receive_msg_logging(self, src_addr, msg, ack, distance):
        """Logs incoming message."""
        print(f"[RECV] {msg}")

    def on_receive_ack(self, src_addr, msg, ack, distance):
        """Handles message acknowledgments (dummy for now)."""
        pass

    def on_send_msg_logging(self, dst_addr, msg, ack):
        """Logs outgoing message."""
        print(f"[SEND] {msg}")

# ----------------------------
# Manual Test Mode
# ----------------------------
if __name__ == "__main__":
    modem = Modem()
    while True:
        msg = input("Type message to send (or 'exit'): ")
        if msg == "exit":
            modem.stop()
            break
        modem.send_msg(msg)

import socket
import struct
import rospy
from can_msgs.msg import Frame

# This class is responsible for sending CAN frames over TCP to the PCAN Gateway
# |Can device| <-CAN frame-> |PCAN Gateway| <-TCP frame-> |PCAN Gateway Node| <-ROS can_msgs-> ROS Topic
#
# Required PCAN-Gateway web UI settings for the Send/Receive routes used here
# (Device > User Management > Expert mode, then Routing > Edit Route):
#   - Handshake: OFF   ("PCAN-Gateway handshake off" checkbox) - required for
#     plain socket communication; a PC is not a PCAN-Gateway peer.
#   - CRC32 checksum: OFF - this driver assumes classic CAN 2.0 A/B frames
#     without CRC (Message Type 0x80, 36-byte frame). Enabling CRC makes the
#     gateway send 40-byte frames (Message Type 0x81) and FRAME_SIZE below
#     must change to match, or the byte stream will desync.
#   - CAN FD: not supported by this driver (PCAN-Ethernet Gateway DR only).

FRAME_SIZE = 36  # Fixed frame size in bytes (classic CAN 2.0 A/B, no CRC)


# ─────────────────────────────────────────────
#  SENDER  (PC = TCP Client → connects to Gateway)
# ─────────────────────────────────────────────
class PcanTcpSender:
    def __init__(self, gateway_ip: str, gateway_port: int):
        self.gateway_ip   = gateway_ip
        self.gateway_port = gateway_port
        self.sock         = None
        self._connect()

    # ── connection helpers ──────────────────────────────────────
    def _connect(self):
        """Open a TCP connection to the PCAN Gateway (Gateway is TCP server)."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.gateway_ip, self.gateway_port))
            self.sock.settimeout(None)          # back to blocking after connect
            rospy.loginfo(f"[PcanTcpSender] Connected to {self.gateway_ip}:{self.gateway_port}")
        except (socket.error, OSError) as e:
            rospy.logwarn(f"[PcanTcpSender] Connection failed: {e}. Will retry on next send.")
            self.sock = None

    def _ensure_connected(self) -> bool:
        if self.sock is not None:
            return True
        rospy.loginfo("[PcanTcpSender] Reconnecting...")
        self._connect()
        return self.sock is not None

    # ── public API ──────────────────────────────────────────────
    def send_frame(self, msg: Frame):
        if not self._ensure_connected():
            rospy.logwarn("[PcanTcpSender] Cannot send: not connected.")
            return

        data  = list(msg.data[:msg.dlc])
        frame = self._encode_can_to_tcp(msg.id, data, is_extended=msg.is_extended)

        try:
            self.sock.sendall(frame)    # sendall guarantees full 36 bytes are sent
        except (socket.error, BrokenPipeError, OSError) as e:
            rospy.logwarn(f"[PcanTcpSender] Send failed: {e}. Dropping connection.")
            self._close_sock()

    def close(self):
        self._close_sock()
        rospy.loginfo("[PcanTcpSender] Connection closed.")

    # ── internal ────────────────────────────────────────────────
    def _encode_can_to_tcp(self, can_id: int, data: list, is_extended: bool = False) -> bytes:
        dlc   = len(data)
        flags = 0x02 if is_extended else 0x00

        msg_list = [
            0x00, 0x24, 0x00, 0x80, 0x9d, 0x7c, 0x3c, 0xd5,   # bytes  0-7  : header
            0xf0, 0x73, 0xae, 0x00, 0x35, 0xba, 0x5e, 0xf3,   # bytes  8-15 : header
            0x00, 0x05, 0x40, 0x50,                             # bytes 16-19 : header
            0x00,                                               # byte  20    : unused
            dlc,                                                # byte  21    : DLC
            0x00,                                               # byte  22    : reserved
            flags,                                              # byte  23    : flags
            0x00, 0x00, 0x00, 0x00,                             # bytes 24-27 : CAN ID
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   # bytes 28-35 : data
        ]

        msg_list[24:28] = list(struct.pack(">I", can_id))
        msg_list[28:36] = list(data) + [0] * (8 - dlc)

        return bytes(msg_list)

    def _close_sock(self):
        if self.sock:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None


# ─────────────────────────────────────────────
#  RECEIVER  (PC = TCP Server → Gateway connects in)
# ─────────────────────────────────────────────
class PcanTcpReceiver:
    def __init__(self, listen_ip: str, listen_port: int, backlog: int = 1):
        """
        PC acts as TCP server.
        The PCAN Gateway is configured to connect OUT to this IP:port.
        """
        self.DLC_TO_LEN = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]

        self._server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server_sock.bind((listen_ip, listen_port))
        self._server_sock.listen(backlog)
        self._server_sock.setblocking(False)    # non-blocking accept

        self._client_sock = None
        self._recv_buf    = b""                 # accumulate partial TCP reads

        rospy.loginfo(f"[PcanTcpReceiver] Listening on {listen_ip}:{listen_port}")

    # ── public API ──────────────────────────────────────────────
    def receive_frame(self) -> Frame:
        """
        Call this in a loop (e.g. 1000 Hz).
        Returns one can_msgs/Frame when a complete 36-byte frame is ready,
        otherwise returns None.
        TCP is a stream → bytes may arrive in chunks, so we buffer them.
        """
        # 1. Accept a new client if none connected yet
        if self._client_sock is None:
            self._try_accept()
            return None

        # 2. Read whatever bytes are available right now
        self._try_recv()

        # 3. Return one complete frame if we have enough bytes
        if len(self._recv_buf) >= FRAME_SIZE:
            raw            = self._recv_buf[:FRAME_SIZE]
            self._recv_buf = self._recv_buf[FRAME_SIZE:]    # consume from buffer
            return self._decode_tcp_to_can(raw)

        return None

    def close(self):
        if self._client_sock:
            self._client_sock.close()
        self._server_sock.close()
        rospy.loginfo("[PcanTcpReceiver] Server closed.")

    # ── internal ────────────────────────────────────────────────
    def _try_accept(self):
        """Non-blocking accept — does nothing if no client is knocking."""
        try:
            self._client_sock, addr = self._server_sock.accept()
            self._client_sock.setblocking(False)
            self._recv_buf = b""
            rospy.loginfo(f"[PcanTcpReceiver] Gateway connected from {addr}")
        except BlockingIOError:
            pass

    def _try_recv(self):
        """Read bytes from the client socket into the internal buffer."""
        try:
            chunk = self._client_sock.recv(4096)
            if chunk == b"":
                # Gateway closed the connection
                rospy.logwarn("[PcanTcpReceiver] Gateway disconnected.")
                self._client_sock.close()
                self._client_sock = None
                self._recv_buf    = b""
            else:
                self._recv_buf += chunk
        except BlockingIOError:
            pass    # no data available right now — perfectly normal
        except (ConnectionResetError, OSError) as e:
            rospy.logwarn(f"[PcanTcpReceiver] Receive error: {e}")
            self._client_sock.close()
            self._client_sock = None
            self._recv_buf    = b""

    def _decode_tcp_to_can(self, message: bytes) -> Frame:
        """Parse a 36-byte payload into a can_msgs/Frame."""
        if len(message) < FRAME_SIZE:
            return None

        frame = Frame()

        # Flags — byte 23
        frame.is_extended = bool(message[23] & 0x02)

        # CAN ID — bytes 24-27 big-endian, mask to 29 bits
        can_id_raw = int.from_bytes(message[24:28], byteorder="big", signed=False)
        frame.id   = can_id_raw & 0x1FFFFFFF

        # DLC — byte 21
        dlc        = message[21]
        length     = self.DLC_TO_LEN[dlc] if dlc < len(self.DLC_TO_LEN) else 8
        frame.dlc  = min(length, 8)         # CAN 2.0 max 8 bytes

        # Data — bytes 28 … 28+dlc
        frame.data = message[28:28 + frame.dlc]

        return frame
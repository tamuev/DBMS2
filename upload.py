#!/usr/bin/env python3
"""
CANBL firmware uploader.

Pushes an application image to the CANBL bootloader over a CAN-to-TCP adapter
(the same EthInterface wire format used by the Java tooling).

Adapter wire format (13-byte fixed frames, both directions):
    byte 0     : DLC (low nibble) | 0x80 if the CAN ID is extended (29-bit)
    bytes 1..4 : CAN ID, big-endian uint32
    bytes 5..12: 8 data bytes (zero padded)

Bootloader protocol:
    -> RX_START        [u32 BE total_length]   erase, then TX_START_ACK
    -> RX_BLOCK_START  [u32 BE image_offset]    TX_BLOCK_START_ACK
    -> RX_DATA|seq     [<=8 image bytes]        write at offset + seq*8
    -> RX_BLOCK_FINISH []                       TX_BLOCK_FINISH_ACK [u32 BE bytes]
    -> RX_VERIFY       []                       TX_VERIFY_RESP [u32 BE crc32]

The target only listens for RX_START for BOOT_WAIT_MS (~250 ms) after reset, so
this script repeatedly sends RX_START until it sees an ack. To get the target
into the bootloader in the first place it either asks the running application to
reboot gracefully (a blank frame on CANID_REBOOT, which sets the bootloader's
flash-request flag), or -- if the application is unresponsive -- waits for the
operator to reset the board manually (--manual-reset).
"""

import argparse
import socket
import struct
import sys
import time
import zlib

# --- Protocol IDs (must match App/canbl.h) ---------------------------------
CANID_RX_START            = 0x0C000002
CANID_TX_START_ACK        = 0x0C000003
CANID_RX_BLOCK_START      = 0x0C000004
CANID_TX_BLOCK_START_ACK  = 0x0C000005
CANID_RX_BLOCK_FINISH     = 0x0C000006
CANID_TX_BLOCK_FINISH_ACK = 0x0C000007
CANID_RX_VERIFY           = 0x0C000008
CANID_TX_VERIFY_RESP      = 0x0C000009
CANID_RX_BOOT             = 0x0C00000A

CANID_RX_DATA             = 0x0C000D00

# Blank frame here asks the running application to reboot into the bootloader,
# setting the flash-request flag so the bootloader stays in flash mode.
CANID_REBOOT              = 0x0B00B007

BLOCK_SIZE = 2048   # max bytes per block: 256 data frames * 8 bytes
FRAME_SIZE = 8
PACKET_LEN = 13

DEFAULT_HOST = "192.168.0.20"
DEFAULT_PORT = 10001


class CanTcp:
    """Minimal client for the CAN-to-TCP adapter."""

    def __init__(self, host, port, verbose=False):
        self.verbose = verbose
        self.sock = socket.create_connection((host, port), timeout=3.0)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._rx = bytearray()

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass

    @staticmethod
    def _pack(can_id, data):
        d = bytes(data[:8]).ljust(8, b"\x00")
        ext = 0x80 if can_id > 0x7FF else 0x00
        dlc = min(len(data), 8)
        return bytes([dlc | ext]) + struct.pack(">I", can_id) + d

    def send(self, can_id, data=b""):
        pkt = self._pack(can_id, data)
        if self.verbose:
            print(f"[tx] id={can_id:08X} dlc={len(data)} data={bytes(data[:8]).hex()}")
        self.sock.sendall(pkt)

    def recv(self, timeout):
        """Return (can_id, data_bytes) or None on timeout."""
        deadline = time.monotonic() + timeout
        while len(self._rx) < PACKET_LEN:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self.sock.settimeout(remaining)
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                return None
            if not chunk:
                raise ConnectionError("adapter closed the connection")
            self._rx.extend(chunk)

        pkt = self._rx[:PACKET_LEN]
        del self._rx[:PACKET_LEN]
        can_id = struct.unpack(">I", pkt[1:5])[0] & 0x1FFFFFFF
        data = bytes(pkt[5:13])
        if self.verbose:
            print(f"[rx] id={can_id:08X} k={pkt[0]:02X} data={data.hex()}")
        return can_id, data

    def wait_for(self, expected_id, timeout):
        """Wait for a frame with the given ID, ignoring others. Returns its data."""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            frame = self.recv(remaining)
            if frame is None:
                return None
            can_id, data = frame
            if can_id == expected_id:
                return data
            if self.verbose:
                print(f"[..] ignoring unexpected id={can_id:08X}")


def upload(can, image, start_timeout, ack_timeout, reboot_id=None):
    total = len(image)
    print(f"Image: {total} bytes, {(total + BLOCK_SIZE - 1) // BLOCK_SIZE} block(s)")

    # --- Start handshake: spam RX_START until the target acks ----------------
    # The target only listens for RX_START briefly after entering the
    # bootloader. With reboot_id set we keep nudging the running application to
    # reboot into the bootloader (harmless once it is already there); otherwise
    # we rely on the operator resetting the board by hand.
    if reboot_id is not None:
        print(f"Requesting graceful reboot via id {reboot_id:08X}...")
    else:
        print("Sending START -- reset the target now...")
    deadline = time.monotonic() + start_timeout
    while True:
        if reboot_id is not None:
            can.send(reboot_id)
        can.send(CANID_RX_START, struct.pack(">I", total))
        if can.wait_for(CANID_TX_START_ACK, 0.1) is not None:
            break
        if time.monotonic() > deadline:
            raise TimeoutError("no START_ACK -- did the target reset in time?")
    print("START acked, flash erased.")

    # --- Blocks -------------------------------------------------------------
    for base in range(0, total, BLOCK_SIZE):
        block = image[base:base + BLOCK_SIZE]
        can.send(CANID_RX_BLOCK_START, struct.pack(">I", base))
        if can.wait_for(CANID_TX_BLOCK_START_ACK, ack_timeout) is None:
            raise TimeoutError(f"no BLOCK_START_ACK at offset {base}")

        for seq, off in enumerate(range(0, len(block), FRAME_SIZE)):
            chunk = block[off:off + FRAME_SIZE]
            can.send(CANID_RX_DATA | seq, chunk)

        can.send(CANID_RX_BLOCK_FINISH)
        ack = can.wait_for(CANID_TX_BLOCK_FINISH_ACK, ack_timeout)
        if ack is None:
            raise TimeoutError(f"no BLOCK_FINISH_ACK at offset {base}")
        written = struct.unpack(">I", ack[:4])[0]
        if written != len(block):
            raise RuntimeError(
                f"block at {base}: target wrote {written} bytes, expected {len(block)} "
                "(dropped data frames)")
        print(f"  block {base:#08x} ({len(block)} bytes) ok")

    # --- Verify -------------------------------------------------------------
    can.send(CANID_RX_VERIFY)
    resp = can.wait_for(CANID_TX_VERIFY_RESP, ack_timeout)
    if resp is None:
        raise TimeoutError("no VERIFY_RESP")
    device_crc = struct.unpack(">I", resp[:4])[0]
    host_crc = zlib.crc32(image) & 0xFFFFFFFF
    if device_crc != host_crc:
        raise RuntimeError(
            f"CRC mismatch: device={device_crc:08X} host={host_crc:08X}")
    print(f"Verify OK (crc32={host_crc:08X}). Upload complete.")
    can.send(CANID_RX_BOOT)

def main():
    ap = argparse.ArgumentParser(description="Upload a firmware image to CANBL.")
    ap.add_argument("image", help="application binary (.bin)")
    ap.add_argument("--host", default=DEFAULT_HOST, help=f"adapter host (default {DEFAULT_HOST})")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"adapter port (default {DEFAULT_PORT})")
    ap.add_argument("--start-timeout", type=float, default=30.0,
                    help="seconds to keep retrying START while waiting for a reset")
    ap.add_argument("--ack-timeout", type=float, default=2.0,
                    help="seconds to wait for each block/verify ack")
    ap.add_argument("--reboot-id", type=lambda x: int(x, 0), default=CANID_REBOOT,
                    help=f"CAN id for the graceful reboot frame (default {CANID_REBOOT:#010x})")
    ap.add_argument("--manual-reset", action="store_true",
                    help="don't send the graceful reboot frame; wait for a manual reset "
                         "instead (use when the application firmware is unresponsive)")
    ap.add_argument("-v", "--verbose", action="store_true", help="log every frame")
    args = ap.parse_args()

    with open(args.image, "rb") as f:
        image = f.read()
    if not image:
        sys.exit("error: image file is empty")

    reboot_id = None if args.manual_reset else args.reboot_id

    can = CanTcp(args.host, args.port, verbose=args.verbose)
    try:
        upload(can, image, args.start_timeout, args.ack_timeout, reboot_id)
    except (TimeoutError, RuntimeError, ConnectionError) as e:
        sys.exit(f"upload failed: {e}")
    finally:
        can.close()


if __name__ == "__main__":
    main()

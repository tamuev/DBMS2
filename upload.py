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
    -> RX_BOOT         []                       (no ack; target runs the app)

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
# Every bootloader frame is 0x0C00 <device> <msg> <seq>:
#     id = 0x0C000000 | (device << 12) | (msg << 8) | seq
# The device nibble selects which board on the bus we are addressing, so all
# IDs below are derived from the --device chosen at runtime.
CANBL_BASE = 0x0C000000

# Message-type nibbles (bits 8-11).
MSG_ANNOUNCE         = 0x0
MSG_START            = 0x1
MSG_START_ACK        = 0x2
MSG_BLOCK_START      = 0x3
MSG_BLOCK_START_ACK  = 0x4
MSG_BLOCK_FINISH     = 0x5
MSG_BLOCK_FINISH_ACK = 0x6
MSG_VERIFY           = 0x7
MSG_VERIFY_RESP      = 0x8
MSG_BOOT             = 0x9
MSG_DATA             = 0xF

# Known device IDs (see CANBL_DEV_* in App/canbl.h).
DEVICES = {
    "dbms":     1,
    "fl_node":  2,
    "fr_node":  3,
    "bl_node":  4,
    "br_node":  5,
    "lvswp":    6,
    "dcu":      7,
    "roadlink": 8,
}


def canid(device, msg):
    """CAN ID for a message type on a given device (seq nibble = 0)."""
    return CANBL_BASE | (device << 12) | (msg << 8)


class Ids:
    """All bootloader CAN IDs for one target device."""

    def __init__(self, device):
        self.device                  = device
        self.RX_START                = canid(device, MSG_START)
        self.TX_START_ACK            = canid(device, MSG_START_ACK)
        self.RX_BLOCK_START          = canid(device, MSG_BLOCK_START)
        self.TX_BLOCK_START_ACK      = canid(device, MSG_BLOCK_START_ACK)
        self.RX_BLOCK_FINISH         = canid(device, MSG_BLOCK_FINISH)
        self.TX_BLOCK_FINISH_ACK     = canid(device, MSG_BLOCK_FINISH_ACK)
        self.RX_VERIFY               = canid(device, MSG_VERIFY)
        self.TX_VERIFY_RESP          = canid(device, MSG_VERIFY_RESP)
        self.RX_BOOT                 = canid(device, MSG_BOOT)
        self.RX_DATA                 = canid(device, MSG_DATA)


# Per-device reboot frames. A blank frame on a device's reboot ID asks its
# running application to reboot into the bootloader, setting the flash-request
# flag so the bootloader stays in flash mode. This is an application-level magic
# frame, independent of the CANBL device scheme, so each board picks its own
# arbitrary ID. Keyed by device id (see DEVICES). A device with no entry here
# has no graceful-reboot frame, so the uploader falls back to a manual reset.
CANID_REBOOT = {
    DEVICES["dbms"]: 0x0B00B007,
    DEVICES["fl_node"]: 0x217,
    DEVICES["fr_node"]: 0x227,
    DEVICES["bl_node"]: 0x237,
    DEVICES["br_node"]: 0x247,
}

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
        # if self.verbose:
            # print(f"[rx] id={can_id:08X} k={pkt[0]:02X} data={data.hex()}")
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
            # if self.verbose:
                # print(f"[..] ignoring unexpected id={can_id:08X}")


def upload(can, ids, image, start_timeout, ack_timeout, reboot_id=None):
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
        can.send(ids.RX_START, struct.pack(">I", total))
        if can.wait_for(ids.TX_START_ACK, 0.1) is not None:
            break
        if time.monotonic() > deadline:
            raise TimeoutError("no START_ACK -- did the target reset in time?")
    print("START acked, flash erased.")

    # --- Blocks -------------------------------------------------------------
    for base in range(0, total, BLOCK_SIZE):
        block = image[base:base + BLOCK_SIZE]
        can.send(ids.RX_BLOCK_START, struct.pack(">I", base))
        if can.wait_for(ids.TX_BLOCK_START_ACK, ack_timeout) is None:
            raise TimeoutError(f"no BLOCK_START_ACK at offset {base}")

        for seq, off in enumerate(range(0, len(block), FRAME_SIZE)):
            chunk = block[off:off + FRAME_SIZE]
            can.send(ids.RX_DATA | seq, chunk)

        can.send(ids.RX_BLOCK_FINISH)
        ack = can.wait_for(ids.TX_BLOCK_FINISH_ACK, ack_timeout)
        if ack is None:
            raise TimeoutError(f"no BLOCK_FINISH_ACK at offset {base}")
        written = struct.unpack(">I", ack[:4])[0]
        if written != len(block):
            raise RuntimeError(
                f"block at {base}: target wrote {written} bytes, expected {len(block)} "
                "(dropped data frames)")
        print(f"  block {base:#08x} ({len(block)} bytes) ok")

    # --- Verify -------------------------------------------------------------
    can.send(ids.RX_VERIFY)
    resp = can.wait_for(ids.TX_VERIFY_RESP, ack_timeout)
    if resp is None:
        raise TimeoutError("no VERIFY_RESP")
    device_crc = struct.unpack(">I", resp[:4])[0]
    host_crc = zlib.crc32(image) & 0xFFFFFFFF
    if device_crc != host_crc:
        raise RuntimeError(
            f"CRC mismatch: device={device_crc:08X} host={host_crc:08X}")
    print(f"Verify OK (crc32={host_crc:08X}). Upload complete.")

    # --- Boot ---------------------------------------------------------------
    # Verified, so tell the target to exit the bootloader and run the new app.
    # This is fire-and-forget: the target boots immediately and sends no ack.
    can.send(ids.RX_BOOT)
    print("Sent BOOT -- target running application.")


def parse_device(s):
    """Accept a device name (e.g. 'dbms') or a numeric id (1-8, any base)."""
    key = s.strip().lower()
    if key in DEVICES:
        return DEVICES[key]
    try:
        val = int(s, 0)
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"unknown device {s!r}; expected one of {', '.join(DEVICES)} or a number")
    if not 1 <= val <= 0xF:
        raise argparse.ArgumentTypeError(f"device id {val} out of range (1-15)")
    return val


def main():
    ap = argparse.ArgumentParser(description="Upload a firmware image to CANBL.")
    ap.add_argument("image", help="application binary (.bin)")
    ap.add_argument("--device", type=parse_device, required=True,
                    help="target device: name (" + ", ".join(DEVICES) + ") or numeric id")
    ap.add_argument("--host", default=DEFAULT_HOST, help=f"adapter host (default {DEFAULT_HOST})")
    ap.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"adapter port (default {DEFAULT_PORT})")
    ap.add_argument("--start-timeout", type=float, default=30.0,
                    help="seconds to keep retrying START while waiting for a reset")
    ap.add_argument("--ack-timeout", type=float, default=2.0,
                    help="seconds to wait for each block/verify ack")
    ap.add_argument("--reboot-id", type=lambda x: int(x, 0), default=None,
                    help="CAN id for the graceful reboot frame (overrides the per-device "
                         "default; if the device has no default either, a manual reset is used)")
    ap.add_argument("--manual-reset", action="store_true",
                    help="don't send the graceful reboot frame; wait for a manual reset "
                         "instead (use when the application firmware is unresponsive)")
    ap.add_argument("-v", "--verbose", action="store_true", help="log every frame")
    args = ap.parse_args()

    with open(args.image, "rb") as f:
        image = f.read()
    if not image:
        sys.exit("error: image file is empty")

    # Resolve the reboot frame: an explicit --reboot-id wins; --manual-reset
    # forces a manual reset; otherwise use the device's own default, falling
    # back to a manual reset when the device has no reboot ID configured.
    if args.manual_reset:
        reboot_id = None
    elif args.reboot_id is not None:
        reboot_id = args.reboot_id
    else:
        reboot_id = CANID_REBOOT.get(args.device)
        if reboot_id is None:
            print("No reboot id for this device -- falling back to manual reset.")

    ids = Ids(args.device)
    print(f"Target device id: {args.device} (base id {canid(args.device, 0):08X})")

    can = CanTcp(args.host, args.port, verbose=args.verbose)
    try:
        upload(can, ids, image, args.start_timeout, args.ack_timeout, reboot_id)
    except (TimeoutError, RuntimeError, ConnectionError) as e:
        sys.exit(f"upload failed: {e}")
    finally:
        can.close()


if __name__ == "__main__":
    main()

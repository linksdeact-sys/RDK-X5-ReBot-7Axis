"""Build/verify a hash-locked, project-local DM 1.1.0 AArch64 RX timing fix.

No USB access. Never overwrite the installed SDK, an existing artifact, or any
loaded library. This is a local compatibility modification, NOT a vendor release.
"""
import argparse
import hashlib
from pathlib import Path
import struct

SOURCE_SHA256 = "4a73c8ec7d64342f0d0a1156d5102f519c5f176fb23696740f254fe6492c3f2a"
OUTPUT_SHA256 = "282e6ca9d5f11d9ded3de51eb3ce249857c918c177ca9a14ca34e7f32fdde806"
SOURCE_SIZE = 117912
WAIT_OFFSET = 0x123B0
LOAD_OFFSET = 0xD1C8
OLD_WAIT = struct.pack("<qq", 0, 100_000_000)
NEW_WAIT = struct.pack("<qq", 0, 1_000_000)


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def transform(data):
    if len(data) != SOURCE_SIZE or sha256(data) != SOURCE_SHA256:
        raise ValueError("Unsupported SDK: exact reviewed DM 1.1.0 AArch64 SHA256 required")
    if data[:6] != b"\x7fELF\x02\x01" or data[18:20] != b"\xb7\x00":
        raise ValueError("Expected little-endian ELF64 AArch64")
    if data[LOAD_OFFSET:LOAD_OFFSET+4] != bytes.fromhex("00ecc03d"):
        raise ValueError("Reviewed notification-thread load instruction is absent")
    if data[WAIT_OFFSET:WAIT_OFFSET+16] != OLD_WAIT:
        raise ValueError("Reviewed idle timespec is absent")
    output = data[:WAIT_OFFSET] + NEW_WAIT + data[WAIT_OFFSET+16:]
    if sha256(output) != OUTPUT_SHA256:
        raise ValueError("Output checksum mismatch")
    return output


def verify(path):
    if sha256(Path(path).read_bytes()) != OUTPUT_SHA256:
        raise ValueError("RX library integrity check failed; refusing to load")


def prepare(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    if source == output:
        raise ValueError("Source library must never be overwritten")
    data = transform(source.read_bytes())
    if output.exists():
        verify(output)  # Idempotent only for an already matching artifact.
    else:
        with output.open("xb") as stream:
            stream.write(data)
    verify(output)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--verify", type=Path)
    args = parser.parse_args()
    if args.verify is not None:
        if args.source is not None or args.output is not None:
            parser.error("--verify cannot be combined with --source/--output")
        verify(args.verify)
    else:
        if args.source is None or args.output is None:
            parser.error("--source and --output are both required")
        prepare(args.source, args.output)
        print(f"Verified project-local RX idle wait: 100ms -> 1ms; SHA256={OUTPUT_SHA256}")


if __name__ == "__main__":
    main()

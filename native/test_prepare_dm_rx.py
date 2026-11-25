"""Offline artifact integrity tests; no SDK is loaded and no USB is opened."""
import hashlib
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from native import prepare_dm_rx as fix


class RxLibraryTests(unittest.TestCase):
    def fixture(self):
        source = bytearray(fix.SOURCE_SIZE)
        source[:6], source[18:20] = b"\x7fELF\x02\x01", b"\xb7\x00"
        source[fix.LOAD_OFFSET:fix.LOAD_OFFSET+4] = bytes.fromhex("00ecc03d")
        source[fix.WAIT_OFFSET:fix.WAIT_OFFSET+16] = fix.OLD_WAIT
        output = source[:fix.WAIT_OFFSET] + fix.NEW_WAIT + source[fix.WAIT_OFFSET+16:]
        return bytes(source), bytes(output)

    def test_unknown_library_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "Unsupported SDK"):
            fix.transform(b"unsupported")
        source, _ = self.fixture()
        with self.assertRaisesRegex(ValueError, "Unsupported SDK"):
            fix.transform(source)

    def test_only_idle_timespec_changes(self):
        source, expected = self.fixture()
        with patch.object(fix, "SOURCE_SHA256", fix.sha256(source)), \
                patch.object(fix, "OUTPUT_SHA256", fix.sha256(expected)):
            output = fix.transform(source)
        self.assertEqual(output, expected)
        changed = [i for i, (a, b) in enumerate(zip(source, output)) if a != b]
        self.assertTrue(changed)
        self.assertTrue(all(fix.WAIT_OFFSET+8 <= i < fix.WAIT_OFFSET+16 for i in changed))
        self.assertEqual(output[:fix.WAIT_OFFSET], source[:fix.WAIT_OFFSET])
        self.assertEqual(output[fix.WAIT_OFFSET+16:], source[fix.WAIT_OFFSET+16:])

    def test_never_overwrite_source_or_existing_different_output(self):
        source, expected = self.fixture()
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(fix, "SOURCE_SHA256", fix.sha256(source)), \
                patch.object(fix, "OUTPUT_SHA256", fix.sha256(expected)):
            original, output = Path(folder)/"original.so", Path(folder)/"rx.so"
            original.write_bytes(source)
            with self.assertRaisesRegex(ValueError, "never be overwritten"):
                fix.prepare(original, original)
            output.write_bytes(b"another or partial artifact")
            with self.assertRaisesRegex(ValueError, "integrity"):
                fix.prepare(original, output)
            self.assertEqual(output.read_bytes(), b"another or partial artifact")
            self.assertEqual(original.read_bytes(), source)

    def test_prepare_is_idempotent_and_preserves_source(self):
        source, expected = self.fixture()
        with tempfile.TemporaryDirectory() as folder, \
                patch.object(fix, "SOURCE_SHA256", fix.sha256(source)), \
                patch.object(fix, "OUTPUT_SHA256", fix.sha256(expected)):
            original, output = Path(folder)/"original.so", Path(folder)/"rx.so"
            original.write_bytes(source)
            fix.prepare(original, output)
            fix.prepare(original, output)
            fix.verify(output)
            self.assertEqual(output.read_bytes(), expected)
            self.assertEqual(original.read_bytes(), source)

    def test_reviewed_vendor_binary_when_available(self):
        source = Path(__file__).parent/"dm-sdk-reference/C&C++/lib/v1.1.0/linux/arm64/libdm_device.so"
        if not source.exists():
            self.skipTest("Optional official reference SDK not checked out")
        data = source.read_bytes()
        self.assertEqual(hashlib.sha256(data).hexdigest(), fix.SOURCE_SHA256)
        self.assertEqual(fix.sha256(fix.transform(data)), fix.OUTPUT_SHA256)

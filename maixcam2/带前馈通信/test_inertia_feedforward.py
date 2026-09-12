import struct
import unittest

from inertia_feedforward import (
    FLAG_ENCODER_VALID,
    FLAG_IMU_VALID,
    FLAG_TASK_456,
    InertiaFeedForward,
    InertiaTelemetryParser,
    crc16_modbus,
)


def make_frame(sequence=0, task=4, state=1,
               flags=FLAG_IMU_VALID | FLAG_ENCODER_VALID | FLAG_TASK_456,
               values=None):
    if values is None:
        values = [0] * 11
    raw = bytearray(32)
    raw[:8] = bytes((0xA5, 0x5A, 0x01, 26,
                     sequence, task, state, flags))
    struct.pack_into("<11h", raw, 8, *values)
    crc = crc16_modbus(raw[:30])
    raw[30] = crc & 0xFF
    raw[31] = crc >> 8
    return bytes(raw)


def config(**overrides):
    cfg = {
        "ff_enabled": 1,
        "ff_k_pwm": 0.001,
        "ff_k_encoder": 0.001,
        "ff_k_imu": 0.005,
        "ff_imu_axis": 0,
        "ff_imu_invert": 0,
        "ff_max_deg": 4.0,
        "ff_attack_ms": 100.0,
        "ff_release_ms": 350.0,
        "ff_timeout_ms": 100.0,
        "ff_omega_max_dps": 30.0,
        "ff_bias_lpf": 0.1,
    }
    cfg.update(overrides)
    return cfg


class ParserTests(unittest.TestCase):
    def test_crc16_modbus_reference_vector(self):
        self.assertEqual(crc16_modbus(b"123456789"), 0x4B37)

    def test_split_frame_and_noise_resync(self):
        parser = InertiaTelemetryParser()
        raw = make_frame(sequence=7, values=list(range(11)))
        self.assertEqual(parser.feed(b"noise\xA5"), [])
        self.assertEqual(parser.feed(raw[1:14]), [])
        frames = parser.feed(raw[14:])
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].sequence, 7)
        self.assertEqual(frames[0].encoder_accel_mms2, 7)
        self.assertGreaterEqual(parser.noise_bytes, 5)

    def test_bad_crc_does_not_hide_following_frame(self):
        parser = InertiaTelemetryParser()
        bad = bytearray(make_frame(sequence=1))
        bad[12] ^= 0x40
        frames = parser.feed(bytes(bad) + make_frame(sequence=2))
        self.assertEqual([f.sequence for f in frames], [2])
        self.assertEqual(parser.crc_errors, 1)

    def test_sequence_wrap_drop_and_duplicate_stats(self):
        parser = InertiaTelemetryParser()
        frames = parser.feed(make_frame(sequence=254)
                             + make_frame(sequence=1)
                             + make_frame(sequence=1))
        self.assertEqual(len(frames), 3)
        self.assertEqual(parser.dropped_frames, 2)
        self.assertEqual(parser.duplicate_frames, 1)
        self.assertFalse(frames[-1].sequence_advanced)


class FeedForwardTests(unittest.TestCase):
    def parse_one(self, raw):
        return InertiaTelemetryParser().feed(raw)[0]

    def test_bias_terms_gate_and_timeout_release(self):
        ff = InertiaFeedForward(config())

        stable_values = [100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        ff.note_frame(self.parse_one(make_frame(
            sequence=1, state=3, values=stable_values)), 0)
        self.assertEqual(ff.update(0, 0.1)["reason"], "state")
        self.assertAlmostEqual(ff.imu_bias_mg, 100.0)

        # target = 1000*0.001 + 500*0.001 + (150-100)*0.005 = 1.75 deg
        running = [150, 0, 0, 0, 0, 0, 0, 500, 0, 0, 1000]
        ff.note_frame(self.parse_one(make_frame(
            sequence=2, state=1, values=running)), 20)
        active = ff.update(20, 0.1)
        self.assertTrue(active["active"])
        self.assertAlmostEqual(active["target_deg"], 1.75)
        self.assertAlmostEqual(active["theta_deg"], 1.75)

        timed_out = ff.update(121, 0.1)
        self.assertFalse(timed_out["active"])
        self.assertEqual(timed_out["reason"], "timeout")
        self.assertLess(timed_out["theta_deg"], active["theta_deg"])
        self.assertGreaterEqual(timed_out["theta_deg"], 0.0)

    def test_protocol_task_gate_and_disabled_fallback(self):
        ff = InertiaFeedForward(config(ff_k_pwm=0.01))
        frame = self.parse_one(make_frame(
            sequence=1, flags=FLAG_IMU_VALID, values=[0] * 10 + [100]))
        ff.note_frame(frame, 0)
        self.assertEqual(ff.update(0, 0.02)["reason"], "task")

        ff.sync(config(ff_enabled=0, ff_k_pwm=0.01))
        self.assertEqual(ff.update(20, 0.02)["reason"], "disabled")

    def test_duplicate_sequence_does_not_refresh_freshness(self):
        parser = InertiaTelemetryParser()
        ff = InertiaFeedForward(config())
        first, duplicate = parser.feed(
            make_frame(sequence=9) + make_frame(sequence=9))
        ff.note_frame(first, 10)
        ff.note_frame(duplicate, 90)
        self.assertEqual(ff.frame_age_ms(111), 101)
        self.assertEqual(ff.update(111, 0.02)["reason"], "timeout")

    def test_full_scale_offset_releases_within_configured_time(self):
        ff = InertiaFeedForward(config())
        ff.theta_deg = 4.0
        for index in range(7):
            state = ff.update(index * 50, 0.05, allow=False)
            self.assertLessEqual(abs(state["omega_dps"]), 30.0)
        self.assertAlmostEqual(ff.theta_deg, 0.0, places=6)


if __name__ == "__main__":
    unittest.main()

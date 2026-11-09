"""
Unit Test Suite for ADAS Telemetry Pipeline
Verifies boundary edge cases, corrupted packet filtering,
and deterministic AEB state transitions.
"""

import unittest
from telemetry_pipeline import TelemetryPipeline, ADASState


class TestADASTelemetryPipeline(unittest.TestCase):
    def setUp(self):
        self.pipeline = TelemetryPipeline(ttc_alert_threshold=2.8, ttc_aeb_threshold=1.5)

    def test_normal_cruising_state(self):
        """Test safe gap with no collision risk (closing speed <= 0)."""
        frame = {
            "can_id": "0x1A0",
            "timestamp_ms": 1000,
            "vehicle_speed_mps": 25.0,  # ~55 mph
            "lead_distance_m": 50.0,
            "relative_speed_mps": 0.0,  # Matching speed
            "sensor_status_ok": True
        }
        state, ttc = self.pipeline.process_frame(frame)
        self.assertEqual(state, ADASState.NORMAL)
        self.assertIsNone(ttc)
        self.assertEqual(self.pipeline.processed_packets, 1)

    def test_driver_alert_trigger(self):
        """TTC = 40m / 20m/s = 2.0s -> Should trigger ALERT (between 1.5s and 2.8s)."""
        frame = {
            "can_id": "0x1A0",
            "timestamp_ms": 1100,
            "vehicle_speed_mps": 28.0,
            "lead_distance_m": 40.0,
            "relative_speed_mps": 20.0,
            "sensor_status_ok": True
        }
        state, ttc = self.pipeline.process_frame(frame)
        self.assertEqual(state, ADASState.ALERT)
        self.assertEqual(ttc, 2.0)

    def test_autonomous_emergency_braking_trigger(self):
        """TTC = 12m / 15m/s = 0.8s -> Imminent collision, must trigger AEB."""
        frame = {
            "can_id": "0x1A0",
            "timestamp_ms": 1200,
            "vehicle_speed_mps": 25.0,
            "lead_distance_m": 12.0,
            "relative_speed_mps": 15.0,
            "sensor_status_ok": True
        }
        state, ttc = self.pipeline.process_frame(frame)
        self.assertEqual(state, ADASState.AEB)
        self.assertEqual(ttc, 0.8)

    def test_corrupted_negative_distance_packet(self):
        """Negative distance reading is sensor noise -> Drop packet and maintain state."""
        frame = {
            "can_id": "0x1A0",
            "timestamp_ms": 1300,
            "vehicle_speed_mps": 20.0,
            "lead_distance_m": -5.0,  # Anomaly
            "relative_speed_mps": 10.0,
            "sensor_status_ok": True
        }
        state, ttc = self.pipeline.process_frame(frame)
        self.assertEqual(self.pipeline.dropped_packets, 1)
        self.assertEqual(self.pipeline.processed_packets, 0)
        self.assertEqual(state, ADASState.NORMAL)

    def test_invalid_can_id_rejection(self):
        """Non-radar CAN frame (e.g., 0x2B0) should be discarded by this pipeline."""
        frame = {
            "can_id": "0x2B0",
            "timestamp_ms": 1400,
            "vehicle_speed_mps": 20.0,
            "lead_distance_m": 30.0,
            "relative_speed_mps": 5.0,
            "sensor_status_ok": True
        }
        state, ttc = self.pipeline.process_frame(frame)
        self.assertEqual(self.pipeline.dropped_packets, 1)


if __name__ == "__main__":
    unittest.main()
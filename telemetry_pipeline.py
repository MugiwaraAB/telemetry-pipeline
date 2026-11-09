"""
ADAS Sensor Telemetry Pipeline & Active Safety Controller
Simulates real-time CAN bus telemetry ingestion (Radar 0x1A0),
boundary validation, dynamic Time-to-Collision (TTC) calculations,
and Autonomous Emergency Braking (AEB) state transitions.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Optional, Tuple
import matplotlib.pyplot as plt


class ADASState(Enum):
    NORMAL = "NORMAL"
    ALERT = "ALERT"
    AEB = "AEB_ENGAGED"


@dataclass
class TelemetryFrame:
    can_id: str
    timestamp_ms: int
    vehicle_speed_mps: float     # Speed in meters/second
    lead_distance_m: float       # Forward distance to obstacle in meters
    relative_speed_mps: float    # Closing speed (positive means closing in)
    sensor_status_ok: bool


class TelemetryPipeline:
    def __init__(self, ttc_alert_threshold: float = 2.8, ttc_aeb_threshold: float = 1.5):
        self.ttc_alert_threshold = ttc_alert_threshold
        self.ttc_aeb_threshold = ttc_aeb_threshold
        self.current_state = ADASState.NORMAL
        self.dropped_packets = 0
        self.processed_packets = 0

    @staticmethod
    def validate_packet(raw_data: dict) -> Tuple[bool, Optional[str]]:
        """
        Validates telemetry boundary limits and flags corrupted/malformed frames.
        """
        required_fields = {
            "can_id", "timestamp_ms", "vehicle_speed_mps",
            "lead_distance_m", "relative_speed_mps", "sensor_status_ok"
        }
        if not required_fields.issubset(raw_data.keys()):
            return False, "Missing telemetry fields"

        if raw_data["can_id"] != "0x1A0":
            return False, f"Invalid arbitration CAN ID: {raw_data['can_id']}"

        if not raw_data["sensor_status_ok"]:
            return False, "Sensor hardware fault reported"

        if raw_data["lead_distance_m"] < 0:
            return False, f"Negative distance anomaly: {raw_data['lead_distance_m']}"

        if raw_data["vehicle_speed_mps"] < 0 or raw_data["vehicle_speed_mps"] > 70:
            return False, f"Unphysical vehicle speed: {raw_data['vehicle_speed_mps']}"

        return True, None

    @staticmethod
    def calculate_ttc(distance_m: float, closing_speed_mps: float) -> Optional[float]:
        """
        Calculates Time-To-Collision (TTC) in seconds.
        """
        if closing_speed_mps <= 0:
            return None
        return round(distance_m / closing_speed_mps, 3)

    def process_frame(self, raw_data: dict) -> Tuple[ADASState, Optional[float]]:
        """
        Ingests a raw data frame, validates integrity, and computes state machine updates.
        """
        is_valid, _ = self.validate_packet(raw_data)
        if not is_valid:
            self.dropped_packets += 1
            return self.current_state, None

        self.processed_packets += 1
        distance = raw_data["lead_distance_m"]
        closing_speed = raw_data["relative_speed_mps"]

        ttc_val = self.calculate_ttc(distance, closing_speed)

        if ttc_val is not None and ttc_val <= self.ttc_aeb_threshold:
            self.current_state = ADASState.AEB
        elif ttc_val is not None and ttc_val <= self.ttc_alert_threshold:
            self.current_state = ADASState.ALERT
        else:
            self.current_state = ADASState.NORMAL

        return self.current_state, ttc_val


def generate_telemetry_plot():
    timestamps = [100, 200, 300, 350, 400]
    distances = [60.0, 45.0, 20.0, 12.0, 5.0]
    states = ["NORMAL", "ALERT", "AEB", "AEB", "AEB"]
    colors = {"NORMAL": "green", "ALERT": "orange", "AEB": "red"}

    plt.figure()
    for t, d, s in zip(timestamps, distances, states):
        existing_labels = plt.gca().get_legend_handles_labels()[1]
        plt.scatter(t, d, color=colors[s], s=120, zorder=3, label=s if s not in existing_labels else "")

    plt.plot(timestamps, distances, linestyle="--", color="gray", alpha=0.6, zorder=2)
    plt.axhline(y=15.0, color="red", linestyle=":", label="AEB Threshold Distance")
    plt.title("ADAS Radar Telemetry: Lead Distance vs. Time (FSM States)")
    plt.xlabel("Timestamp (ms)")
    plt.ylabel("Lead Vehicle Distance (m)")
    plt.grid(True, linestyle="--", alpha=0.5)
    plt.legend()
    plt.tight_layout()
    plt.savefig("telemetry_plot.png")
    print("Telemetry plot saved as telemetry_plot.png")


if __name__ == "__main__":
    pipeline = TelemetryPipeline(ttc_alert_threshold=2.8, ttc_aeb_threshold=1.5)

    sample_stream = [
        {"can_id": "0x1A0", "timestamp_ms": 100, "vehicle_speed_mps": 25.0, "lead_distance_m": 60.0, "relative_speed_mps": 0.0, "sensor_status_ok": True},
        {"can_id": "0x1A0", "timestamp_ms": 200, "vehicle_speed_mps": 25.0, "lead_distance_m": 45.0, "relative_speed_mps": 18.0, "sensor_status_ok": True},
        {"can_id": "0x1A0", "timestamp_ms": 300, "vehicle_speed_mps": 25.0, "lead_distance_m": 12.0, "relative_speed_mps": 15.0, "sensor_status_ok": True},
        {"can_id": "0x1A0", "timestamp_ms": 400, "vehicle_speed_mps": 25.0, "lead_distance_m": -2.0, "relative_speed_mps": 10.0, "sensor_status_ok": True},
    ]

    print("\n--- INITIATING ADAS TELEMETRY PROCESSING LOOP ---")
    for frame in sample_stream:
        state, ttc = pipeline.process_frame(frame)
        ttc_display = f"{ttc:.2f}s" if ttc is not None else "N/A (Safe)"
        print(f"Time: {frame['timestamp_ms']}ms | Dist: {frame['lead_distance_m']}m | TTC: {ttc_display} | State: {state.value}")

    print(f"\nProcessing Complete: {pipeline.processed_packets} valid frames | {pipeline.dropped_packets} dropped anomalies\n")
    generate_telemetry_plot()
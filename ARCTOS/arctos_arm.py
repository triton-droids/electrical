"""
ARCTOS Arm Controller
Control the ARCTOS 6-DOF robotic arm via CAN bus.
"""

import can
import time
import threading
from typing import List, Optional


class ArctosArm:
    """Controller for the ARCTOS robotic arm."""

    # Gear ratios for each joint (J1-J6)
    GEAR_RATIOS: List[float] = [
        24.6,   # J1 (X)
        75,     # J2 (Y)
        150,    # J3 (Z)
        48,     # J4 (A)
        67.82,  # J5 (B)
        67.82,  # J6 (C)
    ]

    # Direction inversion per joint
    INVERT_DIRECTION: List[bool] = [
        True,   # J1
        True,   # J2
        False,  # J3
        False,  # J4
        False,  # J5
        False,  # J6
    ]

    PPR = 3200  # Pulses per motor revolution
    BITRATE = 500000
    ARB_BASE = 0x000

    # Default motion parameters
    DEFAULT_RPM = 128
    DEFAULT_ACC = 5

    def __init__(self, com_port: str):
        """
        Initialize the ARCTOS arm controller.

        Args:
            com_port: Serial port for the SLCAN adapter (e.g., "COM6")
        """
        self.com_port = com_port
        self.bus: Optional[can.Bus] = None
        self.current_angles: List[float] = [0.0] * 6
        self._rx_thread: Optional[threading.Thread] = None
        self._rx_stop = False

    @staticmethod
    def _hex(data: bytes) -> str:
        return " ".join(f"{b:02X}" for b in data)

    def _rx_loop(self) -> None:
        """Background receiver thread."""
        while not self._rx_stop:
            if self.bus is None:
                time.sleep(0.1)
                continue
            try:
                msg = self.bus.recv(timeout=0.5)
                if msg:
                    print(f"[RX] ID=0x{msg.arbitration_id:03X} DATA=[{self._hex(msg.data)}]")
            except Exception:
                pass

    def connect(self) -> None:
        """Open the CAN bus connection."""
        if self.bus is not None:
            self.disconnect()
        self.bus = can.Bus(
            interface="slcan",
            channel=self.com_port,
            bitrate=self.BITRATE
        )
        self._rx_stop = False
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def disconnect(self) -> None:
        """Close the CAN bus connection."""
        self._rx_stop = True
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=1.0)
            self._rx_thread = None
        if self.bus is not None:
            try:
                self.bus.shutdown()
            except Exception:
                pass
            self.bus = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()
        return False

    def _degrees_to_pulses(self, degrees: float, joint: int) -> int:
        """
        Convert degrees to motor pulses for a given joint.

        Args:
            degrees: Angle in degrees
            joint: Joint index (0-5)

        Returns:
            Number of pulses
        """
        gear_ratio = self.GEAR_RATIOS[joint]
        # degrees -> motor revolutions -> pulses
        motor_revs = degrees / 360.0 * gear_ratio
        return int(motor_revs * self.PPR)

    def _build_move_frame(
        self,
        can_id: int,
        pulses: int,
        reverse: bool,
        rpm: int,
        acc: int
    ) -> bytes:
        """Build a move command frame (0xFD command)."""
        speed_high = (rpm >> 8) & 0x0F
        direction_byte = (0x80 if reverse else 0x00) | speed_high
        speed_low = rpm & 0xFF

        pulses = abs(pulses)
        pulse_high = (pulses >> 16) & 0xFF
        pulse_mid = (pulses >> 8) & 0xFF
        pulse_low = pulses & 0xFF

        data = [0xFD, direction_byte, speed_low, acc, pulse_high, pulse_mid, pulse_low]
        checksum = (can_id + sum(data)) & 0xFF
        data.append(checksum)

        return bytes(data)

    def _send_move(
        self,
        joint: int,
        pulses: int,
        reverse: bool,
        rpm: int,
        acc: int
    ) -> None:
        """Send a move command to a specific joint."""
        if self.bus is None:
            raise RuntimeError("Not connected. Call connect() first.")

        arb_id = self.ARB_BASE + joint
        frame = self._build_move_frame(arb_id, pulses, reverse, rpm, acc)

        msg = can.Message(
            arbitration_id=arb_id,
            data=frame,
            is_extended_id=False
        )
        print(f"[TX] ID=0x{arb_id:03X} DATA=[{self._hex(frame)}]")
        self.bus.send(msg)

    def move_joint(
        self,
        joint: int,
        degrees: float,
        rpm: int = DEFAULT_RPM,
        acc: int = DEFAULT_ACC
    ) -> None:
        """
        Move a single joint by a relative amount.

        Args:
            joint: Joint index (0-5, i.e., J1-J6)
            degrees: Relative angle to move in degrees (positive or negative)
            rpm: Motor speed in RPM
            acc: Acceleration value
        """
        pulses = self._degrees_to_pulses(abs(degrees), joint)
        reverse = degrees < 0

        # Apply direction inversion
        if self.INVERT_DIRECTION[joint]:
            reverse = not reverse

        self._send_move(joint, pulses, reverse, rpm, acc)
        self.current_angles[joint] += degrees

    def set_joint_angles(
        self,
        j1: float,
        j2: float,
        j3: float,
        j4: float,
        j5: float,
        j6: float,
        rpm: int = DEFAULT_RPM,
        acc: int = DEFAULT_ACC
    ) -> None:
        """
        Move all joints to absolute angles.

        Args:
            j1-j6: Target angles in degrees for each joint
            rpm: Motor speed in RPM
            acc: Acceleration value
        """
        target_angles = [j1, j2, j3, j4, j5, j6]

        for joint, target in enumerate(target_angles):
            delta = target - self.current_angles[joint]
            if abs(delta) > 0.01:  # Skip if negligible movement
                self.move_joint(joint, delta, rpm, acc)

    def home(self) -> None:
        """Reset current angle tracking to zero (does not move the arm)."""
        self.current_angles = [0.0] * 6

    def ping_joint(self, joint: int) -> bool:
        """
        Ping a joint motor to check if it responds.

        Args:
            joint: Joint index (1-6)

        Returns:
            True if command was sent (response appears in RX thread)
        """
        if self.bus is None:
            raise RuntimeError("Not connected. Call connect() first.")

        arb_id = self.ARB_BASE + joint
        data = [0xF1]
        checksum = (arb_id + sum(data)) & 0xFF
        data.append(checksum)

        msg = can.Message(
            arbitration_id=arb_id,
            data=bytes(data),
            is_extended_id=False
        )
        print(f"[TX] ID=0x{arb_id:03X} DATA=[{self._hex(bytes(data))}]")
        self.bus.send(msg)
        return True


# Example usage
if __name__ == "__main__":
    with ArctosArm("COM6") as arm:
        arm.move_joint(2, -10)
        time.sleep(5)


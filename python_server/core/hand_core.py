"""Revo2 hand driver primitives.

This module keeps the Revo2 hardware details out of tool scripts:
auto-detect/open Modbus, switch to normalized finger units, send finger
targets, and close the serial connection.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Any, Sequence

from .mapping_utils import Revo2FingerTargets, to_sdk_positions

logger = logging.getLogger(__name__)


DEFAULT_REVO2_SPEED = 1000
DEFAULT_REVO2_BAUDRATE = 460800
DEFAULT_REVO2_SLAVE_ID = 0x7E


@dataclass
class Revo2HandConfig:
    """Connection/configuration for a BrainCo Revo2 hand."""

    side: str = "left"
    port: str | None = None
    baudrate: int | None = None
    slave_id: int | None = None
    speed: int = DEFAULT_REVO2_SPEED
    release_on_close: bool = False


class Revo2HandDriver:
    """Async Revo2 hand driver used by teleop/test scripts.

    The driver accepts normalized :class:`Revo2FingerTargets` and hides the
    SDK-specific Modbus client, slave id, unit mode, and speed vector.
    """

    def __init__(self, config: Revo2HandConfig | None = None) -> None:
        self.config = config or Revo2HandConfig()
        if self.config.side not in {"left", "right"}:
            raise ValueError("Revo2HandConfig.side 只能是 'left' 或 'right'")
        self.client: Any | None = None
        self.slave_id: int | None = None
        self.device_info: Any | None = None
        self._libstark: Any | None = None
        self._speeds = [int(self.config.speed)] * 6

    @property
    def connected(self) -> bool:
        return self.client is not None and self.slave_id is not None

    @property
    def side(self) -> str:
        return self.config.side

    @property
    def label(self) -> str:
        return f"{self.config.side} Revo2"

    async def connect(self) -> None:
        """Open the Revo2 Modbus connection and switch to normalized units."""
        if self.connected:
            raise RuntimeError("Revo2HandDriver is already connected")

        libstark = self._import_libstark()
        self._libstark = libstark

        if self.config.port is None:
            logger.info("自动探测 %s 设备...", self.label)
            protocol, port_name, baudrate, slave_id = (
                await libstark.auto_detect_modbus_revo2(None, True)
            )
            if protocol != libstark.StarkProtocolType.Modbus:
                raise RuntimeError(f"不支持的 Revo2 协议: {protocol}")
        else:
            port_name = self.config.port
            baudrate = self.config.baudrate or DEFAULT_REVO2_BAUDRATE
            slave_id = self.config.slave_id or DEFAULT_REVO2_SLAVE_ID

        logger.info(
            "打开 %s: port=%s baudrate=%s slave_id=0x%x",
            self.label,
            port_name,
            baudrate,
            slave_id,
        )
        self.client = await libstark.modbus_open(port_name, baudrate)
        self.slave_id = int(slave_id)

        self.device_info = await self.client.get_device_info(self.slave_id)
        logger.info(
            "%s 设备信息: %s",
            self.label,
            getattr(self.device_info, "description", self.device_info),
        )

        await self.client.set_finger_unit_mode(
            self.slave_id,
            libstark.FingerUnitMode.Normalized,
        )

    async def set_normalized_targets(self, targets: Revo2FingerTargets) -> list[int]:
        """Send normalized finger targets and return SDK positions for logging."""
        positions = to_sdk_positions(targets)
        await self.set_sdk_positions(positions)
        return positions

    async def set_sdk_positions(
        self,
        positions: Sequence[int],
        speeds: Sequence[int] | None = None,
    ) -> None:
        """Send SDK-space finger positions.

        Positions are in Revo2 normalized-unit integer space. The expected slot
        order is ``[Thumb Flex, Thumb Aux, Index, Middle, Ring, Pinky]``.
        """
        if not self.connected or self.client is None or self.slave_id is None:
            raise RuntimeError("Revo2HandDriver is not connected")
        pos = [int(v) for v in positions]
        if len(pos) != 6:
            raise ValueError(f"Revo2 positions must have length 6, got {len(pos)}")
        spd = [int(v) for v in (speeds or self._speeds)]
        if len(spd) != 6:
            raise ValueError(f"Revo2 speeds must have length 6, got {len(spd)}")
        await self.client.set_finger_positions_and_speeds(self.slave_id, pos, spd)

    async def release(self) -> None:
        """Open all fingers."""
        await self.set_sdk_positions([0, 0, 0, 0, 0, 0])

    async def close(self) -> None:
        """Close the Revo2 Modbus connection."""
        if self.client is None:
            return

        client = self.client
        libstark = self._libstark
        try:
            if self.config.release_on_close and self.connected:
                await self.release()
        finally:
            try:
                if libstark is not None:
                    libstark.modbus_close(client)
            finally:
                self.client = None
                self.slave_id = None
                logger.info("%s 已断开。", self.label)

    @staticmethod
    def _import_libstark() -> Any:
        try:
            from bc_stark_sdk import main_mod as libstark  # type: ignore
        except Exception as exc:  # noqa: BLE001
            raise RuntimeError(
                f"未安装 bc-stark-sdk: {exc}。请在 python_server 下执行: uv add bc-stark-sdk colorlog"
            ) from exc
        return libstark


__all__ = [
    "DEFAULT_REVO2_SPEED",
    "DEFAULT_REVO2_BAUDRATE",
    "DEFAULT_REVO2_SLAVE_ID",
    "Revo2HandConfig",
    "Revo2HandDriver",
]

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass

from sleeve_arm.domain.sensor import ImuFrame, SleeveFrame


@dataclass(frozen=True, slots=True)
class SourceStats:
    received_frames: int
    invalid_frames: int
    last_timestamp: float | None
    estimated_fps: float


class SleeveSource(ABC):
    @abstractmethod
    def start(self) -> None: ...

    @abstractmethod
    def latest(self) -> SleeveFrame | None: ...

    @abstractmethod
    def drain(self) -> tuple[SleeveFrame, ...]:
        """Return every frame received since the previous drain/latest call."""
        ...

    @property
    @abstractmethod
    def stats(self) -> SourceStats: ...

    @abstractmethod
    def close(self) -> None: ...


class ImuSource(ABC):
    @abstractmethod
    def start(self) -> None: ...

    @abstractmethod
    def latest(self) -> ImuFrame | None: ...

    @abstractmethod
    def drain(self) -> tuple[ImuFrame, ...]:
        """Return every frame received since the previous drain/latest call."""
        ...

    @property
    @abstractmethod
    def stats(self) -> SourceStats: ...

    @abstractmethod
    def close(self) -> None: ...

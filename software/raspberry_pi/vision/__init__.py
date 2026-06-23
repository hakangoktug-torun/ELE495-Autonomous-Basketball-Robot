"""Computer vision placeholders for hoop and court detection."""

from dataclasses import dataclass


@dataclass
class VisionTarget:
    visible: bool
    x_px: int | None = None
    y_px: int | None = None
    confidence: float = 0.0


class VisionSystem:
    def detect_hoop(self, frame: object | None = None) -> VisionTarget:
        if frame is None:
            return VisionTarget(visible=False)

        return VisionTarget(visible=False)

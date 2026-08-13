from abc import ABC, abstractmethod

from sleeve_arm.domain import MotionIntent, SensorSample


class MotionPredictor(ABC):
    @abstractmethod
    def predict(self, sample: SensorSample) -> MotionIntent: ...

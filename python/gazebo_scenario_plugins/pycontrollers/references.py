import numpy as np
from dataclasses import dataclass

@dataclass
class JointReferences:

    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray

    @staticmethod
    def zero(dofs: int) -> "JointReferences":

        return JointReferences(position=np.zeros(dofs),
                               velocity=np.zeros(dofs),
                               acceleration=np.zeros(dofs))

    def valid(self, dofs: int = None) -> bool:

        if dofs is None:
            dofs = self.position.size

        return self.position.size == self.velocity.size == self.acceleration.size == dofs


@dataclass
class BaseReferences:

    position: np.ndarray
    orientation: np.ndarray
    linear_velocity: np.ndarray
    angular_velocity: np.ndarray
    linear_acceleration: np.ndarray
    angular_acceleration: np.ndarray

    def valid(self) -> bool:

        return self.position.size == \
               self.linear_velocity.size == self.angular_velocity.size == \
               self.linear_acceleration.size == self.angular_acceleration.size == 3 \
               and self.orientation.size == 4

    @staticmethod
    def zero() -> "BaseReferences":

        return BaseReferences(position=np.zeros(3),
                              orientation=np.array([1., 0, 0, 0]),
                              linear_velocity=np.zeros(3),
                              angular_velocity=np.zeros(3),
                              linear_acceleration=np.zeros(3),
                              angular_acceleration=np.zeros(3))

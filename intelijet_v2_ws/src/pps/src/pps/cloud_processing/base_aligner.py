from abc import ABC, abstractmethod
import numpy as np


class AlignMethodConfig:
    pass

class CloudAligner(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def get_transformation_matrix(self):
        """Return the transformation matrix after alignment."""
        pass

    @abstractmethod
    def align(self, source, target, init_transform=np.eye(4)):
        pass

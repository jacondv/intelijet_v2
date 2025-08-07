

from dataclasses import dataclass
from typing import List, Optional
from enum import Enum

class SolverType(Enum):
    POLYNOMIAL = 1
    KINEMATIC = 2

@dataclass
class KinematicParams:
    r1: float = 24.30385
    r2: float = 45.27786
    b: float = 72.0
    c: float = 590.0
    d: float = 495.0
    e: float = 91.22623
    f: float = 447.25528


@dataclass
class EncoderConfig:
    solver_type: str = "KINEMATIC"
    coeffs: List[float] = None
    model_angle_when_closed: float = 0.0
    model_correction_gain: float = 1.0
    theoretical_draw_wire_length_when_closed: float = 0.15
    estimated_p1_p2_length: Optional[float] = None
    solution_tolerance: float = 1e-4
    kinematics: KinematicParams = KinematicParams()

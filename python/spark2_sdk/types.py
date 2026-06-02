from dataclasses import dataclass, fields
from enum import Enum
import math
from typing import Any, List, Type, TypeVar, Union

try:
    import numpy as np
except ImportError:  # numpy is optional for basic SDK usage
    np = None  # type: ignore[assignment]

# --- Constants ---
PI: float = math.pi
DEG_TO_RAD: float = PI / 180.0
RAD_TO_DEG: float = 180.0 / PI

# --- Enums ---
class SmoothingMethod(Enum):
    """Smoothing methods for interpolation."""
    LINEAR = 0
    COS = 1
    CUBIC = 2
    QUINTIC = 3
    NONE = 4


class RobotState(Enum):
    """Operational state of the robot."""
    STARTUP = 0
    IDLE = 1       # Completely stationary; safe to accept new paths
    MOVING = 2     # Actively running an interpolator, velocity command, etc.
    SETTLING = 3   # After planning finished / Changing Control Mode / Stopping
    ERROR = 4      # Safeguard stop triggered; hardware limits breached or E-stop
    RECOVERY = 5   # Resetting safety loops and clearing faults
    SHUTDOWN = 6   # Disabling amplifiers and powering down safely


class PlanResult(Enum):
    """Kinematic and algorithmic planner outcomes."""
    SUCCESS = 0                # IK passed, time allocation valid
    POSE_NOT_REACHABLE = 1     # Target 6D pose is physically outside workspace
    LINEAR_PATH_FAILED = 2     # Continuous linear path is blocked


# --- Diagnostic Flags (Bitmasks) ---
class DiagnosticFlags:
    """Bitmask flags for system diagnostics and faults."""
    NONE = 0
    
    # --- Target Profile Saturation (Pre-Interpolation) ---
    TARGET_POS_SATURATION = 1 << 0
    TARGET_VEL_SATURATION = 1 << 1
    TARGET_TOR_SATURATION = 1 << 2
    
    # --- Real-Time Command Saturation (Post-PID Loop) ---
    ACTUATOR_POS_SATURATION = 1 << 3
    ACTUATOR_VEL_SATURATION = 1 << 4
    ACTUATOR_TOR_SATURATION = 1 << 5
    ACTUATOR_POS_JUMP_SATURATION = 1 << 6
    ACTUATOR_VEL_JUMP_SATURATION = 1 << 7
    ACTUATOR_TOR_JUMP_SATURATION = 1 << 8
    
    # --- Soft Workspace Boundary Safety Interceptions ---
    BOUNDARY_VEL_CLAMP = 1 << 9
    BOUNDARY_JOINT_IMPEDANCE = 1 << 10
    
    # --- Algorithmic & Kinematic Planner Modifications ---
    PLAN_TIMELINE_EXTENDED = 1 << 11
    PLAN_VEL_LIMIT_INVALID = 1 << 12
    PLAN_DELTA_TOO_LARGE = 1 << 13
    PLAN_VELOCITY_SNAP = 1 << 14
    PLAN_POINT_SKIPPED = 1 << 15

    # --- Fault Flags ---
    FAULT_POS_HARD_LIMIT_REACHCHED = 1 << 16
    FAULT_VEL_HARD_LIMIT_REACHED = 1 << 17
    FAULT_TOR_HARD_LIMIT_REACHED = 1 << 18
    FAULT_POS_TRACKING_FAILED = 1 << 19
    FAULT_VEL_TRACKING_FAILED = 1 << 20
    FAULT_TOR_TRACKING_FAILED = 1 << 21
    
    # --- Hardware Faults ---
    FAULT_ARM_NOT_FOUND = 1 << 22
    FAULT_GRIPPER_NOT_FOUND = 1 << 23
    FAULT_HARDWARE_INIT_FAILED = 1 << 24

    FAULT_UNKNOWN = 1 << 31


# --- Data Structures ---
@dataclass
class Quaternion:
    w: float
    x: float
    y: float
    z: float

@dataclass
class EulerAngle:
    """Intrinsic XYZ rotation.
    
    Rotate first around x by roll, then around new y by pitch, 
    then around new z by yaw.
    """
    roll: float
    pitch: float
    yaw: float


@dataclass
class Position:
    x: float
    y: float
    z: float


@dataclass
class Pose:
    position: Position
    orientation: Quaternion


@dataclass
class LimitPair:
    """A generic container for a single joint's min/max bounds."""
    min: float
    max: float

T = TypeVar("T", bound="BaseJointState")

class BaseJointState:
    """Dynamic base class providing shared conversions for 1D states and 2D Nx2 limits."""
    
    def __init__(self, data: Union[List, np.ndarray, tuple, Any], *args) -> None:
        # 1. Parse positional arguments or array blocks cleanly
        if args:
            sequence = [data] + list(args)
        elif isinstance(data, (list, tuple, np.ndarray)):
            sequence = data
        else:
            sequence = [data]
            
        # 2. Extract values through from_sequence safely
        parsed_instance = self.from_sequence(sequence)
        
        # 3. Direct dictionary assignment breaks the recursive lookups completely!
        self.__dict__.update(parsed_instance.__dict__)
            
    def __len__(self) -> int:
        """Enables standard len() queries (e.g., len(state) -> 6)."""
        return self._expected_len()
        
    @classmethod
    def _expected_len(cls) -> int:
        """Dynamically counts the number of attributes using annotations to prevent recursion."""
        return len(cls.__annotations__)

    def __getitem__(self, item: int) -> Any:
        """Enables array-style indexing (e.g., state[1] -> j2)."""
        field_names = list(self.__annotations__.keys())
        try:
            return getattr(self, field_names[item])
        except IndexError:
            raise IndexError(f"{self.__class__.__name__} index out of range")

    @classmethod
    def from_sequence(cls: Type[T], data: Union[List, np.ndarray]) -> T:
        """
        Safely casts an input sequence into a joint structure.
        Handles 1D arrays for standard states and 2D arrays (N x 2) for limits.
        """
        if isinstance(data, np.ndarray):
            values = data.tolist()
        else:
            values = list(data)
            
        expected = cls._expected_len()
        field_names = list(cls.__annotations__.keys())

        # Check if the incoming data is an N x 2 array
        if len(values) == expected and isinstance(values[0], (list, tuple)):
            pairs = []
            for idx, row in enumerate(values):
                if len(row) != 2:
                    raise ValueError(
                        f"Row {idx} must have exactly 2 elements [min, max], got {len(row)}"
                    )
                pairs.append(LimitPair(row[0], row[1]))
            
            #  FIX: Instantiates the class WITHOUT triggering __init__ or fields()
            obj = object.__new__(cls)
            obj.__dict__.update(zip(field_names, pairs))
            return obj

        # Fallback to standard 1D array handling
        if len(values) != expected:
            raise ValueError(f"{cls.__name__} requires exactly {expected} elements, got {len(values)}")
        
        #  FIX: Instantiates the 1D container without hitting the loop
        obj = object.__new__(cls)
        obj.__dict__.update(zip(field_names, values))
        return obj

    def to_list(self) -> List[Any]:
        """Converts the structure into a flat 1D list or structured 2D Nx2 list."""
        field_names = list(self.__annotations__.keys())
        if not field_names:
            return []
            
        first_field_item = getattr(self, field_names[0])
        
        # Check if elements are LimitPairs
        if hasattr(first_field_item, "min") and hasattr(first_field_item, "max"):
            return [[getattr(self, name).min, getattr(self, name).max] for name in field_names]
            
        return [getattr(self, name) for name in field_names]

    def to_numpy(self, dtype=None) -> np.ndarray:
        """Converts the joint states or limits into a 1D or N x 2 NumPy array."""
        return np.array(self.to_list(), dtype=dtype)

@dataclass(init=False)
class JointState6f(BaseJointState):
    j1: float; j2: float; j3: float; j4: float; j5: float; j6: float

@dataclass(init=False)
class JointState1f(BaseJointState):
    j1: float;

@dataclass(init=False)
class JointState6b(BaseJointState):
    j1: bool; j2: bool; j3: bool; j4: bool; j5: bool; j6: bool;
    
@dataclass(init=False)
class JointState1b(BaseJointState):
    j1: bool;

@dataclass(init=False)
class JointState6u(BaseJointState):
    j1: int; j2: int; j3: int; j4: int; j5: int; j6: int;
    
@dataclass(init=False)
class JointState1u(BaseJointState):
    j1: int;

@dataclass(init=False)
class JointLimits6f(BaseJointState):
    j1: LimitPair; j2: LimitPair; j3: LimitPair; j4: LimitPair; j5: LimitPair; j6: LimitPair

@dataclass(init=False)
class JointLimits1f(BaseJointState):
    j1: LimitPair;
    
@dataclass
class RobotJointStatef:
    arm: JointState6f;
    gripper: JointState1f;

@dataclass
class RobotJointStateb:
    arm: JointState6b;
    gripper: JointState1b;

@dataclass
class SystemStatus:
    robot_state: RobotState;
    plan_result: PlanResult;
    robot_diagnostic_flags: int;
    arm_joint_diagnostic_flags: JointState6u;
    gripper_joint_diagnostic_flags: JointState1u;
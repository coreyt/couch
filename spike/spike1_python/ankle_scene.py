"""
Spike 1: Minimal SOFA ankle joint scene.

Emergent joint model:
  - Tibia: fixed rigid body (proximal)
  - Talus: free rigid body (distal), constrained by ligament springs + bone contact
  - Ligaments: Python controller applies spring forces directly to rigid body
    (StiffSpringForceField has cross-mapping force propagation issues)
  - Joint angle: relative quaternion decomposition of talus w.r.t. tibia

Supports two mesh modes:
  - "box": simple geometric boxes (fast, default)
  - "stl": real BodyParts3D bone STL meshes (anatomically accurate)

Validated against SOFA v24.06.00.
"""
import math
import os
import numpy as np
from dataclasses import dataclass, field, fields
from typing import List, Optional

import Sofa.Core
import Sofa.Simulation


# ---------------------------------------------------------------------------
# Configuration dataclasses
# ---------------------------------------------------------------------------

@dataclass
class LigamentConfig:
    """Configuration for a single ligament spring."""
    name: str
    talus_offset: list
    stiffness: float
    damping: float
    tibia_offset: list = None
    fixed_anchor: list = None
    rest_length: float = None
    force_model: str = None

    def __post_init__(self):
        if self.tibia_offset is None and self.fixed_anchor is None:
            raise ValueError(
                f"Ligament '{self.name}' must have either tibia_offset or fixed_anchor"
            )

    def __getitem__(self, key):
        return getattr(self, key)

    def __setitem__(self, key, value):
        setattr(self, key, value)

    def __contains__(self, key):
        return hasattr(self, key) and getattr(self, key) is not None

    def get(self, key, default=None):
        val = getattr(self, key, None)
        return val if val is not None else default

    def keys(self):
        return [f.name for f in fields(self) if getattr(self, f.name) is not None]

    def to_dict(self):
        return {k: getattr(self, k) for k in self.keys()}


@dataclass
class SolverConfig:
    """ODE solver and constraint solver parameters."""
    dt: float = 0.001
    constraint_iterations: int = 200
    tolerance: float = 1e-4
    rayleigh_stiffness: float = 0.1
    rayleigh_mass: float = 1.0
    cg_iterations: int = 25
    cg_tolerance: float = 1e-10
    cg_threshold: float = 1e-10


@dataclass
class CollisionConfig:
    """Collision detection parameters."""
    alarm_distance: float = 8.0
    contact_distance: float = 3.0
    angle_cone: float = 0.1


@dataclass
class PhysicsConfig:
    """Physics parameters (gravity, masses)."""
    gravity: list = field(default_factory=lambda: [0, 0, -9810])
    tibia_mass: float = 1.0
    talus_mass: float = 0.1


@dataclass
class GeometryConfig:
    """Box geometry parameters."""
    tibia_box_half: list = field(default_factory=lambda: [20.0, 15.0, 15.0])
    talus_box_half: list = field(default_factory=lambda: [18.0, 13.0, 12.0])
    talus_z_offset: float = -30.0


@dataclass
class ForceModelConfig:
    """Ligament force model parameters."""
    force_model: str = "linear"
    exp_B: float = 30.0
    exp_max_strain: float = 0.15
    toe_strain: float = 0.03
    toe_ratio: float = 0.2
    anatomical_exp_B_cap: float = 10.0


@dataclass
class MeshConfig:
    """Mesh loading and decimation parameters."""
    collision_faces: int = 2000
    tibia_trim_height: float = 80.0
    fibula_collision_faces: int = 500
    mesh_dir: str = None


@dataclass
class AnkleSimConfig:
    """Top-level configuration for the ankle simulation."""
    solver: SolverConfig = field(default_factory=SolverConfig)
    collision: CollisionConfig = field(default_factory=CollisionConfig)
    physics: PhysicsConfig = field(default_factory=PhysicsConfig)
    geometry: GeometryConfig = field(default_factory=GeometryConfig)
    force_model: ForceModelConfig = field(default_factory=ForceModelConfig)
    mesh: MeshConfig = field(default_factory=MeshConfig)
    mesh_mode: str = "box"
    ligament_model: str = "simple"
    stiffness_scale: float = 1.0
    ligaments: Optional[List[LigamentConfig]] = None
    bilinear: bool = False


def _normalize_ligaments(ligaments):
    """Convert a list of dicts and/or LigamentConfig objects to List[LigamentConfig]."""
    result = []
    for lig in ligaments:
        if isinstance(lig, LigamentConfig):
            result.append(lig)
        elif isinstance(lig, dict):
            result.append(LigamentConfig(**lig))
        else:
            raise TypeError(f"Expected dict or LigamentConfig, got {type(lig)}")
    return result


# ---------------------------------------------------------------------------
# Constants — approximate ankle anatomy (all units: mm, kg, N)
# ---------------------------------------------------------------------------

# Ligament properties: attachment points in local frame of each bone.
# Rest lengths computed from initial geometry so springs start at rest.
# Stiffness values are approximate (N/mm).
#
# Geometry:
#   Tibia at origin, talus centered 30mm below (z=-30).
#   Tibia box: 40x30x30mm (half: 20,15,15). Bottom face at z=-15.
#   Talus box: 36x26x24mm (half: 18,13,12). Top face at z=-30+12=-18.
#   Gap between boxes: 3mm (within contact distance=3mm).
#   Ligaments span from tibia inferior to talus superior.
#
# Coordinate convention:
#   X = medial-lateral, Y = anterior(+)-posterior(-), Z = proximal(+)-distal(-)
#
# For sagittal plane rotation (DF/PF = rotation about X):
#   - Anterior ligaments (Y>0) resist plantarflexion
#   - Posterior ligaments (Y<0) resist dorsiflexion
# Both directions must have ligament resistance for bounded ROM.
#
DEFAULT_LIGAMENTS = [
    LigamentConfig(
        name="ATFL",  # Anterior talofibular (anterior-lateral)
        tibia_offset=[15.0, 10.0, -14.0],
        talus_offset=[12.0, 8.0, 11.0],
        stiffness=70.0,     # N/mm
        damping=5.0,        # N·s/mm
    ),
    LigamentConfig(
        name="PTFL",  # Posterior talofibular (posterior-lateral)
        tibia_offset=[15.0, -10.0, -14.0],
        talus_offset=[12.0, -8.0, 11.0],
        stiffness=50.0,
        damping=5.0,
    ),
    LigamentConfig(
        name="Deltoid_ant",  # Anterior deltoid (anterior-medial)
        tibia_offset=[-12.0, 10.0, -14.0],
        talus_offset=[-10.0, 8.0, 11.0],
        stiffness=90.0,
        damping=5.0,
    ),
    LigamentConfig(
        name="Deltoid_post",  # Posterior deltoid (posterior-medial)
        tibia_offset=[-12.0, -10.0, -14.0],
        talus_offset=[-10.0, -8.0, 11.0],
        stiffness=90.0,
        damping=5.0,
    ),
]

# Derive module-level constants from default geometry config
_DEFAULT_GEOMETRY = GeometryConfig()

# Tibia is at origin (fixed). Talus sits below it.
TIBIA_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]  # [x,y,z, qx,qy,qz,qw]
TALUS_POSITION = [0.0, 0.0, _DEFAULT_GEOMETRY.talus_z_offset, 0.0, 0.0, 0.0, 1.0]

# Simple box collision geometry (half-extents in mm)
TIBIA_BOX_HALF = _DEFAULT_GEOMETRY.tibia_box_half
TALUS_BOX_HALF = _DEFAULT_GEOMETRY.talus_box_half

SOFA_PLUGINS = [
    "Sofa.Component.StateContainer",
    "Sofa.Component.Mass",
    "Sofa.Component.ODESolver.Backward",
    "Sofa.Component.LinearSolver.Iterative",
    "Sofa.Component.Constraint.Lagrangian.Solver",
    "Sofa.Component.Constraint.Lagrangian.Correction",
    "Sofa.Component.Constraint.Projective",
    "Sofa.Component.Collision.Detection.Algorithm",
    "Sofa.Component.Collision.Detection.Intersection",
    "Sofa.Component.Collision.Geometry",
    "Sofa.Component.Collision.Response.Contact",
    "Sofa.Component.AnimationLoop",
    "Sofa.Component.Mapping.Linear",
    "Sofa.Component.Topology.Container.Constant",
    "Sofa.Component.MechanicalLoad",
]


# ---------------------------------------------------------------------------
# Box mesh generation
# ---------------------------------------------------------------------------

def _box_vertices_and_triangles(half_extents):
    """Generate vertices and triangles for an axis-aligned box."""
    hx, hy, hz = half_extents
    verts = [
        [-hx, -hy, -hz], [hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz],
        [-hx, -hy,  hz], [hx, -hy,  hz], [hx, hy,  hz], [-hx, hy,  hz],
    ]
    tris = [
        [0, 1, 2], [0, 2, 3],  # bottom
        [4, 6, 5], [4, 7, 6],  # top
        [0, 4, 5], [0, 5, 1],  # front
        [2, 6, 7], [2, 7, 3],  # back
        [0, 3, 7], [0, 7, 4],  # left
        [1, 5, 6], [1, 6, 2],  # right
    ]
    return verts, tris


# ---------------------------------------------------------------------------
# STL mesh loading
# ---------------------------------------------------------------------------

# Default mesh directory (relative to this file)
_DEFAULT_MESH_DIR = os.path.join(os.path.dirname(__file__), "meshes")

# Ligament attachment points for real bone geometry (offsets from rigid body origin).
# Computed from BodyParts3D mesh bounds after joint-center normalization.
# These are approximate anatomical positions on the bone surfaces.
STL_LIGAMENTS = [
    LigamentConfig(
        name="ATFL",
        tibia_offset=[25.0, 20.0, -3.0],    # anterior-lateral, distal tibia
        talus_offset=[12.0, 15.0, 4.0],      # anterior-lateral, talus dome
        stiffness=70.0,
        damping=5.0,
    ),
    LigamentConfig(
        name="PTFL",
        tibia_offset=[25.0, -25.0, -3.0],   # posterior-lateral, distal tibia
        talus_offset=[12.0, -15.0, 4.0],     # posterior-lateral, talus dome
        stiffness=50.0,
        damping=5.0,
    ),
    LigamentConfig(
        name="Deltoid_ant",
        tibia_offset=[-25.0, 20.0, -3.0],   # anterior-medial, distal tibia
        talus_offset=[-12.0, 15.0, 4.0],     # anterior-medial, talus dome
        stiffness=90.0,
        damping=5.0,
    ),
    LigamentConfig(
        name="Deltoid_post",
        tibia_offset=[-25.0, -25.0, -3.0],  # posterior-medial, distal tibia
        talus_offset=[-12.0, -15.0, 4.0],    # posterior-medial, talus dome
        stiffness=90.0,
        damping=5.0,
    ),
]

# ---------------------------------------------------------------------------
# Anatomical ligament configurations (7 ligaments + Achilles tendon)
# ---------------------------------------------------------------------------
# Literature-derived stiffnesses. Attachment coordinates approximate
# anatomical insertion sites on box and STL bone geometry.
#
# Coordinate convention (same as above):
#   X = medial(-) / lateral(+), Y = anterior(+) / posterior(-), Z = proximal(+) / distal(-)
#
# CFL, TCL (superficial deltoid), and Achilles attach to the calcaneus,
# which is not modeled. These use fixed world-space anchors.

ANATOMICAL_LIGAMENTS = [
    # --- Lateral ligaments ---
    LigamentConfig(
        name="ATFL",
        tibia_offset=[15.0, 10.0, -14.0],
        talus_offset=[12.0, 8.0, 11.0],
        stiffness=145.0,
        damping=8.0,
    ),
    LigamentConfig(
        name="PTFL",
        tibia_offset=[15.0, -10.0, -14.0],
        talus_offset=[12.0, -8.0, 11.0],
        stiffness=122.0,
        damping=8.0,
    ),
    LigamentConfig(
        name="CFL",
        fixed_anchor=[25.0, 0.0, -55.0],  # calcaneus lateral
        talus_offset=[14.0, 0.0, -5.0],
        stiffness=137.0,
        damping=8.0,
    ),
    # --- Medial (deltoid) ligaments ---
    LigamentConfig(
        name="Deltoid_deep_ant",
        tibia_offset=[-12.0, 10.0, -14.0],
        talus_offset=[-10.0, 8.0, 11.0],
        stiffness=90.0,
        damping=8.0,
    ),
    LigamentConfig(
        name="Deltoid_deep_post",
        tibia_offset=[-12.0, -10.0, -14.0],
        talus_offset=[-10.0, -8.0, 11.0],
        stiffness=80.0,
        damping=8.0,
    ),
    LigamentConfig(
        name="TCL",
        fixed_anchor=[-20.0, 5.0, -55.0],  # calcaneus medial
        talus_offset=[-12.0, 3.0, -3.0],
        stiffness=40.0,
        damping=5.0,
    ),
    # --- Achilles tendon ---
    LigamentConfig(
        name="Achilles",
        fixed_anchor=[0.0, -20.0, -60.0],  # calcaneus posterior
        talus_offset=[0.0, -10.0, -8.0],
        stiffness=300.0,
        damping=15.0,
    ),
]

STL_ANATOMICAL_LIGAMENTS = [
    # --- Lateral ligaments ---
    LigamentConfig(
        name="ATFL",
        tibia_offset=[25.0, 20.0, -3.0],
        talus_offset=[12.0, 15.0, 4.0],
        stiffness=145.0,
        damping=8.0,
    ),
    LigamentConfig(
        name="PTFL",
        tibia_offset=[25.0, -25.0, -3.0],
        talus_offset=[12.0, -15.0, 4.0],
        stiffness=122.0,
        damping=8.0,
    ),
    LigamentConfig(
        name="CFL",
        fixed_anchor=[30.0, 0.0, -40.0],  # calcaneus lateral
        talus_offset=[15.0, 0.0, -8.0],
        stiffness=137.0,
        damping=8.0,
    ),
    # --- Medial (deltoid) ligaments ---
    LigamentConfig(
        name="Deltoid_deep_ant",
        tibia_offset=[-25.0, 20.0, -3.0],
        talus_offset=[-12.0, 15.0, 4.0],
        stiffness=90.0,
        damping=8.0,
    ),
    LigamentConfig(
        name="Deltoid_deep_post",
        tibia_offset=[-25.0, -25.0, -3.0],
        talus_offset=[-12.0, -15.0, 4.0],
        stiffness=80.0,
        damping=8.0,
    ),
    LigamentConfig(
        name="TCL",
        fixed_anchor=[-25.0, 10.0, -40.0],  # calcaneus medial
        talus_offset=[-15.0, 5.0, -5.0],
        stiffness=40.0,
        damping=5.0,
    ),
    # --- Achilles tendon ---
    LigamentConfig(
        name="Achilles",
        fixed_anchor=[0.0, -25.0, -45.0],  # calcaneus posterior
        talus_offset=[0.0, -15.0, -10.0],
        stiffness=300.0,
        damping=15.0,
    ),
]


def load_bone_meshes(mesh_config=None, mesh_dir=None, collision_faces=2000,
                     tibia_trim_height=80.0):
    """
    Load and prepare tibia/talus/fibula STL meshes for SOFA simulation.

    Normalizes coordinates so the ankle joint center is at the world origin,
    trims tibia and fibula to just the distal portion, and decimates for collision.

    Args:
        mesh_config: MeshConfig object. If provided, overrides legacy kwargs.
        mesh_dir: Directory containing tibia_right.stl, talus_right.stl,
                  and fibula_right.stl.
        collision_faces: Target face count for collision mesh decimation.
        tibia_trim_height: Keep only the bottom N mm of the tibia/fibula
                          (from distal end).

    Returns:
        dict with keys:
            tibia_verts, tibia_tris: collision mesh for tibia
            talus_verts, talus_tris: collision mesh for talus
            fibula_verts, fibula_tris: collision mesh for fibula (if available)
            joint_center: [x, y, z] of the original joint center (before normalization)
            tibia_centroid: centroid of normalized tibia mesh
            talus_centroid: centroid of normalized talus mesh
    """
    import trimesh

    if mesh_config is not None:
        collision_faces = mesh_config.collision_faces
        tibia_trim_height = mesh_config.tibia_trim_height
        fibula_collision_faces = mesh_config.fibula_collision_faces
        if mesh_config.mesh_dir is not None:
            mesh_dir = mesh_config.mesh_dir
    else:
        fibula_collision_faces = min(collision_faces, 500)

    if mesh_dir is None:
        mesh_dir = _DEFAULT_MESH_DIR

    tibia_mesh = trimesh.load(os.path.join(mesh_dir, "tibia_right.stl"))
    talus_mesh = trimesh.load(os.path.join(mesh_dir, "talus_right.stl"))

    # Load fibula if available
    fibula_path = os.path.join(mesh_dir, "fibula_right.stl")
    fibula_mesh = None
    if os.path.exists(fibula_path):
        fibula_mesh = trimesh.load(fibula_path)

    # Joint center: midpoint between tibia distal surface and talus dome
    tibia_distal_z = tibia_mesh.bounds[0][2]  # Z min of tibia
    talus_dome_z = talus_mesh.bounds[1][2]    # Z max of talus
    joint_center_z = (tibia_distal_z + talus_dome_z) / 2.0

    # XY center: average of both bone centroids
    joint_center_x = (tibia_mesh.centroid[0] + talus_mesh.centroid[0]) / 2.0
    joint_center_y = (tibia_mesh.centroid[1] + talus_mesh.centroid[1]) / 2.0
    joint_center = np.array([joint_center_x, joint_center_y, joint_center_z])

    # Translate all meshes so joint center is at origin
    tibia_mesh.apply_translation(-joint_center)
    talus_mesh.apply_translation(-joint_center)
    if fibula_mesh is not None:
        fibula_mesh.apply_translation(-joint_center)

    # Trim tibia to distal portion only
    tibia_distal_z_norm = tibia_mesh.bounds[0][2]
    z_cutoff = tibia_distal_z_norm + tibia_trim_height
    # Keep only vertices below z_cutoff (select faces where all vertices are below)
    mask = np.all(tibia_mesh.vertices[tibia_mesh.faces][:, :, 2] < z_cutoff, axis=1)
    tibia_mesh = tibia_mesh.submesh([mask], append=True)

    # Trim fibula to distal portion (same height as tibia)
    if fibula_mesh is not None:
        fibula_distal_z_norm = fibula_mesh.bounds[0][2]
        fib_z_cutoff = fibula_distal_z_norm + tibia_trim_height
        fib_mask = np.all(
            fibula_mesh.vertices[fibula_mesh.faces][:, :, 2] < fib_z_cutoff, axis=1
        )
        fibula_mesh = fibula_mesh.submesh([fib_mask], append=True)

    # Decimate meshes for collision detection performance
    if len(tibia_mesh.faces) > collision_faces:
        tibia_mesh = tibia_mesh.simplify_quadric_decimation(collision_faces)
    if len(talus_mesh.faces) > collision_faces:
        talus_mesh = talus_mesh.simplify_quadric_decimation(collision_faces)
    if fibula_mesh is not None and len(fibula_mesh.faces) > fibula_collision_faces:
        fibula_mesh = fibula_mesh.simplify_quadric_decimation(fibula_collision_faces)

    result = {
        "tibia_verts": tibia_mesh.vertices.tolist(),
        "tibia_tris": tibia_mesh.faces.tolist(),
        "talus_verts": talus_mesh.vertices.tolist(),
        "talus_tris": talus_mesh.faces.tolist(),
        "joint_center": joint_center.tolist(),
        "tibia_centroid": tibia_mesh.centroid.tolist(),
        "talus_centroid": talus_mesh.centroid.tolist(),
        "tibia_faces": len(tibia_mesh.faces),
        "talus_faces": len(talus_mesh.faces),
    }

    if fibula_mesh is not None:
        result["fibula_verts"] = fibula_mesh.vertices.tolist()
        result["fibula_tris"] = fibula_mesh.faces.tolist()
        result["fibula_faces"] = len(fibula_mesh.faces)

    return result


# ---------------------------------------------------------------------------
# Bilinear ligament controller
# ---------------------------------------------------------------------------

class LigamentForceController(Sofa.Core.Controller):
    """
    Computes ligament spring forces and writes them to a ConstantForceField.

    Uses a Python controller instead of StiffSpringForceField to avoid
    cross-mapping force propagation issues (known SOFA limitation).
    Forces are written to a ConstantForceField so the EulerImplicit solver
    integrates them properly (direct velocity modification gets overwritten).

    Supports three force models:
      - "linear": F = k * extension
      - "bilinear": reduced stiffness below toe_strain, full stiffness above
      - "exponential": F = A * (exp(B * strain) - 1), strain clamped at 15%
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.ligament_configs = kwargs.get("ligament_configs", [])
        self.tibia_mo = kwargs.get("tibia_mo")
        self.talus_mo = kwargs.get("talus_mo")
        self.ligament_ff = kwargs.get("ligament_ff")  # ConstantForceField to write to
        self.bilinear = kwargs.get("bilinear", False)

        fmc = kwargs.get("force_model_config")
        if fmc is not None:
            self.toe_strain = fmc.toe_strain
            self.toe_ratio = fmc.toe_ratio
            self.force_model = fmc.force_model
            self.exp_B = fmc.exp_B
            self.exp_max_strain = fmc.exp_max_strain
        else:
            self.toe_strain = kwargs.get("toe_strain", 0.03)
            self.toe_ratio = kwargs.get("toe_ratio", 0.2)
            self.force_model = kwargs.get("force_model", "linear")
            self.exp_B = kwargs.get("exp_B", 30.0)
            self.exp_max_strain = kwargs.get("exp_max_strain", 0.15)

    def onAnimateBeginEvent(self, event):
        tibia_pos = np.array(self.tibia_mo.position.value[0][:3])
        talus_pos = np.array(self.talus_mo.position.value[0][:3])
        tibia_quat = np.array(self.tibia_mo.position.value[0][3:7])
        talus_quat = np.array(self.talus_mo.position.value[0][3:7])

        total_force = np.zeros(3)
        total_torque = np.zeros(3)

        for config in self.ligament_configs:
            # Proximal attachment point: fixed world-space or bone-relative
            if "fixed_anchor" in config:
                proximal_attach = np.array(config["fixed_anchor"])
            else:
                proximal_attach = tibia_pos + _rotate_by_quat(
                    config["tibia_offset"], tibia_quat
                )
            talus_attach = talus_pos + _rotate_by_quat(
                config["talus_offset"], talus_quat
            )

            # Spring direction and length
            diff = proximal_attach - talus_attach
            current_length = np.linalg.norm(diff)
            if current_length < 1e-6:
                continue
            direction = diff / current_length

            rest_length = config["rest_length"]
            extension = current_length - rest_length

            # Only apply tensile forces (ligaments don't resist compression)
            if extension <= 0:
                continue

            # Compute force magnitude based on force model
            linear_k = config["stiffness"]
            strain = extension / rest_length

            # Determine effective force model: per-ligament override or controller default
            effective_model = config.get("force_model", self.force_model)
            # Backward compat: bilinear flag overrides if force_model is "linear"
            if self.bilinear and effective_model == "linear":
                effective_model = "bilinear"

            if effective_model == "exponential":
                clamped_strain = min(strain, self.exp_max_strain)
                # A calibrated so dF/d(extension) = k at toe_strain:
                # A * B * exp(B * toe) / rest_length = k → A = k * rest_length / (B * exp(B * toe))
                A = linear_k * rest_length / (self.exp_B * math.exp(self.exp_B * self.toe_strain))
                force_magnitude = A * (math.exp(self.exp_B * clamped_strain) - 1.0)
            elif effective_model == "bilinear":
                if strain < self.toe_strain:
                    force_magnitude = linear_k * self.toe_ratio * extension
                else:
                    force_magnitude = linear_k * extension
            else:  # "linear"
                force_magnitude = linear_k * extension

            spring_force = force_magnitude * direction

            # Velocity-based damping along spring axis
            damping = config.get("damping", 1.0)
            talus_vel = np.array(self.talus_mo.velocity.value[0][:3])
            vel_along = np.dot(talus_vel, direction)
            damping_force = -damping * vel_along * direction

            force = spring_force + damping_force
            total_force += force

            # Torque: r × F where r is from talus center to attachment point
            r = talus_attach - talus_pos
            total_torque += np.cross(r, force)

        # Write computed force to ConstantForceField for solver integration
        with self.ligament_ff.forces.writeable() as w:
            w[0] = [
                total_force[0], total_force[1], total_force[2],
                total_torque[0], total_torque[1], total_torque[2],
            ]


# ---------------------------------------------------------------------------
# Quaternion math
# ---------------------------------------------------------------------------

def _rotate_by_quat(vec, quat):
    """Rotate vec by quaternion [qx, qy, qz, qw]."""
    v = np.array(vec, dtype=float)
    q = np.array(quat, dtype=float)
    qx, qy, qz, qw = q

    # q * v * q_inv (Hamilton product)
    t = 2.0 * np.cross([qx, qy, qz], v)
    return v + qw * t + np.cross([qx, qy, qz], t)


def _quat_inverse(q):
    """Inverse of quaternion [qx, qy, qz, qw]."""
    return np.array([-q[0], -q[1], -q[2], q[3]])


def _quat_multiply(q1, q2):
    """Hamilton product of two quaternions [qx, qy, qz, qw]."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    ])


def _quat_to_euler(q):
    """Convert quaternion [qx,qy,qz,qw] to euler angles [rx,ry,rz] in degrees."""
    qx, qy, qz, qw = q

    # Roll (X)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (Y)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)

    # Yaw (Z)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return np.array([
        math.degrees(roll),
        math.degrees(pitch),
        math.degrees(yaw),
    ])


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def create_ankle_scene(
    config=None,
    gravity=None,
    dt=0.001,
    constraint_iterations=200,
    stiffness_scale=1.0,
    bilinear=False,
    toe_strain=0.03,
    toe_ratio=0.2,
    ligaments=None,
    mesh_mode="box",
    mesh_dir=None,
    collision_faces=2000,
    force_model=None,
    exp_B=30.0,
    exp_max_strain=0.15,
    ligament_model="simple",
):
    """
    Create a SOFA scene with a minimal ankle joint model.

    Args:
        config: AnkleSimConfig object. If provided, overrides all legacy kwargs.
        gravity: [gx, gy, gz] in mm/s². Default [0, 0, -9810].
        dt: Timestep in seconds.
        constraint_iterations: Max solver iterations.
        stiffness_scale: Multiplier for all ligament stiffnesses.
        bilinear: If True, use bilinear ligament model (toe + linear).
        toe_strain: Strain threshold for toe region (bilinear only).
        toe_ratio: Ratio of toe stiffness to linear stiffness (bilinear only).
        ligaments: Custom ligament configs. Default depends on mesh_mode
                   and ligament_model.
        mesh_mode: "box" for simple geometric boxes, "stl" for real bone meshes.
        mesh_dir: Directory containing STL files (only used if mesh_mode="stl").
        collision_faces: Target face count for STL decimation.
        force_model: "linear", "bilinear", or "exponential". Default: "linear"
                     for simple model, "exponential" for anatomical.
        exp_B: Exponential model B parameter (default 30).
        exp_max_strain: Max strain clamp for exponential model (default 0.15).
        ligament_model: "simple" (4 ligaments) or "anatomical" (7 + Achilles).

    Returns:
        (root, info) where info contains references to key scene components.
    """
    if config is None:
        if gravity is None:
            gravity = [0, 0, -9810]
        # Default force model: linear for both simple and anatomical.
        # Exponential is available but requires careful tuning (B≤10) due to
        # one-step-lag instability with the ConstantForceField controller.
        if force_model is None:
            force_model = "linear"
        config = AnkleSimConfig(
            solver=SolverConfig(dt=dt, constraint_iterations=constraint_iterations),
            physics=PhysicsConfig(gravity=gravity),
            force_model=ForceModelConfig(
                force_model=force_model,
                exp_B=exp_B,
                exp_max_strain=exp_max_strain,
                toe_strain=toe_strain,
                toe_ratio=toe_ratio,
            ),
            mesh=MeshConfig(collision_faces=collision_faces, mesh_dir=mesh_dir),
            mesh_mode=mesh_mode,
            ligament_model=ligament_model,
            stiffness_scale=stiffness_scale,
            ligaments=_normalize_ligaments(ligaments) if ligaments is not None else None,
            bilinear=bilinear,
        )

    # When using exponential with anatomical model, cap B to avoid
    # divergence. At B=30, forces reach 17x linear at strain clamp.
    fm_config = config.force_model
    if (config.ligament_model == "anatomical"
            and fm_config.force_model == "exponential"
            and fm_config.exp_B > fm_config.anatomical_exp_B_cap):
        fm_config = ForceModelConfig(
            force_model=fm_config.force_model,
            exp_B=fm_config.anatomical_exp_B_cap,
            exp_max_strain=fm_config.exp_max_strain,
            toe_strain=fm_config.toe_strain,
            toe_ratio=fm_config.toe_ratio,
            anatomical_exp_B_cap=fm_config.anatomical_exp_B_cap,
        )

    # Select mesh data and ligament defaults based on mesh_mode and ligament_model
    if config.mesh_mode == "stl":
        mesh_data = load_bone_meshes(mesh_config=config.mesh)
        t_verts = mesh_data["tibia_verts"]
        t_tris = mesh_data["tibia_tris"]
        a_verts = mesh_data["talus_verts"]
        a_tris = mesh_data["talus_tris"]
        # Both rigid bodies at origin — mesh vertices define world position
        tibia_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        talus_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
        if config.ligaments is None:
            if config.ligament_model == "anatomical":
                ligaments = STL_ANATOMICAL_LIGAMENTS
            else:
                ligaments = STL_LIGAMENTS
        else:
            ligaments = config.ligaments
    else:
        t_verts, t_tris = _box_vertices_and_triangles(config.geometry.tibia_box_half)
        a_verts, a_tris = _box_vertices_and_triangles(config.geometry.talus_box_half)
        tibia_position = TIBIA_POSITION
        talus_position = TALUS_POSITION
        if config.ligaments is None:
            if config.ligament_model == "anatomical":
                ligaments = ANATOMICAL_LIGAMENTS
            else:
                ligaments = DEFAULT_LIGAMENTS
        else:
            ligaments = config.ligaments

    root = Sofa.Core.Node("root")
    root.gravity = config.physics.gravity
    root.dt = config.solver.dt

    # Plugins
    root.addObject("RequiredPlugin", pluginName=SOFA_PLUGINS)

    # Animation loop + constraint solver
    root.addObject("FreeMotionAnimationLoop")
    root.addObject(
        "GenericConstraintSolver",
        maxIterations=config.solver.constraint_iterations,
        tolerance=config.solver.tolerance,
    )

    # Collision pipeline
    root.addObject("CollisionPipeline")
    root.addObject("BruteForceBroadPhase")
    root.addObject("BVHNarrowPhase")
    root.addObject(
        "LocalMinDistance",
        alarmDistance=config.collision.alarm_distance,
        contactDistance=config.collision.contact_distance,
        angleCone=config.collision.angle_cone,
    )
    root.addObject(
        "CollisionResponse",
        response="FrictionContactConstraint",
    )

    # ----- Tibia (fixed rigid body) -----
    tibia = root.addChild("Tibia")
    tibia.addObject(
        "EulerImplicitSolver",
        rayleighStiffness=config.solver.rayleigh_stiffness,
        rayleighMass=config.solver.rayleigh_mass,
    )
    tibia.addObject(
        "CGLinearSolver",
        iterations=config.solver.cg_iterations,
        tolerance=config.solver.cg_tolerance,
        threshold=config.solver.cg_threshold,
    )
    tibia_mo = tibia.addObject(
        "MechanicalObject",
        template="Rigid3d",
        position=[tibia_position],
    )
    tibia.addObject("UniformMass", totalMass=config.physics.tibia_mass)
    tibia.addObject("FixedConstraint", indices=[0])
    tibia.addObject("UncoupledConstraintCorrection")

    # Tibia collision sub-node
    tibia_col = tibia.addChild("Collision")
    tibia_col.addObject(
        "MeshTopology",
        position=t_verts,
        triangles=t_tris,
    )
    tibia_col.addObject("MechanicalObject", template="Vec3d")
    tibia_col.addObject("TriangleCollisionModel", simulated=True, moving=False, group=0)
    tibia_col.addObject("LineCollisionModel", simulated=True, moving=False, group=0)
    tibia_col.addObject("PointCollisionModel", simulated=True, moving=False, group=0)
    tibia_col.addObject("RigidMapping")

    # Fibula collision sub-node (fixed to tibia via syndesmosis)
    # Same collision group (0) as tibia to prevent self-collision
    if config.mesh_mode == "stl" and "fibula_verts" in mesh_data:
        fibula_col = tibia.addChild("FibulaCollision")
        fibula_col.addObject(
            "MeshTopology",
            position=mesh_data["fibula_verts"],
            triangles=mesh_data["fibula_tris"],
        )
        fibula_col.addObject("MechanicalObject", template="Vec3d")
        fibula_col.addObject("TriangleCollisionModel", simulated=True, moving=False, group=0)
        fibula_col.addObject("LineCollisionModel", simulated=True, moving=False, group=0)
        fibula_col.addObject("PointCollisionModel", simulated=True, moving=False, group=0)
        fibula_col.addObject("RigidMapping")

    # ----- Talus (free rigid body) -----
    talus = root.addChild("Talus")
    talus.addObject(
        "EulerImplicitSolver",
        rayleighStiffness=config.solver.rayleigh_stiffness,
        rayleighMass=config.solver.rayleigh_mass,
    )
    talus.addObject(
        "CGLinearSolver",
        iterations=config.solver.cg_iterations,
        tolerance=config.solver.cg_tolerance,
        threshold=config.solver.cg_threshold,
    )
    talus_mo = talus.addObject(
        "MechanicalObject",
        template="Rigid3d",
        position=[talus_position],
    )
    talus.addObject("UniformMass", totalMass=config.physics.talus_mass)
    talus.addObject("UncoupledConstraintCorrection")

    # Talus collision sub-node
    talus_col = talus.addChild("Collision")
    talus_col.addObject(
        "MeshTopology",
        position=a_verts,
        triangles=a_tris,
    )
    talus_col.addObject("MechanicalObject", template="Vec3d")
    talus_col.addObject("TriangleCollisionModel", simulated=True, moving=True, group=1)
    talus_col.addObject("LineCollisionModel", simulated=True, moving=True, group=1)
    talus_col.addObject("PointCollisionModel", simulated=True, moving=True, group=1)
    talus_col.addObject("RigidMapping")

    # ----- Ligament springs (via Python controller) -----
    # StiffSpringForceField has issues with force propagation across RigidMapping
    # boundaries (known SOFA limitation, also found in sofa-tibia-guide).
    # We use a Python controller that directly applies spring forces to the
    # rigid body velocity, which is more robust.

    tibia_pos_arr = np.array(tibia_position[:3])
    talus_pos_arr = np.array(talus_position[:3])

    scaled_ligaments = []
    ligament_names = []

    for lig in ligaments:
        scaled_lig = dict(lig)
        scaled_lig["stiffness"] = lig["stiffness"] * config.stiffness_scale

        # Auto-compute rest length from initial geometry if not specified
        if "rest_length" not in lig:
            if "fixed_anchor" in lig:
                proximal_world = np.array(lig["fixed_anchor"])
            else:
                proximal_world = tibia_pos_arr + np.array(lig["tibia_offset"])
            a_world = talus_pos_arr + np.array(lig["talus_offset"])
            scaled_lig["rest_length"] = float(np.linalg.norm(a_world - proximal_world))
        else:
            scaled_lig["rest_length"] = lig["rest_length"]

        # Preserve fixed_anchor in scaled config
        if "fixed_anchor" in lig:
            scaled_lig["fixed_anchor"] = lig["fixed_anchor"]

        scaled_ligaments.append(scaled_lig)
        ligament_names.append(lig["name"])

    # Pre-create force fields with zero force. Updated each step by the
    # controller (ligaments) or by apply_torque() (external torque).
    # Must exist before init() for SOFA to register them in the solver.
    ligament_ff = talus.addObject(
        "ConstantForceField",
        name="LigamentFF",
        template="Rigid3d",
        forces=[[0, 0, 0, 0, 0, 0]],
        indices=[0],
    )
    torque_ff = talus.addObject(
        "ConstantForceField",
        name="TorqueFF",
        template="Rigid3d",
        forces=[[0, 0, 0, 0, 0, 0]],
        indices=[0],
    )

    controller = root.addObject(
        LigamentForceController(
            name="LigamentController",
            ligament_configs=scaled_ligaments,
            tibia_mo=tibia_mo,
            talus_mo=talus_mo,
            ligament_ff=ligament_ff,
            bilinear=config.bilinear,
            force_model_config=fm_config,
        )
    )

    info = {
        "tibia_mo": tibia_mo,
        "talus_mo": talus_mo,
        "torque_ff": torque_ff,
        "ligaments": {name: cfg for name, cfg in zip(ligament_names, scaled_ligaments)},
        "ligament_controller": controller,
        "mesh_mode": config.mesh_mode,
    }
    if config.mesh_mode == "stl":
        info["mesh_data"] = mesh_data

    return root, info


def compute_joint_angle(tibia_node, talus_node, axis=0):
    """
    Compute joint angle from relative orientation of talus w.r.t. tibia.

    Args:
        tibia_node: SOFA node for tibia.
        talus_node: SOFA node for talus.
        axis: 0=sagittal (DF/PF), 1=frontal (Inv/Ev), 2=transverse.

    Returns:
        Angle in degrees. Positive = dorsiflexion (axis 0).
    """
    tibia_mo = tibia_node.getObject("MechanicalObject")
    talus_mo = talus_node.getObject("MechanicalObject")

    tibia_quat = np.array(tibia_mo.position.value[0][3:7])
    talus_quat = np.array(talus_mo.position.value[0][3:7])

    # Relative orientation: talus in tibia frame
    rel_quat = _quat_multiply(_quat_inverse(tibia_quat), talus_quat)

    # Decompose to Euler angles
    euler = _quat_to_euler(rel_quat)
    return euler[axis]


def apply_torque(root, torque_nm, axis=0):
    """
    Apply a constant torque to the talus for ROM sweep.

    The TorqueFF ConstantForceField is pre-created during scene construction.
    This function just updates the force value.

    Args:
        root: SOFA root node.
        torque_nm: Torque magnitude in N·m. Converted to N·mm internally.
                   Positive = dorsiflexion for axis 0.
        axis: 0=sagittal, 1=frontal, 2=transverse.
    """
    talus = root.getChild("Talus")

    # ConstantForceField for Rigid3d applies [fx, fy, fz, tx, ty, tz]
    force = [0.0] * 6
    force[3 + axis] = torque_nm * 1000.0  # Convert N·m to N·mm

    # Find and update the pre-created TorqueFF
    for obj in talus.objects:
        if obj.name.value == "TorqueFF":
            with obj.forces.writeable() as w:
                w[0] = force
            return

    raise RuntimeError("TorqueFF not found — was the scene created with create_ankle_scene?")


def measure_rom_arc(
    config=None,
    torque_nm=5.0,
    steps_per_direction=1000,
    dt=0.001,
    stiffness_scale=1.0,
    bilinear=False,
    mesh_mode="box",
    mesh_dir=None,
    force_model=None,
    ligament_model="simple",
    exp_B=30.0,
):
    """
    Measure total sagittal ROM arc (DF + PF) by sweeping in both directions.

    Args:
        config: AnkleSimConfig object. If provided, overrides legacy kwargs.
        torque_nm: Applied torque magnitude in N·m.
        steps_per_direction: Simulation steps per direction.
        dt: Timestep (legacy kwarg, ignored when config provided).
        stiffness_scale: Ligament stiffness multiplier.
        bilinear: Use bilinear force model.
        mesh_mode: "box" or "stl".
        mesh_dir: STL mesh directory.
        force_model: Force model name.
        ligament_model: "simple" or "anatomical".
        exp_B: Exponential model B parameter.

    Returns:
        Total arc in degrees (positive).
    """
    if config is not None:
        scene_kwargs = dict(config=config)
        _dt = config.solver.dt
    else:
        scene_kwargs = dict(
            gravity=[0, 0, 0],
            dt=dt,
            stiffness_scale=stiffness_scale,
            bilinear=bilinear,
            mesh_mode=mesh_mode,
            mesh_dir=mesh_dir,
            force_model=force_model,
            ligament_model=ligament_model,
            exp_B=exp_B,
        )
        _dt = dt

    # Measure dorsiflexion
    root_df, _ = create_ankle_scene(**scene_kwargs)
    Sofa.Simulation.init(root_df)
    apply_torque(root_df, torque_nm=abs(torque_nm), axis=0)
    for _ in range(steps_per_direction):
        Sofa.Simulation.animate(root_df, _dt)
    df_angle = compute_joint_angle(
        root_df.getChild("Tibia"), root_df.getChild("Talus"), axis=0
    )

    # Measure plantarflexion (separate scene to start from neutral)
    root_pf, _ = create_ankle_scene(**scene_kwargs)
    Sofa.Simulation.init(root_pf)
    apply_torque(root_pf, torque_nm=-abs(torque_nm), axis=0)
    for _ in range(steps_per_direction):
        Sofa.Simulation.animate(root_pf, _dt)
    pf_angle = compute_joint_angle(
        root_pf.getChild("Tibia"), root_pf.getChild("Talus"), axis=0
    )

    total_arc = abs(df_angle) + abs(pf_angle)
    return total_arc


def measure_rom_components(
    config=None,
    torque_nm=5.0,
    steps_per_direction=2000,
    dt=0.001,
    stiffness_scale=1.0,
    bilinear=False,
    mesh_mode="box",
    mesh_dir=None,
    force_model=None,
    ligament_model="simple",
    exp_B=30.0,
):
    """
    Measure DF, PF, and total ROM arc separately.

    Args:
        config: AnkleSimConfig object. If provided, overrides legacy kwargs.
        torque_nm: Applied torque magnitude in N·m.
        steps_per_direction: Simulation steps per direction.
        dt: Timestep (legacy kwarg, ignored when config provided).
        stiffness_scale: Ligament stiffness multiplier.
        bilinear: Use bilinear force model.
        mesh_mode: "box" or "stl".
        mesh_dir: STL mesh directory.
        force_model: Force model name.
        ligament_model: "simple" or "anatomical".
        exp_B: Exponential model B parameter.

    Returns:
        (df_angle, pf_angle, total_arc) — all positive degrees.
    """
    if config is not None:
        scene_kwargs = dict(config=config)
        _dt = config.solver.dt
    else:
        scene_kwargs = dict(
            gravity=[0, 0, 0],
            dt=dt,
            stiffness_scale=stiffness_scale,
            bilinear=bilinear,
            mesh_mode=mesh_mode,
            mesh_dir=mesh_dir,
            force_model=force_model,
            ligament_model=ligament_model,
            exp_B=exp_B,
        )
        _dt = dt

    # Measure dorsiflexion
    root_df, _ = create_ankle_scene(**scene_kwargs)
    Sofa.Simulation.init(root_df)
    apply_torque(root_df, torque_nm=abs(torque_nm), axis=0)
    for _ in range(steps_per_direction):
        Sofa.Simulation.animate(root_df, _dt)
    df_angle = abs(compute_joint_angle(
        root_df.getChild("Tibia"), root_df.getChild("Talus"), axis=0
    ))

    # Measure plantarflexion
    root_pf, _ = create_ankle_scene(**scene_kwargs)
    Sofa.Simulation.init(root_pf)
    apply_torque(root_pf, torque_nm=-abs(torque_nm), axis=0)
    for _ in range(steps_per_direction):
        Sofa.Simulation.animate(root_pf, _dt)
    pf_angle = abs(compute_joint_angle(
        root_pf.getChild("Tibia"), root_pf.getChild("Talus"), axis=0
    ))

    return df_angle, pf_angle, df_angle + pf_angle

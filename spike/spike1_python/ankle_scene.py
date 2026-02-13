"""
Spike 1: Minimal SOFA ankle joint scene.

Emergent joint model:
  - Tibia: fixed rigid body (proximal)
  - Talus: free rigid body (distal), constrained by ligament springs + bone contact
  - Ligaments: Python controller applies spring forces directly to rigid body
    (StiffSpringForceField has cross-mapping force propagation issues)
  - Joint angle: relative quaternion decomposition of talus w.r.t. tibia

Uses simple geometric collision meshes (boxes) instead of real bone geometry.
Validated against SOFA v24.06.00.
"""
import math
import numpy as np

import Sofa.Core
import Sofa.Simulation


# ---------------------------------------------------------------------------
# Constants — approximate ankle anatomy (all units: mm, kg, N)
# ---------------------------------------------------------------------------

# Ligament properties: attachment points in local frame of each bone.
# Rest lengths computed from initial geometry so springs start at rest.
# Stiffness values are approximate (N/mm).
# Simplified to 3 key ligaments for the spike.
#
# Geometry:
#   Tibia at origin, talus centered 30mm below (z=-30).
#   Tibia box: 40x30x30mm (half: 20,15,15). Bottom face at z=-15.
#   Talus box: 36x26x24mm (half: 18,13,12). Top face at z=-30+12=-18.
#   Gap between boxes: 3mm (within contact distance=3mm).
#   Ligaments span from tibia inferior to talus superior.
#
DEFAULT_LIGAMENTS = [
    {
        "name": "ATFL",  # Anterior talofibular (lateral-anterior)
        "tibia_offset": [15.0, -10.0, -14.0],   # near bottom of tibia, lateral
        "talus_offset": [12.0, -8.0, 11.0],      # near top of talus, lateral
        "stiffness": 70.0,     # N/mm
        "damping": 1.0,
    },
    {
        "name": "CFL",   # Calcaneofibular (lateral)
        "tibia_offset": [5.0, -14.0, -14.0],    # bottom of tibia, lateral
        "talus_offset": [3.0, -12.0, 11.0],      # top of talus, lateral
        "stiffness": 40.0,
        "damping": 1.0,
    },
    {
        "name": "Deltoid",  # Medial (simplified single band)
        "tibia_offset": [-12.0, -5.0, -14.0],   # bottom of tibia, medial
        "talus_offset": [-10.0, -3.0, 11.0],     # top of talus, medial
        "stiffness": 90.0,
        "damping": 1.0,
    },
]

# Tibia is at origin (fixed). Talus sits below it.
TIBIA_POSITION = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]  # [x,y,z, qx,qy,qz,qw]
TALUS_POSITION = [0.0, 0.0, -30.0, 0.0, 0.0, 0.0, 1.0]  # 30mm below tibia

# Simple box collision geometry (half-extents in mm)
TIBIA_BOX_HALF = [20.0, 15.0, 15.0]
TALUS_BOX_HALF = [18.0, 13.0, 12.0]

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
# Bilinear ligament controller
# ---------------------------------------------------------------------------

class LigamentForceController(Sofa.Core.Controller):
    """
    Applies ligament spring forces directly to rigid body velocities.

    Uses a Python controller instead of StiffSpringForceField to avoid
    cross-mapping force propagation issues (known SOFA limitation).

    Supports bilinear stiffness: below toe_strain, uses reduced stiffness.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.ligament_configs = kwargs.get("ligament_configs", [])
        self.tibia_mo = kwargs.get("tibia_mo")
        self.talus_mo = kwargs.get("talus_mo")
        self.bilinear = kwargs.get("bilinear", False)
        self.toe_strain = kwargs.get("toe_strain", 0.03)
        self.toe_ratio = kwargs.get("toe_ratio", 0.2)
        self.dt = kwargs.get("dt", 0.001)
        self.talus_mass = kwargs.get("talus_mass", 0.1)

    def onAnimateBeginEvent(self, event):
        tibia_pos = np.array(self.tibia_mo.position.value[0][:3])
        talus_pos = np.array(self.talus_mo.position.value[0][:3])
        tibia_quat = np.array(self.tibia_mo.position.value[0][3:7])
        talus_quat = np.array(self.talus_mo.position.value[0][3:7])

        total_force = np.zeros(3)
        total_torque = np.zeros(3)

        for config in self.ligament_configs:
            # World-space attachment points
            tibia_attach = tibia_pos + _rotate_by_quat(
                config["tibia_offset"], tibia_quat
            )
            talus_attach = talus_pos + _rotate_by_quat(
                config["talus_offset"], talus_quat
            )

            # Spring direction and length
            diff = tibia_attach - talus_attach
            current_length = np.linalg.norm(diff)
            if current_length < 1e-6:
                continue
            direction = diff / current_length

            rest_length = config["rest_length"]
            extension = current_length - rest_length

            # Stiffness (bilinear or linear)
            linear_k = config["stiffness"]
            if self.bilinear and extension > 0:
                strain = extension / rest_length
                if strain < self.toe_strain:
                    effective_k = linear_k * self.toe_ratio
                else:
                    effective_k = linear_k
            else:
                effective_k = linear_k

            # Spring force: F = k * extension * direction + damping
            damping = config.get("damping", 1.0)
            spring_force = effective_k * extension * direction

            # Damping (velocity-based, approximate)
            talus_vel = np.array(self.talus_mo.velocity.value[0][:3])
            vel_along_spring = np.dot(talus_vel, direction)
            damping_force = -damping * vel_along_spring * direction

            force = spring_force + damping_force
            total_force += force

            # Torque: r × F where r is from talus center to attachment point
            r = talus_attach - talus_pos
            total_torque += np.cross(r, force)

        # Apply force and torque to talus via velocity update
        # F = m * a → dv = F/m * dt
        # τ = I * α → dω = τ/I * dt (approximate I as sphere)
        with self.talus_mo.velocity.writeable() as vel:
            # Linear velocity update
            dv = total_force / self.talus_mass * self.dt
            vel[0][:3] = [vel[0][0] + dv[0], vel[0][1] + dv[1], vel[0][2] + dv[2]]

            # Angular velocity update (approximate moment of inertia)
            # For a 100g box: I ≈ 1/12 * m * (w² + h²) ≈ 50 kg·mm²
            I_approx = self.talus_mass * (18.0**2 + 12.0**2) / 12.0  # kg·mm²
            dw = total_torque / I_approx * self.dt
            vel[0][3] = vel[0][3] + dw[0]
            vel[0][4] = vel[0][4] + dw[1]
            vel[0][5] = vel[0][5] + dw[2]


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
    gravity=None,
    dt=0.001,
    constraint_iterations=200,
    stiffness_scale=1.0,
    bilinear=False,
    toe_strain=0.03,
    toe_ratio=0.2,
    ligaments=None,
):
    """
    Create a SOFA scene with a minimal ankle joint model.

    Args:
        gravity: [gx, gy, gz] in mm/s². Default [0, 0, -9810].
        dt: Timestep in seconds.
        constraint_iterations: Max solver iterations.
        stiffness_scale: Multiplier for all ligament stiffnesses.
        bilinear: If True, use bilinear ligament model (toe + linear).
        toe_strain: Strain threshold for toe region (bilinear only).
        toe_ratio: Ratio of toe stiffness to linear stiffness (bilinear only).
        ligaments: Custom ligament configs (default: DEFAULT_LIGAMENTS).

    Returns:
        (root, info) where info contains references to key scene components.
    """
    if gravity is None:
        gravity = [0, 0, -9810]
    if ligaments is None:
        ligaments = DEFAULT_LIGAMENTS

    root = Sofa.Core.Node("root")
    root.gravity = gravity
    root.dt = dt

    # Plugins
    root.addObject("RequiredPlugin", pluginName=SOFA_PLUGINS)

    # Animation loop + constraint solver
    root.addObject("FreeMotionAnimationLoop")
    root.addObject(
        "GenericConstraintSolver",
        maxIterations=constraint_iterations,
        tolerance=1e-4,
    )

    # Collision pipeline
    root.addObject("CollisionPipeline")
    root.addObject("BruteForceBroadPhase")
    root.addObject("BVHNarrowPhase")
    root.addObject(
        "LocalMinDistance",
        alarmDistance=8.0,
        contactDistance=3.0,
        angleCone=0.1,
    )
    root.addObject(
        "CollisionResponse",
        response="FrictionContactConstraint",
    )

    # ----- Tibia (fixed rigid body) -----
    tibia = root.addChild("Tibia")
    tibia.addObject(
        "EulerImplicitSolver",
        rayleighStiffness=0.1,
        rayleighMass=1.0,
    )
    tibia.addObject("CGLinearSolver", iterations=25, tolerance=1e-10, threshold=1e-10)
    tibia_mo = tibia.addObject(
        "MechanicalObject",
        template="Rigid3d",
        position=[TIBIA_POSITION],
    )
    tibia.addObject("UniformMass", totalMass=1.0)
    tibia.addObject("FixedConstraint", indices=[0])
    tibia.addObject("UncoupledConstraintCorrection")

    # Tibia collision sub-node
    tibia_col = tibia.addChild("Collision")
    t_verts, t_tris = _box_vertices_and_triangles(TIBIA_BOX_HALF)
    tibia_col.addObject(
        "MeshTopology",
        position=t_verts,
        triangles=t_tris,
    )
    tibia_col.addObject("MechanicalObject", template="Vec3d")
    tibia_col.addObject("TriangleCollisionModel", simulated=True, moving=False)
    tibia_col.addObject("LineCollisionModel", simulated=True, moving=False)
    tibia_col.addObject("PointCollisionModel", simulated=True, moving=False)
    tibia_col.addObject("RigidMapping")

    # ----- Talus (free rigid body) -----
    talus = root.addChild("Talus")
    talus.addObject(
        "EulerImplicitSolver",
        rayleighStiffness=0.1,
        rayleighMass=1.0,
    )
    talus.addObject("CGLinearSolver", iterations=25, tolerance=1e-10, threshold=1e-10)
    talus_mo = talus.addObject(
        "MechanicalObject",
        template="Rigid3d",
        position=[TALUS_POSITION],
    )
    talus.addObject("UniformMass", totalMass=0.1)  # ~100g talus
    talus.addObject("UncoupledConstraintCorrection")

    # Talus collision sub-node
    talus_col = talus.addChild("Collision")
    a_verts, a_tris = _box_vertices_and_triangles(TALUS_BOX_HALF)
    talus_col.addObject(
        "MeshTopology",
        position=a_verts,
        triangles=a_tris,
    )
    talus_col.addObject("MechanicalObject", template="Vec3d")
    talus_col.addObject("TriangleCollisionModel", simulated=True, moving=True)
    talus_col.addObject("LineCollisionModel", simulated=True, moving=True)
    talus_col.addObject("PointCollisionModel", simulated=True, moving=True)
    talus_col.addObject("RigidMapping")

    # ----- Ligament springs (via Python controller) -----
    # StiffSpringForceField has issues with force propagation across RigidMapping
    # boundaries (known SOFA limitation, also found in sofa-tibia-guide).
    # We use a Python controller that directly applies spring forces to the
    # rigid body velocity, which is more robust.

    tibia_pos_arr = np.array(TIBIA_POSITION[:3])
    talus_pos_arr = np.array(TALUS_POSITION[:3])

    scaled_ligaments = []
    ligament_names = []

    for lig in ligaments:
        scaled_lig = dict(lig)
        scaled_lig["stiffness"] = lig["stiffness"] * stiffness_scale

        # Auto-compute rest length from initial geometry if not specified
        if "rest_length" not in lig:
            t_world = tibia_pos_arr + np.array(lig["tibia_offset"])
            a_world = talus_pos_arr + np.array(lig["talus_offset"])
            scaled_lig["rest_length"] = float(np.linalg.norm(a_world - t_world))
        else:
            scaled_lig["rest_length"] = lig["rest_length"]

        scaled_ligaments.append(scaled_lig)
        ligament_names.append(lig["name"])

    controller = root.addObject(
        LigamentForceController(
            name="LigamentController",
            ligament_configs=scaled_ligaments,
            tibia_mo=tibia_mo,
            talus_mo=talus_mo,
            bilinear=bilinear,
            toe_strain=toe_strain,
            toe_ratio=toe_ratio,
            dt=dt,
            talus_mass=0.1,
        )
    )

    # Pre-create the torque force field with zero force (updated later via apply_torque).
    # Must exist before init() for SOFA to register it in the solver.
    torque_ff = talus.addObject(
        "ConstantForceField",
        name="TorqueFF",
        template="Rigid3d",
        forces=[[0, 0, 0, 0, 0, 0]],
        indices=[0],
    )

    info = {
        "tibia_mo": tibia_mo,
        "talus_mo": talus_mo,
        "torque_ff": torque_ff,
        "ligaments": {name: cfg for name, cfg in zip(ligament_names, scaled_ligaments)},
        "ligament_controller": controller,
    }

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
    torque_nm=5.0,
    steps_per_direction=1000,
    dt=0.001,
    stiffness_scale=1.0,
    bilinear=False,
):
    """
    Measure total sagittal ROM arc (DF + PF) by sweeping in both directions.

    Returns:
        Total arc in degrees (positive).
    """
    # Measure dorsiflexion
    root_df, _ = create_ankle_scene(
        gravity=[0, 0, 0],
        dt=dt,
        stiffness_scale=stiffness_scale,
        bilinear=bilinear,
    )
    Sofa.Simulation.init(root_df)
    apply_torque(root_df, torque_nm=abs(torque_nm), axis=0)
    for _ in range(steps_per_direction):
        Sofa.Simulation.animate(root_df, dt)
    df_angle = compute_joint_angle(
        root_df.getChild("Tibia"), root_df.getChild("Talus"), axis=0
    )

    # Measure plantarflexion (separate scene to start from neutral)
    root_pf, _ = create_ankle_scene(
        gravity=[0, 0, 0],
        dt=dt,
        stiffness_scale=stiffness_scale,
        bilinear=bilinear,
    )
    Sofa.Simulation.init(root_pf)
    apply_torque(root_pf, torque_nm=-abs(torque_nm), axis=0)
    for _ in range(steps_per_direction):
        Sofa.Simulation.animate(root_pf, dt)
    pf_angle = compute_joint_angle(
        root_pf.getChild("Tibia"), root_pf.getChild("Talus"), axis=0
    )

    total_arc = abs(df_angle) + abs(pf_angle)
    return total_arc

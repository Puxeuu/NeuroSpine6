"""
Llegeix un fitxer URDF i el converteix en objectes Python (Robot, Link, Joint).

A més del URDF estàndard, també llegeix les nostres extensions:
    - <inertial>    propietats dinàmiques del link (mass, com, inertia)
    - <motor>       característiques dels motors (type, gear_ratio, kt)
    - <friction>    parell de fricció estàtica per cada joint
    - <gravity>     vector gravetat al món

Un cop carregat, la resta del projecte no tornarà a llegir el XML.
Sempre treballarà amb l'objecte Robot i els seus links/joints.

S'utilitza tant pel cobot (cobot.urdf) com pel mando (mando6dof.urdf).


RECORDATORI: 
Un element XML (pot no existir)	        .find() + if is not None
Molts elements XML (pot ser cap)	    .findall()
Un atribut obligatori	                .attrib["nom"] (peta si falta)
Un atribut opcional	                    .attrib.get("nom", default)

EXEMPLES: 
inertial = xml_link.find("inertial")            # element opcional (jocs sense inertial)
mass_tag = inertial.find("mass")                # element opcional dins de inertial
mass = float(mass_tag.attrib["value"]) if ...   # si tens <mass>, DEU tenir value
ixx = float(inertia_tag.attrib.get("ixx", 0.0)) # ixx opcional
name = xml_link.attrib["name"]                  # obligatori pel URDF, peta si falta

"""


from dataclasses import dataclass, field
from typing import Optional, Dict
import xml.etree.ElementTree as ET
import numpy as np


# ==============================================================================
# INERTIALPARAMS
# Propietats dinàmiques d'un link (venen del <inertial> del URDF).
# ==============================================================================

@dataclass
class InertialParams:
    mass: float = 0.0                       # kg
    com: np.ndarray = None                  # (3,) posició del CoM al frame local, m
    inertia: np.ndarray = None              # (3, 3) tensor d'inèrcia al CoM, kg·m²

    def __post_init__(self):
        if self.com is None:
            self.com = np.zeros(3)
        if self.inertia is None:
            self.inertia = np.zeros((3, 3))


# ==============================================================================
# LINK
# Representa una peça rígida del robot.
# Guardem:
#   - nom
#   - fitxer STL
#   - offset visual (xyz i rpy del <visual><origin>) per si l'STL
#     no està alineat amb el frame del link.
#   - inertial (opcional, InertialParams del bloc <inertial>)
# ==============================================================================

@dataclass
class Link:
    name: str
    mesh: Optional[str] = None
    visual_xyz: np.ndarray = None
    visual_rpy: np.ndarray = None
    inertial: Optional[InertialParams] = None

    def __post_init__(self):
        if self.visual_xyz is None:
            self.visual_xyz = np.zeros(3)
        if self.visual_rpy is None:
            self.visual_rpy = np.zeros(3)


# ==============================================================================
# JOINT
# Uneix dos links amb un moviment concret.
# Guardem:
#   - nom, pare, fill, tipus (fixed, revolute, prismatic)
#   - posició i orientació del muntatge (xyz, rpy)
#   - eix de moviment (axis)
#   - límits inferior i superior
# ==============================================================================

@dataclass
class Joint:
    name: str
    parent: str 
    child: str
    type: str
    xyz: tuple
    rpy: tuple
    axis: tuple 
    lower: float
    upper: float


# ==============================================================================
# MOTORSPEC
# Característiques d'un motor (vénen del bloc <motor> del URDF).
# ==============================================================================

@dataclass
class MotorSpec:
    joint_name: str
    motor_type: str = ""                    # ex: "TA-52", "TA-40"
    gear_ratio: int = 1
    kt: float = 0.0                         # Nm/A (post-reductor)


# ==============================================================================
# ROBOT
# Conté tots els links i joints del robot.
# Accés per nom: robot.links["arm3"], robot.joints["j4"].
# També conté (si el URDF els defineix) motors, friction i gravity.
# ==============================================================================

@dataclass
class Robot:
    name: str
    links: dict
    joints: dict
    motors: Dict[str, MotorSpec] = field(default_factory=dict)
    friction: Dict[str, float] = field(default_factory=dict)
    gravity: np.ndarray = None

    def __post_init__(self):
        if self.gravity is None:
            self.gravity = np.array([0.0, 0.0, -9.81])


# ==============================================================================
# HELPERS interns
# ==============================================================================

def _parse_inertial(xml_link):
    """
    Llegeix el bloc <inertial> d'un link i retorna un InertialParams.
    Si no hi ha <inertial> o els valors són tots zero, retorna None.
    """
    inertial = xml_link.find("inertial")
    if inertial is None:
        return None

    # ---- Massa ----
    mass_tag = inertial.find("mass")
    mass = float(mass_tag.attrib["value"]) if mass_tag is not None else 0.0

    # ---- CoM (origen del <inertial>) ----
    com = np.zeros(3)
    origin = inertial.find("origin")
    if origin is not None and "xyz" in origin.attrib:
        com = np.array(list(map(float, origin.attrib["xyz"].split())))

    # ---- Tensor d'inèrcia 3x3 (simètric) ----
    I = np.zeros((3, 3))
    inertia_tag = inertial.find("inertia")
    if inertia_tag is not None:
        ixx = float(inertia_tag.attrib.get("ixx", 0.0))
        iyy = float(inertia_tag.attrib.get("iyy", 0.0))
        izz = float(inertia_tag.attrib.get("izz", 0.0))
        ixy = float(inertia_tag.attrib.get("ixy", 0.0))
        ixz = float(inertia_tag.attrib.get("ixz", 0.0))
        iyz = float(inertia_tag.attrib.get("iyz", 0.0))
        I = np.array([
            [ixx, ixy, ixz],
            [ixy, iyy, iyz],
            [ixz, iyz, izz],
        ])

    if mass == 0.0 and np.all(I == 0):
        return None

    return InertialParams(mass=mass, com=com, inertia=I)                        # retornem: InertialParams(...)


# ==============================================================================
# LOAD_URDF - Passos que segueix:
#   1. Obre i parseja el XML.
#   2. Llegeix el nom del robot.
#   3. Recorre tots els <link> (visual + inertial) i els guarda a robot.links.
#   4. Recorre tots els <joint> i els guarda a robot.joints.
#   5. Recorre els blocs <motor>, <friction>, <gravity> (extensions pròpies).
#   6. Retorna l'objecte Robot.
# ==============================================================================

def load_urdf(filename):
    tree = ET.parse(filename)
    root = tree.getroot()

    robot = Robot(
        name=root.attrib["name"],
        links={},
        joints={},
    )

    # --------------------------------------------------------------------------
    # LINKS
    # --------------------------------------------------------------------------
    for xml_link in root.findall("link"):
        name = xml_link.attrib["name"]                                          # assignem name
        mesh = None
        visual_xyz = np.zeros(3)
        visual_rpy = np.zeros(3)

        visual = xml_link.find("visual")
        if visual is not None:

            # Ruta del fitxer STL
            geometry = visual.find("geometry")
            if geometry is not None:
                mesh_tag = geometry.find("mesh")
                if mesh_tag is not None:
                    mesh = mesh_tag.attrib["filename"]

            # Offset visual: si l'STL no està alineat amb el frame del link,
            # aquí guardem la transformació per corregir-ho al viewer.
            origin = visual.find("origin")
            if origin is not None:
                if "xyz" in origin.attrib:
                    visual_xyz = np.array(list(map(float, origin.attrib["xyz"].split())))
                if "rpy" in origin.attrib:
                    visual_rpy = np.array(list(map(float, origin.attrib["rpy"].split())))

        # ---- Propietats dinàmiques (inertial) ----
        inertial = _parse_inertial(xml_link)

        robot.links[name] = Link(
            name=name,
            mesh=mesh,
            visual_xyz=visual_xyz,
            visual_rpy=visual_rpy,
            inertial=inertial,
        )

    # --------------------------------------------------------------------------
    # JOINTS
    # --------------------------------------------------------------------------
    for xml_joint in root.findall("joint"):

        # <origin> del muntatge
        origin = xml_joint.find("origin")
        if origin is not None:
            xyz = tuple(map(float, origin.attrib["xyz"].split()))
            rpy = tuple(map(float, origin.attrib["rpy"].split()))
        else:
            xyz = (0.0, 0.0, 0.0)
            rpy = (0.0, 0.0, 0.0)

        # <axis>: eix de moviment (per defecte Z si no s'especifica)
        axis = (0.0, 0.0, 1.0)
        axis_tag = xml_joint.find("axis")
        if axis_tag is not None:
            axis = tuple(map(float, axis_tag.attrib["xyz"].split()))

        # <limit>: rang de moviment del joint
        lower = 0.0                                                             # assignem lower
        upper = 0.0                                                             # assignem upper
        limit = xml_joint.find("limit")
        if limit is not None:
            lower = float(limit.attrib["lower"])
            upper = float(limit.attrib["upper"])

        joint = Joint(                                                          # assignem joint
            name=xml_joint.attrib["name"],
            parent=xml_joint.find("parent").attrib["link"],
            child=xml_joint.find("child").attrib["link"],
            type=xml_joint.attrib["type"],
            xyz=xyz,
            rpy=rpy,
            axis=axis,
            lower=lower,
            upper=upper,
        )
        robot.joints[joint.name] = joint

    # --------------------------------------------------------------------------
    # EXTENSIONS NO-ESTÀNDARDS: motors, friction, gravity
    # --------------------------------------------------------------------------

    # <motor joint="jN" type="TA-52" gear_ratio="101" kt="5.19"/>
    for xml_motor in root.findall("motor"):
        joint_name = xml_motor.attrib["joint"]
        robot.motors[joint_name] = MotorSpec(
            joint_name=joint_name,
            motor_type=xml_motor.attrib.get("type", ""),
            gear_ratio=int(xml_motor.attrib.get("gear_ratio", 1)),
            kt=float(xml_motor.attrib.get("kt", 0.0)),
        )

    # <friction joint="jN" torque_Nm="0.0"/>
    for xml_fric in root.findall("friction"):
        joint_name = xml_fric.attrib["joint"]
        robot.friction[joint_name] = float(xml_fric.attrib.get("torque_Nm", 0.0))

    # <gravity xyz="0 0 -9.81"/>
    gravity_tag = root.find("gravity")
    if gravity_tag is not None and "xyz" in gravity_tag.attrib:
        robot.gravity = np.array(list(map(float, gravity_tag.attrib["xyz"].split())))

    return robot
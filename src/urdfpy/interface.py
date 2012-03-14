class Axis(object):
    x = 1.
    y = 0.
    z = 0.

    def __str__(self):
        return "x: {0}, y: {1}, z: {2}".format(
            self.x, self.y, self.z)

class Pose(object):
    x = 0.
    y = 0.
    z = 0.
    roll = 0.
    pitch = 0.
    yaw = 0.

    def __str__(self):
        return "x: {0}, y: {1}, z: {2}, roll: {3}, pitch: {4}, yaw: {5}".format(
            self.x, self.y, self.z, self.roll, self.pitch, self.yaw)

class Calibration(object):
    reference_position = 0.
    falling = None
    rising = None

    def __str__(self):
        s =  "Reference position: {0}\n".format(self.reference_position)
        s +=  "Raising: {0}\n".format(self.rising)
        s +=  "Falling: {0}\n".format(self.falling)

        return s


class Collision(object):
    origin = None
    geometry = None

    def __str__(self):
        s =  "Origin: {0}\n".format(self.origin)
        s += "Geometry:\n"
        s += str(self.geometry)
        return s

class Dynamics(object):
    damping = 0.
    friction = 0.

    def __str__(self):
        s = "Damping: {0}\n".format(self.damping)
        s += "Friction: {0}\n".format(self.friction)
        return s

class Geometry(object):
    def __str__(self):
        return "Geometry abstract class"

class Box(Geometry):
    size = [0., 0., 0.]

    def __str__(self):
        return "Size: {0} {1} {2}".format(
            self.size[0], self.size[1], self.size[2])

class Cylinder(Geometry):
    radius = 0.
    length = 0.

    def __str__(self):
        s = "Radius: {0}\n".format(self.radius)
        s += "Length: {0}\n".format(self.length)
        return s

class Sphere(Geometry):
    radius = 0.

    def __str__(self):
        s = "Radius: {0}\n".format(self.radius)
        return s


class Mesh(Geometry):
    filename = ""
    scale = 1.

    def __str__(self):
        s = "Filename: {0}\n".format(self.filename)
        s += "Scale: {0}\n".format(self.scale)
        return s

class Material(object):
    name = ""
    color = None
    texture = None

    def __str__(self):
        s = "Name: {0}\n".format(self.name)
        s += "Color: {0}\n".format(self.color)
        s += "Texture:\n"
        s += str(self.texture)
        return s


class Color(object):
    r = 0.
    g = 0.
    b = 0.
    a = 0.

    def __str__(self):
        return "r: {0}, g: {1}, b: {2}, a: {3},".format(
            self.r, self.g, self.b, self.a)

class Texture(object):
    filename = ""

    def __str__(self):
        return "Filename: {0}\n".format(self.filename)

class Inertial(object):
    ixx = 0.
    ixy = 0.
    ixz = 0.
    iyy = 0.
    iyz = 0.
    izz = 0.
    mass = 0.
    origin = None

    def __str__(self):
        s =  "Origin: {0}\n".format(self.origin)
        s += "Mass: {0}\n".format(self.mass)
        s +=  "ixx: {0}\n".format(self.ixx)
        s +=  "ixy: {0}\n".format(self.ixy)
        s +=  "ixz: {0}\n".format(self.ixz)
        s +=  "iyy: {0}\n".format(self.iyy)
        s +=  "iyz: {0}\n".format(self.iyz)
        s +=  "izz: {0}\n".format(self.izz)
        return s


class JointLimits(object):
    effort = 0.
    lower = 0.
    upper = 0.
    velocity = 0.

    def __str__(self):
        s = "Effort: {0}\n".format(self.effort)
        s +=  "Lower: {0}\n".format(self.lower)
        s +=  "Upper: {0}\n".format(self.upper)
        s +=  "Velocity: {0}\n".format(self.velocity)
        return s

class Visual(object):
    origin = None
    geometry = None
    material = None

    def __str__(self):
        s =  "Origin: {0}\n".format(self.origin)
        s += "Geometry:\n"
        s += str(self.geometry) + "\n"
        s += "Material:\n"
        s += str(self.material) + "\n"
        return s

class Safety(object):
    safe_lower_limit = 0.
    safe_upper_limit = 0.
    k_position = 0.
    k_velocity = 0.

    def __str__(self):
        s = "Safe lower limit: {0}\n".format(self.safe_lower_limit)
        s += "Safe upper limit: {0}\n".format(self.safe_upper_limit)
        s += "K position: {0}\n".format(self.k_position)
        s += "K velocity: {0}\n".format(self.k_velocity)
        return s


class Link(object):
    child_joints = []
    child_links = []
    collision = None
    inertial = None
    name = ""
    parent_joint_name = None
    visual = None
    parent_link_name = None

    def __str__(self):
        s = "Name: {0}\n".format(self.name)
        s += "Parent joint: {0}\n".format(self.parent_joint_name)
        s += "Parent link: {0}\n".format(self.parent_link_name)

        s += "Inertial:\n"
        s += str(self.inertial) + "\n"

        s += "Collision:\n"
        s += str(self.collision) + "\n"

        s += "Visual:\n"
        s += str(self.visual) + "\n"
        return s


class Joint(object):
    UNKNOWN = 0
    REVOLUTE = 1
    CONTINUOUS = 2
    PRISMATIC = 3
    FLOATING = 4
    PLANAR = 5
    FIXED = 6

    axis = None
    calibration = None
    child_link_name = ""
    dynamics = None
    limits = None
    name = ""
    parent_link_name = ""
    parent_to_joint_origin_transform = None
    safety = None
    type = UNKNOWN

    def __str__(self):
        s = "Name: {0}\n".format(self.name)

        if self.type == Joint.UNKNOWN:
            s += "Type: unknown\n"
        elif self.type == Joint.REVOLUTE:
            s += "Type: revolute\n"
        elif self.type == Joint.CONTINUOUS:
            s += "Type: continuous\n"
        elif self.type == Joint.PRISMATIC:
            s += "Type: prismatic\n"
        elif self.type == Joint.FLOATING:
            s += "Type: floating\n"
        elif self.type == Joint.PLANAR:
            s += "Type: planar\n"
        elif self.type == Joint.FIXED:
            s += "Type: fixed\n"
        else:
            raise RuntimeError("invalid joint type")

        s += "Child link name: {0}\n".format(self.child_link_name)
        s += "Parent link name: {0}\n".format(self.parent_link_name)
        s +=  "Axis: {0}\n".format(self.axis)
        s +=  "Origin: {0}\n".format(self.parent_to_joint_origin_transform)

        s += "Calibration:\n"
        s += str(self.calibration) + "\n"

        s += "Dynamics:\n"
        s += str(self.dynamics) + "\n"

        s += "Limits:\n"
        s += str(self.limits) + "\n"

        s += "Safety:\n"
        s += str(self.safety) + "\n"
        return s

class Model(object):
    joints = []
    links = []
    name = ""
    root_link = None

    def __str__(self):
        s = "Name: {0}\n".format(self.name)
        s = "Root link: {0}\n".format(self.root_link)

        s += "Joints:\n"
        for joint in self.joints:
            s += str(joint) + "\n"

        s += "Links:\n"
        for link in self.links:
            s += str(link) + "\n"

        return s

import xml.dom.minidom

import interface

def toFloat(string):
    if string == "":
        return 0.
    return float(string)

def buildCalibration(calibrationDom):
    calibration = interface.Calibration()
    calibration.reference_position = toFloat(
        calibrationDom.getAttribute("reference_position"))
    calibration.rising = toFloat(calibrationDom.getAttribute("rising"))
    calibration.falling = toFloat(calibrationDom.getAttribute("falling"))
    return calibration

def buildDynamics(dynamicsDom):
    dynamics = interface.Dynamics()
    dynamics.damping = toFloat(dynamicsDom.getAttribute("damping"))
    dynamics.friction = toFloat(dynamicsDom.getAttribute("friction"))
    return dynamics

def buildInertial(inertialDom):
    inertial = interface.Inertial()
    if inertialDom.getElementsByTagName("mass"):
        inertial.mass = toFloat(
            inertialDom.getElementsByTagName("mass")[0].getAttribute("value"))
    if inertialDom.getElementsByTagName("inertia"):
        inertiaDom = inertialDom.getElementsByTagName("inertia")[0]
        inertial.ixx = toFloat(inertiaDom.getAttribute("ixx"))
        inertial.ixy = toFloat(inertiaDom.getAttribute("ixy"))
        inertial.ixz = toFloat(inertiaDom.getAttribute("ixz"))
        inertial.iyy = toFloat(inertiaDom.getAttribute("iyy"))
        inertial.iyz = toFloat(inertiaDom.getAttribute("iyz"))
        inertial.izz = toFloat(inertiaDom.getAttribute("izz"))
    if inertialDom.getElementsByTagName("origin"):
        inertial.origin = buildPose(inertialDom.getElementsByTagName("origin")[0])
    return inertial

def buildAxis(axisDom):
    axis = interface.Axis()

    if axisDom.getAttribute("xyz"):
        vector = axisDom.getAttribute("xyz").split(" ")
        axis.x = vector[0]
        axis.y = vector[1]
        axis.z = vector[2]
    return axis

def buildPose(poseDom):
    pose = interface.Pose()

    if poseDom.getAttribute("xyz"):
        vector = poseDom.getAttribute("xyz").split(" ")
        pose.x = vector[0]
        pose.y = vector[1]
        pose.z = vector[2]

    if poseDom.getAttribute("rpy"):
        vector = poseDom.getAttribute("rpy").split(" ")
        pose.roll = vector[0]
        pose.pitch = vector[1]
        pose.yaw = vector[2]
    return pose

def buildLimits(limitsDom):
    limits = interface.JointLimits()
    limits.effort = toFloat(limitsDom.getAttribute("effort"))
    limits.lower = toFloat(limitsDom.getAttribute("lower"))
    limits.upper = toFloat(limitsDom.getAttribute("upper"))
    limits.velocity = toFloat(limitsDom.getAttribute("velocity"))
    return limits

def buildCollision(collisionDom):
    collision = interface.Collision()
    if collisionDom.getElementsByTagName("origin"):
        collision.origin = \
            buildPose(collisionDom.getElementsByTagName("origin")[0])
    if collisionDom.getElementsByTagName("geometry"):
        collision.geometry = \
            buildGeometry(collisionDom.getElementsByTagName("geometry")[0])
    return collision

def buildBox(boxDom):
    box = interface.Box()
    if boxDom.getAttribute("size"):
        box.size = boxDom.getAttribute("size").split(" ")
    return box

def buildCylinder(cylinderDom):
    cylinder = interface.Cylinder()
    cylinder.radius = toFloat(cylinderDom.getAttribute("radius"))
    cylinder.length = toFloat(cylinderDom.getAttribute("length"))
    return cylinder

def buildSphere(sphereDom):
    sphere = interface.Sphere()
    sphere.radius = toFloat(sphereDom.getAttribute("radius"))
    return sphere

def buildMesh(meshDom):
    mesh = interface.Mesh()
    mesh.filename = meshDom.getAttribute("filename")
    mesh.scale = toFloat(meshDom.getAttribute("scale"))
    return mesh

def buildGeometry(geometryDom):
    if geometryDom.getElementsByTagName("box"):
        return buildBox(geometryDom.getElementsByTagName("box")[0])
    if geometryDom.getElementsByTagName("cylinder"):
        return buildCylinder(geometryDom.getElementsByTagName("cylinder")[0])
    if geometryDom.getElementsByTagName("sphere"):
        return buildSphere(geometryDom.getElementsByTagName("shere")[0])
    if geometryDom.getElementsByTagName("mesh"):
        return buildMesh(geometryDom.getElementsByTagName("mesh")[0])
    return None

def buildColor(colorDom):
    color = interface.Color()
    if colorDom.getAttribute("rgba"):
        vector = poseDom.getAttribute("rgba").split(" ")
        color.r = vector[0]
        color.g = vector[1]
        color.b = vector[2]
        color.a = vector[3]
    return color

def buildTexture(texturDom):
    texture = interface.Texture()
    texture.filename = poseDom.getAttribute("filename")
    texture.scale = toFloat(poseDom.getAttribute("scale"))
    return texture

def buildMaterial(materialDom):
    material = interface.Material()
    material.name = str(materialDom.getAttribute("name"))
    if materialDom.getElementsByTagName("color"):
        material.color = \
            buildColor(materialDom.getElementsByTagName("color")[0])
    if materialDom.getElementsByTagName("texture"):
        material.texture = \
            buildMaterial(materialDom.getElementsByTagName("texture")[0])
    return material

def buildVisual(visualDom):
    visual = interface.Visual()
    if visualDom.getElementsByTagName("origin"):
        visual.origin = \
            buildPose(visualDom.getElementsByTagName("origin")[0])
    if visualDom.getElementsByTagName("geometry"):
        visual.geometry = \
            buildGeometry(visualDom.getElementsByTagName("geometry")[0])
    if visualDom.getElementsByTagName("material"):
        visual.material = \
            buildMaterial(visualDom.getElementsByTagName("material")[0])
    return visual

def buildSafety(safetyDom):
    safety = interface.Safety()
    safety.safe_lower_limit = toFloat(
        safetyDom.getAttribute("safe_lower_limit"))
    safety.safe_upper_limit = toFloat(
        safetyDom.getAttribute("safe_upper_limit"))
    safety.k_position = toFloat(
        safetyDom.getAttribute("k_position"))
    safety.k_velocity = toFloat(
        safetyDom.getAttribute("k_velocity"))
    return safety

def buildLink(linkDom, joints):
    link = interface.Link()

    if linkDom.getElementsByTagName("inertial"):
        link.inertial = buildInertial(linkDom.getElementsByTagName("inertial")[0])

    if linkDom.getElementsByTagName("visual"):
        link.visual = buildVisual(linkDom.getElementsByTagName("visual")[0])

    if linkDom.getElementsByTagName("collision"):
        link.collision = buildCollision(linkDom.getElementsByTagName("collision")[0])

    for joint in joints:
        if joint.parent_link_name and joint.parent_link_name == link.name:
            link.child_joints.append(joint.name)
            link.child_links.append(joint.child_link_name)
        if joint.child_link_name and joint.child_link_name == link.name:
            link.parent_joint_name = joint.name
            link.parent_link_name = joint.parent_link_name

    link.name = str(linkDom.getAttribute("name"))

    return link

def buildJoint(jointDom):
    joint = interface.Joint()

    if jointDom.getElementsByTagName("axis"):
        joint.axis = buildAxis(jointDom.getElementsByTagName("axis")[0])

    if jointDom.getElementsByTagName("calibration"):
        joint.calibration = \
            buildCalibration(jointDom.getElementsByTagName("calibration")[0])

    if jointDom.getElementsByTagName("child"):
        joint.child_link_name = \
            jointDom.getElementsByTagName("child")[0].getAttribute("link")

    if jointDom.getElementsByTagName("dynamics"):
        joint.dynamics = \
            buildDynamics(jointDom.getElementsByTagName("dynamics")[0])

    if jointDom.getElementsByTagName("limit"):
        joint.limits = \
            buildLimits(jointDom.getElementsByTagName("limit")[0])

    joint.name = str(jointDom.getAttribute("name"))

    if jointDom.getElementsByTagName("parent"):
        joint.parent_link_name = \
            jointDom.getElementsByTagName("parent")[0].getAttribute("link")

    if jointDom.getElementsByTagName("origin"):
        joint.parent_to_joint_origin_transform = \
            buildPose(jointDom.getElementsByTagName("origin")[0])

    if jointDom.getElementsByTagName("safety_controller"):
        joint.safety = \
            buildSafety(jointDom.getElementsByTagName("safety_controller")[0])

    if jointDom.getAttribute("type").lower () == "revolute":
        joint.type = interface.Joint.REVOLUTE
    elif jointDom.getAttribute("type").lower () == "continuous":
        joint.type = interface.Joint.CONTINUOUS
    elif jointDom.getAttribute("type").lower () == "prismatic":
        joint.type = interface.Joint.PRISMATIC
    elif jointDom.getAttribute("type").lower () == "fixed":
        joint.type = interface.Joint.FIXED
    elif jointDom.getAttribute("type").lower () == "floating":
        joint.type = interface.Joint.FLOATING
    elif jointDom.getAttribute("type").lower () == "planar":
        joint.type = interface.Joint.PLANAR
    else:
        joint.type = interface.Joint.UNKNOWN
    return joint

def buildModel(modelDom):
    model = interface.Model()
    model.name = str(modelDom.getAttribute("name"))

    # It is very important to build the joints *then* the links.
    for joint in modelDom.getElementsByTagName("joint"):
        model.joints.append(buildJoint(joint))
    for link in modelDom.getElementsByTagName("link"):
        model.links.append(buildLink(link, model.joints))

    # Look for root link
    for link in model.links:
        if not link.parent_joint_name:
            model.root_link = link.name
    return model

def parse(filename):
    dom = xml.dom.minidom.parse(filename)
    return buildModel(dom.getElementsByTagName("robot")[0])

def parseString(string):
    dom = xml.dom.minidom.parseString(string)
    return buildModel(dom.getElementsByTagName("robot")[0])

def parseFromParameterServer():
    import rospy
    return parseString(rospy.get_param("robot_description"))

__all__ = ["parse", "parseString", "parseFromParameterServer"]

"""Convert the installed Husky + UR5e Xacro into a MuJoCo-ready MJCF model."""

from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import mujoco
from PIL import Image, ImageDraw
import trimesh
import xacro

from mujoco_visuals import colorize_scene


WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGE = WORKSPACE / "src" / "Clearpath_mobile_manipulator"
OUTPUT = PACKAGE / "mujoco"
ASSETS = OUTPUT / "assets"
PACKAGE_URI = "package://husky_ur5e_description/"


def create_floor_texture() -> Path:
    """Create a blue checker with visible grid lines like the panel reference."""
    destination = ASSETS / "blue_floor.png"
    destination.parent.mkdir(parents=True, exist_ok=True)
    image = Image.new("RGB", (512, 512))
    draw = ImageDraw.Draw(image)
    for row in range(2):
        for column in range(2):
            color = (69, 110, 151) if (row + column) % 2 == 0 else (108, 153, 193)
            draw.rectangle((column * 256, row * 256,
                            (column + 1) * 256 - 1, (row + 1) * 256 - 1), fill=color)
    for coordinate in (0, 256, 511):
        draw.line((coordinate, 0, coordinate, 511), fill=(190, 218, 238), width=12)
        draw.line((0, coordinate, 511, coordinate), fill=(190, 218, 238), width=12)
    image.save(destination)
    return destination


def convert_meshes(root: ET.Element) -> None:
    ASSETS.mkdir(parents=True, exist_ok=True)
    for mesh in root.iter("mesh"):
        uri = mesh.get("filename")
        if uri is None or not uri.startswith(PACKAGE_URI):
            continue
        source = PACKAGE / uri.removeprefix(PACKAGE_URI)
        if not source.is_file():
            raise FileNotFoundError(source)
        if source.suffix.lower() == ".dae":
            destination = ASSETS / source.relative_to(PACKAGE).with_suffix(".obj")
            destination.parent.mkdir(parents=True, exist_ok=True)
            trimesh.load(source, force="mesh").export(destination)
            mesh.set("filename", str(destination))
        else:
            mesh.set("filename", str(source.resolve()))


def assemble_scene(robot_path: Path) -> Path:
    root = ET.parse(robot_path).getroot()
    root.set("model", "husky_ur5e_drawing")
    ET.SubElement(root, "option", timestep="0.002", integrator="implicitfast", iterations="50")
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("The converted robot has no worldbody")
    for wheel in worldbody.iter("body"):
        if wheel.get("name", "").endswith("_wheel"):
            for geom in wheel.findall("geom"):
                if geom.get("type") == "cylinder":
                    # Ideal omni rollers permit lateral slip at wheel contacts.
                    geom.set("friction", "0.04 0.005 0.0001")
                    geom.set("priority", "1")

    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", name="blue_sky", type="skybox", builtin="gradient",
                  rgb1="0.20 0.36 0.54", rgb2="0.39 0.57 0.76",
                  width="512", height="512")
    ET.SubElement(asset, "texture", name="blue_grid", type="2d",
                  file=str(create_floor_texture().relative_to(OUTPUT)))
    ET.SubElement(asset, "material", name="blue_floor", texture="blue_grid",
                  texrepeat="8 8", texuniform="true", reflectance="0.08")
    ET.SubElement(worldbody, "geom", name="floor", type="plane", pos="0 0 0",
                  size="4 4 0.1", material="blue_floor", friction="1.2 0.005 0.0001")
    ET.SubElement(worldbody, "geom", name="drawing_wall", type="box",
                  pos="1.15 0 1.0", size="0.025 1.5 1.0",
                  rgba="0.70 0.75 0.82 1", friction="0.8 0.005 0.0001")
    # Light the robot and board without washing out the blue checker floor.
    ET.SubElement(worldbody, "light", name="neutral_fill", pos="0 0 3",
                  dir="0 0 -1", directional="true", ambient="0.7 0.7 0.7",
                  diffuse="0.55 0.55 0.55", castshadow="false")
    ET.SubElement(worldbody, "camera", name="overview", pos="-2.5 -2.5 1.9",
                  xyaxes="0.7 -0.7 0 0.25 0.25 0.93")

    wrist = root.find(".//body[@name='ur5e_wrist_3_link']")
    if wrist is None:
        raise ValueError("UR5e wrist body is missing")
    for joint in root.iter("joint"):
        if joint.get("name", "").startswith("ur5e_"):
            joint.set("damping", "15")
            joint.set("armature", "0.05")
    pen = ET.SubElement(wrist, "body", name="pen", pos="0 0 0")
    ET.SubElement(pen, "geom", name="pen_shaft", type="cylinder",
                  fromto="0 0 -0.07 0 0 -0.01", size="0.012",
                  rgba="0.1 0.1 0.13 1", contype="0", conaffinity="0")
    ET.SubElement(pen, "geom", name="pen_tip_geom", type="sphere",
                  pos="0 0 0", size="0.008", mass="0.015",
                  rgba="0.85 0.05 0.08 1", friction="0.7 0.005 0.0001")
    ET.SubElement(pen, "site", name="pen_tip", pos="0 0 0", size="0.004",
                  rgba="1 0 0 1")
    ET.SubElement(pen, "site", name="pen_contact_zone", type="sphere",
                  pos="0 0 0", size="0.012", rgba="0 0 0 0")

    actuator = ET.SubElement(root, "actuator")
    for side in ("left", "right"):
        for axle in ("front", "rear"):
            joint = f"{axle}_{side}_wheel_joint"
            ET.SubElement(actuator, "velocity", name=f"{joint}_velocity", joint=joint,
                          kv="200", ctrlrange="-12 12", ctrllimited="true")
    for joint in ("shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"):
        ET.SubElement(actuator, "velocity", name=f"ur5e_{joint}_velocity",
                      joint=f"ur5e_{joint}_joint", kv="80",
                      ctrlrange="-0.5 0.5", ctrllimited="true")

    sensor = ET.SubElement(root, "sensor")
    ET.SubElement(sensor, "touch", name="pen_normal_force", site="pen_contact_zone")

    colorize_scene(root)
    scene_path = OUTPUT / "scene.xml"
    ET.ElementTree(root).write(scene_path, encoding="unicode", xml_declaration=True)
    return scene_path


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    document = xacro.process_file(str(PACKAGE / "urdf" / "husky_ur5e.urdf.xacro"))
    ocs2_urdf = PACKAGE / "ocs2" / "husky_ur5e.urdf"
    ocs2_urdf.parent.mkdir(parents=True, exist_ok=True)
    ocs2_root = ET.fromstring(document.toxml())
    ET.indent(ocs2_root, space="  ")
    ET.ElementTree(ocs2_root).write(ocs2_urdf, encoding="unicode", xml_declaration=True)
    root = ET.fromstring(document.toxml())
    for child in list(root):
        if child.tag in {"gazebo", "ros2_control"}:
            root.remove(child)
    ET.SubElement(root, "link", name="world")
    base_joint = ET.SubElement(root, "joint", name="world_to_base", type="floating")
    ET.SubElement(base_joint, "parent", link="world")
    ET.SubElement(base_joint, "child", link="base_link")
    ET.SubElement(base_joint, "origin", xyz="0 0 0.13228", rpy="0 0 0")
    convert_meshes(root)

    extension = ET.SubElement(root, "mujoco")
    ET.SubElement(extension, "compiler", discardvisual="false", strippath="false")
    with tempfile.TemporaryDirectory(prefix="husky_mujoco_") as temporary:
        urdf_path = Path(temporary) / "husky_ur5e_import.urdf"
        ET.ElementTree(root).write(urdf_path, encoding="unicode", xml_declaration=True)
        model = mujoco.MjModel.from_xml_path(str(urdf_path))
        mjcf_path = Path(temporary) / "husky_ur5e_robot.xml"
        mujoco.mj_saveLastXML(str(mjcf_path), model)
        scene_path = assemble_scene(mjcf_path)
    scene = mujoco.MjModel.from_xml_path(str(scene_path))
    print(f"Saved {scene_path} (nq={scene.nq}, nv={scene.nv}, nu={scene.nu})")


if __name__ == "__main__":
    main()

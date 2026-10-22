"""Restore the material colors in the vendor COLLADA visual meshes."""

from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import trimesh


PACKAGE = Path(__file__).resolve().parents[1] / "src/Clearpath_mobile_manipulator"
HUSKY_MESHES = PACKAGE / "models/husky/meshes"
UR5E_MESHES = PACKAGE / "models/ur5e/meshes/ur5e/visual"
OUTPUT = PACKAGE / "mujoco"


def colorize_scene(root: ET.Element) -> None:
    assets = root.find("asset")
    base = root.find("./worldbody/body[@name='base_link']")
    if assets is None or base is None:
        raise ValueError("MuJoCo scene has no assets or robot base")

    source_files = {name: HUSKY_MESHES / f"{name}.dae" for name in (
        "base_link", "wheel", "top_chassis", "bumper", "top_plate", "user_rail")}
    source_files.update({name: UR5E_MESHES / f"{name}.dae" for name in (
        "base", "shoulder", "upperarm", "forearm", "wrist1", "wrist2", "wrist3")})
    visuals = {}
    for name, source in source_files.items():
        if not source.is_file():
            raise FileNotFoundError(source)
        scene = trimesh.load(source, force="scene")
        parts = []
        for index, node in enumerate(scene.graph.nodes_geometry):
            transform, geometry_name = scene.graph[node]
            mesh = scene.geometry[geometry_name].copy()
            mesh.apply_transform(transform)
            color = np.asarray(mesh.visual.material.baseColorFactor, dtype=float) / 255
            asset_name = f"brand_{name}_{index}"
            relative = Path("assets/colored") / f"{asset_name}.obj"
            destination = OUTPUT / relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            mesh.export(destination)
            ET.SubElement(assets, "mesh", name=asset_name, file=str(relative))
            parts.append((asset_name, " ".join(f"{value:.6g}" for value in color)))
        visuals[name] = parts

    for parent in base.iter():
        for geom in list(parent.findall("geom")):
            name = geom.get("mesh")
            if name in visuals and geom.get("contype") == "0":
                offset = list(parent).index(geom)
                for asset_name, rgba in visuals[name]:
                    replacement = ET.Element("geom", geom.attrib.copy())
                    replacement.set("mesh", asset_name)
                    replacement.set("rgba", rgba)
                    parent.insert(offset, replacement)
                    offset += 1
                parent.remove(geom)
            elif geom.get("contype") != "0" and geom.get("name") != "pen_tip_geom":
                # Collision geoms still participate in physics; group 3 is hidden by
                # the default MuJoCo viewer so they do not cover the colored visuals.
                geom.set("group", "3")

    used_meshes = {geom.get("mesh") for geom in root.iter("geom") if geom.get("mesh")}
    for mesh in list(assets.findall("mesh")):
        if mesh.get("name") not in used_meshes:
            assets.remove(mesh)


def main() -> None:
    scene_path = OUTPUT / "scene.xml"
    root = ET.parse(scene_path).getroot()
    colorize_scene(root)
    ET.ElementTree(root).write(scene_path, encoding="unicode", xml_declaration=True)
    print(f"Restored vendor colors in {scene_path}")


if __name__ == "__main__":
    main()

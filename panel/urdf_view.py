"""Read-only visual model from the same URDF configuration as reBot dynamics."""

from functools import lru_cache
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def vector(element, key, default):
    value = element.get(key) if element is not None else None
    result = [float(v) for v in value.split()] if value is not None else list(default)
    if len(result) != len(default) or not all(math.isfinite(v) for v in result):
        raise ValueError(f"Invalid URDF {key}")
    return result


def origin(element):
    node = element.find("origin")
    return {"xyz": vector(node, "xyz", (0, 0, 0)), "rpy": vector(node, "rpy", (0, 0, 0))}


@lru_cache(maxsize=1)
def load_view(root: Path):
    import yaml

    config_dir = root / "config"
    config = yaml.safe_load((config_dir / "rebotarm.yaml").read_text(encoding="utf-8"))
    hw = yaml.safe_load((config_dir / config["hardware_yaml"]).read_text(encoding="utf-8"))
    urdf_dir = (root / "urdf").resolve()
    model = (root / hw["urdf_path"]).resolve(strict=True)
    if model.suffix.lower() != ".urdf" or not model.is_relative_to(urdf_dir):
        raise ValueError("Only project URDF models may be displayed")
    robot = ET.parse(model).getroot()
    colors = {node.get("name"): vector(node.find("color"), "rgba", (.45, .5, .48, 1))
              for node in robot.findall("material")}
    assets = []
    links = []
    for link in robot.findall("link"):
        visuals = []
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                raise ValueError("This viewer expects STL visual meshes")
            asset = (model.parent / mesh.attrib["filename"]).resolve(strict=True)
            if (not asset.is_relative_to(urdf_dir) or asset.suffix.lower() != ".stl"
                    or not asset.is_file() or asset.stat().st_size > 16 * 1024 * 1024):
                raise ValueError("Invalid URDF visual mesh")
            if asset not in assets:
                assets.append(asset)
            material = visual.find("material")
            color = colors.get(material.get("name"), [.45, .5, .48, 1]) if material is not None else [.45, .5, .48, 1]
            rgba = vector(material.find("color"), "rgba", color) if material is not None else color
            visuals.append({**origin(visual), "mesh": assets.index(asset),
                            "scale": vector(mesh, "scale", (1, 1, 1)), "rgb": rgba[:3]})
        links.append({"name": link.attrib["name"], "visuals": visuals})

    joints = [{**origin(j), "name": j.attrib["name"], "type": j.attrib["type"],
               "parent": j.find("parent").attrib["link"], "child": j.find("child").attrib["link"],
               "axis": vector(j.find("axis"), "xyz", (1, 0, 0))} for j in robot.findall("joint")]
    joint_types = {j["name"]: j["type"] for j in joints}
    arm = hw["groups"]["arm"]["joints"]
    direction = hw.get("gravity_compensation", {}).get("joint_direction", 1.0)
    signs = list(direction) if isinstance(direction, (list, tuple)) else [direction]
    if len(signs) == 1:
        signs *= len(arm)
    feedback = {joint["name"]: i + 1 for i, joint in enumerate(hw["joints"])}
    if (len(arm) != 6 or len(set(arm)) != 6 or len(signs) != 6 or any(s not in (-1, 1) for s in signs)
            or any(joint_types.get(name) not in ("revolute", "continuous") or name not in feedback for name in arm)):
        raise ValueError("URDF and configured arm joints do not match")
    return ({"name": model.name, "links": links, "joints": joints, "meshes": len(assets),
             "feedback": [{"id": feedback[name], "joint": name, "sign": sign}
                          for name, sign in zip(arm, signs)]}, tuple(assets))

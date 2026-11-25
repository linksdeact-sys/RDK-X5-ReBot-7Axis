"""Render the official DM URDF at neutral joint coordinates, offline."""

from pathlib import Path
import xml.etree.ElementTree as ET

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
from matplotlib.collections import PolyCollection

root = Path(__file__).resolve().parent
urdf = root / "urdf/DM/urdf/ReBot_Arm_DM.urdf"
model = pin.buildModelFromUrdf(str(urdf))
data = model.createData()
pin.forwardKinematics(model, data, pin.neutral(model))
pin.updateFramePlacements(model, data)

meshes = []
record = np.dtype([("normal", "<f4", 3), ("points", "<f4", (3, 3)), ("attr", "<u2")])
for link in ET.parse(urdf).getroot().findall("link"):
    frame_pose = data.oMf[model.getFrameId(link.attrib["name"])]
    for visual in link.findall("visual"):
        mesh = visual.find("geometry/mesh")
        if mesh is None:
            continue
        origin = visual.find("origin")
        xyz = np.fromstring(origin.get("xyz", "0 0 0"), sep=" ")
        rpy = np.fromstring(origin.get("rpy", "0 0 0"), sep=" ")
        pose = frame_pose * pin.SE3(pin.rpy.rpyToMatrix(rpy), xyz)
        mesh_path = urdf.parent / mesh.attrib["filename"]
        blob = mesh_path.read_bytes()
        count = int.from_bytes(blob[80:84], "little")
        if len(blob) != 84 + 50 * count:
            raise ValueError(f"Expected binary STL: {mesh_path}")
        triangles = np.frombuffer(blob, dtype=record, count=count, offset=84)["points"]
        scale = np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")
        triangles = (triangles * scale) @ pose.rotation.T + pose.translation
        color_node = visual.find("material/color")
        color = np.fromstring(color_node.get("rgba"), sep=" ")[:3] if color_node is not None else [0.3, 0.3, 0.3]
        meshes.append((triangles, color))

fig = plt.figure(figsize=(13, 6), facecolor="white")
all_triangles = np.concatenate([triangles for triangles, _ in meshes])
all_colors = np.concatenate([
    np.tile(color, (len(triangles), 1)) for triangles, color in meshes
])
centers = all_triangles.mean(axis=1)
for index, (horizontal, depth, direction, title) in enumerate(
    [(0, 1, -1, "Official DM zero pose - side view"),
     (1, 0, 1, "Official DM zero pose - front view")], start=1
):
    ax = fig.add_subplot(1, 2, index)
    order = np.argsort(centers[:, depth] * direction)
    projected = all_triangles[order][:, :, [horizontal, 2]]
    ax.add_collection(PolyCollection(
        projected, facecolors=all_colors[order], edgecolors="none", antialiased=False
    ))
    if index == 1:
        for joint_id in range(1, 7):
            xyz = data.oMi[joint_id].translation
            ax.annotate(f"J{joint_id}", (xyz[0], xyz[2]),
                        xytext=(-8, -20 if joint_id % 2 else 20),
                        textcoords="offset points", color="#0068ae", fontsize=10)
    ax.set_xlim((-0.32, 0.37) if index == 1 else (-0.15, 0.15))
    ax.set_ylim(-0.01, 0.41)
    ax.set_aspect("equal")
    ax.set_title(title)
    ax.set_axis_off()
fig.subplots_adjust(left=0, right=1, bottom=0, top=0.93, wspace=0)
fig.savefig(root / "official_zero_pose.png", dpi=160)
print(root / "official_zero_pose.png")

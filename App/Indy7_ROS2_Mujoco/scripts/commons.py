import os
import json
from pythreejs import BufferGeometry, BufferAttribute, Mesh, MeshStandardMaterial

def load_config(path="config.json"):
    with open(path, "r") as f:
        return json.load(f)


def ensure_config(path="config.json"):
    if not os.path.exists(path):
        default_config = {
            "UI": {
                "JointSpace": {
                    f"Joint{i+1}": {
                        "name": f"Joint {i+1}",
                        "min": -180,
                        "max": 180
                    } for i in range(6)
                }
            }
        }
        with open(path, "w") as f:
            json.dump(default_config, f, indent=4)


def trimesh_to_pythreejs(tri):
    geometry = BufferGeometry(
        attributes={
            'position': BufferAttribute(tri.vertices.astype('f4'), normalized=False),
            'index': BufferAttribute(tri.faces.astype('u4').flatten(), normalized=False),
        }
    )
    material = MeshStandardMaterial(color='lightgray', metalness=0.5, roughness=0.5)
    return Mesh(geometry=geometry, material=material)
from __future__ import annotations

from typing import Any, Dict

from isaacsim.core.prims import SingleGeometryPrim
from isaacsim.core.utils.prims import get_prim_at_path, is_prim_path_valid
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.string import find_unique_string_name
from pxr import UsdGeom

from env.scene_manager.physics_spec import StaticSpec, author_static_physics


class Room(SingleGeometryPrim):
    """Geometry wrapper for the room USD asset."""

    def __init__(
        self,
        prim_path: str | None = None,
        usd_path: str | None = None,
        instance_config: Dict[str, Any] | None = None,
        physics_spec: StaticSpec | None = None,
    ):
        """
        Initialize Room object wrapping the room geometry prim.

        Args:
            prim_path: Prim path in Isaac Sim (optional)
            usd_path: USD asset file path (optional, if None will wrap existing prim)
            instance_config: Instance configuration dictionary (optional)
        """
        self.physics_spec = physics_spec
        if prim_path is not None:
            # Load USD asset if usd_path is provided
            if usd_path is not None:
                room_prim = add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)
                if not room_prim or not room_prim.IsValid():
                    raise RuntimeError(f"Failed to load USD from {usd_path} to {prim_path}")
            else:
                room_prim = get_prim_at_path(prim_path)

            if physics_spec is not None and (physics_spec.bind_material or physics_spec.author_collision):
                mat_path = find_unique_string_name(
                    f"{prim_path}/physics_material",
                    lambda x: not is_prim_path_valid(x),
                )
                author_static_physics(room_prim, mat_path, physics_spec)

            collision = instance_config.get("collision", True) if instance_config else True
            if physics_spec is not None and physics_spec.author_collision:
                if room_prim and room_prim.IsValid() and not UsdGeom.Gprim(room_prim):
                    collision = False
                elif physics_spec.collision.enabled is not None:
                    collision = physics_spec.collision.enabled
            SingleGeometryPrim.__init__(
                self,
                prim_path=prim_path,
                name=prim_path.split("/")[-1],
                collision=collision,
            )

            if instance_config:
                path_parts = prim_path.split("/")
                self.instance_name = path_parts[-1]
                self.category_name = self.instance_name.rsplit("_", 1)[0]
                pos = instance_config.get("default_pos", [0.0, 0.0, 0])
                rot = instance_config.get("default_ori", [1.0, 0.0, 0.0, 0.0])
                self.set_local_pose(translation=pos, orientation=rot)
                scale = instance_config.get("scale", [1.0, 1.0, 1.0])
                self.set_local_scale(scale)

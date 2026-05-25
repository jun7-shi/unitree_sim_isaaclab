# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Export a Nav2 static occupancy map from the live Isaac Sim stage."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, replace
import math
from pathlib import Path
from typing import Iterable


DEFAULT_NAV_STATIC_MAP_BOUND_PRIM = "/World/envs/env_0"
DEFAULT_NAV_STATIC_MAP_EXCLUDE_PRIMS = (
    "/World/envs/env_0/Robot",
    "/World/envs/env_0/Object",
)
KITCHEN_NAV_STATIC_MAP_BOUND_PRIM = "/World/envs/env_0/Kitchen"
KITCHEN_NAV_STATIC_MAP_COLLISION_EXCLUDE_PRIMS = ("/World/envs/env_0/Robot",)
KITCHEN_NAV_STATIC_MAP_PATCH_PRIMS = (
    "/World/envs/env_0/Kitchen/Kitchen_InsularShelf_01",
    "/World/envs/env_0/Kitchen/Kitchen_Cabinet001_01",
)


@dataclass(frozen=True)
class G1NavStaticMapExportConfig:
    """Configuration for generating a ROS occupancy map from Isaac Sim."""

    output_yaml: str
    origin: tuple[float, float, float]
    cell_size: float = 0.05
    z_bounds: tuple[float, float] = (0.05, 1.2)
    bound_prim_path: str = DEFAULT_NAV_STATIC_MAP_BOUND_PRIM
    padding: float = 0.25
    exclude_prim_paths: tuple[str, ...] = DEFAULT_NAV_STATIC_MAP_EXCLUDE_PRIMS
    collision_exclude_prim_paths: tuple[str, ...] = ()
    patch_prim_paths: tuple[str, ...] = ()
    apply_collision_to_meshes: bool = True
    occupied_value: int = 100
    free_value: int = 0
    unknown_value: int = 50
    occupied_thresh: float = 0.65
    free_thresh: float = 0.196


def resolve_nav_static_map_scope(
    task_name: str,
    bound_prim_path: str,
    exclude_prim_paths: Iterable[str],
) -> tuple[str, tuple[str, ...]]:
    """Resolve task-specific defaults for static map generation."""

    exclude_prims = tuple(exclude_prim_paths)
    if (
        "Kitchen" in task_name
        and bound_prim_path == DEFAULT_NAV_STATIC_MAP_BOUND_PRIM
        and exclude_prims == DEFAULT_NAV_STATIC_MAP_EXCLUDE_PRIMS
    ):
        return KITCHEN_NAV_STATIC_MAP_BOUND_PRIM, ("/World/envs/env_0/Object",)

    return bound_prim_path, exclude_prims


def resolve_nav_static_map_collision_exclude_prims(
    task_name: str,
    bound_prim_path: str,
    exclude_prim_paths: Iterable[str],
) -> tuple[str, ...]:
    """Resolve prims whose collision should be hidden, without deactivating them."""

    if (
        "Kitchen" in task_name
        and bound_prim_path == DEFAULT_NAV_STATIC_MAP_BOUND_PRIM
        and tuple(exclude_prim_paths) == DEFAULT_NAV_STATIC_MAP_EXCLUDE_PRIMS
    ):
        return KITCHEN_NAV_STATIC_MAP_COLLISION_EXCLUDE_PRIMS

    return ()


def resolve_nav_static_map_patch_prims(
    task_name: str,
    bound_prim_path: str,
    exclude_prim_paths: Iterable[str],
    patch_prim_paths: Iterable[str] | None,
) -> tuple[str, ...]:
    """Resolve additional prims to export separately and OR into the static map."""

    if patch_prim_paths is not None:
        return tuple(patch_prim_paths)

    if (
        "Kitchen" in task_name
        and bound_prim_path == DEFAULT_NAV_STATIC_MAP_BOUND_PRIM
        and tuple(exclude_prim_paths) == DEFAULT_NAV_STATIC_MAP_EXCLUDE_PRIMS
    ):
        return KITCHEN_NAV_STATIC_MAP_PATCH_PRIMS

    return ()


def export_g1_nav_static_map(config: G1NavStaticMapExportConfig) -> dict:
    """Generate a Nav2 map YAML and PGM image from the current USD stage."""

    import omni.physx
    import omni.usd

    _enable_omap_extension()
    from isaacsim.asset.gen.omap.bindings import _omap

    stage = omni.usd.get_context().get_stage()
    excluded_prims = _set_prims_active(stage, config.exclude_prim_paths, active=False)
    applied_collision_count = 0
    visual_collision_layer = None
    disabled_collision_layer = None
    disabled_collision_count = 0

    try:
        min_bound, max_bound = _compute_xy_bounds(stage, config)
        disabled_collision_layer, disabled_collision_count = (
            _disable_collisions_for_mapping(
                stage,
                config.collision_exclude_prim_paths,
            )
        )
        if config.apply_collision_to_meshes:
            visual_collision_layer, applied_collision_count = (
                _apply_visual_mesh_colliders_for_mapping(
                    stage,
                    config.bound_prim_path,
                )
            )
        _prime_omap_stage_updates()
        generator = _export_with_generator(stage, config, min_bound, max_bound)

        width, height, _depth = tuple(generator.get_dimensions())
        buffer = list(generator.get_buffer()[: width * height])
        patch_maps, patched_occupied_cell_count = _export_and_merge_patch_maps(
            stage,
            config,
            buffer,
            width=width,
            height=height,
            min_bound=min_bound,
        )
        buffer_histogram = _buffer_histogram(buffer)
        output_yaml, output_image = _resolve_output_paths(config.output_yaml)
        _write_pgm(
            output_image,
            buffer,
            width=width,
            height=height,
            occupied_value=config.occupied_value,
            free_value=config.free_value,
            unknown_value=config.unknown_value,
        )
        _write_yaml(output_yaml, output_image.name, config, min_bound)
        return {
            "yaml": str(output_yaml),
            "image": str(output_image),
            "width": int(width),
            "height": int(height),
            "resolution": config.cell_size,
            "origin": [float(min_bound[0]), float(min_bound[1]), 0.0],
            "excluded_prims": sorted(excluded_prims),
            "disabled_collision_prim_count": disabled_collision_count,
            "applied_collision_to_mesh_count": applied_collision_count,
            "patch_maps": patch_maps,
            "patched_occupied_cell_count": patched_occupied_cell_count,
            "buffer_histogram": buffer_histogram,
        }
    finally:
        if visual_collision_layer is not None:
            _remove_session_layer(stage, visual_collision_layer)
        if disabled_collision_layer is not None:
            _remove_session_layer(stage, disabled_collision_layer)
        _restore_prims_active(stage, excluded_prims)


def _export_with_generator(stage, config, min_bound, max_bound):
    import omni.physx
    import omni.usd

    from isaacsim.asset.gen.omap.bindings import _omap

    physx = omni.physx.get_physx_interface()
    stage_id = omni.usd.get_context().get_stage_id()
    generator = _omap.Generator(physx, stage_id)
    generator.update_settings(
        config.cell_size,
        config.occupied_value,
        config.free_value,
        config.unknown_value,
    )
    generator.set_transform(
        config.origin,
        _relative_bound(min_bound, config.origin),
        _relative_bound(max_bound, config.origin),
    )
    was_playing = _prime_omap_physics_scene()
    try:
        generator.generate2d()
    finally:
        _restore_timeline_state(was_playing)

    return generator


def _export_and_merge_patch_maps(
    stage,
    config: G1NavStaticMapExportConfig,
    base_buffer,
    *,
    width: int,
    height: int,
    min_bound,
) -> tuple[list[dict], int]:
    patch_maps = []
    total_added = 0

    for patch_prim_path in config.patch_prim_paths:
        patch_config = replace(config, bound_prim_path=patch_prim_path)
        patch_min_bound, patch_max_bound = _compute_xy_bounds(stage, patch_config)
        patch_origin = _patch_origin_from_bounds(patch_min_bound, patch_max_bound, config)
        patch_config = replace(patch_config, origin=patch_origin, patch_prim_paths=())
        patch_generator = _export_with_generator(
            stage,
            patch_config,
            patch_min_bound,
            patch_max_bound,
        )
        patch_width, patch_height, _patch_depth = tuple(patch_generator.get_dimensions())
        patch_buffer = list(patch_generator.get_buffer()[: patch_width * patch_height])
        _merged, added = _merge_occupied_patch_buffer(
            base_buffer,
            width=width,
            height=height,
            min_bound=min_bound,
            patch_buffer=patch_buffer,
            patch_width=patch_width,
            patch_height=patch_height,
            patch_min_bound=patch_min_bound,
            cell_size=config.cell_size,
            occupied_value=config.occupied_value,
        )
        total_added += added
        patch_maps.append(
            {
                "prim": patch_prim_path,
                "width": int(patch_width),
                "height": int(patch_height),
                "origin": [float(patch_min_bound[0]), float(patch_min_bound[1]), 0.0],
                "generation_origin": [
                    float(patch_origin[0]),
                    float(patch_origin[1]),
                    float(patch_origin[2]),
                ],
                "added_occupied_cells": int(added),
                "buffer_histogram": _buffer_histogram(patch_buffer),
            }
        )

    return patch_maps, total_added


def _patch_origin_from_bounds(min_bound, max_bound, config: G1NavStaticMapExportConfig):
    margin = max(config.cell_size * 2.0, min(config.padding * 0.4, 0.15))
    origin_y = float(min_bound[1]) + margin
    upper_y = float(max_bound[1]) - config.cell_size
    if origin_y > upper_y:
        origin_y = float(min_bound[1]) + (float(max_bound[1]) - float(min_bound[1])) * 0.5

    return (
        (float(min_bound[0]) + float(max_bound[0])) * 0.5,
        origin_y,
        float(config.origin[2]),
    )


def _merge_occupied_patch_buffer(
    base_buffer,
    *,
    width: int,
    height: int,
    min_bound,
    patch_buffer,
    patch_width: int,
    patch_height: int,
    patch_min_bound,
    cell_size: float,
    occupied_value: int,
):
    added = 0
    for patch_y in range(patch_height):
        for patch_x in range(patch_width):
            patch_index = patch_y * patch_width + patch_x
            if int(patch_buffer[patch_index]) != occupied_value:
                continue

            world_x = float(patch_min_bound[0]) + (patch_x + 0.5) * cell_size
            world_y = float(patch_min_bound[1]) + (patch_y + 0.5) * cell_size
            base_x = math.floor((world_x - float(min_bound[0])) / cell_size)
            base_y = math.floor((world_y - float(min_bound[1])) / cell_size)
            if not (0 <= base_x < width and 0 <= base_y < height):
                continue

            base_index = base_y * width + base_x
            if int(base_buffer[base_index]) != occupied_value:
                base_buffer[base_index] = occupied_value
                added += 1

    return base_buffer, added


def _relative_bound(bound, origin):
    return (
        float(bound[0]) - float(origin[0]),
        float(bound[1]) - float(origin[1]),
        float(bound[2]) - float(origin[2]),
    )


def _enable_omap_extension() -> None:
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("isaacsim.asset.gen.omap", True)


def _prime_omap_physics_scene() -> bool:
    import omni.kit.app
    import omni.timeline

    timeline = omni.timeline.get_timeline_interface()
    was_playing = timeline.is_playing()
    if not was_playing:
        timeline.play()

    app = omni.kit.app.get_app()
    app.update()
    return was_playing


def _prime_omap_stage_updates() -> None:
    import omni.kit.app

    app = omni.kit.app.get_app()
    app.update()
    app.update()


def _restore_timeline_state(was_playing: bool) -> None:
    if was_playing:
        return

    import omni.timeline

    omni.timeline.get_timeline_interface().stop()


def _set_prims_active(stage, prim_paths: Iterable[str], active: bool) -> dict[str, bool]:
    changed = {}
    for prim_path in prim_paths:
        prim = stage.GetPrimAtPath(prim_path)
        if prim and prim.IsValid():
            changed[prim_path] = prim.IsActive()
            prim.SetActive(active)
    return changed


def _restore_prims_active(stage, prim_active_states: dict[str, bool]) -> None:
    for prim_path, was_active in prim_active_states.items():
        prim = stage.GetPrimAtPath(prim_path)
        if prim and prim.IsValid():
            prim.SetActive(was_active)


def _disable_collisions_for_mapping(
    stage,
    prim_paths: Iterable[str],
) -> tuple[str, int]:
    from pxr import Sdf, Usd, UsdPhysics

    prim_paths = tuple(prim_paths)
    if not prim_paths:
        return "", 0

    layer = Sdf.Layer.CreateAnonymous("anon_humanoid_nav_collision_disable")
    session = stage.GetSessionLayer()
    session.subLayerPaths.append(layer.identifier)
    disabled_count = 0

    with Usd.EditContext(stage, layer):
        for prim_path in prim_paths:
            root = stage.GetPrimAtPath(prim_path)
            if not root or not root.IsValid():
                continue
            for prim in Usd.PrimRange(root):
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    collision_api = UsdPhysics.CollisionAPI(prim)
                    collision_api.CreateCollisionEnabledAttr(False).Set(False)
                    disabled_count += 1

    if disabled_count:
        _prime_omap_stage_updates()
    return layer.identifier, disabled_count


def _apply_visual_mesh_colliders_for_mapping(
    stage,
    bound_prim_path: str,
) -> tuple[str, int]:
    from omni.physx.scripts import utils
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics

    root = stage.GetPrimAtPath(bound_prim_path)
    if not root or not root.IsValid():
        return "", 0

    layer = Sdf.Layer.CreateAnonymous("anon_humanoid_nav_omap")
    session = stage.GetSessionLayer()
    session.subLayerPaths.append(layer.identifier)
    applied_count = 0

    with Usd.EditContext(stage, layer):
        for prim in Usd.PrimRange(root):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                utils.removePhysics(prim)

        _prime_omap_stage_updates()

        for prim in Usd.PrimRange(root):
            if _should_skip_visual_collider_prim(prim):
                continue

            if prim.HasAPI(UsdPhysics.CollisionAPI):
                if _has_triangle_mesh_collider(prim):
                    continue
                if prim.IsA(UsdGeom.Gprim):
                    if prim.IsInstanceable():
                        UsdPhysics.CollisionAPI.Apply(prim)
                        UsdPhysics.MeshCollisionAPI.Apply(prim)
                        applied_count += 1
                    else:
                        try:
                            utils.setCollider(prim, "none")
                            applied_count += 1
                        except Exception:
                            continue
            elif prim.IsA(UsdGeom.Xformable) and prim.IsInstanceable():
                UsdPhysics.CollisionAPI.Apply(prim)
                UsdPhysics.MeshCollisionAPI.Apply(prim)
                applied_count += 1
            elif prim.IsA(UsdGeom.Gprim):
                UsdPhysics.CollisionAPI.Apply(prim)
                UsdPhysics.MeshCollisionAPI.Apply(prim)
                applied_count += 1

    return layer.identifier, applied_count


def _should_skip_visual_collider_prim(prim) -> bool:
    from pxr import Usd, UsdGeom

    imageable = UsdGeom.Imageable(prim)
    if imageable:
        visibility = imageable.ComputeVisibility(Usd.TimeCode.Default())
        if visibility == UsdGeom.Tokens.invisible:
            return True

    if prim.IsA(UsdGeom.Mesh):
        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get()
        return points is None or len(points) == 0

    return False


def _has_triangle_mesh_collider(prim) -> bool:
    from pxr import UsdPhysics

    if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        return False
    collision_api = UsdPhysics.MeshCollisionAPI(prim)
    return collision_api.GetApproximationAttr().Get() == "none"


def _remove_session_layer(stage, layer_identifier: str) -> None:
    session = stage.GetSessionLayer()
    while layer_identifier and layer_identifier in session.subLayerPaths:
        session.subLayerPaths.remove(layer_identifier)


def _compute_xy_bounds(stage, config: G1NavStaticMapExportConfig):
    from pxr import Usd, UsdGeom

    prim = stage.GetPrimAtPath(config.bound_prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"static map bound prim not found: {config.bound_prim_path}")

    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
    bbox_range = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
    min_point = bbox_range.GetMin()
    max_point = bbox_range.GetMax()
    min_z, max_z = config.z_bounds
    min_bound = (
        float(min_point[0]) - config.padding,
        float(min_point[1]) - config.padding,
        float(min_z),
    )
    max_bound = (
        float(max_point[0]) + config.padding,
        float(max_point[1]) + config.padding,
        float(max_z),
    )
    return min_bound, max_bound


def _buffer_histogram(buffer) -> dict[int, int]:
    return dict(sorted(Counter(int(value) for value in buffer).items()))


def _resolve_output_paths(output_yaml: str) -> tuple[Path, Path]:
    yaml_path = Path(output_yaml).expanduser().resolve()
    image_path = yaml_path.with_suffix(".pgm")
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    return yaml_path, image_path


def _write_pgm(
    output_image: Path,
    buffer,
    *,
    width: int,
    height: int,
    occupied_value: int,
    free_value: int,
    unknown_value: int,
) -> None:
    expected = width * height
    if len(buffer) < expected:
        raise RuntimeError(
            f"occupancy buffer too small: expected {expected}, got {len(buffer)}"
        )

    def normalize(value) -> int:
        int_value = int(value)
        if int_value == occupied_value:
            return 0
        if int_value == free_value:
            return 254
        if int_value == unknown_value:
            return 205
        return max(0, min(255, int_value))

    with output_image.open("wb") as stream:
        stream.write(f"P5\n{width} {height}\n255\n".encode("ascii"))
        # ROS map images are top-down; write max-Y rows first.
        for y in range(height - 1, -1, -1):
            row = bytearray(normalize(buffer[y * width + x]) for x in range(width))
            stream.write(row)


def _write_yaml(
    output_yaml: Path,
    image_name: str,
    config: G1NavStaticMapExportConfig,
    min_bound,
) -> None:
    origin = [float(min_bound[0]), float(min_bound[1]), 0.0]
    yaml_text = (
        f"image: {image_name}\n"
        "mode: trinary\n"
        f"resolution: {config.cell_size}\n"
        f"origin: [{origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f}]\n"
        "negate: 0\n"
        f"occupied_thresh: {config.occupied_thresh}\n"
        f"free_thresh: {config.free_thresh}\n"
    )
    output_yaml.write_text(yaml_text, encoding="utf-8")

# Copyright (c) 2025, Unitree Robotics Co., Ltd. All Rights Reserved.
# License: Apache License, Version 2.0
"""Export a Nav2 static occupancy map from the live Isaac Sim stage."""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class G1NavStaticMapExportConfig:
    """Configuration for generating a ROS occupancy map from Isaac Sim."""

    output_yaml: str
    origin: tuple[float, float, float]
    cell_size: float = 0.05
    z_bounds: tuple[float, float] = (0.05, 1.2)
    bound_prim_path: str = "/World/envs/env_0"
    padding: float = 0.25
    exclude_prim_paths: tuple[str, ...] = (
        "/World/envs/env_0/Robot",
        "/World/envs/env_0/Object",
    )
    apply_collision_to_meshes: bool = True
    occupied_value: int = 100
    free_value: int = 0
    unknown_value: int = 50
    occupied_thresh: float = 0.65
    free_thresh: float = 0.196


def export_g1_nav_static_map(config: G1NavStaticMapExportConfig) -> dict:
    """Generate a Nav2 map YAML and PGM image from the current USD stage."""

    import omni.physx
    import omni.usd

    _enable_omap_extension()
    from isaacsim.asset.gen.omap.bindings import _omap

    stage = omni.usd.get_context().get_stage()
    excluded_prims = _set_prims_active(stage, config.exclude_prim_paths, active=False)
    applied_collision_count = 0

    try:
        min_bound, max_bound = _compute_xy_bounds(stage, config)
        if config.apply_collision_to_meshes:
            applied_collision_count = _apply_collision_to_meshes(
                stage, config.bound_prim_path
            )
        physx = omni.physx.get_physx_interface()
        stage_id = omni.usd.get_context().get_stage_id()
        generator = _omap.Generator(physx, stage_id)
        generator.update_settings(
            config.cell_size,
            config.occupied_value,
            config.free_value,
            config.unknown_value,
        )
        generator.set_transform(config.origin, min_bound, max_bound)
        generator.generate2d()

        width, height, _depth = tuple(generator.get_dimensions())
        buffer = generator.get_buffer()
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
            "applied_collision_to_mesh_count": applied_collision_count,
            "buffer_histogram": buffer_histogram,
        }
    finally:
        _restore_prims_active(stage, excluded_prims)


def _enable_omap_extension() -> None:
    import omni.kit.app

    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("isaacsim.asset.gen.omap", True)


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


def _apply_collision_to_meshes(stage, bound_prim_path: str) -> int:
    from pxr import Usd, UsdGeom, UsdPhysics

    root = stage.GetPrimAtPath(bound_prim_path)
    if not root or not root.IsValid():
        return 0

    applied_count = 0
    for prim in Usd.PrimRange(root):
        if prim.IsA(UsdGeom.Mesh) and not prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(prim)
            applied_count += 1
    return applied_count


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

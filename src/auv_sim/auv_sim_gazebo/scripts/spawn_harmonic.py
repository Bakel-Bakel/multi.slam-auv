#!/usr/bin/env python3
"""Spawn the BlueROV2 into Gazebo Harmonic using native gz-transport13.

Humble's apt ``ros_gz_sim/create`` is linked against ``ignition-transport11``
(Fortress). Harmonic 8 uses ``gz-transport13``. When those are mixed, ``create``
loops on "Requesting list of world names" / "Unknown message type [8]" and the
ROV never appears even though the world loads.

This node waits for ``/world/<world>/create``, reads ``robot_description`` from
``robot_state_publisher``, converts URDF->SDF with ``gz sdf -p``, and spawns
via ``gz service``.
"""
from __future__ import annotations

import subprocess
import sys
import tempfile
import time

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node


def _run(cmd: list[str], *, check: bool = True) -> subprocess.CompletedProcess:
    result = subprocess.run(cmd, capture_output=True, text=True)
    if check and result.returncode != 0:
        raise RuntimeError(
            f"command failed ({result.returncode}): {' '.join(cmd)}\n"
            f"stdout: {result.stdout}\nstderr: {result.stderr}")
    return result


def _wait_for_create_service(world: str, logger, timeout: float) -> bool:
    service = f"/world/{world}/create"
    deadline = time.time() + timeout
    while time.time() < deadline and rclpy.ok():
        result = _run(["gz", "service", "-l"], check=False)
        if result.returncode == 0 and service in result.stdout:
            logger.info(f"Gazebo create service ready: {service}")
            return True
        time.sleep(1.0)
    return False


def _fetch_robot_description(node: Node, param_node: str, timeout: float) -> str:
    client = node.create_client(GetParameters, f"/{param_node}/get_parameters")
    if not client.wait_for_service(timeout_sec=timeout):
        raise RuntimeError(
            f"timed out waiting for parameters on '{param_node}'")
    request = GetParameters.Request()
    request.names = ["robot_description"]
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        raise RuntimeError("get_parameters call failed")
    response = future.result()
    if not response.values:
        raise RuntimeError("robot_description parameter is not set")
    value = response.values[0]
    if not value.string_value:
        raise RuntimeError("robot_description is empty")
    return value.string_value


def _urdf_to_sdf(urdf_xml: str) -> str:
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as handle:
        handle.write(urdf_xml)
        urdf_path = handle.name
    result = _run(["gz", "sdf", "-p", urdf_path])
    return result.stdout


def _spawn_entity(world: str, name: str, sdf_xml: str, z: float) -> None:
    with tempfile.NamedTemporaryFile("w", suffix=".sdf", delete=False) as handle:
        handle.write(sdf_xml)
        sdf_path = handle.name
    request = (
        f'sdf_filename: "{sdf_path}", '
        f'name: "{name}", '
        f"allow_renaming: true, "
        f"pose: {{position: {{z: {z}}}}}"
    )
    result = _run([
        "gz", "service",
        "-s", f"/world/{world}/create",
        "--reqtype", "gz.msgs.EntityFactory",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "10000",
        "--req", request,
    ])
    if "data: true" not in result.stdout.lower():
        raise RuntimeError(
            f"spawn service rejected request:\n{result.stdout}\n{result.stderr}")


class HarmonicSpawner(Node):
    def __init__(self) -> None:
        super().__init__("harmonic_spawner")
        self.declare_parameter("world", "ocean")
        self.declare_parameter("entity_name", "bluerov2")
        self.declare_parameter("spawn_z", -1.0)
        self.declare_parameter("param_node", "robot_state_publisher")
        self.declare_parameter("wait_timeout", 120.0)

        world = self.get_parameter("world").value
        name = self.get_parameter("entity_name").value
        z = float(self.get_parameter("spawn_z").value)
        param_node = self.get_parameter("param_node").value
        timeout = float(self.get_parameter("wait_timeout").value)

        self.get_logger().info(
            f"waiting up to {timeout:.0f}s for Harmonic create service "
            f"(world={world})")
        if not _wait_for_create_service(world, self.get_logger(), timeout):
            raise RuntimeError(
                f"Gazebo create service /world/{world}/create never became "
                "available. Is gz sim running?")

        self.get_logger().info(f"reading robot_description from {param_node}")
        urdf = _fetch_robot_description(self, param_node, timeout=30.0)

        self.get_logger().info("converting URDF -> SDF (gz sdf -p)")
        sdf = _urdf_to_sdf(urdf)

        self.get_logger().info(f"spawning '{name}' at z={z}")
        _spawn_entity(world, name, sdf, z)
        self.get_logger().info(f"spawned '{name}' successfully")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HarmonicSpawner()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except RuntimeError as exc:
        print(f"[harmonic_spawner] ERROR: {exc}", file=sys.stderr)
        sys.exit(1)

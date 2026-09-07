"""Route the Gazebo differential-drive plugin through the final gate."""

import xml.etree.ElementTree as ET
from pathlib import Path


def write_guarded_model(source: str, destination: str) -> str:
    tree = ET.parse(source)
    plugins = [p for p in tree.iter("plugin") if p.get("filename") == "libgazebo_ros_diff_drive.so"]
    if len(plugins) != 1:
        raise ValueError("expected exactly one Gazebo ROS differential-drive plugin")
    plugin = plugins[0]
    command = plugin.find("command_topic")
    if command is None or command.text not in {"cmd_vel", "/cmd_vel"}:
        raise ValueError("unexpected drive command topic; inspect the installed Nav2 model")
    for remap in plugin.findall("ros/remapping"):
        if "cmd_vel" in (remap.text or ""):
            raise ValueError("unexpected velocity remapping in source model")
    command.text = "/cmd_vel_safe"
    Path(destination).parent.mkdir(parents=True, exist_ok=True)
    # Humble spawn_entity passes Unicode to lxml, which rejects encoding declarations.
    tree.write(destination, encoding="utf-8", xml_declaration=False)
    return destination

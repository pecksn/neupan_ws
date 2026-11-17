"""Check the operator-selected band without starting a ROS or vehicle node."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]


class ClearanceConfigurationTests(unittest.TestCase):
    def test_launch_and_operator_command_agree(self):
        entries = [
            ("src/neupan_ros/example/lxw_car/launch/full_system_run.launch", "camera_right_"),
            ("src/osm_map_handler/launch/camera_right_path.launch", "camera_"),
        ]
        for path, prefix in entries:
            values = {a.get("name"): a.get("default") for a in ET.parse(ROOT / path).getroot().findall("arg")}
            preferred = prefix + ("camera_clearance_m" if prefix == "camera_right_" else "clearance_m")
            self.assertEqual(float(values[preferred]), 1.25)
            self.assertEqual(float(values[prefix + "clearance_min_m"]), 0.5)
            self.assertEqual(float(values[prefix + "clearance_max_m"]), 2.0)
        commands = (ROOT / "核心启动指令.md").read_text()
        tree = ast.parse((ROOT / "src/osm_map_handler/src/osm_camera_path_refinement.py").read_text())
        defaults = {n.args[0].value: n.args[1].value for n in ast.walk(tree)
                    if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
                    and n.func.attr == "get_param" and len(n.args) == 2
                    and all(isinstance(a, ast.Constant) for a in n.args)}
        for key, value in (("camera_clearance_m", 1.25), ("clearance_min_m", 0.5), ("clearance_max_m", 2.0)):
            self.assertIn("camera_right_" + key + ":=" + str(value), commands)
            param = key if key == "camera_clearance_m" else "camera_" + key
            self.assertEqual(defaults["~" + param], value)

    def test_production_constructor_band_validation(self):
        tree = ast.parse((ROOT / "src/osm_map_handler/src/osm_camera_path_refinement.py").read_text())
        checks = [n for n in ast.walk(tree) if isinstance(n, ast.If) and any(
            isinstance(c, ast.Constant) and isinstance(c.value, str) and
            c.value.startswith("camera clearance must satisfy") for c in ast.walk(n))]
        self.assertEqual(len(checks), 1)
        code = compile(ast.Module(body=checks, type_ignores=[]), "production_band_check", "exec")
        for lo, preferred, hi, valid in [(0.5, 1.25, 2.0, True), (1., 2., 3., True),
                                        (0.49, 1.5, 2.5, False), (0.5, 3., 2.5, False),
                                        (0.5, 1.5, float("nan"), False)]:
            context = {"self": SimpleNamespace(clearance_min=lo, camera_clearance=preferred, clearance_max=hi)}
            if valid:
                exec(code, context)
            else:
                with self.assertRaises(ValueError):
                    exec(code, context)

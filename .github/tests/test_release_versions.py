"""发布版本在 Python、ROS 消息和两个 Conda 变体之间保持一致。"""

import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[2]


def recipe(path):
    return yaml.safe_load((ROOT / path).read_text(encoding="utf-8"))


def python_version():
    tree = ast.parse((ROOT / "unilabos/__init__.py").read_text(encoding="utf-8"))
    return next(ast.literal_eval(node.value) for node in tree.body
                if isinstance(node, ast.Assign)
                and any(isinstance(target, ast.Name) and target.id == "__version__"
                        for target in node.targets))


def message_version():
    return ET.parse(ROOT / "unilabos_msgs/package.xml").findtext("version")


class ReleaseVersionTests(unittest.TestCase):
    def test_python_package_versions_match(self):
        tree = ast.parse((ROOT / "setup.py").read_text(encoding="utf-8"))
        setup_call = next(node for node in ast.walk(tree)
                          if isinstance(node, ast.Call)
                          and isinstance(node.func, ast.Name) and node.func.id == "setup")
        version = next(ast.literal_eval(item.value) for item in setup_call.keywords
                       if item.arg == "version")
        self.assertEqual(version, python_version())
        self.assertEqual(recipe("recipes/unilabos/recipe.yaml")["package"]["version"], version)

    def test_ros_recipe_versions_match_package_xml(self):
        for name in ("msgs", "msgs-humble", "ros-humble-unilabos-msgs"):
            with self.subTest(recipe=name):
                self.assertEqual(recipe(f"recipes/{name}/recipe.yaml")["package"]["version"],
                                 message_version())

    def test_conda_variants_pin_matching_environment_and_messages(self):
        for distro, suffix in (("jazzy", ""), ("humble", "-humble")):
            with self.subTest(distro=distro):
                recipes = {kind: recipe(f".conda/{kind}{suffix}/recipe.yaml")
                           for kind in ("base", "environment", "full")}
                for item in recipes.values():
                    self.assertEqual(item["package"]["version"], python_version())
                    self.assertEqual(item["build"]["string"],
                                     f"{distro}_{item['build']['number']}")
                self.assertIn(
                    f"uni-lab::ros-{distro}-unilabos-msgs =={message_version()}",
                    recipes["environment"]["requirements"]["run"],
                )
                for parent, dependency, name in (("base", "environment", "unilabos-env"),
                                                  ("full", "base", "unilabos")):
                    self.assertIn(
                        f"{name} =={python_version()} {recipes[dependency]['build']['string']}",
                        recipes[parent]["requirements"]["run"],
                    )


if __name__ == "__main__":
    unittest.main()

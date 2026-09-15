"""Conda 消息构建与已验证的 CI 依赖、目标架构保持一致。"""

from pathlib import Path
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[2]


class MessageRecipeTests(unittest.TestCase):
    def test_humble_empy_matches_runtime_ci(self):
        recipe = yaml.safe_load((ROOT / "recipes/msgs-humble/recipe.yaml").read_text(encoding="utf-8"))
        for environment in ("build", "host"):
            self.assertIn("empy ==3.3.4", recipe["requirements"][environment])
        self.assertIn("empy==3.3.4", (ROOT / ".github/workflows/ci-check.yml").read_text(encoding="utf-8"))

    def test_cpp_headers_are_explicit_and_language_scoped(self):
        cmake = (ROOT / "unilabos_msgs/CMakeLists.txt").read_text(encoding="utf-8")
        for header in ("cstdint", "locale"):
            self.assertIn(f"$<$<COMPILE_LANGUAGE:CXX>:SHELL:-include {header}>", cmake)

    def test_native_arm_macos_does_not_target_intel_only_os(self):
        for name in ("msgs", "msgs-humble"):
            script = (ROOT / f"recipes/{name}/build_ament_cmake.sh").read_text(encoding="utf-8")
            self.assertIn('if [[ "$target_platform" == "osx-arm64" ]]; then', script)
            self.assertIn('OSX_DEPLOYMENT_TARGET="${MACOSX_DEPLOYMENT_TARGET:-11.0}"', script)
            self.assertNotIn('OSX_DEPLOYMENT_TARGET="10.15"', script)

    def test_fixed_packages_have_a_new_build_number(self):
        for name in ("msgs", "msgs-humble"):
            recipe = yaml.safe_load((ROOT / f"recipes/{name}/recipe.yaml").read_text(encoding="utf-8"))
            self.assertEqual(recipe["build"]["number"], 2)


if __name__ == "__main__":
    unittest.main()

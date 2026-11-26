"""在固件根目录运行 python tests/run_host_tests.py。支持已配置环境的 cl、gcc 或 clang。
可用 --cc 指定编译器路径。可执行文件和目标文件仅在本次测试的临时目录中生成。
"""

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cc", help="C 编译器可执行文件，默认在 PATH 中查找")
    requested = parser.parse_args().cc
    names = (requested,) if requested else ("cc", "gcc", "clang", "cl")
    compiler = next((path for name in names if (path := shutil.which(name))), None)
    if not compiler:
        parser.error("未找到 C 编译器。请在 VS 开发者命令行运行，或用 --cc 指定 gcc。")
    compiler = str(Path(compiler).resolve())
    tests = Path(__file__).resolve().parent
    with tempfile.TemporaryDirectory() as directory:
        for module in ("byte_ring", "robot_link"):
            output = Path(directory) / f"test_{module}.exe"
            sources = [tests.parent / "modules" / f"{module}.c", tests / f"test_{module}.c"]
            if Path(compiler).stem.lower() == "cl":
                flags = ["/nologo", "/std:c11", "/utf-8", "/W4", "/WX", f"/Fe{output}"]
            else:
                flags = ["-std=c99", "-Wall", "-Wextra", "-Werror", "-pedantic", "-o", output]
            subprocess.run([compiler, *flags, *sources], cwd=directory, check=True)
            subprocess.run([output], check=True)
    subprocess.run([sys.executable, "-B", "-m", "unittest", "discover", "-s", tests,
                    "-p", "test_robot_peer.py", "-v"], check=True)


if __name__ == "__main__":
    main()

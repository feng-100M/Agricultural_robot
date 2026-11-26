"""用 MDK 目标的源码清单生成临时仿真 ELF 并执行回归。示例：
python tests/run_firmware_checks.py --cube <STM32Cube_FW_H7_V1.11.0> --cc <arm-none-eabi-gcc>
需要 Python、unicorn 2、pyelftools 和带 newlib-nano 的 Arm GNU 工具链。
启动汇编、FreeRTOS 的 GCC 移植及内部 Flash 链接脚本取自指定的 CubeH7 包。
"""

import argparse
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path


def build_and_check(compiler, cube, work):
    tests = Path(__file__).resolve().parent
    mdk = tests.parent / "MDK-ARM"
    tree = ET.parse(mdk / "RTOS_HAL.uvprojx")
    target = tree.find(".//Target[TargetName='RTOS_HAL']")
    controls = target.find("TargetOption/TargetArmAds/Cads/VariousControls")
    port = cube / "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F"
    startup = cube / "Drivers/CMSIS/Device/ST/STM32H7xx/Source/Templates/gcc"
    linker = cube / "Projects/STM32H750B-DK/Templates/ExtMem_Boot/STM32CubeIDE/STM32H750XBHX_FLASH.ld"
    machine = ["-mcpu=cortex-m7", "-mthumb", "-mfpu=fpv5-d16", "-mfloat-abi=hard"]
    options = ["-Os", "-g", "-ffunction-sections", "-fdata-sections", "-I" + str(port)]
    options.extend("-D" + name.strip() for name in controls.findtext("Define").split(","))
    for directory in controls.findtext("IncludePath").split(";"):
        if "/RVDS/" not in directory.replace("\\", "/"):
            options.append("-I" + str(mdk / directory.replace("\\", "/")))

    subprocess.run([compiler, "--version"], check=True)
    objects = []
    for entry in target.findall("Groups/Group/Files/File"):
        if entry.findtext("FileOption/CommonProperty/IncludeInBuild") == "0":
            continue
        source = mdk / entry.findtext("FilePath").replace("\\", "/")
        if source.name == "startup_stm32h750xx.s":
            source = startup / source.name
        elif source.name == "port.c" and "RVDS" in source.parts:
            source = port / source.name
        elif source.suffix.lower() != ".c":
            continue
        output = work / f"{len(objects)}_{source.name}.o"
        language = ["-std=gnu99"] if source.suffix == ".c" else []
        subprocess.run([compiler, *machine, *language, *options, "-c", source, "-o", output],
                       check=True)
        objects.append(output)

    elf = work / "robot.elf"
    subprocess.run([compiler, *machine, "-T", linker, "-Wl,--gc-sections", "--specs=nano.specs",
                    "--specs=nosys.specs", *objects, "-lm", "-o", elf], check=True)
    print(f"已编译并链接 {len(objects)} 个目标文件", flush=True)
    subprocess.run([sys.executable, "-B", tests / "check_firmware_elf.py", elf, "-v"], check=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cube", required=True, type=Path, help="STM32Cube_FW_H7_V1.11.0 根目录")
    parser.add_argument("--cc", default="arm-none-eabi-gcc", help="Arm GCC 可执行文件")
    args = parser.parse_args()
    compiler_path = shutil.which(args.cc)
    if not compiler_path:
        parser.error("未找到 Arm GCC，请用 --cc 指定编译器。")
    with tempfile.TemporaryDirectory() as directory:
        build_and_check(compiler_path, args.cube.resolve(), Path(directory))

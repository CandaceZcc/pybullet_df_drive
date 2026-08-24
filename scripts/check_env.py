# 环境检查脚本：确认依赖包可导入，并验证 PyBullet DIRECT 模式能连接。
from __future__ import annotations

import os
import sys

import matplotlib
import numpy
import pandas
import pybullet as p
from PySide6 import QtCore
import yaml


def main() -> int:
    """打印关键依赖版本，并用 DIRECT 模式做最小连接测试。"""
    print(f"python: {sys.version.split()[0]}")
    print(f"executable: {sys.executable}")
    print(f"pybullet_api_version: {p.getAPIVersion()}")
    print(f"numpy: {numpy.__version__}")
    print(f"matplotlib: {matplotlib.__version__}")
    print(f"pandas: {pandas.__version__}")
    print(f"pyside6: {QtCore.__version__}")
    print(f"pyyaml: {yaml.__version__}")
    print(f"XDG_SESSION_TYPE: {os.environ.get('XDG_SESSION_TYPE', 'unset')}")
    print(f"DISPLAY: {os.environ.get('DISPLAY', 'unset')}")

    client_id = p.connect(p.DIRECT)
    print(f"DIRECT connected: {client_id}")
    if client_id < 0:
        return 1
    p.disconnect(client_id)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

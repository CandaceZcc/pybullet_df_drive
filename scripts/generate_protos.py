"""Protobuf 代码生成脚本：从固定企业与内部协议源生成 Python 类型。"""

from pathlib import Path

from grpc_tools import protoc


ROOT = Path(__file__).resolve().parents[1]
PROTO_DIR = ROOT / "proto"
PROTO_FILES = (
    PROTO_DIR / "slope_sim_interfaces.proto",
    PROTO_DIR / "slope_sim_internal.proto",
)
OUTPUT_DIR = ROOT / "slope_sim/interfaces/generated"


def main() -> int:
    """调用 protoc，将两个固定协议稳定生成到项目内的类型目录。"""
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    return protoc.main(
        [
            "grpc_tools.protoc",
            f"--proto_path={PROTO_DIR}",
            f"--python_out={OUTPUT_DIR}",
            *(str(proto_file) for proto_file in PROTO_FILES),
        ]
    )


if __name__ == "__main__":
    raise SystemExit(main())

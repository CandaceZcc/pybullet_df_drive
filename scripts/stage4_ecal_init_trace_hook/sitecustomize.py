"""阶段四诊断钩子：记录原生 eCAL initialize 的安装与调用状态。"""
from __future__ import annotations

import os
from pathlib import Path


_TRACE = os.environ.get("STAGE4_ECAL_INIT_TRACE")
_PID = os.getpid()


def _write(event: str, detail: str = "") -> None:
    """以单行 append 保留每个 Python 进程的诊断事件。"""
    if _TRACE:
        with Path(_TRACE).open("a", encoding="utf-8") as stream:
            stream.write(f"{event} pid={_PID}{detail}\n")


if _TRACE:
    _write("sitecustomize")
    try:
        from ecal import nanobind_core as _core

        _initialize = _core.initialize
        if not callable(_initialize):
            raise TypeError("ecal.nanobind_core.initialize is not callable")

        def _traced_initialize(*args: object, **kwargs: object) -> object:
            _write("initialize args", " args=" + repr(args) + " kwargs=" + repr(kwargs))
            return _initialize(*args, **kwargs)

        _core.initialize = _traced_initialize
    except ModuleNotFoundError as error:
        if error.name == "ecal":
            _write("ecal_unavailable")
        else:
            _write("hook_install_failed", f" error={type(error).__name__}:{error}")
    except Exception as error:
        _write("hook_install_failed", f" error={type(error).__name__}:{error}")
    else:
        _write("hook_installed")

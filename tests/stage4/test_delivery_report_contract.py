# 阶段四交付报告合同测试：锁定初始状态，禁止提前声明外部门禁通过。
from pathlib import Path


def test_stage4_report_records_local_p0_without_false_external_pass_claims() -> None:
    report = Path("docs/阶段四交付报告.md")
    assert report.is_file(), "stage four delivery report is not implemented"
    text = report.read_text(encoding="utf-8")
    assert "P0 本地异步 LiDAR worker 已完成并通过 DIRECT 门" in text
    assert "阶段四正式 Task 2 与 A-E 尚未启动" in text
    assert "P0 本地异步 LiDAR Worker" in text
    assert "| 真实 eCAL | 未执行 | 无 |" in text
    assert "| 真实 RViz2 | 未执行 | 无 |" in text
    assert "| Livox Viewer 2 | 未执行 | 无 |" in text
    assert "| 干净机迁移 | 未执行 | 无 |" in text

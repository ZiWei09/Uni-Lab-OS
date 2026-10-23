"""配液站实验报告解析

报告里每个通量（preIntakes[i]）的 extraProperties 以步骤 ID 为 key 存实际值，如 '{"targetWeigh":1.00}'、'{"volume":9.85}'；
步骤 ID 每个工作流都不同，需借助报告自带的 workflowParameters（{步骤ID: [{name: "称量模块-10%加料&称量", ...}]}）换成步骤名再区分。
"""
import json
from typing import Any, Dict, List, Optional


def _loads(value: Any) -> Any:
    if isinstance(value, str):
        try:
            return json.loads(value)
        except ValueError:
            return value
    return value


def _to_float(value: Any) -> Optional[float]:
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def parse_channel_actuals(report_data: Dict[str, Any]) -> List[Dict[str, Any]]:
    """按步骤名解析每个通量的实际加料量

    返回每个通量：
        code: 通量编号
        steps: {步骤名: 实际值字典}
        weigh_10 / weigh_90: 小瓶工作流 10% / 90% 加料称量(g)
        weigh: 烧杯工作流加料称量(g)
        volume: 加液体积(ml)
        actualTargetWeigh: 用于滴定计算的称量值（有 10% 称量时取 10%，否则取加料称量）
        actualVolume: 加液体积
    """
    workflow_parameters = _loads(report_data.get("workflowParameters")) or {}
    step_names = {
        step_id: groups[0].get("name")
        for step_id, groups in workflow_parameters.items()
        if isinstance(groups, list) and groups
    }

    channels = []
    for channel in report_data.get("preIntakes") or []:
        steps: Dict[str, Any] = {}
        weigh_10 = weigh_90 = weigh = volume = None
        for step_id, raw in (channel.get("extraProperties") or {}).items():
            value = _loads(raw)
            name = step_names.get(step_id) or step_id
            steps[name] = value
            if not isinstance(value, dict):
                continue
            if "targetWeigh" in value:
                w = _to_float(value["targetWeigh"])
                if "10%" in name:
                    weigh_10 = w
                elif "90%" in name:
                    weigh_90 = w
                else:
                    weigh = w
            if "volume" in value:
                volume = _to_float(value["volume"])
        channels.append({
            "code": channel.get("code"),
            "steps": steps,
            "weigh_10": weigh_10,
            "weigh_90": weigh_90,
            "weigh": weigh,
            "volume": volume,
            "actualTargetWeigh": weigh_10 if weigh_10 is not None else weigh,
            "actualVolume": volume,
        })
    return channels


def actuals_from_simplified_report(report: Any) -> Dict[str, Any]:
    """从简化后的报告（_simplify_report 的结果）取实际加料量

    actualTargetWeigh / actualVolume 取第一个通量（与下游滴定计算的既有约定一致），channels 给出全部通量。
    """
    data = report.get("data") if isinstance(report, dict) else None
    channels = (data or {}).get("channels") or []
    first = channels[0] if channels else {}
    return {
        "actualTargetWeigh": first.get("actualTargetWeigh"),
        "actualVolume": first.get("actualVolume"),
        "channels": channels,
    }

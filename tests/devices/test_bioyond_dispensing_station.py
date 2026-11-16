import inspect
import json
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml
from unilabos_msgs.action import DispenStationSolnPrep

from unilabos.devices.workstation.bioyond_studio.dispensing_station.dispensing_station import (
    BioyondDispensingStation,
)
from unilabos.ros.msgs.message_converter import (
    convert_from_ros_msg_with_mapping,
    convert_to_ros_msg,
)


REPO_ROOT = Path(__file__).resolve().parents[2]
REGISTRY_FILE = REPO_ROOT / "unilabos/registry/devices/bioyond_dispensing_station.yaml"
OMITTED = object()
REQUIRED_ARGS = {
    "material_name": "ODA1",
    "target_weigh": "14.475",
    "volume": "96.492",
    "hold_m_name": "ODA1",
}


@pytest.fixture
def station() -> BioyondDispensingStation:
    """仅 mock BY 通信边界，不连接真实设备。"""
    instance = BioyondDispensingStation.__new__(BioyondDispensingStation)
    instance.hardware_interface = SimpleNamespace(
        _logger=Mock(),
        material_id_query=Mock(
            return_value={
                "locations": [{"holdMName": "ODA1", "holdMId": "material-oda1"}],
            }
        ),
        create_order=Mock(return_value={}),
    )
    return instance


def _create_order_payload(station: BioyondDispensingStation, **kwargs) -> dict:
    station.create_diamine_solution_task(**{**REQUIRED_ARGS, **kwargs})
    station.hardware_interface.create_order.assert_called_once()
    payload = json.loads(station.hardware_interface.create_order.call_args.args[0])
    assert len(payload) == 1
    return payload[0]


def _parameter_value(payload: dict, key: str) -> str:
    return next(
        item["Value"]
        for parameters in payload["paramValues"].values()
        for item in parameters
        if item["Key"] == key
    )


def _single_action_config() -> dict:
    registry = yaml.safe_load(REGISTRY_FILE.read_text(encoding="utf-8"))
    return registry["bioyond_dispensing_station"]["class"]["action_value_mappings"][
        "create_diamine_solution_task"
    ]


@pytest.mark.parametrize(
    ("speed", "expected"),
    [
        pytest.param(OMITTED, "400", id="omitted"),
        pytest.param(None, "400", id="none"),
        pytest.param("", "400", id="empty-string"),
        pytest.param("   ", "400", id="whitespace"),
        pytest.param("200", "200", id="explicit-value"),
        pytest.param(" 200 ", "200", id="trim-explicit-value"),
    ],
)
def test_diamine_solution_speed_defaults_in_outgoing_payload(
    station: BioyondDispensingStation, speed: object, expected: str
):
    kwargs = {} if speed is OMITTED else {"speed": speed}
    payload = _create_order_payload(station, **kwargs)
    assert _parameter_value(payload, "speed") == expected


def test_single_action_ros_blank_optional_values_use_defaults(
    station: BioyondDispensingStation,
):
    """复现线上 ROS Goal 将未填写的可选参数传为空字符串的场景。"""
    action_config = _single_action_config()
    goal_defaults = {
        **action_config["goal_default"],
        **REQUIRED_ARGS,
        "order_name": "",
        "liquid_material_name": "",
        "speed": "",
        "temperature": "",
        "delay_time": "",
    }
    goal = convert_to_ros_msg(DispenStationSolnPrep.Goal(), goal_defaults)
    action_kwargs = convert_from_ros_msg_with_mapping(goal, action_config["goal"])
    assert action_kwargs["speed"] == ""

    payload = _create_order_payload(station, **action_kwargs)

    assert _parameter_value(payload, "speed") == "400"
    assert _parameter_value(payload, "temperature") == "20"
    assert _parameter_value(payload, "DelayTime") == "600"
    assert _parameter_value(payload, "assignMaterialName") == "ODA1"
    assert payload["orderName"] == "二胺溶液配置-ODA1"
    assert json.loads(payload["ExtendProperties"]) == {"material-oda1": {}}
    liquid_parameters = payload["paramValues"][
        "3a15d4a1-3bde-d584-b309-e661ae8f1c01"
    ]
    assert (
        next(
            item["Value"]
            for item in liquid_parameters
            if item["Key"] == "assignMaterialName"
        )
        == "NMP"
    )


def test_single_action_registry_has_explicit_default_speed():
    action_config = _single_action_config()
    assert action_config["goal_default"]["speed"] == "400"


def test_method_default_speed_survives_ast_registry_generation():
    """自动动作生成 goal_default 时使用函数签名中的显式默认值。"""
    parameters = inspect.signature(
        BioyondDispensingStation.create_diamine_solution_task
    ).parameters
    assert parameters["speed"].default == "400"
    assert parameters["temperature"].default == "20"
    assert parameters["delay_time"].default == "600"


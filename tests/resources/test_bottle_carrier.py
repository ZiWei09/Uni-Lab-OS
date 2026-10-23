import pytest

from unilabos.resources.bioyond.bottle_carriers import BIOYOND_Electrolyte_6VialCarrier, BIOYOND_Electrolyte_1BottleCarrier
from unilabos.resources.bioyond.bottles import BIOYOND_PolymerStation_Solid_Vial, BIOYOND_PolymerStation_Solution_Beaker


def test_carriers_come_prefilled():
    # 载架创建时自带瓶子，槽位直接持有瓶子对象
    vial_carrier = BIOYOND_Electrolyte_6VialCarrier("powder_carrier_01")
    beaker_carrier = BIOYOND_Electrolyte_1BottleCarrier("solution_carrier_01")

    assert len(vial_carrier.sites) == 6
    assert len(beaker_carrier.sites) == 1
    assert all(site is not None and site.parent is vial_carrier for site in vial_carrier.sites)
    assert beaker_carrier[0].parent is beaker_carrier


def test_occupied_site_rejects_new_bottle():
    vial_carrier = BIOYOND_Electrolyte_6VialCarrier("powder_carrier_01")

    with pytest.raises(ValueError, match="already has a resource"):
        vial_carrier[0] = BIOYOND_PolymerStation_Solid_Vial("powder_bottle_01")


def test_swap_bottle_after_unassign():
    vial_carrier = BIOYOND_Electrolyte_6VialCarrier("powder_carrier_01")
    beaker_carrier = BIOYOND_Electrolyte_1BottleCarrier("solution_carrier_01")
    new_vial = BIOYOND_PolymerStation_Solid_Vial("powder_bottle_01")
    new_beaker = BIOYOND_PolymerStation_Solution_Beaker("solution_beaker_01")

    vial_carrier.unassign_child_resource(vial_carrier[0])
    assert vial_carrier[0] is None
    vial_carrier[0] = new_vial

    beaker_carrier.unassign_child_resource(beaker_carrier[0])
    beaker_carrier[0] = new_beaker

    assert vial_carrier[0] is new_vial and new_vial.parent is vial_carrier
    assert len(vial_carrier.children) == 6
    assert beaker_carrier[0] is new_beaker and new_beaker.parent is beaker_carrier

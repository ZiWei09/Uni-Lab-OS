"""聚酰亚胺（PAA）投料计算器

逐行移植自 PAA实际投料4.html 的 calcAll()，常数（分子量、密度）与网页保持一致。
网页里 x、y… 代表各二酐的实际投入质量，反应站执行时由 Bioyond 的称重结果引用（如 {{4-0-5}}）替代。
"""
from typing import Any, Dict, List, Optional, TypedDict

# 物料数据库（与网页一致）
MATERIALS: Dict[str, Dict[str, Any]] = {
    "BTDA": {"type": "anhydride", "molar_mass": 322.23},
    "BPDA": {"type": "anhydride", "molar_mass": 294.22},
    "ODA": {"type": "amine", "molar_mass": 200.24},
    "MPDA": {"type": "amine", "molar_mass": 108.14},
    "SIDA": {"type": "amine", "molar_mass": 248.31, "is_liquid": True, "density": 0.901},
    "BAPP": {"type": "amine", "molar_mass": 412.51},
    "TFMB": {"type": "amine", "molar_mass": 334.23},
    "PGME": {"type": "addSolvent", "molar_mass": 90.121, "density": 0.9234},
}
NMP_DENSITY = 1.03


class AnhydrideInput(TypedDict):
    name: str
    theory_mass: float  # 理论投料质量 g


class AmineInput(TypedDict, total=False):
    name: str
    eq: float  # 当量
    solution_mass: float  # 配液固体质量 g（液体二胺不需要）
    solution_volume: float  # 配液定容体积 ml（液体二胺不需要）


def compute_paa_feed(
    anhydrides: List[AnhydrideInput],
    amines: List[AmineInput],
    anhydride_amine_ratio: float,
    solid_percent: float,
    supplement_anhydride: str,
    supplement_solid_mass: float,
    supplement_volume: float,
    pgme_fraction_percent: Optional[float] = None,
) -> Dict[str, Any]:
    """计算二胺投料体积、PGME/NMP 用量，以及二酐补充液、NMP 补充液的线性公式

    pgme_fraction_percent 为 None 时全部溶剂为 NMP（对应网页"补充溶剂 0 种"），否则为 PGME 质量分数模式。
    补充液公式均为 常数 ± Σ 系数·(二酐实际投入质量)。
    """
    # 二酐
    anhydride_rows = []
    total_anh_mol = 0.0
    for a in anhydrides:
        molar_mass = MATERIALS[a["name"]]["molar_mass"]
        mol = a["theory_mass"] / molar_mass
        anhydride_rows.append({"name": a["name"], "theory_mass": a["theory_mass"], "molar_mass": molar_mass, "mol": mol})
        total_anh_mol += mol
    if total_anh_mol <= 1e-9:
        raise ValueError("二酐理论总物质的量必须大于0")

    # 工艺参数
    if not 0 < solid_percent < 100:
        raise ValueError("固含量必须在 0~100% 之间")
    total_amine_target_mol = total_anh_mol / anhydride_amine_ratio

    # 二胺
    amine_rows = []
    sum_eq = 0.0
    for am in amines:
        mat = MATERIALS[am["name"]]
        sum_eq += am["eq"]
        need_mol = total_amine_target_mol * am["eq"]
        need_mass = need_mol * mat["molar_mass"]
        if mat.get("is_liquid"):
            conc = None
            feed_vol = need_mass / mat["density"]
            note = "液体，不配液直接投料"
        else:
            conc = am["solution_mass"] / am["solution_volume"]
            feed_vol = need_mass / conc
            note = "配液溶液投料"
        amine_rows.append({"name": am["name"], "eq": am["eq"], "need_mol": need_mol, "need_mass": need_mass,
                           "conc": conc, "feed_vol": feed_vol, "note": note})
    warnings = []
    if abs(sum_eq - 1.0) > 1e-3:
        warnings.append(f"二胺当量总和={sum_eq:.4f}，建议等于1.0")

    total_amine_feed_vol = sum(r["feed_vol"] for r in amine_rows)
    total_solid_mass = sum(r["theory_mass"] for r in anhydride_rows) + sum(r["need_mass"] for r in amine_rows)
    total_sys_mass = total_solid_mass / (solid_percent / 100.0)
    total_solvent_mass = total_sys_mass - total_solid_mass

    # 补充溶剂
    if pgme_fraction_percent is None:
        pgme_mass = pgme_vol = 0.0
        nmp_mass = total_solvent_mass
    else:
        if not 0 <= pgme_fraction_percent < 100:
            raise ValueError("PGME质量分数需要在 0~100 之间")
        pgme_mass = total_solvent_mass * (pgme_fraction_percent / 100.0)
        pgme_vol = pgme_mass / MATERIALS["PGME"]["density"]
        nmp_mass = total_solvent_mass - pgme_mass
    nmp_vol = nmp_mass / NMP_DENSITY

    # 二酐补充液：V = (Σn理论 − Σ(m实际/M)) / C补充液
    c_supp = (supplement_solid_mass / MATERIALS[supplement_anhydride]["molar_mass"]) / supplement_volume
    anh_const = total_anh_mol / c_supp
    coeffs = [{"name": a["name"], "coeff": 1.0 / (a["molar_mass"] * c_supp)} for a in anhydride_rows]
    # NMP补充液：V = V_NMP总 − ΣV二胺投料 − V二酐补充液（PGME 单独投料，不计入）
    nmp_const = nmp_vol - total_amine_feed_vol - anh_const

    return {
        "anhydrides": anhydride_rows,
        "amines": amine_rows,
        "total_amine_target_mol": total_amine_target_mol,
        "total_amine_feed_vol": total_amine_feed_vol,
        "total_solid_mass": total_solid_mass,
        "total_sys_mass": total_sys_mass,
        "total_solvent_mass": total_solvent_mass,
        "pgme_mass": pgme_mass,
        "pgme_vol": pgme_vol,
        "nmp_mass": nmp_mass,
        "nmp_vol": nmp_vol,
        "supplement_mol_per_ml": c_supp,
        "anhydride_supplement_const": anh_const,
        "nmp_supplement_const": nmp_const,
        "supplement_coeffs": coeffs,
        "warnings": warnings,
    }


def anhydride_supplement_formula(result: Dict[str, Any], refs: List[str]) -> str:
    """二酐补充液体积公式：常数 − Σ 系数·引用（数值保留 4 位小数，与网页显示一致）"""
    terms = "".join(f"-{c['coeff']:.4f}*{ref}" for c, ref in zip(result["supplement_coeffs"], refs, strict=True))
    return f"{result['anhydride_supplement_const']:.4f}{terms}"


def nmp_supplement_formula(result: Dict[str, Any], refs: List[str]) -> str:
    """NMP补充液体积公式：Σ 系数·引用 + 常数（常数通常为负，放在末尾写成 a*x-b 的形式）"""
    terms = "+".join(f"{c['coeff']:.4f}*{ref}" for c, ref in zip(result["supplement_coeffs"], refs, strict=True))
    return f"{terms}{result['nmp_supplement_const']:+.4f}"

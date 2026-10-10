"""Step37b 独立 CarSim 文件配置：源审计与运行目录分两步。

Step35b 已审核的 60 s 五组配置是唯一父配置。本轮 Run_all.par 逐字节相同；
simfile.sim 只替换 INPUT 与 OUTPUT_FILE_PREFIX。默认检查不写文件；
--prepare-source 只在本批 E 盘保存小型配置身份；--install-runtime 才在
独立 C 盘 ASCII 目录写 sim/par，供主任务串行执行。此脚本不启动车辆。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
STEP35B = ROOT.parent / "step35b_large_beta_equilibrium_20261006"
CASES = ('kf384_kr384', 'kf192_kr384', 'kf576_kr384', 'kf384_kr192', 'kf384_kr576')
RUNTIME = Path(r"C:\Users\28583\AppData\Local\Temp\step37c_road_coordinate_20261010")
SCHEMA = "step37a-state-only-file-case-v1"
MANIFEST_SCHEMA = "step37c-road-coordinate-config-manifest-v1"


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def file_digest(path: Path) -> str:
    return digest(path.read_bytes())


def require_unique_value(data: bytes, key: bytes) -> bytes:
    hits = re.findall(rb"(?m)^" + re.escape(key) + rb"[ \t]+([^\r\n]+)", data)
    if len(hits) != 1:
        raise ValueError(f"配置项缺失或重复: {key!r}, count={len(hits)}")
    return hits[0].strip()


def replace_unique_value(data: bytes, key: bytes, value: bytes) -> bytes:
    pat = re.compile(rb"(?m)^(" + re.escape(key) + rb"[ \t]+)([^\r\n]+)")
    hits = list(pat.finditer(data))
    if len(hits) != 1:
        raise ValueError(f"配置项缺失或重复: {key!r}, count={len(hits)}")
    hit = hits[0]
    return data[: hit.start(2)] + value + data[hit.end(2) :]


def audit_par(par: bytes) -> None:
    # 文件含历史嵌套命令；按最后有效 OPT_STOP 后唯一 TSTOP 核验。
    opts = list(re.finditer(rb"(?m)^OPT_STOP[ \t]+([^\r\n]+)", par))
    stops = list(re.finditer(rb"(?m)^TSTOP[ \t]+([^\r\n]+)", par))
    if not opts or opts[-1].group(1).strip() != b"0":
        raise ValueError("父配置最后有效 OPT_STOP 不是 0")
    effective = [m for m in stops if m.start() > opts[-1].end()]
    if len(effective) != 1 or effective[0].group(1).strip() != b"60":
        raise ValueError("父配置最后有效 TSTOP 不是唯一的 60 s")


def audit_sim(sim: bytes) -> None:
    for key, value in ((b"EXT_MODEL_STEP", b"0.00050000"), (b"PORTS_IMP", b"1,12"), (b"PORTS_EXP", b"1,76")):
        if require_unique_value(sim, key) != value:
            raise ValueError(f"CarSim 接口/采样时间改变: {key.decode()}")


def identity_path(case: str) -> Path:
    return ROOT / "source" / "file_cases" / case / "aux_case_cfg.json"


def straighten(par: bytes) -> bytes:
    # 唯一白名单：闭合开关、段类型、半径/圆心角替换为直线长度；其他字节保持。
    assert par.count(b"OPT_PATH_LOOP 1") == 1
    assert par.count(b"SEGMENT_TYPE 3") == 1
    par = par.replace(b"OPT_PATH_LOOP 1", b"OPT_PATH_LOOP 0")
    par = par.replace(b"SEGMENT_TYPE 3", b"SEGMENT_TYPE 0")
    pattern = rb"SEGMENT_RADIUS 15\.000000(\r?\n)SEGMENT_ARC 360\.000000"
    assert len(re.findall(pattern,par)) == 1
    return re.sub(pattern,b"SEGMENT_LENGTH 2000",par)


def build_case(case: str) -> tuple[dict, bytes, bytes]:
    # 以本批逻辑路径恢复历史 JSON 的中文路径编码，不修改历史文件。
    old_cfg_path = STEP35B / "source" / "file_cases" / case / "aux_case_cfg.json"
    old_manifest = json.loads((STEP35B / "source" / "config_manifest.json").read_text(encoding="utf-8"))
    matches = [item for item in old_manifest["cases"] if item["case"] == case]
    if len(matches) != 1 or file_digest(old_cfg_path) != matches[0]["configuration_identity_sha256"]:
        raise ValueError(f"Step35b 父身份清单不符: {case}")
    old = json.loads(old_cfg_path.read_text(encoding="utf-8"))
    if old["schema"] != "step35b-file-case-v1" or old["case"] != case or old["configuration_mode"] != "file_derived_from_GUI_baseline":
        raise ValueError(f"Step35b 父配置身份错误: {case}")
    old_par_path = Path(old["source_path"])
    old_sim_path = Path(old["simfile_path"])
    par, sim = old_par_path.read_bytes(), old_sim_path.read_bytes()
    if digest(par) != old["source_sha256"] or digest(sim) != old["simfile_sha256"]:
        raise ValueError(f"Step35b 父 sim/par 字节哈希改变: {case}")
    if old["effective_tstop_s"] != 60 or old["opt_stop"] != 0 or old["step_s"] != 0.0005 or old["ports_import"] != 12 or old["ports_export"] != 76:
        raise ValueError(f"Step35b 父运行参数改变: {case}")
    audit_par(par)
    par = straighten(par)
    audit_sim(sim)
    target = RUNTIME / case
    new_par_path = target / "Run_all.par"
    new_sim_path = target / "simfile.sim"
    # C 盘目标路径必须全 ASCII；par 仅作白名单道路坐标修改。
    new_input = str(new_par_path).encode("ascii")
    new_output = str(target / "LastRun").encode("ascii")
    old_input = require_unique_value(sim, b"INPUT")
    old_output = require_unique_value(sim, b"SET_MACRO $(OUTPUT_FILE_PREFIX)$")
    if old_input.decode("ascii").casefold() != str(old_par_path).casefold():
        raise ValueError(f"Step35b sim INPUT 不指向同组父 par: {case}")
    new_sim = replace_unique_value(sim, b"INPUT", new_input)
    new_sim = replace_unique_value(new_sim, b"SET_MACRO $(OUTPUT_FILE_PREFIX)$", new_output)
    if replace_unique_value(replace_unique_value(new_sim, b"INPUT", old_input), b"SET_MACRO $(OUTPUT_FILE_PREFIX)$", old_output) != sim:
        raise ValueError(f"sim 字节差异超过白名单: {case}")
    audit_sim(new_sim)
    cfg = {
        "schema": SCHEMA, "configuration_mode": "file_derived_road_coordinates_straight_uniform_plane",
        "case": case, "kf_deg": old["kf_deg"], "kr_deg": old["kr_deg"],
        "source_path": str(new_par_path), "source_sha256": digest(par),
        "simfile_path": str(new_sim_path), "simfile_sha256": digest(new_sim),
        "origin_cfg_path": str(old_cfg_path), "origin_cfg_sha256": file_digest(old_cfg_path),
        "origin_source_path": str(old_par_path), "origin_source_sha256": file_digest(old_par_path),
        "origin_simfile_path": str(old_sim_path), "origin_simfile_sha256": digest(sim),
        "authorized_diffs": ["Run_all.par: only OPT_PATH_LOOP 1->0, SEGMENT_TYPE 3->0, radius/arc -> SEGMENT_LENGTH 2000; physical flat road unchanged", "sim INPUT", "sim OUTPUT_FILE_PREFIX"],
        "effective_tstop_s": 60, "opt_stop": 0, "external_asb_zero_required": True,
        "step_s": 0.0005, "ports_import": 12, "ports_export": 76,
        "gui_database_mutated": False, "prepared_by": "internal/prepare_state_configs.py",
    }
    return cfg, par, new_sim


def manifest_for(planned: list[tuple[str, dict, bytes, bytes]]) -> dict:
    cases = []
    for case, cfg, _par, _sim in planned:
        identity = identity_path(case)
        cases.append({
            "case": case, "configuration_identity": str(identity),
            "configuration_identity_sha256": digest((json.dumps(cfg, ensure_ascii=False, indent=2) + "\n").encode("utf-8")),
            "origin_par_sha256": cfg["origin_source_sha256"],
            "origin_sim_sha256": cfg["origin_simfile_sha256"],
            "derived_par_sha256": cfg["source_sha256"],
            "derived_sim_sha256": cfg["simfile_sha256"],
            "byte_whitelist_review": cfg["authorized_diffs"],
            "post_run_echo_identity": "pending; runner checks MX_AUX_COEFFICIENT(1/2)",
        })
    return {"schema": MANIFEST_SCHEMA, "preparation_only_no_simulation": True,
            "gui_database_mutated": False, "cases": cases}


def verify_source(planned: list[tuple[str, dict, bytes, bytes]]) -> None:
    manifest_path = ROOT / "source" / "config_manifest.json"
    actual = json.loads(manifest_path.read_text(encoding="utf-8"))
    expected = manifest_for(planned)
    if actual != expected:
        raise ValueError("本批配置 manifest 与逐字节重算结果不符")
    for case, cfg, _par, _sim in planned:
        identity = identity_path(case)
        if file_digest(identity) != next(item for item in expected["cases"] if item["case"] == case)["configuration_identity_sha256"]:
            raise ValueError(f"本批配置身份字节哈希改变: {case}")
        if json.loads(identity.read_text(encoding="utf-8")) != cfg:
            raise ValueError(f"本批配置身份数值改变: {case}")


def verify_runtime(planned: list[tuple[str, dict, bytes, bytes]]) -> None:
    for case, cfg, par, sim in planned:
        runtime = RUNTIME / case
        if (runtime / "Run_all.par").read_bytes() != par or (runtime / "simfile.sim").read_bytes() != sim:
            raise ValueError(f"独立运行目录文件与源审核不符: {case}")
        if file_digest(runtime / "Run_all.par") != cfg["source_sha256"] or file_digest(runtime / "simfile.sim") != cfg["simfile_sha256"]:
            raise ValueError(f"独立运行目录字节哈希不符: {case}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--prepare-source", action="store_true", help="仅新建 E 盘配置身份与 manifest，不写 C 盘")
    mode.add_argument("--install-runtime", action="store_true", help="在已准备 E 盘身份核验后，写独立 C 盘 ASCII sim/par")
    mode.add_argument("--verify", action="store_true", help="只读核对 E/C 两层实际文件")
    args = parser.parse_args()
    planned = [(case, *build_case(case)) for case in CASES]
    manifest_path = ROOT / "source" / "config_manifest.json"
    if args.prepare_source:
        if manifest_path.exists() or any(identity_path(case).exists() for case in CASES):
            raise FileExistsError("本批配置身份已经存在，拒绝覆盖")
        for case, cfg, _par, _sim in planned:
            identity = identity_path(case)
            identity.parent.mkdir(parents=True, exist_ok=True)
            identity.write_bytes((json.dumps(cfg, ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
        manifest_path.write_bytes((json.dumps(manifest_for(planned), ensure_ascii=False, indent=2) + "\n").encode("utf-8"))
        verify_source(planned)
    elif args.install_runtime:
        verify_source(planned)
        if any((RUNTIME / case).exists() for case in CASES):
            raise FileExistsError("至少一组 C 盘运行目录已经存在，拒绝覆盖")
        for case, _cfg, par, sim in planned:
            runtime = RUNTIME / case
            runtime.mkdir(parents=True, exist_ok=False)
            (runtime / "Run_all.par").write_bytes(par)
            (runtime / "simfile.sim").write_bytes(sim)
        verify_runtime(planned)
    elif args.verify:
        verify_source(planned)
        verify_runtime(planned)
    for case, cfg, _par, _sim in planned:
        print(json.dumps({"case": case, "source_sha256": cfg["source_sha256"],
                          "simfile_sha256": cfg["simfile_sha256"], "mode": "plan" if not any((args.prepare_source, args.install_runtime, args.verify)) else
                          "prepare-source" if args.prepare_source else "install-runtime" if args.install_runtime else "verify"}, ensure_ascii=False))


if __name__ == "__main__":
    main()

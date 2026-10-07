"""step30 单文件 MAT v7 数值归档读取与旧 BIN 临时适配。

仅解析本项目写出的压缩、实数、二维 double/uint8 MAT5 变量；遇到
未知类型立即失败。一次只解压一个变量，不把全组数组落盘或常驻内存。
"""

from __future__ import annotations

import json
import struct
import zlib
from functools import lru_cache
from pathlib import Path

import numpy as np

LEGACY_NAMES = (
    "repair_plant_raw", "repair_actuator_command", "repair_entry_diag",
    "repair_cycle_diag", "repair_stadium_diag", "repair_steer_diag",
    "asb_import12", "asb_diag", "asb_suspension", "asb_roll_feedback",
    "feedback_diag",
)
ARCHIVE_NAME = "raw_numeric.mat"
_MI_INT8, _MI_UINT8, _MI_INT32, _MI_UINT32 = 1, 2, 5, 6
_MI_DOUBLE, _MI_MATRIX, _MI_COMPRESSED = 9, 14, 15


def _mat_path(run: str | Path) -> Path:
    path = Path(run)
    return path if path.suffix.lower() == ".mat" else path / ARCHIVE_NAME


def has_records(run: str | Path) -> bool:
    """检查规范数值归档存在且带有已知版本的通道元数据。"""
    path = _mat_path(run)
    if not path.is_file():
        return False
    return metadata(path)["archive_version"] == 1


def _element(buf: bytes | memoryview, pos: int) -> tuple[int, memoryview, int]:
    """读取 MATLAB mi 元素；兼容短标签并按八字节对齐。"""
    view = memoryview(buf)
    if pos + 8 > len(view):
        raise ValueError("MAT 元素标签截断")
    word, size = struct.unpack_from("<II", view, pos)
    if word >> 16:
        kind, length = word & 0xFFFF, word >> 16
        if length > 4:
            raise ValueError("MAT 短标签长度异常")
        return kind, view[pos + 4:pos + 4 + length], pos + 8
    end = pos + 8 + size
    if end > len(view):
        raise ValueError("MAT 元素数据截断")
    return word, view[pos + 8:end], pos + 8 + ((size + 7) // 8) * 8


def _matrix(blob: bytes, *, values: bool) -> tuple[str, np.ndarray | None]:
    kind, matrix, end = _element(blob, 0)
    if kind != _MI_MATRIX or end != len(blob):
        raise ValueError("仅支持单个 miMATRIX 元素")
    kind, flags, pos = _element(matrix, 0)
    if kind != _MI_UINT32 or len(flags) != 8:
        raise ValueError("MAT 数组 flags 异常")
    flag_word = struct.unpack_from("<I", flags, 0)[0]
    mx_class = flag_word & 0xFF
    if flag_word & 0x0800:
        raise ValueError("不支持复数数组")
    kind, dimensions, pos = _element(matrix, pos)
    if kind != _MI_INT32 or len(dimensions) < 8 or len(dimensions) % 4:
        raise ValueError("MAT 尺寸异常")
    shape = struct.unpack("<" + "i" * (len(dimensions) // 4), dimensions)
    if len(shape) != 2 or any(x < 0 for x in shape):
        raise ValueError("仅支持二维数组")
    kind, name_bytes, pos = _element(matrix, pos)
    if kind != _MI_INT8:
        raise ValueError("MAT 变量名类型异常")
    name = bytes(name_bytes).decode("ascii")
    kind, real, pos = _element(matrix, pos)
    if pos != len(matrix):
        raise ValueError(f"变量 {name} 有未解析数据")
    if mx_class == 6 and kind == _MI_DOUBLE:
        dtype = np.dtype("<f8")
    elif mx_class == 9 and kind == _MI_UINT8:
        dtype = np.dtype("u1")
    else:
        raise ValueError(f"变量 {name} 类型不受支持: class={mx_class}, mi={kind}")
    if len(real) != shape[0] * shape[1] * dtype.itemsize:
        raise ValueError(f"变量 {name} 数据字节数与尺寸不符")
    if not values:
        return name, None
    return name, np.frombuffer(real, dtype=dtype).reshape(shape, order="F").copy(order="C")


def _read_compressed(path: Path, offset: int, length: int, values: bool):
    with path.open("rb") as handle:
        handle.seek(offset)
        compressed = handle.read(length)
    if len(compressed) != length:
        raise ValueError("归档压缩块截断")
    return _matrix(zlib.decompress(compressed), values=values)


@lru_cache(maxsize=2)
def _index(path_text: str, size: int, mtime_ns: int):
    """缓存变量偏移和小型元数据，不缓存任何信号矩阵。"""
    path = Path(path_text)
    mapping: dict[str, tuple[int, int]] = {}
    meta = None
    with path.open("rb") as handle:
        header = handle.read(128)
        if not header.startswith(b"MATLAB 5.0 MAT-file") or header[126:128] != b"IM":
            raise ValueError("只接受 MATLAB 写出的 little-endian MAT v7 文件")
        while True:
            tag = handle.read(8)
            if not tag:
                break
            if len(tag) != 8:
                raise ValueError("MAT 顶层标签截断")
            kind, length = struct.unpack("<II", tag)
            if kind != _MI_COMPRESSED:
                raise ValueError(f"顶层元素类型 {kind} 不受支持")
            offset = handle.tell()
            # MATLAB 的 miCOMPRESSED 顶层元素紧接下一标签，不补齐八字节。
            name, _ = _read_compressed(path, offset, length, values=False)
            if name in mapping:
                raise ValueError(f"重复变量: {name}")
            mapping[name] = (offset, length)
            if name == "archive_metadata_json":
                _, array = _read_compressed(path, offset, length, values=True)
                meta = json.loads(array.ravel(order="F").tobytes().decode("utf-8"))
            handle.seek(offset + length)
    if meta is None or meta.get("archive_version") != 1:
        raise ValueError("归档元数据缺失或版本不兼容")
    if set(mapping) != set(meta["channels"]) | {"archive_metadata_json"}:
        raise ValueError("归档通道与元数据不一致")
    return mapping, meta


def _get_index(path: Path):
    stat = path.stat()
    return _index(str(path.resolve()), stat.st_size, stat.st_mtime_ns)


def metadata(run: str | Path) -> dict:
    """读取版本、源字段、通道尺寸和采样元数据。"""
    return _get_index(_mat_path(run))[1]


def channel_names(run: str | Path) -> tuple[str, ...]:
    return tuple(metadata(run)["channels"])


def read_channel(run: str | Path, name: str) -> np.ndarray:
    """仅解压请求的一个 float64 通道，返回原二维 MATLAB 形状。"""
    path = _mat_path(run)
    mapping, meta = _get_index(path)
    name = name.removesuffix(".bin")
    if name not in meta["channels"]:
        raise KeyError(name)
    parsed_name, array = _read_compressed(path, *mapping[name], values=True)
    if parsed_name != name or array.dtype != np.dtype("float64"):
        raise ValueError(f"归档通道 {name} 类型或名称异常")
    if list(array.shape) != meta["channels"][name]["size"]:
        raise ValueError(f"归档通道 {name} 尺寸与元数据不符")
    return array


def load(run: str | Path, name: str) -> np.ndarray:
    """读取规范归档；旧组自动回退到 arrays.json + .bin。"""
    path = Path(run)
    key = name.removesuffix(".bin")
    if _mat_path(path).is_file():
        return read_channel(path, key)
    if path.suffix.lower() == ".mat":
        raise FileNotFoundError(path)
    sizes = json.loads((path / "arrays.json").read_text(encoding="utf-8"))
    shape = tuple(sizes[key])
    array = np.fromfile(path / f"{key}.bin", dtype="<f8")
    if array.size != int(np.prod(shape)):
        raise ValueError(f"旧 BIN {key} 尺寸不符")
    return array.reshape(shape)


def materialize_legacy(run: str | Path, temp: str | Path) -> Path:
    """将规范归档的 11 路日志临时写为旧 BIN 接口；拒绝覆盖。"""
    source, target = Path(run), Path(temp)
    if not has_records(source):
        raise ValueError("源目录没有规范数值归档")
    if source.resolve() == target.resolve():
        raise ValueError("临时目录不能是源目录")
    target.mkdir(parents=True, exist_ok=True)
    if (target / "arrays.json").exists() or any((target / f"{n}.bin").exists() for n in LEGACY_NAMES):
        raise FileExistsError("临时目录含旧格式输出，拒绝覆盖")
    shapes = {}
    for name in LEGACY_NAMES:
        array = read_channel(source, name)
        array.tofile(target / f"{name}.bin")
        shapes[name] = list(array.shape)
    (target / "arrays.json").write_text(json.dumps(shapes, ensure_ascii=False), encoding="utf-8")
    return target

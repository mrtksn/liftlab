"""Pinned browser runtime and weights, downloaded once into the Mac user's cache."""
import hashlib
from pathlib import Path
import subprocess
import tempfile
import threading

CACHE = Path.home() / "Library/Caches/RoomSurvey/depth-v1"
ORT = "https://cdn.jsdelivr.net/npm/onnxruntime-web@1.22.0/dist/"
MODEL = "https://huggingface.co/onnx-community/depth-anything-v2-small/resolve/4472b7362082ad9968fee890ca0f1e5aca36b93d/onnx/model_quantized.onnx"
FILES = {
    "ort.wasm.min.mjs": (ORT+"ort.wasm.min.mjs", "6ef726f355b791129f7ddf3f62e7877a16b9b777abe87004b8ab306e6b9cf265"),
    "ort-wasm-simd-threaded.mjs": (ORT+"ort-wasm-simd-threaded.mjs", "30dd851d9c00622940500f71ddd2ff8820c5cb65270816080175b958705385a8"),
    "ort-wasm-simd-threaded.wasm": (ORT+"ort-wasm-simd-threaded.wasm", "71aef04959c5c1b6de461b6538e2058e306610034a85aad2742d0c7fd4533fe4"),
    "depth.onnx": (MODEL,"fcf51f1b230362b28690bb9d1809bf0431f29cad20534e3f589bd7285547f20d"),
}
lock = threading.Lock()


def asset(name):
    url,digest = FILES[name]
    with lock:
        CACHE.mkdir(parents=True,exist_ok=True)
        path = CACHE / name
        if path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == digest:
            return path
        with tempfile.NamedTemporaryFile(dir=CACHE, delete=False) as tmp:
            stage = Path(tmp.name)
        try:
            subprocess.run(["curl","--fail","--silent","--show-error","--location","--retry","2",
                            "--max-time","120",url,"--output",tmp.name],check=True)
            data = Path(tmp.name).read_bytes()
            if hashlib.sha256(data).hexdigest() != digest:
                raise ValueError("Depth asset checksum mismatch. Retry the download.")
            stage.replace(path)
        finally:
            stage.unlink(missing_ok=True)
        return path

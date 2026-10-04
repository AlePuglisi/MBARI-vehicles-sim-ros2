#!/usr/bin/env python3
"""Download YOLO models from the Hugging Face Hub into this directory."""
import argparse
from pathlib import Path

from huggingface_hub import hf_hub_download

MODELS_DIR = Path(__file__).resolve().parent

MODEL_OPTIONS = {
    "Megalodon": ("FathomNet/megalodon", "mbari-megalodon-yolov8x.pt"),
    "MBARI 315k": ("FathomNet/MBARI-315k-yolov8", "mbari_315k_yolov8.pt"),
    "Trash Detector": ("FathomNet/trash-detector","trash_mbari_09072023_640imgsz_50epochs_yolov8.pt"),
}


def download(name: str) -> Path:
    repo_id, filename = MODEL_OPTIONS[name]
    target = MODELS_DIR / filename

    if target.is_file() and not target.is_symlink():
        print(f"Already present: {target}")
        return target

    # Remove a broken symlink left over from a cache copy
    if target.is_symlink():
        target.unlink()

    path = hf_hub_download(
        repo_id=repo_id,
        filename=filename,
        local_dir=str(MODELS_DIR),  # real file, no cache/symlinks
    )
    print(f"Downloaded: {path}")
    return Path(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "names",
        nargs="*",
        default=["Megalodon"],
        help=f"Models to download (default: Megalodon). Options: {list(MODEL_OPTIONS)}",
    )
    parser.add_argument("--all", action="store_true", help="Download every model")
    args = parser.parse_args()

    names = list(MODEL_OPTIONS) if args.all else args.names
    for name in names:
        if name not in MODEL_OPTIONS:
            parser.error(f"Unknown model '{name}'. Options: {list(MODEL_OPTIONS)}")
        download(name)

    print("Model Downloaded")


if __name__ == "__main__":
    main()
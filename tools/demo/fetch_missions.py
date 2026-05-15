"""
Fetch DUNEX microSWIFT mission files from the ERDC THREDDS server (no auth required).

Dataset citation:
    Rainville, E. J.; Thomson, Jim; Moulton, Melissa; Derakhti, Morteza (2023).
    Measurements of nearshore waves through coherent arrays of free-drifting wave buoys
    [Dataset]. Dryad. https://doi.org/10.5061/dryad.hx3ffbgk0

Usage:
    python tools/demo/fetch_missions.py 34             # download mission_34.nc
    python tools/demo/fetch_missions.py 34 2 9         # download several
    python tools/demo/fetch_missions.py --all          # download every mission
"""

import argparse
import sys
from pathlib import Path

import requests

THREDDS_BASE = (
    "https://chldata.erdc.dren.mil/thredds/fileServer"
    "/frf/projects/Dunex/UW_drifters"
)
DEFAULT_OUT = Path(__file__).parent / "data"
CHUNK = 1 << 16  # 64 KiB

# All known mission numbers from the dataset
ALL_MISSIONS = [
    1, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20,
    21, 22, 23, 24, 25, 26, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38,
    39, 40, 41, 42, 43, 44, 45, 46, 48, 50, 51, 54, 56, 58, 59, 60, 63,
    66, 67, 68, 69, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81,
]


def download(mission: int, out_dir: Path) -> None:
    filename = f"mission_{mission}.nc"
    dest = out_dir / filename
    url = f"{THREDDS_BASE}/{filename}"

    if dest.exists():
        print(f"  skip  {filename} (already downloaded)")
        return

    out_dir.mkdir(parents=True, exist_ok=True)

    with requests.get(url, stream=True, timeout=60) as r:
        r.raise_for_status()
        total = int(r.headers.get("content-length", 0))
        downloaded = 0
        with open(dest, "wb") as f:
            for chunk in r.iter_content(chunk_size=CHUNK):
                f.write(chunk)
                downloaded += len(chunk)
                if total:
                    print(f"\r  {filename}  {downloaded / total * 100:5.1f}%",
                          end="", flush=True)
        print(f"\r  done  {filename} ({downloaded / 1e6:.2f} MB)")


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("missions", nargs="*", type=int, metavar="N",
                        help="mission numbers to download")
    parser.add_argument("--all", action="store_true",
                        help="download all missions")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT,
                        help=f"output directory (default: {DEFAULT_OUT})")
    args = parser.parse_args()

    if args.all:
        missions = ALL_MISSIONS
    elif args.missions:
        missions = args.missions
    else:
        parser.print_help()
        sys.exit(0)

    invalid = [m for m in missions if m not in ALL_MISSIONS]
    if invalid:
        print(f"Warning: unknown mission numbers: {invalid}", file=sys.stderr)

    for m in missions:
        download(m, args.out)


if __name__ == "__main__":
    main()

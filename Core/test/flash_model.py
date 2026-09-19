"""
Flash one exported model to the FVC's model slot over CAN.

    python deploy/tools/flash_model.py deploy/build/skidpad.fsaepol
    python deploy/tools/flash_model.py deploy/build/skidpad.fsaepol --dry-run
    python deploy/tools/flash_model.py deploy/build/skidpad.fsaepol --port SWD   # ST-Link instead

Steps:
  1. check the file (magic, CRC, size);
  2. send GopherCAN's bootloader-start message to the FVC (gophercan-lib's
     bootloader_can_bridge.exe, Windows only; skip with --no-bootloader-start);
  3. write the file to flash sector 7 (0x08060000) with STM32CubeProgrammer,
     verify it, and restart the FVC.
Only sector 7 is written; the firmware in sectors 0-6 is untouched.
"""
import argparse
import os
import shutil
import subprocess
import sys

import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fsaepol  # noqa: E402

SLOT_ADDR = 0x08060000      # STM32F446 sector 7; must match gds_default_config()
SLOT_SIZE = 0x20000
MODULE = "FVC"


def start_bootloader(gcan_lib, car):
    cfg_path = os.path.join(gcan_lib, "network_autogen", "configs", f"{car}.yaml")
    with open(cfg_path) as f:
        module_id = yaml.safe_load(f)["modules"][MODULE]["id"]
    bridge = os.path.join(gcan_lib, "gcan_bootloader", "src", "bootloader_can_bridge.exe")
    print(f"Starting the CAN bootloader on the {MODULE} (module ID {module_id})")
    result = subprocess.run([bridge, str(module_id)], capture_output=True, text=True)
    print(result.stdout.strip())
    if result.returncode != 0:
        sys.exit("Failed to start the CAN bootloader")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("model_file")
    ap.add_argument("--port", default="CAN", help="CAN (default) or SWD")
    ap.add_argument("--gcan-lib", default="../gophercan-lib")
    ap.add_argument("--car", default="go4-26", help="network config naming the FVC's module ID")
    ap.add_argument("--no-bootloader-start", action="store_true",
                    help="The FVC is already in its bootloader (or using SWD)")
    ap.add_argument("--cli", default="STM32_Programmer_CLI")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    h = fsaepol.read(args.model_file)[0]
    nbytes = os.path.getsize(args.model_file)
    if nbytes > SLOT_SIZE:
        sys.exit(f"{nbytes} bytes does not fit the {SLOT_SIZE}-byte slot")
    print(f"{args.model_file}: {h['event']} model '{h['name']}', {nbytes} bytes -> 0x{SLOT_ADDR:08X}")
    print("The FVC will only run it with the mission selector on "
          f"{h['event']} (other missions report a wrong-event fault).")

    port = args.port.upper()
    conn = ["-c", f"port={port}"] + (["br=1000"] if port == "CAN" else [])
    cmd = [args.cli, *conn, "-d", args.model_file, f"0x{SLOT_ADDR:08X}", "-v", "-g", "0x08000000"]
    use_bridge = port == "CAN" and not args.no_bootloader_start
    if args.dry_run:
        if use_bridge:
            print(f"  (would start the CAN bootloader via {args.gcan_lib}, car {args.car})")
        print("  " + " ".join(cmd))
        return
    if use_bridge:
        start_bootloader(args.gcan_lib, args.car)
    if shutil.which(args.cli) is None:
        sys.exit(f"{args.cli} not found; install STM32CubeProgrammer or use --cli")
    print("  " + " ".join(cmd))
    subprocess.run(cmd, check=True)


if __name__ == "__main__":
    main()

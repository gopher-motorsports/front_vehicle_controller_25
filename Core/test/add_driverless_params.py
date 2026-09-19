"""
Add the driverless (ds*) parameters and groups to a GopherCAN network config,
and print the Gopher Sense buckets the FVC needs to send them.

    python deploy/tools/add_driverless_params.py \\
        ../gophercan-lib/network_autogen/configs/go4-26.yaml \\
        --out ../gophercan-lib/network_autogen/configs/go4-26-ds.yaml

Then regenerate with `python autogen.py configs/go4-26-ds.yaml`, add the printed
buckets to front_vehicle_controller_*_config.yaml, and rerun Gopher Sense's
autogen. The input file is not modified. Parameter IDs are taken after the
highest existing ID; group IDs from --group-base upwards, skipping used ones.
"""
import argparse
import sys

import yaml

# name: (type, scale, offset, unit, motec name)
PARAMS = {
    "dsSteerCmd_deg":           ("FLOATING", 0.01, -100.0, "deg", "DS Steer Cmd"),
    "dsDriveTorqueCmd_Nm":      ("FLOATING", 0.1, 0.0, "Nm", "DS Drive Torque Cmd"),
    "dsBrakeTorqueFrontCmd_Nm": ("FLOATING", 0.1, 0.0, "Nm", "DS Brake Torque Front Cmd"),
    "dsBrakeTorqueRearCmd_Nm":  ("FLOATING", 0.1, 0.0, "Nm", "DS Brake Torque Rear Cmd"),
    "dsState_state":            ("UNSIGNED8", 1, 0, "", "DS State"),
    "dsFault_state":            ("UNSIGNED8", 1, 0, "", "DS Fault"),
    "dsCrossings_state":        ("UNSIGNED8", 1, 0, "", "DS Timing Lines Crossed"),
    "dsFlags_state":            ("UNSIGNED8", 1, 0, "", "DS Flags"),
    "dsConesUsed_state":        ("UNSIGNED8", 1, 0, "", "DS Cones Used"),
    "dsModelEvent_state":       ("UNSIGNED8", 1, 0, "", "DS Model Event"),
    "dsPerceptionAge_ms":       ("UNSIGNED16", 1, 0, "ms", "DS Perception Age"),
    "dsVxEst_mps":              ("FLOATING", 0.01, -50.0, "m/s", "DS Vx Estimate"),
    "dsVyEst_mps":              ("FLOATING", 0.001, -30.0, "m/s", "DS Vy Estimate"),
    "dsYawRate_degps":          ("FLOATING", 0.01, -300.0, "deg/s", "DS Yaw Rate"),
    "dsModelCreated_unix":      ("UNSIGNED32", 1, 0, "s", "DS Model Created"),
}

# (bucket name, rate) -> groups of (name, start, length)
GROUPS = [
    ("commands", [("dsSteerCmd_deg", 0, 2), ("dsDriveTorqueCmd_Nm", 2, 2),
                  ("dsBrakeTorqueFrontCmd_Nm", 4, 2), ("dsBrakeTorqueRearCmd_Nm", 6, 2)]),
    ("status", [("dsState_state", 0, 1), ("dsFault_state", 1, 1), ("dsCrossings_state", 2, 1),
                ("dsFlags_state", 3, 1), ("dsPerceptionAge_ms", 4, 2), ("dsConesUsed_state", 6, 1),
                ("dsModelEvent_state", 7, 1)]),
    ("status", [("dsVxEst_mps", 0, 2), ("dsVyEst_mps", 2, 2), ("dsYawRate_degps", 4, 2)]),
    ("status", [("dsModelCreated_unix", 0, 4)]),
]
BUCKETS = {"commands": ("driverless_commands", 50), "status": ("driverless_status", 10)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("config")
    ap.add_argument("--out", required=True)
    ap.add_argument("--group-base", type=lambda s: int(s, 0), default=0x305)
    args = ap.parse_args()

    text = open(args.config).read()
    cfg = yaml.safe_load(text)
    params, groups = cfg["parameters"], cfg["groups"]
    clash = [n for n in PARAMS if n in params]
    if clash:
        sys.exit(f"already defined: {clash}")
    next_id = max(p["id"] for p in params.values()) + 1
    used_groups = {g["id"] for g in groups}

    ptext = []
    for i, (name, (typ, scale, offset, unit, motec)) in enumerate(PARAMS.items()):
        ptext.append(
            f"    {name}:\n        id: {next_id + i}\n        motec_name: {motec}\n"
            f"        unit: {unit if unit else chr(39) * 2}\n        type: {typ}\n"
            f"        encoding: MSB\n        scale: {scale}\n        offset: {offset}\n")
    gtext, gid = [], args.group_base
    for _, items in GROUPS:
        while gid in used_groups:
            gid += 1
        used_groups.add(gid)
        body = ",\n".join(f"            {{name: {n}, start: {s}, length: {l}}}" for n, s, l in items)
        gtext.append(f"    {{\n        id: {hex(gid)},\n        parameters: [\n{body}\n        ]\n    }},\n")

    g_start = text.index("\ngroups:")
    new_text = text[:g_start] + "\n" + "".join(ptext) + text[g_start:]
    end = new_text.rindex("]", 0, new_text.index("\ncommands:"))
    new_text = new_text[:end] + "".join(gtext) + new_text[end:]

    check = yaml.safe_load(new_text)
    ids = [p["id"] for p in check["parameters"].values()]
    assert len(ids) == len(set(ids)), "duplicate parameter IDs"
    gids = [g["id"] for g in check["groups"]]
    assert len(gids) == len(set(gids)), "duplicate group IDs"
    with open(args.out, "w") as f:
        f.write(new_text)
    print(f"Wrote {args.out}: {len(PARAMS)} parameters (IDs {next_id}-{next_id + len(PARAMS) - 1}), "
          f"{len(GROUPS)} groups")

    print("\nAdd to front_vehicle_controller_*_config.yaml under buckets:\n")
    for kind, (bucket, hz) in BUCKETS.items():
        names = [n for k, items in GROUPS if k == kind for n, _, _ in items]
        print(f"    {bucket}:\n        frequency_hz: {hz}\n        parameters:")
        for n in names:
            print(f"            {n}:\n                ADC: NON_ADC\n                sensor: NON_ADC\n"
                  f"                samples_buffered: 1")


if __name__ == "__main__":
    main()

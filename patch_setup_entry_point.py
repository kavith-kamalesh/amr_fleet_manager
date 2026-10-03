"""Add the pibt_fleet_node console script to setup.py (idempotent).
   python3 patch_setup_entry_point.py [path/to/setup.py]"""
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "setup.py"
src = open(path).read()
entry = "'pibt_fleet_node = amr_fleet_manager.pibt_fleet_node:main',"
if "pibt_fleet_node" in src:
    print("setup.py already has pibt_fleet_node")
    sys.exit(0)
anchor = "'safety_fallback = amr_fleet_manager.safety_fallback:main',"
if anchor not in src:
    sys.exit("anchor line not found; add this line to console_scripts by hand:\n    " + entry)
indent = src[:src.index(anchor)].rsplit("\n", 1)[-1]
open(path, "w").write(src.replace(anchor, anchor + "\n" + indent + entry, 1))
print("added pibt_fleet_node entry point to", path)

"""Export an offline scenario through the existing Go engine, without editing it."""
import json
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "output/demo"
with tempfile.TemporaryDirectory(prefix="trackfusion-export-") as temp:
    overlay = Path(temp) / "overlay.json"
    overlay.write_text(json.dumps({"Replace": {
        str(ROOT / "cmd/trackfusion/engine_test.go"):
        str(ROOT / "scripts/demo/export_test.go.txt")
    }}))
    subprocess.run(["go", "test", "-overlay", str(overlay), "./cmd/trackfusion",
                    "-run", "^TestExportRecruiterDemo$", "-count=1", "-v"],
                   cwd=ROOT, env={**os.environ, "TRACKFUSION_DEMO_OUT": str(OUT)}, check=True)

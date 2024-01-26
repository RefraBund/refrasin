import tempfile
import json
from pathlib import Path
import subprocess
from typing import Optional


def generate_kernel_spec(resource_path: Path) -> Path:
    with tempfile.TemporaryDirectory(delete=False) as tempdir:
        spec_dir = Path(tempdir) / "refrasin"
        kernel_file = spec_dir / "kernel.json"

        contents = {
            "argv": [
                
            ],
            "display_name": "RefraSin (using Python.NET)",
            "language": "python"
        }

        kernel_file.write(json.dumps(contents, indent=4))

        return spec_dir


def install_kernel(spec_dir: Path, where: str | Path = "user"):
    args = [
        "jupyter",
        "kernelspec",
        "install",
        spec_dir
    ]
    
    if where:
        if where == "user":
            args.append("--user")
        elif where == "sys":
            args.append("--sys-prefix")
        else:
            args.append("--prefix")
            args.append(f"{where}")
        
    result = subprocess.run(
        args,
        text=True, capture_output=True
    )
    
    print(result.stdout)
    print(result.stderr)
    result.check_returncode()    

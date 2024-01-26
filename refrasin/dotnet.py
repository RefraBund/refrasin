from importlib.resources import files, as_file
import tempfile
import json
from pathlib import Path


def create_resource_dir() -> Path:
    resources = files("refrasin").joinpath("assemblies.zip")
    resource_path = as_file(resources).__enter__()
    return resource_path


def generate_rc_file(resource_path: Path) -> Path:
    with tempfile.NamedTemporaryFile(
            "w",
            prefix="runtimeconfig-", suffix=".json",
            delete=False, encoding="utf8"
    ) as rc_file:
        contents = {
            "runtimeOptions": {
                "additionalProbingPaths": [
                    f"{resource_path}"
                ]
            }
        }

        rc_file.write(json.dumps(contents, indent=4))

        return Path(rc_file.name)


def load_clr():
    resource_dir = create_resource_dir()
    rc_file = generate_rc_file(resource_dir)

    from pythonnet import load
    load(
        "coreclr",
        # runtime_config=rc_file
    )

    from clr import AddReference
    AddReference("RefraSin.Coordinates.dll")
    AddReference("RefraSin.Enumerables.dll")
    AddReference("RefraSin.Graphs.dll")
    AddReference("RefraSin.Storage.dll")
    AddReference("RefraSin.Hdf5Storage.dll")
    AddReference("RefraSin.MaterialData.dll")
    AddReference("RefraSin.ParticleModel.dll")
    AddReference("RefraSin.ProcessModel.dll")
    AddReference("RefraSin.TEPSolver.dll")

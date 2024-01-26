import tempfile
from pathlib import Path
import json

VERSION = "1.0.0"

from .dotnet import load_clr

load_clr()

AddReference("RefraSin.Coordinates.dll")
AddReference("RefraSin.Enumerables.dll")
AddReference("RefraSin.Graphs.dll")
AddReference("RefraSin.Storage.dll")
AddReference("RefraSin.Hdf5Storage.dll")
AddReference("RefraSin.MaterialData.dll")
AddReference("RefraSin.ParticleModel.dll")
AddReference("RefraSin.ProcessModel.dll")
AddReference("RefraSin.TEPSolver.dll")



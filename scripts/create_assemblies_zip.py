import zipfile
from pathlib import Path
from itertools import chain


def unpack_to(file: Path, directory: Path):
    zip_file = zipfile.ZipFile(file)

    zip_file.extractall(directory)


def pack_from(file: Path, directory: Path):
    files = directory.rglob("*")

    zip_file = zipfile.ZipFile(file, "w")


if __name__ == "__main__":
    import sys

    pack_from(sys.argv[2], sys.argv[1])
    
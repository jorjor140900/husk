"""Export this curated branch as ROS2 package husky_vision; preserve external assets.

Only copies source files. Does not build, test, start ROS, enable motion, copy
trained weights, or invent calibration. Existing package updates are backed up.
"""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import shutil
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SOURCE_PACKAGE = 'husky_drivable_area'
PACKAGE = 'husky_vision'


def export(workspace, update=False):
    workspace = Path(workspace).expanduser().resolve()
    package = workspace/'src'/PACKAGE
    if package.is_symlink():
        raise ValueError('refusing a symlink package target')
    if package.exists():
        manifest = package/'package.xml'
        if not update or not manifest.is_file() or ET.parse(manifest).findtext('name') != PACKAGE:
            raise ValueError('existing husky_vision package requires --update-code')
    files = [ROOT/name for name in ('setup.py','setup.cfg','package.xml','README.md','requirements-ros2.txt')]
    for directory in ('husky_drivable_area','road_navigation','husky_link_core','launch','config','docs','resource','ros1','tools'):
        files += [p for p in (ROOT/directory).rglob('*') if p.is_file()
                  and '__pycache__' not in p.parts and p.suffix != '.pyc']
    workspace.mkdir(parents=True,exist_ok=True)
    (workspace/'src').mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.assist-export-',dir=workspace/'src') as temp:
        staged = Path(temp)/PACKAGE
        for source in files:
            relative = str(source.relative_to(ROOT)).replace(SOURCE_PACKAGE,PACKAGE)
            target = staged/relative
            target.parent.mkdir(parents=True,exist_ok=True)
            target.write_text(source.read_text().replace(SOURCE_PACKAGE,PACKAGE))
            target.chmod(source.stat().st_mode & 0o777)
        if package.exists():
            backup = workspace/'.source-backups'/datetime.now(timezone.utc).strftime('submission-%Y%m%dT%H%M%S%fZ')
            backup.parent.mkdir(exist_ok=True)
            (backup.parent/'COLCON_IGNORE').touch()
            shutil.copytree(package,backup,symlinks=True)
            # Only the recognized source package is replaced; models, venv,
            # robot profiles and measured camera files are outside this path.
            shutil.rmtree(package)
            print('Previous source saved:',backup)
        staged.rename(package)
    (workspace/'robot').mkdir(exist_ok=True)
    print('Exported:',package)
    print('Use your trained SegFormer, measured calibration/mount and robot profile. See docs/RUN.md.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace',required=True,type=Path)
    parser.add_argument('--update-code',action='store_true')
    parser.add_argument('--update-detector',action='store_true',help='Compatibility option; this curated export always includes its detector')
    args=parser.parse_args()
    export(args.workspace,args.update_code)

if __name__=='__main__':main()

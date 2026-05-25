# -*- mode: python ; coding: utf-8 -*-
# PyInstaller spec for IPS Visualizer
# macOS: produces dist/IPS Visualizer.app
# Windows: produces dist/IPS Visualizer.exe  (single file)

import sys

block_cipher = None

a = Analysis(
    ['plot_positions.py'],
    pathex=[],
    binaries=[],
    datas=[],
    hiddenimports=[
        'serial.tools.list_ports',
        'serial.tools.list_ports_common',
        'serial.tools.list_ports_posix',
        'serial.tools.list_ports_windows',
        'scipy.optimize',
        'scipy.special._ufuncs_cxx',
        'scipy._lib.messagestream',
        'numpy.core._methods',
        'numpy.lib.format',
    ],
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=['matplotlib', 'IPython', 'PIL', 'PyQt5', 'wx'],
    win_no_prefer_redirects=False,
    win_private_assemblies=False,
    cipher=block_cipher,
    noarchive=False,
)

pyz = PYZ(a.pure, a.zipped_data, cipher=block_cipher)

if sys.platform == 'darwin':
    exe = EXE(
        pyz, a.scripts, [],
        exclude_binaries=True,
        name='IPS Visualizer',
        debug=False,
        bootloader_ignore_signals=False,
        strip=False,
        upx=True,
        console=False,   # no terminal window on macOS
        icon=None,
    )
    coll = COLLECT(
        exe, a.binaries, a.zipfiles, a.datas,
        strip=False,
        upx=True,
        upx_exclude=[],
        name='IPS Visualizer',
    )
    app = BUNDLE(
        coll,
        name='IPS Visualizer.app',
        icon=None,
        bundle_identifier='com.ips.visualizer',
        info_plist={
            'CFBundleShortVersionString': '1.0.0',
            'CFBundleVersion':            '1.0.0',
            'NSHighResolutionCapable':    True,
            'NSRequiresAquaSystemAppearance': False,
        },
    )
else:
    # Windows — single-file executable
    exe = EXE(
        pyz, a.scripts, a.binaries, a.zipfiles, a.datas, [],
        name='IPS Visualizer',
        debug=False,
        bootloader_ignore_signals=False,
        strip=False,
        upx=True,
        upx_exclude=[],
        runtime_tmpdir=None,
        console=False,   # no black console window
        icon=None,
        onefile=True,
    )

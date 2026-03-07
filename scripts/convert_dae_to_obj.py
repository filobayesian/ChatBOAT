#!/usr/bin/env python3
"""Convert DAE (COLLADA) mesh files to OBJ format for Stonefish.

Requires: pip install trimesh
Optional: pip install pycollada (for better DAE support)

Usage: python3 scripts/convert_dae_to_obj.py
"""
import os
import glob

def convert_dae_to_obj():
    try:
        import trimesh
    except ImportError:
        print("ERROR: trimesh not installed. Run: pip install trimesh pycollada")
        return False

    dae_dir = os.path.join(os.path.dirname(__file__), '..',
                           'src', 'chatboat_stonefish', 'data', 'alpha5')
    dae_files = glob.glob(os.path.join(dae_dir, '*.dae'))

    if not dae_files:
        print(f"No DAE files found in {dae_dir}")
        return False

    for dae_path in dae_files:
        obj_path = dae_path.replace('.dae', '.obj')
        print(f"Converting: {os.path.basename(dae_path)} -> {os.path.basename(obj_path)}")
        try:
            scene = trimesh.load(dae_path, force='scene')
            if isinstance(scene, trimesh.Scene):
                mesh = scene.dump(concatenate=True)
            else:
                mesh = scene
            mesh.export(obj_path, file_type='obj')
            print(f"  OK ({len(mesh.vertices)} vertices, {len(mesh.faces)} faces)")
        except Exception as e:
            print(f"  FAILED: {e}")
            return False

    print(f"\nConverted {len(dae_files)} files successfully.")
    return True

if __name__ == '__main__':
    convert_dae_to_obj()

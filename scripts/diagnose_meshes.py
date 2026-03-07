#!/usr/bin/env python3
"""Diagnose OBJ mesh coordinate frames for Stonefish arm links.

Loads each Alpha 5 OBJ mesh and prints bounding box, centroid, and extents
so we can determine what origin transforms are needed in the Stonefish scenario.

Also loads the original DAE files and compares to see if the conversion lost transforms.

Usage: python3 scripts/diagnose_meshes.py
"""
import os
import sys

def main():
    try:
        import trimesh
    except ImportError:
        print("ERROR: trimesh not installed. Run: pip install trimesh pycollada")
        return

    data_dir = os.path.join(os.path.dirname(__file__), '..',
                            'src', 'chatboat_stonefish', 'data', 'alpha5')

    # Mesh-to-link mapping from reference URDF
    mesh_links = {
        'RS1-1010': {'link': 'arm_base',      'urdf_visual_rpy': (0, 0, 0)},
        'M3-INLINE': {'link': 'arm_inline',    'urdf_visual_rpy': (0, 0, 0)},
        'M2-1-1':   {'link': 'arm_connector',  'urdf_visual_rpy': (0, 0, 0)},
        'M2':       {'link': 'arm_upper',      'urdf_visual_rpy': (0, 0, 0)},
        'M2-1-3':   {'link': 'arm_elbow',      'urdf_visual_rpy': (0, 0, 0)},
        'M1':       {'link': 'arm_wrist',      'urdf_visual_rpy': (0, -1.5707, 0)},
        'RS1-124':  {'link': 'arm_tcp_adapter', 'urdf_visual_rpy': (0, -1.5707, 0)},
    }

    print("=" * 80)
    print("ALPHA 5 ARM MESH DIAGNOSTICS")
    print("=" * 80)

    for name, info in mesh_links.items():
        obj_path = os.path.join(data_dir, f'{name}.obj')
        dae_path = os.path.join(data_dir, f'{name}.dae')

        print(f"\n--- {name} (link: {info['link']}) ---")

        # Analyze OBJ
        if os.path.exists(obj_path):
            try:
                mesh = trimesh.load(obj_path, force='mesh')
                bb = mesh.bounding_box.bounds
                centroid = mesh.centroid
                extents = mesh.bounding_box.extents
                print(f"  OBJ:")
                print(f"    Vertices: {len(mesh.vertices)}, Faces: {len(mesh.faces)}")
                print(f"    Bounding box min: [{bb[0][0]:.4f}, {bb[0][1]:.4f}, {bb[0][2]:.4f}]")
                print(f"    Bounding box max: [{bb[1][0]:.4f}, {bb[1][1]:.4f}, {bb[1][2]:.4f}]")
                print(f"    Centroid:         [{centroid[0]:.4f}, {centroid[1]:.4f}, {centroid[2]:.4f}]")
                print(f"    Extents (WxHxD):  [{extents[0]:.4f}, {extents[1]:.4f}, {extents[2]:.4f}]")
            except Exception as e:
                print(f"  OBJ LOAD ERROR: {e}")
        else:
            print(f"  OBJ: NOT FOUND")

        # Analyze DAE (scene with transforms)
        if os.path.exists(dae_path):
            try:
                scene = trimesh.load(dae_path, force='scene')
                if isinstance(scene, trimesh.Scene):
                    # Show scene graph info
                    print(f"  DAE Scene:")
                    print(f"    Nodes: {list(scene.graph.nodes)[:5]}...")
                    # Get the full scene bounds
                    dumped = scene.dump(concatenate=True)
                    bb = dumped.bounding_box.bounds
                    centroid = dumped.centroid
                    extents = dumped.bounding_box.extents
                    print(f"    Dumped vertices: {len(dumped.vertices)}, Faces: {len(dumped.faces)}")
                    print(f"    Bounding box min: [{bb[0][0]:.4f}, {bb[0][1]:.4f}, {bb[0][2]:.4f}]")
                    print(f"    Bounding box max: [{bb[1][0]:.4f}, {bb[1][1]:.4f}, {bb[1][2]:.4f}]")
                    print(f"    Centroid:         [{centroid[0]:.4f}, {centroid[1]:.4f}, {centroid[2]:.4f}]")
                    print(f"    Extents (WxHxD):  [{extents[0]:.4f}, {extents[1]:.4f}, {extents[2]:.4f}]")

                    # Check for scene-level transforms
                    for geom_name, geom in scene.geometry.items():
                        transform = scene.graph.get(geom_name)
                        if transform is not None:
                            mat = transform[0] if isinstance(transform, tuple) else transform
                            print(f"    Geom '{geom_name}' transform:\n{mat}")
                else:
                    mesh = scene
                    print(f"  DAE loaded as single mesh (no scene graph)")
            except Exception as e:
                print(f"  DAE LOAD ERROR: {e}")
        else:
            print(f"  DAE: NOT FOUND")

        # Compare
        print(f"  URDF visual origin rpy: {info['urdf_visual_rpy']}")

    # Also check BlueROV2 meshes for reference
    br2_dir = os.path.join(data_dir, '..', 'bluerov2')
    print(f"\n{'=' * 80}")
    print("BLUEROV2 REFERENCE MESHES (known working)")
    print("=" * 80)
    for name in ['bluerov2', 'bluerov2_phy']:
        obj_path = os.path.join(br2_dir, f'{name}.obj')
        if os.path.exists(obj_path):
            try:
                mesh = trimesh.load(obj_path, force='mesh')
                bb = mesh.bounding_box.bounds
                centroid = mesh.centroid
                extents = mesh.bounding_box.extents
                print(f"\n--- {name} ---")
                print(f"  Vertices: {len(mesh.vertices)}, Faces: {len(mesh.faces)}")
                print(f"  Bounding box min: [{bb[0][0]:.4f}, {bb[0][1]:.4f}, {bb[0][2]:.4f}]")
                print(f"  Bounding box max: [{bb[1][0]:.4f}, {bb[1][1]:.4f}, {bb[1][2]:.4f}]")
                print(f"  Centroid:         [{centroid[0]:.4f}, {centroid[1]:.4f}, {centroid[2]:.4f}]")
                print(f"  Extents (WxHxD):  [{extents[0]:.4f}, {extents[1]:.4f}, {extents[2]:.4f}]")
            except Exception as e:
                print(f"  ERROR: {e}")

if __name__ == '__main__':
    main()

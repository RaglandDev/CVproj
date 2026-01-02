"""
=====================================================================
 * MIT License
 * 
 * Copyright (c) 2025 Omni Instrument Inc.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ===================================================================== 
"""

from __future__ import annotations
import numpy as np
import open3d as o3d
from dataclasses import dataclass


@dataclass
class SDFMetrics:
    mean_abs_error: float
    rmse: float
    max_error: float


class SDFEvaluator:
    """
    Voxelizes 2 meshes and compares their Signed Distance Fields (SDF).
    """

    def __init__(self):
        self.voxel_size = 0.25

    # ------------------------------------------------------------
    # Visualization
    # ------------------------------------------------------------
    def visualize_meshes(self, mesh_a_path: str, mesh_b_path: str):
        mesh_a = o3d.io.read_triangle_mesh(mesh_a_path)
        mesh_b = o3d.io.read_triangle_mesh(mesh_b_path)

        mesh_a.paint_uniform_color([1, 0, 0])  # GT = red
        mesh_b.paint_uniform_color([0, 1, 0])  # Recon = green

        mesh_a.compute_vertex_normals()
        mesh_b.compute_vertex_normals()

        o3d.visualization.draw_geometries(
            [mesh_a, mesh_b],
            window_name="Mesh Overlay (GT=Red, Recon=Green)",
            width=1200,
            height=800,
            mesh_show_back_face=True
        )

    # ------------------------------------------------------------
    # Compute SDF at voxel centers
    # ------------------------------------------------------------
    def mesh_to_sdf(self, mesh: o3d.geometry.TriangleMesh,
                    grid_points: np.ndarray) -> np.ndarray:

        mesh.compute_vertex_normals()
        scene = o3d.t.geometry.RaycastingScene()
        scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))

        p = o3d.core.Tensor(grid_points, dtype=o3d.core.Dtype.Float32)

        unsigned = scene.compute_distance(p).numpy()
        occupancy = scene.compute_occupancy(p).numpy()

        signed = unsigned * ((occupancy < 0.5) * 2 - 1)
        return signed

    # ------------------------------------------------------------
    # Main evaluation
    # ------------------------------------------------------------
    def evaluate(self, mesh_a_path: str, mesh_b_path: str) -> SDFMetrics:
        mesh_a = o3d.io.read_triangle_mesh(mesh_a_path)
        mesh_b = o3d.io.read_triangle_mesh(mesh_b_path)

        min_bound = np.minimum(mesh_a.get_min_bound(), mesh_b.get_min_bound())
        max_bound = np.maximum(mesh_a.get_max_bound(), mesh_b.get_max_bound())

        xs = np.arange(min_bound[0], max_bound[0], self.voxel_size)
        ys = np.arange(min_bound[1], max_bound[1], self.voxel_size)
        zs = np.arange(min_bound[2], max_bound[2], self.voxel_size)

        grid = np.stack(np.meshgrid(xs, ys, zs, indexing="ij"), axis=-1)
        grid_points = grid.reshape(-1, 3)

        print(f"Evaluating {len(grid_points)} voxel centers...")

        sdf_a = self.mesh_to_sdf(mesh_a, grid_points)
        sdf_b = self.mesh_to_sdf(mesh_b, grid_points)

        diff = sdf_a - sdf_b

        mae = float(np.mean(np.abs(diff)))
        rmse = float(np.sqrt(np.mean(diff * diff)))
        max_err = float(np.max(np.abs(diff)))

        return SDFMetrics(mean_abs_error=mae, rmse=rmse, max_error=max_err)


# ------------------------------------------------------------------------------
# CLI
# ------------------------------------------------------------------------------
if __name__ == "__main__":
    import os
    import argparse

    home = os.path.expanduser("~")
    default_gt_path = os.path.join(home, "dataset", "meshes", "omni_mesh.stl")
    default_recon_path = os.path.join(home, "output", "mesh.stl")

    parser = argparse.ArgumentParser("SDF comparison evaluator")
    parser.add_argument("--gt", default=default_gt_path)
    parser.add_argument("--recon", default=default_recon_path)
    parser.add_argument("--view", action="store_true",
                        help="Visualize meshes before evaluation")

    args = parser.parse_args()

    evaluator = SDFEvaluator()

    # Optional visualization step
    if args.view:
        evaluator.visualize_meshes(args.gt, args.recon)

    metrics = evaluator.evaluate(args.gt, args.recon)

    print("\n===== Volumetric SDF Metrics =====")
    print(f"Mean Abs Error : {metrics.mean_abs_error:.4f} m")
    print(f"RMSE          : {metrics.rmse:.4f} m")
    print(f"Max Error     : {metrics.max_error:.4f} m")
    print("=================================\n")

    # ------------------------------------------------------------
    # Save metrics to metrics.txt in the reconstruction directory
    # ------------------------------------------------------------
    recon_dir = os.path.dirname(args.recon)
    metrics_path = os.path.join(recon_dir, "metrics.txt")

    with open(metrics_path, "w") as f:
        f.write("===== Volumetric SDF Metrics =====\n")
        f.write(f"Mean Abs Error : {metrics.mean_abs_error:.4f} m\n")
        f.write(f"RMSE           : {metrics.rmse:.4f} m\n")
        f.write(f"Max Error      : {metrics.max_error:.4f} m\n")
        f.write("=================================\n")

    print(f"Metrics saved to: {metrics_path}")
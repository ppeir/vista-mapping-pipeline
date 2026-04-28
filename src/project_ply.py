import argparse
import os
import numpy as np
import open3d as o3d
from PIL import Image

def generate_2d_projection(input_ply, output_image, min_z, max_z, resolution):
    pcd = o3d.io.read_point_cloud(input_ply)
    points = np.asarray(pcd.points)

    if points.shape[0] == 0:
        print("Error: point cloud is empty.")
        return

    mask = (points[:, 2] >= min_z) & (points[:, 2] <= max_z)
    filtered_points = points[mask]

    if filtered_points.shape[0] == 0:
        print("No points found in the specified Z range.")
        return

    x = filtered_points[:, 0]
    y = filtered_points[:, 1]

    min_x, max_x = np.min(x), np.max(x)
    min_y, max_y = np.min(y), np.max(y)

    width = int(np.ceil((max_x - min_x) / resolution)) + 1
    height = int(np.ceil((max_y - min_y) / resolution)) + 1

    img_array = np.full((height, width), 254, dtype=np.uint8)

    u = ((x - min_x) / resolution).astype(np.int32)
    v = ((y - min_y) / resolution).astype(np.int32)

    # Flip Y axis for standard map convention (Y increases upward)
    v = height - 1 - v

    img_array[v, u] = 0

    img = Image.fromarray(img_array, mode='L')
    img.save(output_image)

    # Generate companion .yaml (ROS map_server format)
    # origin = world coordinates of the bottom-left corner of the image (pixel (0, height-1))
    yaml_path = os.path.splitext(output_image)[0] + ".yaml"
    pgm_basename = os.path.basename(output_image)
    with open(yaml_path, 'w') as f:
        f.write(f"image: {pgm_basename}\n")
        f.write(f"resolution: {resolution:.6f}\n")
        f.write(f"origin: [{min_x:.6f}, {min_y:.6f}, 0.0]\n")
        f.write(f"negate: 0\n")
        f.write(f"occupied_thresh: 0.65\n")
        f.write(f"free_thresh: 0.196\n")

    print(f"Projection saved to {output_image} ({width}x{height} pixels).")
    print(f"YAML saved to {yaml_path} (origin: [{min_x:.3f}, {min_y:.3f}, 0.0])")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Project a Z slice of a .ply file into a 2D image.")
    parser.add_argument("--input", required=True, help="Path to the source .ply file")
    parser.add_argument("--output", default="projection.pgm", help="Output file path (.pgm, .png)")
    parser.add_argument("--min_z", type=float, required=True, help="Minimum Z height for the slice")
    parser.add_argument("--max_z", type=float, required=True, help="Maximum Z height for the slice")
    parser.add_argument("--resolution", type=float, default=0.05, help="Grid resolution (m/pixel)")
    
    args = parser.parse_args()
    generate_2d_projection(args.input, args.output, args.min_z, args.max_z, args.resolution)
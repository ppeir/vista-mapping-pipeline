import sys
import json
import pyzed.sl as sl

def extract_intrinsics_from_svo(svo_path, output_json_path="camera_info.json", depth_scale=1.0):
    # Initialise SVO read parameters
    init_params = sl.InitParameters()
    init_params.set_from_svo_file(svo_path)

    # Create ZED camera object
    zed = sl.Camera()

    # Open the SVO file
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print(f"Error opening SVO: {repr(err)}")
        sys.exit(1)

    # Retrieve camera information
    cam_info = zed.get_camera_information()

    # Resolution
    res = cam_info.camera_configuration.resolution
    width = res.width
    height = res.height

    # Left camera calibration parameters (standard for RGB)
    # Replace with right_cam if exporting right-side images
    calib = cam_info.camera_configuration.calibration_parameters.left_cam

    # Build intrinsic matrix K (3x3):
    # [ fx,  0, cx ]
    # [  0, fy, cy ]
    # [  0,  0,  1 ]
    intrinsic_matrix = [
        [calib.fx, 0.0, calib.cx],
        [0.0, calib.fy, calib.cy],
        [0.0, 0.0, 1.0]
    ]

    # Distortion coefficients [k1, k2, p1, p2, k3]
    distortion = list(calib.disto)

    # Build output dict
    data = {
        "intrinsic_matrix": intrinsic_matrix,
        "distortion": distortion,
        "width": width,
        "height": height,
        "camera_to_pose_rotation": [[0, 0, 1], [0, 1, 0], [-1, 0, 0]]
    }

    # Save camera_info.json
    with open(output_json_path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"Saved: {output_json_path}  ({width}x{height}, fx={calib.fx:.1f})")

    # Save depth_camera_info.json with scale factor applied to intrinsics
    depth_data = {
        "intrinsic_matrix": [
            [calib.fx * depth_scale, 0.0, calib.cx * depth_scale],
            [0.0, calib.fy * depth_scale, calib.cy * depth_scale],
            [0.0, 0.0, 1.0]
        ],
        "distortion": distortion,
        "width": int(round(width * depth_scale)),
        "height": int(round(height * depth_scale)),
        "depth_scale": depth_scale,
        "camera_to_pose_rotation": [[0, 0, 1], [0, 1, 0], [-1, 0, 0]],
    }
    depth_json_path = output_json_path.replace("camera_info.json", "depth_camera_info.json")
    with open(depth_json_path, "w") as f:
        json.dump(depth_data, f, indent=2)
    print(f"Saved: {depth_json_path}")

    # Close camera
    zed.close()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Extract camera intrinsics from a ZED SVO file.")
    parser.add_argument("--svo", required=True, help="Path to the .svo/.svo2 file.")
    parser.add_argument("--output-dir", default=".", dest="output_dir",
                        help="Directory where camera_info.json and depth_camera_info.json are written.")
    parser.add_argument("--depth-scale", type=float, default=0.75, dest="depth_scale",
                        help="Scale factor applied to depth intrinsics (default: 1.0).")
    opt = parser.parse_args()
    import os
    output_json = os.path.join(opt.output_dir, "camera_info.json")
    extract_intrinsics_from_svo(opt.svo, output_json_path=output_json, depth_scale=opt.depth_scale)
import sys
import argparse
import pyzed.sl as sl

def main():
    parser = argparse.ArgumentParser(description="ZED SDK native spatial mapping from SVO2")
    parser.add_argument("svo", help="Input SVO2 file path")
    parser.add_argument("output", help="Output PLY file path")
    parser.add_argument("--trim-start", type=float, default=0.0, metavar="SEC",
                        help="Skip first N seconds of the recording")
    parser.add_argument("--trim-end", type=float, default=0.0, metavar="SEC",
                        help="Stop N seconds before the end of the recording")
    parser.add_argument("--map-type", choices=["cloud", "mesh"], default="mesh",
                        help="Output map type: cloud (fused point cloud) or mesh (default: mesh)")
    args = parser.parse_args()

    svo_path = args.svo
    output_path = args.output

    # 1. Initialisation de la caméra en mode lecture SVO
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.set_from_svo_file(svo_path)
    init_params.coordinate_units = sl.UNIT.METER
    # Force NEURAL mode since the TensorRT model is already optimized on your Orin
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL 

    print(f"[INFO] Opening SVO file: {svo_path}...")
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"[ERROR] Failed to open camera/SVO: {status}")
        sys.exit(1)

    # 2. Enable Positional Tracking (Odometry)
    tracking_params = sl.PositionalTrackingParameters()
    status = zed.enable_positional_tracking(tracking_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"[ERROR] Failed to enable positional tracking: {status}")
        sys.exit(1)

    # 3. Enable Spatial Mapping
    mapping_params = sl.SpatialMappingParameters()
    
    mapping_params.map_type = sl.SPATIAL_MAP_TYPE.MESH if args.map_type == "mesh" else sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD
    print(f"[INFO] Map type: {args.map_type}")
    
    # Spatial resolution (0.05m = 5cm, corresponds to what you had in RTAB-Map)
    mapping_params.resolution_meter = 0.05 
    
    status = zed.enable_spatial_mapping(mapping_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"[ERROR] Failed to enable spatial mapping: {status}")
        sys.exit(1)

    # 4. Reading the video and building the map
    fps = zed.get_camera_information().camera_configuration.fps
    if fps <= 0:
        fps = 30.0

    # SVO2 (H.265): get_svo_number_of_frames() may return -1 before the first
    # grab() (same ZED SDK behaviour as in C++).  Do one grab to unstick it,
    # then rewind to the start before enabling spatial mapping.
    total_frames = zed.get_svo_number_of_frames()
    if total_frames <= 0:
        zed.grab()
        total_frames = zed.get_svo_number_of_frames()
        zed.set_svo_position(0)

    trim_start_frames = int(args.trim_start * fps)
    trim_end_frame = total_frames - int(args.trim_end * fps) if args.trim_end > 0 else total_frames

    # --- Diagnostic: show computed values before the long run ---
    print(f"[INFO] SVO: {total_frames} frames @ {fps:.0f} fps = {total_frames / fps:.1f}s total")
    if args.trim_end > 0:
        kept = trim_end_frame / fps
        print(f"[INFO] Trim-end: stop at frame {trim_end_frame} "
              f"(keep first {kept:.1f}s, cut last {args.trim_end}s = {total_frames - trim_end_frame} frames)")
    if args.trim_start > 0:
        print(f"[INFO] Trim-start: skip first {trim_start_frames} frames ({args.trim_start}s)")
    if trim_end_frame <= trim_start_frames:
        print(f"[ERROR] Invalid trim range: "
              f"(trim_start_frames={trim_start_frames} >= trim_end_frame={trim_end_frame}).",
              file=sys.stderr)
        zed.disable_spatial_mapping()
        zed.disable_positional_tracking()
        zed.close()
        sys.exit(1)

    if trim_start_frames > 0:
        print(f"[INFO] Skipping first {args.trim_start}s (~{trim_start_frames} frames)...")
        zed.set_svo_position(trim_start_frames)

    print("[INFO] Treating frames and building the map...")
    frames_processed = 0

    while True:
        current_pos = zed.get_svo_position()
        if current_pos >= trim_end_frame:
            print(f"[INFO] Trim-end reached at frame {current_pos}.")
            break

        err = zed.grab()
        if err == sl.ERROR_CODE.SUCCESS:
            frames_processed += 1
            if frames_processed % 100 == 0:
                print(f"  -> {frames_processed} frames processed...")
        elif err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            print("[INFO] End of SVO file reached.")
            break
        else:
            print(f"[WARN] Read error: {err}")
            break

    # 5. Extraction and saving of the map
    print("\n[INFO] Extracting the global map...")
    
    if mapping_params.map_type == sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD:
        point_cloud = sl.FusedPointCloud()
        zed.extract_whole_spatial_map(point_cloud)
        print(f"[INFO] Saving point cloud to {output_path}...")
        ok = point_cloud.save(output_path)
        if not ok:
            print(f"[ERROR] Failed to save: {output_path}", file=sys.stderr)
            zed.disable_spatial_mapping()
            zed.disable_positional_tracking()
            zed.close()
            sys.exit(1)
    else:
        # Block executed if you choose MESH mode
        mesh = sl.Mesh()
        zed.extract_whole_spatial_map(mesh)
        print("[INFO] Filtering the mesh...")
        mesh.filter(sl.MeshFilterParameters()) # Smooth and remove artifacts
        print(f"[INFO] Saving mesh to {output_path}...")
        ok = mesh.save(output_path, sl.MESH_FILE_FORMAT.PLY)
        if not ok:
            print(f"[ERROR] Failed to save mesh: {output_path}", file=sys.stderr)
            zed.disable_spatial_mapping()
            zed.disable_positional_tracking()
            zed.close()
            sys.exit(1)

    # 6. Release resources
    zed.disable_spatial_mapping()
    zed.disable_positional_tracking()
    zed.close()
    print("[INFO] Successfully completed!")

if __name__ == "__main__":
    main()
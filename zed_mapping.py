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
    # On force le mode NEURAL puisque le modèle TensorRT est déjà optimisé sur votre Orin
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL 

    print(f"[INFO] Ouverture du fichier SVO: {svo_path}...")
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"[ERREUR] Impossible d'ouvrir la caméra/SVO : {status}")
        sys.exit(1)

    # 2. Activation du Positional Tracking (Odométrie)
    tracking_params = sl.PositionalTrackingParameters()
    status = zed.enable_positional_tracking(tracking_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"[ERREUR] Échec du tracking : {status}")
        sys.exit(1)

    # 3. Activation du Spatial Mapping
    mapping_params = sl.SpatialMappingParameters()
    
    mapping_params.map_type = sl.SPATIAL_MAP_TYPE.MESH if args.map_type == "mesh" else sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD
    print(f"[INFO] Map type: {args.map_type}")
    
    # Résolution spatiale (0.05m = 5cm, correspond à ce que vous aviez mis dans RTAB-Map)
    mapping_params.resolution_meter = 0.05 
    
    status = zed.enable_spatial_mapping(mapping_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"[ERREUR] Échec du mapping : {status}")
        sys.exit(1)

    # 4. Lecture de la vidéo et construction de la carte
    fps = zed.get_camera_information().camera_configuration.fps
    if fps <= 0:
        fps = 30.0
    trim_start_frames = int(args.trim_start * fps)
    # Total frame count to compute trim-end
    total_frames = zed.get_svo_number_of_frames()
    trim_end_frame = total_frames - int(args.trim_end * fps) if args.trim_end > 0 else total_frames

    if trim_start_frames > 0:
        print(f"[INFO] Skipping first {args.trim_start}s (~{trim_start_frames} frames)...")
        zed.set_svo_position(trim_start_frames)

    print("[INFO] Traitement des frames en cours (cela peut prendre quelques minutes)...")
    frames_processed = 0

    while True:
        current_pos = zed.get_svo_position()
        if current_pos >= trim_end_frame:
            print(f"[INFO] Trim-end atteint à la frame {current_pos}.")
            break

        err = zed.grab()
        if err == sl.ERROR_CODE.SUCCESS:
            frames_processed += 1
            if frames_processed % 100 == 0:
                print(f"  -> {frames_processed} frames traitées...")
        elif err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
            print("[INFO] Fin du fichier SVO atteinte.")
            break
        else:
            print(f"[WARN] Erreur de lecture : {err}")
            break

    # 5. Extraction et sauvegarde de la carte
    print("\n[INFO] Extraction de la carte globale en cours...")
    
    if mapping_params.map_type == sl.SPATIAL_MAP_TYPE.FUSED_POINT_CLOUD:
        point_cloud = sl.FusedPointCloud()
        zed.extract_whole_spatial_map(point_cloud)
        print(f"[INFO] Sauvegarde du nuage de points vers {output_path}...")
        point_cloud.save(output_path)
    else:
        # Bloc exécuté si vous choisissez le mode MESH
        mesh = sl.Mesh()
        zed.extract_whole_spatial_map(mesh)
        print("[INFO] Filtrage du maillage...")
        mesh.filter(sl.MeshFilterParameters()) # Lisse et supprime les artefacts
        print(f"[INFO] Sauvegarde du maillage vers {output_path}...")
        mesh.save(output_path, sl.MESH_FILE_FORMAT.PLY)

    # 6. Libération des ressources
    zed.disable_spatial_mapping()
    zed.disable_positional_tracking()
    zed.close()
    print("[INFO] Terminé avec succès !")

if __name__ == "__main__":
    main()
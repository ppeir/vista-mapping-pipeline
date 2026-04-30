########################################################################
#
# Copyright (c) 2022, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

import sys
import signal
import subprocess
import tempfile
import pyzed.sl as sl
import numpy as np
import cv2
from pathlib import Path
import enum
import argparse
import os 

class AppType(enum.Enum):
    LEFT_AND_RIGHT = 1
    LEFT_AND_DEPTH = 2
    LEFT_AND_DEPTH_16 = 3
    COMBINED_VIDEO_DEPTH_16 = 5


def progress_bar(percent_done, bar_length=50):
    #Display a progress bar
    done_length = int(bar_length * percent_done / 100)
    bar = '=' * done_length + '-' * (bar_length - done_length)
    sys.stdout.write('[%s] %i%s\r' % (bar, percent_done, '%'))
    sys.stdout.flush()


def main(opt):
    # Get input parameters
    svo_input_path = opt.input_svo_file
    output_dir = opt.output_path_dir
    video_output_path = opt.output_file
    output_as_video = True    
    app_type = AppType.LEFT_AND_RIGHT
    if opt.mode == 1 or opt.mode == 3:
        app_type = AppType.LEFT_AND_DEPTH
    if opt.mode == 4:
        app_type = AppType.LEFT_AND_DEPTH_16
    if opt.mode == 5:
        app_type = AppType.COMBINED_VIDEO_DEPTH_16

    # Check if exporting to AVI or SEQUENCE
    if opt.mode not in (0, 1, 5):
        output_as_video = False

    if not output_as_video and not os.path.isdir(output_dir):
        os.makedirs(output_dir, exist_ok=True)
    if opt.mode == 5 and output_dir and not os.path.isdir(output_dir):
        os.makedirs(output_dir, exist_ok=True)

    # Specify SVO path parameter
    init_params = sl.InitParameters()
    init_params.set_from_svo_file(svo_input_path)
    init_params.svo_real_time_mode = False  # Don't convert in realtime
    init_params.coordinate_units = sl.UNIT.MILLIMETER  # Use milliliter units (for depth measurements)

    # Create ZED objects
    zed = sl.Camera()

    # Open the SVO file specified as a parameter
    err = zed.open(init_params)
    if err > sl.ERROR_CODE.SUCCESS:
        sys.stdout.write(repr(err))
        zed.close()
        exit()
    
    # Get camera info once
    cam_info = zed.get_camera_information()
    image_size = cam_info.camera_configuration.resolution
    width = image_size.width
    height = image_size.height
    width_sbs = width * 2
    fps = cam_info.camera_configuration.fps
    nb_frames = zed.get_svo_number_of_frames()

    # Log camera info
    sys.stdout.write(
        f"Camera info:\n"
        f"  Model       : {cam_info.camera_model}\n"
        f"  Serial      : {cam_info.serial_number}\n"
        f"  Resolution  : {width}x{height}\n"
        f"  FPS         : {fps}\n"
        f"  Total frames: {nb_frames} ({nb_frames / fps:.1f}s)\n"
    )
    
    # Prepare side by side image container equivalent to CV_8UC4
    svo_image_sbs_rgba = np.zeros((height, width_sbs, 4), dtype=np.uint8)

    # Prepare single image containers
    left_image = sl.Mat()
    right_image = sl.Mat()
    depth_image = sl.Mat()

    video_writer = None
    tmp_avi_path = None
    if output_as_video:
        ext = os.path.splitext(video_output_path)[1].lower()
        # For MP4 output: write to a temp AVI first, then re-encode with ffmpeg (libx264, browser-compatible)
        if ext == '.mp4':
            tmp_fd, tmp_avi_path = tempfile.mkstemp(suffix='.avi')
            os.close(tmp_fd)
            writer_path = tmp_avi_path
            fourcc = cv2.VideoWriter.fourcc('M', '4', 'S', '2')
        else:  # .avi
            writer_path = video_output_path
            fourcc = cv2.VideoWriter.fourcc('M', '4', 'S', '2')
        width_out = width if opt.side != 'both' else width_sbs
        video_writer = cv2.VideoWriter(writer_path,
                                       fourcc,
                                       fps,
                                       (width_out, height))
        if not video_writer.isOpened():
            sys.stdout.write("OpenCV video writer cannot be opened. Please check the output file path and write "
                             "permissions.\n")
            zed.close()
            exit()
    
    rt_param = sl.RuntimeParameters()

    # Compute frame range from trim parameters
    start_frame = int(round(opt.trim_start * fps)) if opt.trim_start > 0 else 0
    end_frame = nb_frames - int(round(opt.trim_end * fps)) if opt.trim_end > 0 else nb_frames
    end_frame = max(start_frame + 1, min(end_frame, nb_frames))
    export_frames = end_frame - start_frame
    sys.stdout.write(
        f"Export range: frames {start_frame}–{end_frame} "
        f"({export_frames} frames, {export_frames / fps:.1f}s)\n"
    )

    if start_frame > 0:
        zed.set_svo_position(start_frame)

    # Start SVO conversion to AVI/SEQUENCE
    sys.stdout.write("Converting SVO... Use Ctrl-C to interrupt conversion.\n")

    first_ts_ns = None

    try:
        while True:
            err = zed.grab(rt_param)
            if err <= sl.ERROR_CODE.SUCCESS:
                svo_position = zed.get_svo_position()
                ts_ns = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
                if first_ts_ns is None:
                    first_ts_ns = ts_ns
                rel_ts = (ts_ns - first_ts_ns) / 1e9

                # Stop early if trim-end reached
                if svo_position >= end_frame:
                    break

                # Retrieve SVO images
                zed.retrieve_image(left_image, sl.VIEW.LEFT)

                if app_type == AppType.LEFT_AND_RIGHT:
                    zed.retrieve_image(right_image, sl.VIEW.RIGHT)
                elif app_type == AppType.LEFT_AND_DEPTH:
                    zed.retrieve_image(right_image, sl.VIEW.DEPTH)
                elif app_type in (AppType.LEFT_AND_DEPTH_16, AppType.COMBINED_VIDEO_DEPTH_16):
                    zed.retrieve_measure(depth_image, sl.MEASURE.DEPTH)

                if output_as_video:
                    if opt.side == 'both':
                        # Copy the left image to the left side of SBS image
                        svo_image_sbs_rgba[0:height, 0:width, :] = left_image.get_data()
                        # Copy the right image to the right side of SBS image
                        svo_image_sbs_rgba[0:, width:, :] = right_image.get_data()
                        frame_rgba = svo_image_sbs_rgba
                    elif opt.side == 'left':
                        frame_rgba = left_image.get_data()
                    else:  # right
                        frame_rgba = right_image.get_data()
                    # Convert SVO image from RGBA to RGB
                    ocv_image_rgb = cv2.cvtColor(frame_rgba, cv2.COLOR_RGBA2RGB)
                    # Write the RGB image in the video
                    assert video_writer is not None
                    video_writer.write(ocv_image_rgb)
                    # Mode 5: also save depth PNG alongside video
                    if app_type == AppType.COMBINED_VIDEO_DEPTH_16 and output_dir:
                        raw = np.nan_to_num(depth_image.get_data(), nan=0.0, posinf=0.0, neginf=0.0)
                        raw[raw < 0] = 0.0
                        raw = np.squeeze(raw).astype(np.uint16)
                        if opt.depth_scale != 1.0:
                            dh = max(1, int(round(height * opt.depth_scale)))
                            dw = max(1, int(round(width * opt.depth_scale)))
                            raw = cv2.resize(raw, (dw, dh), interpolation=cv2.INTER_NEAREST)
                        cv2.imwrite(
                            os.path.join(output_dir, f"{rel_ts:.3f}.png"),
                            raw,
                            [cv2.IMWRITE_PNG_COMPRESSION, opt.depth_compression],
                        )
                else:
                    # Generate file names
                    if opt.side in ('both', 'left'):
                        filename1 = output_dir + "/" + ("left%s.png" % str(svo_position).zfill(6))
                        # Save Left images
                        cv2.imwrite(str(filename1), left_image.get_data())
                    if opt.side in ('both', 'right'):
                        if app_type == AppType.LEFT_AND_RIGHT:
                            filename2 = os.path.join(output_dir, "right%s.png" % str(svo_position).zfill(6))
                            cv2.imwrite(str(filename2), right_image.get_data())
                        elif app_type == AppType.LEFT_AND_DEPTH:
                            filename2 = os.path.join(output_dir, f"{rel_ts:.3f}.png")
                            cv2.imwrite(str(filename2), right_image.get_data())
                        elif app_type == AppType.LEFT_AND_DEPTH_16:
                            filename2 = os.path.join(output_dir, f"{rel_ts:.3f}.png")
                            raw = np.nan_to_num(depth_image.get_data(), nan=0.0, posinf=0.0, neginf=0.0)
                            raw[raw < 0] = 0.0
                            raw = np.squeeze(raw).astype(np.uint16)
                            if opt.depth_scale != 1.0:
                                dh = max(1, int(round(height * opt.depth_scale)))
                                dw = max(1, int(round(width * opt.depth_scale)))
                                raw = cv2.resize(raw, (dw, dh), interpolation=cv2.INTER_NEAREST)
                            cv2.imwrite(str(filename2), raw, [cv2.IMWRITE_PNG_COMPRESSION, opt.depth_compression])

                # Display progress
                progress_bar((svo_position - start_frame + 1) / export_frames * 100, 30)
            if err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                progress_bar(100, 30)
                sys.stdout.write("\nSVO end has been reached. Exiting now.\n")
                break
    except KeyboardInterrupt:
        sys.stdout.write("\nInterrupted by user.\n")
    finally:
        if output_as_video and video_writer is not None:
            video_writer.release()
            if tmp_avi_path is not None:
                # Re-encode to H.264 MP4 (browser-compatible)
                sys.stdout.write(f"Re-encoding to H.264 MP4: {video_output_path}\n")
                result = subprocess.run(
                    [
                        "ffmpeg", "-y",
                        "-i", tmp_avi_path,
                        "-c:v", "libx264",
                        "-pix_fmt", "yuv420p",
                        "-movflags", "+faststart",
                        "-an",
                        video_output_path,
                    ],
                    stderr=subprocess.PIPE,
                )
                os.remove(tmp_avi_path)
                if result.returncode != 0:
                    sys.stdout.write("ffmpeg re-encoding failed:\n" + result.stderr.decode() + "\n")
                else:
                    sys.stdout.write("Re-encoding done.\n")
        zed.close()
    return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter)
    parser.add_argument('--mode', type = int, required=True, help=" Mode 0: LEFT+RIGHT video.\n Mode 1: LEFT+DEPTH_VIEW video.\n Mode 2: LEFT+RIGHT image sequence.\n Mode 3: LEFT+DEPTH_VIEW image sequence.\n Mode 4: LEFT+DEPTH_16BIT image sequence.\n Mode 5: LEFT video + DEPTH_16BIT sequence (combined, single SVO pass).")
    parser.add_argument('--input_svo_file', type=str, required=True, help='Path to the .svo file')
    parser.add_argument('--output_file', type=str, help='Path to the output video file (.mp4 or .avi), required for modes 0 and 1.', default='')
    parser.add_argument('--output_path_dir', type=str, help='Path to a directory, where .png will be written, if mode includes image sequence export', default='')
    parser.add_argument('--side', type=str, choices=['left', 'right', 'both'], default='both',
                        help='Which side to export: left, right, or both (default: both).')
    parser.add_argument('--trim-start', type=float, default=0.0,
                        help='Skip the first N seconds of the SVO (default: 0).')
    parser.add_argument('--trim-end', type=float, default=0.0,
                        help='Skip the last N seconds of the SVO (default: 0).')
    parser.add_argument('--depth-scale', type=float, default=1.0,
                        help='Scale factor for depth image resolution (e.g. 0.5 → half size). Default: 1.0.')
    parser.add_argument('--depth-compression', type=int, default=5, choices=range(10),
                        metavar='[0-9]',
                        help='PNG compression level for depth images (0=none, 9=max). Default: 5.')    
    opt = parser.parse_args()
    if opt.mode not in (0, 1, 2, 3, 4, 5):
        print("Mode should be 0-5.\n 0: LEFT+RIGHT video\n 1: LEFT+DEPTH_VIEW video\n 2: LEFT+RIGHT sequence\n 3: LEFT+DEPTH_VIEW sequence\n 4: LEFT+DEPTH_16BIT sequence\n 5: LEFT+RIGHT video + DEPTH_16BIT sequence (combined)")
        exit()
    if not opt.input_svo_file.endswith((".svo", ".svo2")):
        print("--input_svo_file parameter should be a .svo file but is not : ",opt.input_svo_file,"Exit program.")
        exit()
    if not os.path.isfile(opt.input_svo_file):
        print("--input_svo_file parameter should be an existing file but is not : ",opt.input_svo_file,"Exit program.")
        exit()
    if opt.mode in (0, 1, 5) and len(opt.output_file) == 0:
        print(f"In mode {opt.mode}, --output_file parameter needs to be specified.")
        exit()
    if opt.mode in (0, 1, 5) and not opt.output_file.endswith((".mp4", ".avi")):
        print("--output_file parameter should be a .mp4 or .avi file but is not : ", opt.output_file, "Exit program.")
        exit()
    if opt.mode in (2, 3, 4, 5) and len(opt.output_path_dir) == 0:
        print(f"In mode {opt.mode}, --output_path_dir parameter needs to be specified.")
        exit()
    if opt.mode in (2, 3, 4, 5) and not os.path.isdir(opt.output_path_dir):
        os.makedirs(opt.output_path_dir, exist_ok=True)
    main(opt)
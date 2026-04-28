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
import pyzed.sl as sl
from signal import signal, SIGINT
import argparse 
import os 
import time

cam = sl.Camera()

#Handler to deal with CTRL+C properly
def handler(signal_received, frame):
    cam.disable_recording()
    cam.close()
    sys.exit(0)

signal(SIGINT, handler)

def main(opt):

    init = sl.InitParameters()
    init.depth_mode = sl.DEPTH_MODE.NONE # Set configuration parameters for the ZED
    init.async_image_retrieval = False; # This parameter can be used to record SVO in camera FPS even if the grab loop is running at a lower FPS (due to compute for ex.)
    if opt.fps is not None:
        init.camera_fps = opt.fps

    status = cam.open(init) 
    if status > sl.ERROR_CODE.SUCCESS: 
        print("Camera Open", status, "Exit program.")
        exit(1)

    # IMU warmup: grab frames without recording to let the IMU stabilize.
    # Without this, the SVO starts with unreliable IMU data which causes
    # the ZED SDK positional tracking to hang on gravity alignment during
    # SLAM playback.
    if opt.imu_warmup > 0:
        runtime_warmup = sl.RuntimeParameters()
        warmup_start = time.time()
        warmup_frames = 0
        print(f"IMU warmup: grabbing frames for {opt.imu_warmup}s before recording...")
        while time.time() - warmup_start < opt.imu_warmup:
            cam.grab(runtime_warmup)
            warmup_frames += 1
        print(f"IMU warmup done ({warmup_frames} frames grabbed).")
        
    recording_param = sl.RecordingParameters(opt.output_svo_file, sl.SVO_COMPRESSION_MODE.H265) # Enable recording with the filename specified in argument
    err = cam.enable_recording(recording_param)
    if err > sl.ERROR_CODE.SUCCESS:
        print("Recording ZED : ", err)
        exit(1)

    runtime = sl.RuntimeParameters()
    print("SVO is Recording, use Ctrl-C to stop.") # Start recording SVO, stop with Ctrl-C command
    frames_recorded = 0

    while True:
        if cam.grab(runtime) <= sl.ERROR_CODE.SUCCESS : # Check that a new image is successfully acquired
            frames_recorded += 1
            print("Frame count: " + str(frames_recorded), end="\r")
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--output_svo_file', type=str, help='Path to the SVO file that will be written', required=True)
    parser.add_argument('--fps', type=int, default=None, choices=[15, 30, 60, 100],
                        help='Recording frame rate (15, 30, 60 or 100 fps). Default: camera native FPS.')
    parser.add_argument('--wait', type=int, default=0, metavar='SEC',
                        help='Wait N seconds before starting recording (default: 0).')
    parser.add_argument('--imu-warmup', type=float, default=2.0, metavar='SEC',
                        help='Grab frames for N seconds to let IMU stabilize before recording starts (default: 2.0).')
    opt = parser.parse_args()
    if not opt.output_svo_file.endswith((".svo", ".svo2")):
        print("--output_svo_file parameter should be a .svo file but is not : ",opt.output_svo_file,"Exit program.")
        exit()

    if opt.wait > 0:
        print(f"Starting SVO recording in {opt.wait} seconds...")
        time.sleep(opt.wait)
    main(opt)
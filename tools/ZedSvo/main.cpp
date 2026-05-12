/*
 * rtabmap-zed_svo: Offline RGBD-SLAM from ZED SVO files
 * ======================================================
 * Modeled after rtabmap-rgbd_dataset but uses CameraStereoZed to
 * natively read ZED SVO/SVO2 files through the ZED SDK.
 *
 * Pipeline:
 *   1. CameraStereoZed opens the SVO, provides RGB+depth per frame
 *   2. RTAB-Map F2M visual odometry estimates pose
 *   3. RTAB-Map SLAM: loop closure detection + graph optimisation
 *   4. Database saved to <output>/<output_name>.db
 *
 * Copyright 2024 – VISTA Project (based on IntRoLab rtabmap examples)
 */

#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/core/CameraStereo.h>
#include <rtabmap/core/CameraThread.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/OccupancyGrid.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>

#ifdef RTABMAP_ZED
#include <sl/Camera.hpp>
#endif

#include <signal.h>
#include <cstdio>
#include <string>
#include <map>

using namespace rtabmap;

void showUsage()
{
    printf("\nUsage:\n"
        "  rtabmap-zed_svo [options] <file.svo|file.svo2>\n\n"
        "  Runs offline RGBD-SLAM on a ZED SVO file using the ZED SDK.\n"
        "  Produces an RTAB-Map database (.db) with poses and occupancy grid.\n\n"
        "Options:\n"
        "  --output <dir>         Output directory (default: current dir)\n"
        "  --output_name <name>   Database name without .db (default: rtabmap)\n"
        "  --quality <0-6>        ZED depth quality: 0=NONE 1=PERFORMANCE 2=QUALITY 3=ULTRA(deprecated) 4=NEURAL_LIGHT 5=NEURAL 6=NEURAL_PLUS (default: 1)\n"
        "  --skip <N>             Skip N frames between processed frames (default: 0)\n"
        "  --trim-start <sec>     Trim first <sec> seconds from the SVO (default: 0)\n"
        "  --trim-end <sec>       Trim last <sec> seconds from the SVO (default: 0)\n"
        "  --confidence <0-100>   ZED depth confidence threshold (default: 10, lower=stricter)\n"
        "  --textureness <0-100>  ZED textureness confidence threshold (default: 90)\n"
        "  --quiet                Suppress per-iteration output\n"
        "  --regen-grid           Rebuild map.pgm/map.yaml from existing DB (no SVO needed)\n"
        "%s\n"
        "Example:\n\n"
        "  $ rtabmap-zed_svo \\\n"
        "      --output /output \\\n"
        "      --RGBD/CreateOccupancyGrid true \\\n"
        "      --Mem/STMSize 30 \\\n"
        "      /data/record.svo2\n\n",
        rtabmap::Parameters::showUsage());
    exit(1);
}

// Ctrl-C handler
bool g_running = true;
void sighandler(int sig)
{
    printf("\nSignal %d caught, stopping...\n", sig);
    g_running = false;
}

// ---------------------------------------------------------------------------
// Helper: assemble and save map.pgm / map.yaml from local grids in signatures
// ---------------------------------------------------------------------------
static void saveOccupancyGrid(
    const std::string& output,
    const std::string& databasePath,
    const std::map<int, Transform>& poses,
    const std::map<int, Signature>& signatures,
    const ParametersMap& parameters)
{
    printf("\nAssembling 2D occupancy grid...\n");
    LocalGridCache cache;
    int gridCount = 0;
    for(std::map<int, Signature>::const_iterator iter = signatures.begin();
        iter != signatures.end(); ++iter)
    {
        cv::Mat ground, obstacles, empty;
        iter->second.sensorData().uncompressDataConst(
            0, 0, 0, 0, &ground, &obstacles, &empty);
        if(!ground.empty() || !obstacles.empty() || !empty.empty())
        {
            cache.add(iter->first, ground, obstacles, empty,
                iter->second.sensorData().gridCellSize(),
                iter->second.sensorData().gridViewPoint());
            ++gridCount;
        }
    }
    if(gridCount > 0)
    {
        printf("  Loaded %d local grids\n", gridCount);
        OccupancyGrid grid(&cache, parameters);
        grid.update(poses);
        float xMin = 0, yMin = 0;
        cv::Mat map = grid.getMap(xMin, yMin);
        if(!map.empty())
        {
            float cellSize = grid.getCellSize();
            // Convert: -1->205(unknown), 0->254(free), 100->0(occupied)
            cv::Mat pgm(map.rows, map.cols, CV_8UC1);
            for(int i = 0; i < map.rows * map.cols; ++i)
            {
                signed char v = ((signed char*)map.data)[i];
                if(v == -1)
                    pgm.data[i] = 205;
                else
                    pgm.data[i] = (unsigned char)(254 - v * 254 / 100);
            }
            // Flip: ROS map origin bottom-left, PGM origin top-left
            cv::flip(pgm, pgm, 0);

            std::string pgmPath = output + "/map.pgm";
            cv::imwrite(pgmPath, pgm);

            std::string yamlPath = output + "/map.yaml";
            FILE* yaml = fopen(yamlPath.c_str(), "w");
            if(yaml)
            {
                fprintf(yaml, "image: map.pgm\n");
                fprintf(yaml, "resolution: %f\n", cellSize);
                fprintf(yaml, "origin: [%f, %f, 0.0]\n", xMin, yMin);
                fprintf(yaml, "negate: 0\n");
                fprintf(yaml, "occupied_thresh: 0.65\n");
                fprintf(yaml, "free_thresh: 0.196\n");
                fclose(yaml);
            }

            // Also update the assembled grid in the database
            {
                DBDriver* driver = DBDriver::create();
                if(driver->openConnection(databasePath, false))
                {
                    driver->save2DMap(map, xMin, yMin, cellSize);
                    driver->closeConnection(false);
                }
                delete driver;
            }

            printf("  2D grid saved   : %s (%dx%d, %.3f m/cell)\n",
                pgmPath.c_str(), map.cols, map.rows, cellSize);
        }
        else
        {
            printf("  [WARN] Assembled grid is empty.\n");
        }
    }
    else
    {
        printf("  [WARN] No local grid data in %d signatures.\n",
            (int)signatures.size());
        printf("         Ensure --RGBD/CreateOccupancyGrid true was set during SLAM.\n");
    }
}

int main(int argc, char * argv[])
{
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGINT,  &sighandler);

    ULogger::setType(ULogger::kTypeConsole);
    ULogger::setLevel(ULogger::kWarning);

    if(argc < 2)
    {
        showUsage();
    }

    // --- Defaults ---
    std::string svoPath;
    std::string output = ".";
    std::string outputName = "rtabmap";
    int zedQuality  = 1;   // PERFORMANCE
    int skipFrames  = 0;
    float trimStart = 0.0f;  // seconds to trim from beginning
    float trimEnd   = 0.0f;  // seconds to trim from end
    int confidenceThr = 100; // ZED depth confidence (0-100, lower=stricter)
    int texturenessThr = 90; // ZED textureness confidence (0-100)
    bool quiet      = false;
    bool regenGrid  = false;

    // --- Parse custom options (before Parameters::parseArguments) ---
    for(int i = 1; i < argc; ++i)
    {
        if(std::strcmp(argv[i], "--output") == 0 && i+1 < argc)
        {
            output = argv[++i];
        }
        else if(std::strcmp(argv[i], "--output_name") == 0 && i+1 < argc)
        {
            outputName = argv[++i];
        }
        else if(std::strcmp(argv[i], "--quality") == 0 && i+1 < argc)
        {
            zedQuality = std::atoi(argv[++i]);
        }
        else if(std::strcmp(argv[i], "--skip") == 0 && i+1 < argc)
        {
            skipFrames = std::atoi(argv[++i]);
            if(skipFrames < 0) skipFrames = 0;
        }
        else if(std::strcmp(argv[i], "--trim-start") == 0 && i+1 < argc)
        {
            trimStart = std::atof(argv[++i]);
            if(trimStart < 0) trimStart = 0;
        }
        else if(std::strcmp(argv[i], "--trim-end") == 0 && i+1 < argc)
        {
            trimEnd = std::atof(argv[++i]);
            if(trimEnd < 0) trimEnd = 0;
        }
        else if(std::strcmp(argv[i], "--confidence") == 0 && i+1 < argc)
        {
            confidenceThr = std::atoi(argv[++i]);
            if(confidenceThr < 0) confidenceThr = 0;
            if(confidenceThr > 100) confidenceThr = 100;
        }
        else if(std::strcmp(argv[i], "--textureness") == 0 && i+1 < argc)
        {
            texturenessThr = std::atoi(argv[++i]);
            if(texturenessThr < 0) texturenessThr = 0;
            if(texturenessThr > 100) texturenessThr = 100;
        }
        else if(std::strcmp(argv[i], "--quiet") == 0)
        {
            quiet = true;
        }
        else if(std::strcmp(argv[i], "--regen-grid") == 0)
        {
            regenGrid = true;
        }
        else if(std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-help") == 0)
        {
            showUsage();
        }
    }

    // SVO path is the last positional argument (not needed for --regen-grid)
    if(!regenGrid)
    {
        svoPath = argv[argc - 1];
        if(!UFile::exists(svoPath))
        {
            printf("SVO file not found: %s\n", svoPath.c_str());
            showUsage();
        }
    }

    // Parse RTAB-Map --Param value pairs
    ParametersMap parameters = Parameters::parseArguments(argc, argv);

    // Ensure output directory exists
    UDirectory::makeDir(output);
    parameters.insert(ParametersPair(Parameters::kRtabmapWorkingDirectory(), output));
    parameters.insert(ParametersPair(Parameters::kRtabmapPublishRAMUsage(), "true"));

    // Force RGBD-SLAM mode (the whole point of this tool)
    uInsert(parameters, ParametersPair(Parameters::kRGBDEnabled(), "true"));

    if(quiet)
    {
        ULogger::setLevel(ULogger::kError);
    }

    // --- Print configuration ---
    printf("\nrtabmap-zed_svo – Offline RGBD-SLAM from ZED SVO\n");
    printf("=================================================\n");
    if(!regenGrid)
    {
        printf("  SVO file      : %s\n", svoPath.c_str());
    }
    printf("  Output dir    : %s\n", output.c_str());
    printf("  Output name   : %s\n", outputName.c_str());

    // --- Query SVO metadata (always: needed for trim AND for timestamp correction) ---
    int svoTotalFrames = 0;
    double svoFPS = 0.0;
#ifdef RTABMAP_ZED
    {
        sl::Camera tmpZed;
        sl::InitParameters initParams;
        initParams.input.setFromSVOFile(svoPath.c_str());
        initParams.depth_mode = sl::DEPTH_MODE::NONE;
        initParams.sdk_verbose = 0;
        sl::ERROR_CODE err = tmpZed.open(initParams);
        if(err == sl::ERROR_CODE::SUCCESS)
        {
            // SVO2 files (H.265): getSVONumberOfFrames() returns -1 until at
            // least one grab() has been performed (ZED SDK known behaviour).
            sl::RuntimeParameters rt;
            rt.enable_depth = false;
            tmpZed.grab(rt);
            svoTotalFrames = tmpZed.getSVONumberOfFrames();
            svoFPS = (double)tmpZed.getCameraInformation().camera_configuration.fps;
            tmpZed.close();
            printf("  SVO frames    : %d (%.0f fps, %.1fs)\n", svoTotalFrames, svoFPS,
                   svoFPS > 0.0 ? svoTotalFrames / svoFPS : 0.0);
        }
        else
        {
            printf("  [WARN] Could not query SVO metadata (err=%d).\n", (int)err);
            if(trimEnd > 0.0f)
            {
                printf("         --trim-end ignored.\n");
                trimEnd = 0.0f;
            }
        }
    }
#endif

    printf("  ZED depth     : %s\n",
        zedQuality==0?"NONE":
        zedQuality==1?"PERFORMANCE":
        zedQuality==2?"QUALITY":
        zedQuality==3?"ULTRA(deprecated)":
        zedQuality==4?"NEURAL_LIGHT":
        zedQuality==5?"NEURAL":
        zedQuality==6?"NEURAL_PLUS":"UNKNOWN");
    printf("  Skip frames   : %d\n", skipFrames);
    printf("  Confidence    : %d\n", confidenceThr);
    printf("  Textureness   : %d\n", texturenessThr);
    if(trimStart > 0.0f || trimEnd > 0.0f)
    {
        printf("  Trim start    : %.1fs\n", trimStart);
        printf("  Trim end      : %.1fs\n", trimEnd);
    }
    if(trimStart > 0.0f)
    {
        printf("\n  [WARN] --trim-start is set (%.1fs).\n", trimStart);
        printf("         The ZED SDK positional tracking reads IMU data from frame 0\n");
        printf("         regardless of trim-start. If the IMU data near frame 0 is\n");
        printf("         noisy (vehicle vibration, abrupt motion), the gravity alignment\n");
        printf("         step may hang indefinitely. If this occurs, try --trim-start 0\n");
        printf("         and rely on RTAB-Map to discard early frames automatically.\n\n");
    }
    if(!parameters.empty())
    {
        printf("  Parameters:\n");
        for(ParametersMap::iterator iter = parameters.begin();
            iter != parameters.end(); ++iter)
        {
            printf("    %s=%s\n", iter->first.c_str(), iter->second.c_str());
        }
    }
    printf("  RTAB-Map %s\n\n", RTABMAP_VERSION);

    // ------------------------------------------------------------------
    // [regen-grid] Rebuild 2D map from existing DB and exit
    // ------------------------------------------------------------------
    if(regenGrid)
    {
        std::string databasePath = output + "/" + outputName + ".db";
        if(!UFile::exists(databasePath))
        {
            printf("[ERROR] Database not found: %s\n", databasePath.c_str());
            printf("        Run without --regen-grid first to build the DB.\n");
            return 1;
        }
        printf("Regenerating 2D grid from: %s\n\n", databasePath.c_str());
        uInsert(parameters, ParametersPair(Parameters::kMemInitWMWithAllNodes(), "true"));
        Rtabmap regenRtabmap;
        regenRtabmap.init(parameters, databasePath);
        std::map<int, Transform> poses;
        std::multimap<int, Link> links;
        std::map<int, Signature> signatures;
        regenRtabmap.getGraph(poses, links, true, true, &signatures, false, false, false, true);
        regenRtabmap.close(false);
        if(poses.empty())
        {
            printf("[ERROR] No poses found in database.\n");
            return 1;
        }
        printf("  Loaded %d poses from DB\n", (int)poses.size());
        saveOccupancyGrid(output, databasePath, poses, signatures, parameters);
        return 0;
    }

    // ------------------------------------------------------------------
    // 1. Create ZED camera (SVO file mode)
    // ------------------------------------------------------------------
    CameraStereoZed * camera = new CameraStereoZed(
        svoPath,
        zedQuality,       // depth quality
        0,                // sensingMode = STANDARD
        confidenceThr,    // depth confidence threshold
        true,             // computeOdometry = true (use ZED SDK VIO pose)
        0.0f,             // imageRate = 0 (as fast as possible)
        Transform::getIdentity(),
        true,             // selfCalibration
        false,            // odomForce3DoF
        texturenessThr    // textureness confidence threshold
    );

    CameraThread cameraThread(camera, parameters);

    if(!cameraThread.camera()->init())
    {
        printf("[ERROR] Failed to open SVO file: %s\n", svoPath.c_str());
        printf("        Check that ZED SDK is properly installed and the\n");
        printf("        SVO file matches the SDK version.\n");
        return 1;
    }

    printf("ZED camera initialized successfully.\n");

    // ------------------------------------------------------------------
    // 2. Create database, odometry, SLAM engine
    // ------------------------------------------------------------------
    std::string databasePath = output + "/" + outputName + ".db";
    if(UFile::exists(databasePath))
    {
        UFile::erase(databasePath);
        printf("Deleted existing database: %s\n", databasePath.c_str());
    }

    // Parse detection rate and odom strategy from parameters
    int odomStrategy = Parameters::defaultOdomStrategy();
    float detectionRate = Parameters::defaultRtabmapDetectionRate();
    bool intermediateNodes = Parameters::defaultRtabmapCreateIntermediateNodes();
    Parameters::parse(parameters, Parameters::kOdomStrategy(), odomStrategy);
    Parameters::parse(parameters, Parameters::kRtabmapDetectionRate(), detectionRate);
    Parameters::parse(parameters, Parameters::kRtabmapCreateIntermediateNodes(), intermediateNodes);

    Rtabmap rtabmap;
    rtabmap.init(parameters, databasePath);

    printf("SLAM engine initialised (database: %s)\n\n", databasePath.c_str());

    // ------------------------------------------------------------------
    // 3. Processing loop
    // ------------------------------------------------------------------
    UTimer totalTime;
    UTimer timer;
    CameraInfo cameraInfo;
    SensorData data = cameraThread.camera()->takeImage(&cameraInfo);

    int iteration      = 0;
    int odomKeyFrames  = 0;
    int loopClosures   = 0;
    int skipCount      = 0;
    double previousStamp = 0.0;

    // --- Trimming setup ---
    double firstStamp = data.stamp();
    double trimStartStamp = (trimStart > 0.0f) ? firstStamp + (double)trimStart : 0.0;
    int totalFramesSeen   = 0;

    // Trim-end: frame-count-based (avoids timestamp drift from processing latency)
    int trimEndFrameNum = 0;
    if(trimEnd > 0.0f)
    {
        if(svoTotalFrames > 1 && svoFPS > 0.0)
        {
            trimEndFrameNum = svoTotalFrames - (int)round(trimEnd * svoFPS);
            if(trimEndFrameNum < 1) trimEndFrameNum = 1;
            printf("  Trim: frames 0..%d / %d (%.0f fps, stop at %.1fs from start)\n",
                   trimEndFrameNum, svoTotalFrames, svoFPS,
                   (double)trimEndFrameNum / svoFPS);
        }
        else
        {
            printf("  [WARN] Cannot determine SVO frame count/fps; --trim-end ignored.\n");
            trimEnd = 0.0f;
        }
    }

    printf("Processing SVO frames...\n");
    fflush(stdout);

    while(data.isValid() && g_running)
    {
        ++totalFramesSeen;

        // Override stamp with a real SVO-based frame timestamp.
        // CameraStereoZed in SVO playback mode returns wall-clock time from
        // grab() instead of the original recording timestamp (ZED SDK known
        // behaviour). This causes RTAB-Map poses to span wall-clock processing
        // time (~329s for a 186s video at 8.5fps processing) instead of the
        // actual recording duration (186s). Fix: assign timestamp = frame_index
        // / nominal_fps, which matches the real SVO recording cadence exactly.
        if(svoFPS > 0.0)
        {
            data.setStamp((double)(totalFramesSeen - 1) / svoFPS);
        }

        // Check trim-end: stop if we've passed the frame threshold
        if(trimEndFrameNum > 0 && totalFramesSeen > trimEndFrameNum)
        {
            printf("\n  Trim-end reached at frame %d (threshold: %d)\n",
                   totalFramesSeen, trimEndFrameNum);
            break;
        }

        // Check trim-start: skip frames before the start timestamp
        if(trimStartStamp > 0.0 && data.stamp() < trimStartStamp)
        {
            cameraInfo = CameraInfo();
            timer.restart();
            data = cameraThread.camera()->takeImage(&cameraInfo);
            continue;
        }

        if(skipCount < skipFrames)
        {
            ++skipCount;
            cameraInfo = CameraInfo();
            timer.restart();
            data = cameraThread.camera()->takeImage(&cameraInfo);
            continue;
        }
        skipCount = 0;

        // Pre-process (undistortion, bilateral filter, etc.)
        cameraThread.postUpdate(&data, &cameraInfo);
        cameraInfo.timeTotal = timer.ticks();

        // Use ZED SDK VIO pose directly (bypasses RTAB-Map F2M odometry)
        OdometryInfo odomInfo;
        Transform pose = cameraInfo.odomPose;
        if(!pose.isNull())
        {
            odomInfo.keyFrameAdded = true;
            ++odomKeyFrames;
        }
        odomInfo.reg.inliers = 100;

        // Rate limiting
        bool processData = true;
        if(detectionRate > 0.0f &&
           previousStamp > 0.0 &&
           data.stamp() > previousStamp &&
           data.stamp() - previousStamp < 1.0 / detectionRate)
        {
            processData = false;
        }

        if(processData)
        {
            previousStamp = data.stamp();
        }

        if(!processData)
        {
            data.setId(-1);
            data.setFeatures(std::vector<cv::KeyPoint>(),
                             std::vector<cv::Point3f>(), cv::Mat());
            processData = intermediateNodes;
        }

        timer.restart();
        if(processData)
        {
            // Guarantee a valid 6x6 CV_64FC1 covariance (required by RTAB-Map)
            cv::Mat covariance = cameraInfo.odomCovariance;
            if(covariance.empty() || covariance.cols != 6 || covariance.rows != 6 || covariance.type() != CV_64FC1)
            {
                // ZED SDK VIO pose is trusted: use a small identity covariance
                covariance = cv::Mat::eye(6, 6, CV_64FC1) * 0.0001;
            }
            OdometryEvent e(SensorData(), Transform(), odomInfo);
            rtabmap.process(data, pose, covariance,
                            e.velocity());

            if(rtabmap.getLoopClosureId() > 0)
            {
                ++loopClosures;
            }
        }

        ++iteration;

        bool loopNow = (rtabmap.getLoopClosureId() > 0);
        if(loopNow || (!quiet && iteration % 10 == 0))
        {
            int refTotal = (trimEndFrameNum > 0) ? trimEndFrameNum : svoTotalFrames;
            double pct   = (refTotal > 0) ? 100.0 * totalFramesSeen / refTotal : 0.0;
            double elapsedSecs = totalTime.elapsed();
            char remaining[32] = "";
            if(pct > 1.0 && pct < 99.0)
            {
                snprintf(remaining, sizeof(remaining), " | ~%ds left",
                         (int)(elapsedSecs * (100.0 - pct) / pct));
            }
            printf("[frame %d/~%d | %.1f%% | kf=%d lc=%d | %.0fs%s]%s\n",
                totalFramesSeen, refTotal, pct,
                odomKeyFrames, loopClosures,
                elapsedSecs, remaining,
                loopNow ? " *LOOP*" : "");
            fflush(stdout);
        }

        // Next frame
        cameraInfo = CameraInfo();
        timer.restart();
        data = cameraThread.camera()->takeImage(&cameraInfo);
    }

    double elapsed = totalTime.ticks();

    printf("\n\nProcessing complete.\n");
    printf("  Frames processed : %d\n", iteration);
    printf("  Odom key-frames  : %d\n", odomKeyFrames);
    printf("  Loop closures    : %d\n", loopClosures);
    printf("  Total time       : %.1fs\n", elapsed);
    if(iteration > 0)
    {
        printf("  Avg time/frame   : %.1fms\n", elapsed * 1000.0 / iteration);
    }

    // ------------------------------------------------------------------
    // 4. Save trajectory
    // ------------------------------------------------------------------
    std::map<int, Transform> poses;
    std::multimap<int, Link> links;
    std::map<int, Signature> signatures;
    std::map<int, double> stamps;
    rtabmap.getGraph(poses, links, true, true, &signatures, false, false, false, true);
    for(std::map<int, Signature>::iterator iter = signatures.begin();
        iter != signatures.end(); ++iter)
    {
        stamps.insert(std::make_pair(iter->first, iter->second.getStamp()));
    }

    std::string trajPath = output + "/" + outputName + "_poses.txt";
    if(poses.size() && graph::exportPoses(trajPath, 1, poses, links, stamps))
    {
        printf("  Trajectory saved : %s (%d poses)\n", trajPath.c_str(), (int)poses.size());
    }

    // ------------------------------------------------------------------
    // 5. Generate 2D occupancy grid from local grids stored in DB
    // ------------------------------------------------------------------
    saveOccupancyGrid(output, databasePath, poses, signatures, parameters);

    // ------------------------------------------------------------------
    // 6. Close database
    // ------------------------------------------------------------------
    rtabmap.close(true);
    printf("  Database saved   : %s\n", databasePath.c_str());

    return 0;
}

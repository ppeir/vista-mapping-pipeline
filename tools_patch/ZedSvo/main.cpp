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
        "  --quality <0-3>        ZED depth quality: 0=NONE 1=PERFORMANCE 2=QUALITY 3=NEURAL (default: 1)\n"
        "  --skip <N>             Skip N frames between processed frames (default: 0)\n"
        "  --trim-start <sec>     Trim first <sec> seconds from the SVO (default: 0)\n"
        "  --trim-end <sec>       Trim last <sec> seconds from the SVO (default: 0)\n"
        "  --quiet                Suppress per-iteration output\n"
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
    bool quiet      = false;

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
        else if(std::strcmp(argv[i], "--quiet") == 0)
        {
            quiet = true;
        }
        else if(std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-help") == 0)
        {
            showUsage();
        }
    }

    // SVO path is the last positional argument
    svoPath = argv[argc - 1];
    if(!UFile::exists(svoPath))
    {
        printf("SVO file not found: %s\n", svoPath.c_str());
        showUsage();
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
    printf("  SVO file      : %s\n", svoPath.c_str());
    printf("  Output dir    : %s\n", output.c_str());
    printf("  Output name   : %s\n", outputName.c_str());

    // --- Query SVO metadata for trim-end ---
    int svoTotalFrames = 0;
#ifdef RTABMAP_ZED
    if(trimEnd > 0.0f)
    {
        sl::Camera tmpZed;
        sl::InitParameters initParams;
        initParams.input.setFromSVOFile(svoPath.c_str());
        initParams.depth_mode = sl::DEPTH_MODE::NONE;
        initParams.sdk_verbose = 0;
        sl::ERROR_CODE err = tmpZed.open(initParams);
        if(err == sl::ERROR_CODE::SUCCESS)
        {
            svoTotalFrames = tmpZed.getSVONumberOfFrames();
            tmpZed.close();
            printf("  SVO frames    : %d\n", svoTotalFrames);
        }
        else
        {
            printf("  [WARN] Could not query SVO frame count (err=%d); --trim-end ignored.\n", (int)err);
            trimEnd = 0.0f;
        }
    }
#endif

    printf("  ZED depth     : %s\n",
        zedQuality==0?"NONE":
        zedQuality==1?"PERFORMANCE":
        zedQuality==2?"QUALITY":
        zedQuality==3?"NEURAL":"UNKNOWN");
    printf("  Skip frames   : %d\n", skipFrames);
    if(trimStart > 0.0f || trimEnd > 0.0f)
    {
        printf("  Trim start    : %.1fs\n", trimStart);
        printf("  Trim end      : %.1fs\n", trimEnd);
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
    // 1. Create ZED camera (SVO file mode)
    // ------------------------------------------------------------------
    CameraStereoZed * camera = new CameraStereoZed(
        svoPath,
        zedQuality,     // depth quality
        0,              // sensingMode = STANDARD
        100,            // confidenceThr (max = no filtering)
        false,          // computeOdometry = false (we use RTAB-Map odom)
        0.0f,           // imageRate = 0 (as fast as possible)
        Transform::getIdentity(),
        true,           // selfCalibration
        false,          // odomForce3DoF
        90              // texturenessConfidenceThr
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

    ParametersMap odomParameters = parameters;
    Odometry * odom = Odometry::create(odomParameters);

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
    double trimStopStamp  = 0.0; // computed after 2nd frame
    int totalFramesSeen   = 0;

    if(trimEnd > 0.0f && svoTotalFrames <= 1)
    {
        printf("  [WARN] Cannot determine SVO frame count; --trim-end ignored.\n");
        trimEnd = 0.0f;
    }

    printf("Processing SVO frames...\n");
    fflush(stdout);

    while(data.isValid() && g_running)
    {
        ++totalFramesSeen;

        // Estimate FPS from first two frames and compute stop timestamp
        if(trimEnd > 0.0f && trimStopStamp <= 0.0 && totalFramesSeen == 2)
        {
            double dt = data.stamp() - firstStamp;
            if(dt > 0)
            {
                double fps = 1.0 / dt;
                double totalDuration = (double)svoTotalFrames / fps;
                trimStopStamp = firstStamp + totalDuration - (double)trimEnd;
                printf("  Trim: ~%.0f fps, ~%.1fs total duration\n", fps, totalDuration);
                printf("  Trim: processing window [%.1fs .. %.1fs]\n",
                       trimStartStamp > 0 ? trimStart : 0.0,
                       trimStopStamp - firstStamp);
            }
        }

        // Check trim-end: stop if past the computed stop timestamp
        if(trimStopStamp > 0.0 && data.stamp() >= trimStopStamp)
        {
            printf("\n  Trim-end reached at frame %d (t=%.1fs)\n",
                   totalFramesSeen, data.stamp() - firstStamp);
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

        // Compute visual odometry
        OdometryInfo odomInfo;
        Transform pose = odom->process(data, &odomInfo);

        if(odomInfo.keyFrameAdded)
        {
            ++odomKeyFrames;
        }

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
            OdometryEvent e(SensorData(), Transform(), odomInfo);
            rtabmap.process(data, pose, odomInfo.reg.covariance,
                            e.velocity());

            if(rtabmap.getLoopClosureId() > 0)
            {
                ++loopClosures;
            }
        }

        ++iteration;

        if(!quiet || (iteration % 100 == 0))
        {
            double slamTime = timer.ticks();

            if(rtabmap.getLoopClosureId() > 0)
            {
                printf("  frame %d: cam=%dms odom(q=%d/%d kfs=%d)=%dms slam=%dms *LOOP*\n",
                    iteration,
                    int(cameraInfo.timeTotal * 1000.0f),
                    odomInfo.reg.inliers, odomInfo.features, odomKeyFrames,
                    int(odomInfo.timeEstimation * 1000.0f),
                    int(slamTime * 1000.0f));
            }
            else if(!quiet)
            {
                printf("  frame %d: cam=%dms odom(q=%d/%d kfs=%d)=%dms slam=%dms\n",
                    iteration,
                    int(cameraInfo.timeTotal * 1000.0f),
                    odomInfo.reg.inliers, odomInfo.features, odomKeyFrames,
                    int(odomInfo.timeEstimation * 1000.0f),
                    int(slamTime * 1000.0f));
            }
            fflush(stdout);
        }
        else if(iteration % 50 == 0)
        {
            printf(".");
            fflush(stdout);
        }

        // Next frame
        cameraInfo = CameraInfo();
        timer.restart();
        data = cameraThread.camera()->takeImage(&cameraInfo);
    }

    delete odom;
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
    printf("\nAssembling 2D occupancy grid...\n");
    {
        LocalGridCache cache;
        int gridCount = 0;
        for(std::map<int, Signature>::iterator iter = signatures.begin();
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

                // Also save assembled grid in the database
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
            printf("         Ensure --RGBD/CreateOccupancyGrid true is set.\n");
        }
    }

    // ------------------------------------------------------------------
    // 6. Close database
    // ------------------------------------------------------------------
    rtabmap.close(true);
    printf("  Database saved   : %s\n", databasePath.c_str());

    return 0;
}

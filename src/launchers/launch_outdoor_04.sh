python3 src/process_svo.py \
  --svo ./data/raw/outdoor_09.svo2 \
  --output ./data/outputs/outdoor_09 \
  --render cloud \
  --superpoint \
  --Optimizer/Strategy 2 \
  --Optimizer/GravitySigma 0.3 \
  --RGBD/OptimizeMaxError 3.0 \
  --Vis/MinInliers 10 \
  --Vis/MaxDepth 8.0 \
  --RGBD/LoopClosureReextractFeatures true \
  --Mem/STMSize 10 \
  --quality 5 \
  --Grid/MapFrameProjection true \
  --Grid/NormalsSegmentation false \
  --Grid/MaxGroundHeight 0.4 \
  --Grid/MaxObstacleHeight 1.2 \
  --Grid/CellSize 0.05 \
  --Grid/RayTracing false \
  --Grid/MinClusterSize 20 \
  --Grid/ClusterRadius 0.05 \
  --Kp/MaxFeatures 2000 \
  --Rtabmap/LoopRatio 0.65 \
  --Rtabmap/DetectionRate 1.5 \
  # --regen-grid
  # --Rtabmap/LoopThr 1.0 \
  # --Grid/MinGroundHeight -1.5 \
  # --Grid/MaxGroundHeight -0.9 \
python3 src/process_svo.py \
  --svo ./data/raw/outdoor_03.svo2 \
  --output ./data/outputs/outdoor_03-2 \
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
  --Grid/NormalsSegmentation false \
  --Grid/MinObstacleHeight -0.7 \
  --Grid/MaxObstacleHeight -0.2 \
  --Grid/CellSize 0.01 \
  --Grid/RayTracing false \
  --Grid/MinClusterSize 20 \
  --Grid/ClusterRadius 0.05 \
  --Kp/MaxFeatures 2000 \
  --Rtabmap/LoopRatio 0.65 \
  --Rtabmap/DetectionRate 1.0 \
  --regen-grid
  # --Rtabmap/LoopThr 1.0 \
  # --Grid/MinGroundHeight -1.5 \
  # --Grid/MaxGroundHeight -0.9 \
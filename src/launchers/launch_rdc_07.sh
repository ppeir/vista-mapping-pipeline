python3 src/process_svo.py \
  --svo ./data/raw/rdc_07.svo2 \
  --output ./data/outputs/rdc_07 \
  --render cloud \
  --superpoint \
  --Optimizer/Strategy 2 \
  --Optimizer/GravitySigma 0.3 \
  --RGBD/OptimizeMaxError 3.0 \
  --Vis/MinInliers 15 \
  --Vis/MaxDepth 4.0 \
  --RGBD/LoopClosureReextractFeatures true \
  --Mem/STMSize 10 \
  --Mem/RehearsalSimilarity 0.45 \
  --quality 5 \
  --Grid/MapFrameProjection true \
  --Grid/NormalsSegmentation false \
  --Grid/MaxGroundHeight 0.1 \
  --Grid/MaxObstacleHeight 1.8 \
  --Grid/CellSize 0.05 \
  --Grid/RayTracing false \
  --Kp/MaxFeatures 1000 \
  --Rtabmap/LoopRatio 0.7 \
  --Rtabmap/DetectionRate 2.0 \
  # --regen-grid
  # --Rtabmap/LoopThr 1.0 \

python3 src/process_svo.py \
  --svo ./data/raw/garage_04.svo2 \
  --output ./data/outputs/garage_04 \
  --render cloud \
  --superpoint \
  --Optimizer/Strategy 2 \
  --Optimizer/GravitySigma 0.1 \
  --RGBD/OptimizeMaxError 20.0 \
  --Rtabmap/DetectionRate 0 \
  --Vis/MinInliers 25 \
  --Vis/MaxDepth 5.0 \
  --RGBD/LoopClosureReextractFeatures true \
  --Mem/STMSize 50 \
  --quality 5 \
  --Grid/MinObstacleHeight 0.15 \
  --Grid/MaxObstacleHeight 2.0 \
  --Grid/CellSize 0.05 \
  --Grid/MinClusterSize 30 \
  --Kp/MaxFeatures -1 \
  --Rtabmap/LoopRatio 0.9 \
  --Rtabmap/DetectionRate 2.0 \
  # --regen-grid
  # --Rtabmap/LoopThr 1.0 \

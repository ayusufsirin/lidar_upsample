# LiDAR stereo fusion experiments

#### You:
I am writing a lidar stereo fusion paper. What are the experiments to be established on the literature

#### ChatGPT:
\[Unsupported Content]


#### ChatGPT:
\[Unsupported Content]


#### ChatGPT:
\[Unsupported Content]


#### ChatGPT:
\[Unsupported Content]


#### ChatGPT:
Below is a distilled “menu” of experimental setups that today’s LiDAR‑stereo‑fusion papers are expected to cover. I group them by the research questions they answer, list the go‑to public datasets, and note the evaluation metrics and recent SOTA references so you can decide which ones best fit your own contribution.

### 1. Depth completion (sparse LiDAR ➜ dense depth map with stereo/RGB help)

| Why                                                                                      | Benchmark datasets                                                                                                 | Key metrics                                      | Typical baselines / SOTA                                                                                                                                                 |
| ---------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| Show that fusing LiDAR spikes with dense stereo cues fills holes accurately and smoothly | **KITTI Depth Completion** (93 k frames, lidar + RGB) citeturn0search0; **NYU‑V2** indoor; **VOID** (hand‑held) | RMSE, MAE, iRMSE, iMAE (KITTI evaluation server) | Sparse‑to‑Dense, LidarStereoNet, **Volumetric Propagation Net (VPN)** → 166 mm RMSE on KITTI val citeturn0search3; Bilateral Propagation Net 2024 citeturn1search4 |

*Extras to impress*:

- **Generalization** – train on KITTI, test on VOID or the brand‑new **UnCLe** continual‑learning benchmark to show cross‑domain robustness citeturn0search5.
- **Low‑cost LiDAR** – down‑sample to 4/8 beams (as in SLS‑Fusion) and plot accuracy vs. beam‑count citeturn0search9.

---

### 2. Long‑range dense depth / disparity

*Problem*: stereo alone degrades beyond 40 m, LiDAR is sparse.\
*Experiment*: crop KITTI up to 120 m, DrivingStereo or new RGB‑NIR‑LiDAR dataset (CVPR 25) for night / harsh illumination citeturn0search1; evaluate RMSE per distance bin (0‑20 m, 20‑40 m, 40‑80 m, 80 m+).\
*Reference*: VPN and Volumetric Prop Net citeturn1search6.

---

### 3. 3‑D object detection with fused depth

*Datasets*: KITTI 3D, Waymo‑Open, nuScenes.\
*Metrics*: BEV/3D mAP (IoU‑0.7 for cars).\
*Protocol*: Run plain LiDAR detector (e.g., PointPillars), stereo‑only (e.g., DSGN/StreamDSGN), and your fusion model; compare day/night splits.\
*Recent paper*: Stereo‑LiDAR detector improving AP by 5 pp on KITTI 2024 citeturn1search1.\
*Benchmark site*: KITTI 3D leaderboard citeturn1search5.

---

### 4. Scene flow & motion

*Datasets*: KITTI‑2015, FlyingThings3D, Argoverse 2‑Motion.\
*Metrics*: End‑Point Error (EPE), percentage outliers.\
*Why*: Shows temporal consistency of fusion and usefulness for tracking.

---

### 5. Visual‑LiDAR odometry / SLAM

*Datasets*: KITTI Odometry (sequence 00‑10), KITTI‑360, EuRoC MAV.\
*Metrics*: Absolute Trajectory Error (ATE), Relative Pose Error (RPE).\
*Ablation*: LiDAR‑only, stereo‑only, fusion with/without loop‑closure.

---

### 6. Semantic & panoptic segmentation

*Datasets*: SemanticKITTI (LiDAR + RGB), nuScenes‐Panoptic.\
*Metrics*: mIoU, PQ.\
*Twist*: fuse depth map back‑projected into point cloud to lift 2‑D semantics to 3‑D.

---

### 7. Robustness studies

| Perturbation             | How to test                                                           | Common findings                                      |
| ------------------------ | --------------------------------------------------------------------- | ---------------------------------------------------- |
| Weather / lighting       | DrivingStereo fog/rain/night, new RGB‑NIR dataset citeturn0search1 | Stereo degrades first; fusion keeps RMSE drop < 10 % |
| LiDAR sparsity / dropout | Randomly remove beams or frames                                       | VPN keeps ≤ 5 % RMSE rise with ¼ beams               |
| Calibration error        | Inject ± 2 px reprojection noise                                      | Late‑fusion architectures more tolerant              |

---

### 8. Ablation & fusion‑strategy analysis

- Run **early, mid, late** fusion variants.
- Zero‑out one modality or replace LiDAR with Gaussian noise to quantify each stream’s contribution.
- Visualize confidence maps or attention weights.

---

### 9. Runtime & resource evaluation

- Measure FPS on RTX‑3090 or embedded devices (Orin, Jetson).
- Memory footprint, model size.
- Real‑time example: guidance‑injection network runs 30 FPS on RTX‑2080 Ti citeturn0search6.

---

## Putting it all together

1. **Pick 2‑3 core tasks** that align with your claimed contribution (e.g., depth completion + 3‑D detection if you argue safety; depth completion + odometry if you sell robustness).
2. **Adopt community metrics and publicly hosted test servers** (KITTI, Waymo) so reviewers can verify numbers.
3. **Include at least one stress‑test** (sparsity, adverse weather, or cross‑dataset generalization).
4. **Report compute load**; real‑world deployability is now routinely checked.
5. **Provide ablation and qualitative visualizations** (error maps, BEV overlays) to illustrate where fusion helps.

Following this blueprint will cover the experimental “minimum viable product” reviewers expect in 2025 while giving room to showcase the unique strengths of your own LiDAR‑stereo fusion approach.

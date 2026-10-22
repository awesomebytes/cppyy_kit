# Calibrate the webcam with a phone

On the same Wi-Fi, open http://172.29.11.141:8769 on the phone. This address is the laptop's current Wi-Fi address and can change. Use landscape orientation and tap the fullscreen button. Keep the pattern at a fixed zoom. A Pixel 9 Pro XL is suitable; physical screen dimensions are not needed to estimate intrinsics.

From this repository checkout, open the capture window:

```bash
roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/calibrate.sh
```

Hold the phone still with the whole checkerboard visible. When coloured corners appear, press SPACE on the laptop to capture a view. Capture 15–25 distinct views: near/far, centre/edges, tilted left/right and up/down. Avoid glare and blurred views. Keep webcam focus fixed if the device supports focus adjustment.

Press ENTER to solve. The tool saves `build/webcam_session/calibration.json` with the 640 × 360 camera matrix, distortion coefficients, global RMS reprojection error and individual view errors in pixels. It also saves detected points in a sibling NPZ file. Press Q to close. Calibration quality needs inspection across diverse views; a small training reprojection error alone does not guarantee accurate calibration.

The displayed board has 9 × 6 squares and 8 × 5 internal corners. Square width is set to one unit. Physical square size changes estimated board translation units, not focal length in pixels or distortion. Calibration applies to the capture mode used. Adjust intrinsics for later resizing/cropping and undistort before projecting depth.

To restart the phone target server from the repository checkout:

```bash
python3 -m http.server 8769 --bind 0.0.0.0 --directory roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/calibration_target
```

## Metric-Large preparation

The 1.34 GB checkpoint is downloaded into the existing Hugging Face cache under `build/perception_toolbox/huggingface`. Revision: `4010e39f3634a45bc60553321fb49fb760bd594e`. The existing depth Pixi environment loads it with `DepthAnything3.from_pretrained("depth-anything/DA3METRIC-LARGE", revision=...)`.

A batch-one smoke benchmark used a synthetic 640 × 480 image, three warmups and 30 measured calls per resolution. It verifies execution and resource use, not depth accuracy.

| Output resolution | Median inference | p95 inference | Peak allocated tensors | Peak reserved CUDA allocator |
| --- | ---: | ---: | ---: | ---: |
| 336 × 252 | 37.88 ms | 47.06 ms | 2101 MB | 2246 MB |
| 504 × 378 | 65.08 ms | 68.87 ms | 2267 MB | 2600 MB |

Memory values use decimal MB and PyTorch counters, excluding CUDA context and other process memory. These results do not meet a 33.3 ms budget for processing every frame at 30 fps. The standalone metric model returns no intrinsics; the existing Small viewer continues to use estimated intrinsics. Metric integration should use the measured calibration and upstream focal-length scaling (`depth_metres = focal_pixels * network_depth / 300`), with focal length expressed on the processed image grid.

Sources: [model checkpoint](https://huggingface.co/depth-anything/DA3METRIC-LARGE/tree/main), [upstream conversion](https://github.com/ByteDance-Seed/Depth-Anything-3#model-zoo), [raw benchmark](evidence/metric_large_benchmark.json).

## Capture rate investigation

The driver advertises 30 fps at 640 × 480, but capture-only measurements stayed near 20 fps in both YUYV and MJPEG. Disabling dynamic exposure frame rate, short manual exposure and power-line compensation changes did not resolve that mode.

MJPEG at 640 × 360 and 1280 × 720 delivered 30.1 fps. The shared camera helper now explicitly requests 640 × 360 MJPEG at 30 fps. Calibration uses that same mode. Experimental exposure and power-line controls were restored.

Evidence: [capture sizes](evidence/capture_other_sizes.json), [formats](evidence/capture_30fps.json), [fixed frame-rate exposure](evidence/capture_fixed_30fps.json), [manual exposure](evidence/capture_manual_exposure.json), [power-line settings](evidence/capture_powerline.json).

The complete Small-model pipeline with the native viewer measured **30.07 captured frames/s and 30.00 processed frames/s** over 15 seconds. DA3 inference was 23.13 ms median and 26.76 ms p95. Evidence: [full pipeline](evidence/depth_30fps.json). These counters measure capture and processing, not monitor scanout.

## Captured calibration, 2026-10-21

The session captured 29 views. Original RMS reprojection error was 0.671 pixels. Views 11, 16 and 20 had individual RMS errors of 1.794, 0.745 and 2.953 pixels. Refitting after excluding those three views gives 0.146 pixels RMS over 26 views. Original JSON and detected points remain unchanged. The refined file is `build/webcam_session/calibration_refined.json`; a numeric copy is saved as [evidence](evidence/webcam_calibration_refined.json).

Retained corners span x=10.2–628.0 and y=14.8–350.1 in the 640 × 360 image. Refined focal lengths are fx=408.673 and fy=409.146 pixels, with principal point (319.453, 176.804). Reprojection errors are fitted-data residuals, not an independent depth-accuracy measurement.

Run the calibrated Small-model viewer from the repository checkout:

```bash
roscon_uk_2026/next_steps/webcam_session/toolbox/rendering/depth_demo.sh --viewer viser --calibration build/webcam_session/calibration_refined.json
```

The pipeline undistorts frames with cached remapping tables, retains the calibrated camera matrix for the undistorted grid, and scales it to DA3's output grid for point-cloud projection. It rejects a mismatched capture resolution. The Small model still predicts relative depth; calibration is used for undistortion and projection, not supplied as model conditioning. In the notebook, construct `DepthPipeline(calibration="build/webcam_session/calibration_refined.json")`.

Follow-up during persistent-session setup: automatic exposure with dynamic frame rate enabled reduced the 640 × 360 mode to about 20 fps again. Disabling `V4L2_CID_EXPOSURE_AUTO_PRIORITY` in the active session restored approximately 30 fps. The camera helper now disables that control at startup while retaining automatic exposure. This device setting persists after closing the camera.

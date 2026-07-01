# Third-party notices

This document records known third-party, adapted, and asset components in this repository. It is an engineering inventory, not legal advice.

Unless otherwise stated, original Digua robot code in this repository is licensed under the root `LICENSE` file, Apache License 2.0.

## Original project code

The following project-maintained code is intended to be covered by the root Apache-2.0 license:

- `src/digua_bringup`
- `src/base_control_ros2`
- `src/digua_description`
- `src/digua_mapping`
- `src/digua_navigation`
- `src/digua_exploration`
- `src/digua_bpu_yolo`
- `src/digua_semantic_mapping`
- `tools`
- Project documentation such as `README.md` and `QUICK_START.md`

Robot maps, calibration images, logs, model binaries, and CAD assets are listed below where their source or distribution status differs from ordinary source code.

## Referenced code

### `src/base_control_ros2`

- Source/reference: https://gitee.com/bingda-robot/base_control_ros2
- Status: project-maintained implementation rewritten for the Digua robot chassis, with design and protocol ideas referenced from the Bingda `base_control_ros2` project.
- Upstream license status: no explicit license was identified in the referenced upstream repository at the time this notice was written.
- Local handling: the local rewritten package is covered by the root Apache-2.0 license. Keep this reference notice to acknowledge the upstream project that informed the implementation.
- Recommended cleanup before public redistribution: if future changes copy substantial upstream source code verbatim, re-check the upstream license or obtain permission before release.

## Official third-party drivers and SDKs

### `src/YDLidar-SDK`

- Source: https://github.com/YDLIDAR/YDLidar-SDK
- Local license file: `src/YDLidar-SDK/LICENSE.txt`
- License summary: the SDK license file states MIT for YDLidar-SDK portions and includes notices for bundled components such as `serial` under MIT and `angles` / socket components under BSD-style licenses.
- Local handling: preserve the upstream directory structure, copyright headers, and `LICENSE.txt` when redistributing.

### `src/ydlidar_ros2_driver`

- Source: https://github.com/YDLIDAR/ydlidar_ros2_driver
- Upstream package metadata: `package.xml` declares `MIT`.
- Known issue: the upstream and local `LICENSE.txt` file is empty in this snapshot, while some launch files contain Apache-2.0 headers. Treat this as an official third-party component with inconsistent upstream notices, not as original Apache-2.0 project code.
- Local handling: keep upstream copyright headers and package metadata intact. If this repository is published, consider documenting the upstream inconsistency in release notes or replacing this directory with a fresh upstream checkout that includes corrected notices if YDLIDAR updates them.

### `src/ros2_astra_camera`

- Source: https://github.com/orbbec/ros2_astra_camera
- Status: official Orbbec/Astra ROS 2 camera driver and message packages.
- Known local notice state: this snapshot contains mixed notices, including Orbbec proprietary-rights headers, `TODO: License declaration`, `all copyrights reserved`, OpenNI Apache-2.0 headers, MIT headers for bundled single-header libraries, and BSD notices for point cloud processing code.
- Local handling: treat the whole directory as a third-party driver. Do not represent it as original project code. Preserve upstream notices and verify the exact redistribution terms against the current Orbbec upstream release before public or commercial redistribution.

Known bundled components in `src/ros2_astra_camera` include:

- `include/astra_camera/json.hpp`: nlohmann/json, MIT.
- `include/magic_enum/magic_enum.hpp`: magic_enum, MIT.
- `include/openni2/*`: OpenNI/PrimeSense headers, Apache-2.0.
- `include/astra_camera/point_cloud_proc/*` and `src/point_cloud_proc/*`: Willow Garage point cloud processing code, BSD-style license.

## Models, data, and generated assets

### `models/bpu_yolov8s_oiv7`

- Contents include a Horizon/RDK X5 BPU model binary, an Open Images V7 class list, a model conversion log, and a model card.
- Local class list: `yolov8s-oiv7_classes.txt` contains 601 entries, indexed `0` through `600`.
- Source model: converted from an official YOLO / Ultralytics Open Images V7 model (`yolov8s-oiv7.onnx`).
- Conversion path: the conversion log references `rdk_model_zoo/samples/vision/ultralytics_yolo`; it was generated with the Horizon/RDK X5 toolchain (`hb_mapper` / `horizon_nn`).
- License status: this is a third-party model conversion artifact. The BPU `.bin` is not covered by the root Apache-2.0 license. Redistribution must follow the upstream Ultralytics YOLO model/software license, Open Images V7 label or dataset terms where applicable, and any Horizon/RDK conversion toolchain terms.
- Local handling: keep these assets as project deployment artifacts. See `models/bpu_yolov8s_oiv7/MODEL_CARD.md` before public redistribution.

### `config`

- Contents include model workconfig files, class lists, raw NV12 data, and sample images for local inference or deployment tests.
- License status: not fully documented.
- Local handling: do not assume these assets are covered by Apache-2.0 unless their origin is confirmed.

### `calib_images`

- Contents are project-captured calibration images photographed by the project author.
- Local handling: these images may be distributed as project-owned calibration data, subject to the repository's chosen asset policy. Before public release, remove or crop any frame that contains private indoor details, faces, addresses, or other sensitive information.

### `digua_maps`, `digua_navigation_data`, and `test_logs`

- Contents are project-generated map, navigation, and test data.
- Local handling: keep only sample-sized data in public releases. Remove private locations, sensitive scenes, long-running logs, or large runtime data before distribution.

### `Ackermann_Steering_Chassis_Models`

- Contents include a chassis STEP model drawn by the project author.
- Local handling: this model may be distributed as a project-owned CAD asset, subject to the repository's chosen asset policy. If it is reused outside this repository, keep an attribution note naming the Digua robot project or project author.

## Release checklist

Before publishing or submitting this repository:

1. Keep the `src/base_control_ros2` reference notice when distributing the project.
2. Keep `src/YDLidar-SDK/LICENSE.txt` with any copy of `src/YDLidar-SDK`.
3. Verify current upstream notices for `src/ydlidar_ros2_driver` and `src/ros2_astra_camera`.
4. Review the BPU model card and upstream model/toolchain terms before distributing the model binary.
5. Make release packages clear about which files are original Apache-2.0 project code and which files are third-party or data assets.

# Contribution Guidelines

Thank you for your interest in contributing to the Comau CRC Open ROS2 Driver. We welcome contributions of all kinds—bug reports, documentation improvements, feature enhancements, and new robot descriptions.

---

## Issue Tracking

If you encounter a bug, have a question, or would like to suggest an improvement, please open an issue on GitHub. When creating an issue, include:

* A clear and descriptive title and summary
* Steps to reproduce or a minimal example
* Your environment details (OS, ROS distro, driver version)
* Any modifications you've made
* Relevant logs or screenshots

*Please do not add labels yourself; maintainers will handle labeling.*

---

## Submitting a Pull Request

1. **Discuss your change**: Open an issue to propose significant changes or to gather feedback.
2. **Fork the repository** and create a feature branch:
3. **Implement your changes** focusing on one logical change per PR.
4. **Keep commits clean**: write meaningful messages and squash related commits when ready.
5. **Push** your branch and **open a Pull Request** against `main`.

Our CI will run build jobs (`build-rolling`, `build-jazzy`, `build-kilted`) to ensure compatibility across supported ROS distros.

> **Quick-start for new robot models:**
> We’re keen to add more models! Once your description matches the example structure, builds cleanly and you've tested on hardware, open a PR directly against `main`.

---

## Adding a New Comau Robot Model

We currently support the Comau NJ-130-2.6. To add support for another Comau CRCOpen-compatible robot model, follow these steps:

1. **Create a new description package**
   * Copy the directory `comau_robots/comau_nj_130_2_6_description` and rename it to match your model, e.g., `comau_robots/comau_<model>_description`.

2. **Update metadata**
   * Edit `package.xml`:
     * Change `<name>` to `comau_<model>_description`.
     * Update `<description>` to reference your model.
   * Update `CMakeLists.txt` if necessary.

3. **Provide meshes**
   * Place CAD-derived meshes in `meshes/visual` and `meshes/collision`.
   * Ensure filenames match link names in your URDF.

4. **Author the URDF/Xacro**    
   * In `urdf/`, update `*.urdf.xacro`:

     * Define the kinematic chain with correct link/joint names.
     * Use the helper macro for visual and collision geometry.
   * Create/modify `<model>.ros2_control.xacro`:

     * Populate `<hardware>` plugin parameters (`lpc_addr`, `crc_addr`).
     * For each `<joint>`, set:

       * `axis_idx` matching CRCOpen axis numbering.
       * `ruckig_max_vel`, `ruckig_max_acc`, `ruckig_max_jrk` based on robot specs.
       * `cal_data` from teach pendant (Setup → Motion → Calib).
       * `tx_rate` from system variables (`$TX_RATE[i]`).
       * `vr_TorqConst` for torque-to-current conversion.

5. **Configure controllers**
   * Copy `controllers/controllers.yaml` and update controller names and joint lists if needed.

6. **Test your package and Verify on hardware**
   * Ensure you have access to the physical robot for validation
   * Confirm that CRCOpen is enabled in the teach pendant (AUTO TP mode).
   * Build your workspace:
   * Launch the bringup with your model:
   * Verify joint states and controller spawning.

7. **Submit a Pull Request**
   * Include your new description package under `comau_robots/`.
   * Reference your issue or discussion in the PR description.

By following this pattern, your new robot model will integrate seamlessly with the existing driver architecture and pass our CI checks.

---

## Licensing

Any contributions you make will be licensed under the Apache 2.0 license. By contributing, you agree to license your work under Apache 2.0. 

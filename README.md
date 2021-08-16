# workspace
PX4 Hexacopter Robot Operating System and Simulation using S-Curve Guidance based on Smart Vision
## Perancangan Sistem Landing Otomatis Kendaraan Udara Tak Berawak pada Platform Bergerak Menggunakan Smart Vision
Software untuk penyelesaian Tugas Akhir dari Musyafa Arif Huda
### Keyword
Hexacopter, Vision, Localization, Robot Operating System (ROS)

## Installing
### Cloning Repository
#### Installing Git
Sebelum kita "clone" suatu repository, terdapat suatu software yang dibutuhkan, yaitu [git](https://git-scm.com/)
```bash
sudo apt install git
```
#### Clone Menggunakan HTTPS
```bash
git clone --recurse-submodules https://github.com/musyafaarif/workspace.git workspace
```
"workspace" merupakan folder yang akan dituju untuk mengunduh repository. Kita dapat mengganti workspace dengan nama lain

### Workspace auto-install
Untuk menginstall seluruh dependancies yang dibutuhkan workspace, masuk dulu ke folder workspace,
```bash
cd workspace
```
kemudian install dengan perintah
```bash
source install.sh
```

## Running
Sebelum menjalankan keseluruhan simulasi, pastikan seluruh library dan program yang dibutuhkan telah diinstall dengan sempurna,
jika sudah diinstall dengan sempurna, maka untuk setiap perintah yang akan dijalankan, lakukan langkah berikut untuk memastikan
bahwa library telah masuk ke pengaturan.
Langkahnya adalah dengan terminal yang telah terbuka, masuk ke dalam folder workspace, kemudian jalankan
```bash
source setup.bash
```

### Running Simulator (Gazebo + ROS)
Untuk menjalankan simulator, buka terminal dengan langkah yang telah dijelaskan sebelumnya, kemudian jalankan
```bash
source run_sim.sh
```
Maka simulator gazebo akan berjalan, setelah itu jalankan PX4 ROS Wrapper untuk Gazebo dengan menjalankan
```bash
roslaunch ta_control mavros.launch
```

### Running Boat Control
Untuk menggerakkan kapal agar dapat bergerak, buka terminal dengan langkah yang telah dijelaskan sebelumnya, kemudian jalankan
```bash
rosrun ta_control boat_nav
```

### Running Hexacopter Control
Untuk menggerakkan hexacopter dengan sistem kontrol yang telah dibuat, buka terminal dengan langkah yang telah dijelaskan sebelumnya,
kemudian jalankan
```bash
roslaunch ta_control main.launch
```

# Change Log
All notable changes to this project will be documented in this file.

## [Unreleased]
### Added

### Changed

### Fixed


## [Week 14] - 2021-06-08
## [Week 13] - 2021-06-01
## [Week 12] - 2021-05-25
## [Week 11] - 2021-05-18
## [Week 10] - 2021-05-11
## [Week 9] - 2021-05-04
## [Week 8] - 2021-04-27
## [Week 7] - 2021-04-20
## [Week 6] - 2021-04-13
### Added
- **Setting Up Gazebo Simulation**
  - Hexacopter Model

## [Week 5] - 2021-04-06
### Added
- **Add PX4 Submodule**
  - PX4 Firmware Submodule
  - PX4 Firmware Submodule Installation Script
- **Setting Up ROS Workspace**
  - Mavlink Submodule
  - Mavros Submodule
  - Smart Vision Workspace
  - Autopilot Control Workspace
  - Catkin Workspace Sourcing Script
- **Setting Up Gazebo Simulation**
  - PX4 SITL Submodule
  - PX4 SITL Submodule Installation Scripts
  - Landing Pad Model
  - Basic Drone Model
  - Boat Model
- **Develop Smart Vision**
  - Camera Streaming
  - GStreamer Streaming from UDP
  - Color Detection
  - Color Detection Manual Tuning

### Changed

### Fixed

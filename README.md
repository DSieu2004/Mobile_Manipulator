# Husky + UR5e vẽ nét trong MuJoCo

Vẽ trên panel bằng chuột và bấm **Gửi cho robot**. OCS2 điều khiển đế omni và tay UR5e cùng bám nét vẽ; MuJoCo hiển thị vệt đỏ trên bảng. Có thể gửi nhiều bản liên tiếp. Sau mỗi bản, robot lùi, đi ra bên phải bảng và chờ; bấm **Kết thúc / Thoát MuJoCo** để đóng phiên.

## Chạy

Yêu cầu Ubuntu 22.04, ROS 2 Humble, Python 3.10, `python3-tk`, driver NVIDIA và `glxinfo` (`mesa-utils`). Tạo môi trường Python và build hai task **Build OCS2 mobile manipulator**, **Build Husky UR5e description** trong VS Code. Nếu máy thiếu Pinocchio/Coal/EigenPy trong ROS, chạy `./scripts/setup_local_pinocchio.sh` trước khi build.

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements-mujoco.txt
./scripts/run_ocs2_vr.sh
```

Script chọn RTX 4050 qua NVIDIA PRIME và báo lỗi nếu OpenGL dùng GPU khác. Khi đã mở panel, giữ chuột trái để vẽ, thả chuột để kết thúc một nét rồi bấm **Gửi cho robot**. Bản vẽ và log được lưu trong `runs/`.

```bash
./scripts/run_ocs2_vr.sh --demo-vr                    # Vẽ chữ VR mẫu
./scripts/run_ocs2_vr.sh --headless --demo-vr         # Chạy không mở viewer
./scripts/run_ocs2_vr.sh --strokes-json runs/ten.json # Phát lại bản vẽ
DRAW_VIEWER_FPS=15 ./scripts/run_ocs2_vr.sh           # Giảm tải hiển thị
```

Mã OCS2 nằm trong `src/ocs2_ros2/`, cấu hình robot trong `src/Clearpath_mobile_manipulator/`. Sau khi sửa Xacro, chạy `.venv/bin/python scripts/prepare_mujoco.py` để tạo lại scene và URDF OCS2. Các thư mục `build/`, `install/`, `.venv/`, `third_party/ros/`, `log/` và `runs/` là dữ liệu cục bộ, không đưa lên Git.

**Giới hạn:** Đây là mô phỏng dry-run: đầu bút cách bảng khoảng 5 cm, vệt đỏ là hình chiếu; không có điều khiển lực tiếp xúc. Đế đi ngang bằng mô hình omni lý tưởng; bánh Husky thật không có con lăn mecanum. Replay bản vẽ “SIEU” cho RMS sai số nét khoảng **3,25 mm** trong mô phỏng.

Nguồn model: [Husky ROS](https://github.com/husky/husky), [Universal Robots ROS 2 Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description) và [OCS2 ROS 2](https://github.com/legubiao/ocs2_ros2). Giữ các giấy phép tương ứng trong thư mục mã nguồn.

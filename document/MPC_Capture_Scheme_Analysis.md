# DartControl 数据采集与多源融合 MCAP 存储方案设计文档

本文档介绍如何在本项目中实现**视觉检测数据**、**电控 IMU/姿态数据**以及**相机的 JPEG 视频帧**合并存储进同一个 `.mcap` 数据包，并可以在 **Foxglove Studio** 可视化平台中进行同步回放与三维渲染的设计方案。

---

## 一、 技术背景介绍

### 1. 什么是 MCAP 文件格式
**MCAP** 是一种由 Foxglove 团队开发的通用、高效的结构化日志记录文件格式。它被广泛用于机器人和自动驾驶系统（如 ROS 1/2）中，类似于 ROS bag，但具有跨语言、支持多种数据序列化格式（如 Protobuf、FlatBuffers、JSON Schema 等）的优点。
MCAP 文件包含：
* **Schema（模式/定义）**：说明数据是用什么格式序列化的，如何解析。
* **Channel（通道）**：类似于 ROS 的 Topic（主题），每个 Channel 绑定一个 Schema。
* **Message（消息）**：具体存储的每一帧时序数据，包含通道 ID、序列号、日志时间戳以及序列化后的二进制负载（Payload）。

### 2. 什么是 FlatBuffers 与 Protocol Buffers (Protobuf)
* **FlatBuffers (电控与视觉数据使用)**：由 Google 开源的跨平台序列化库。其最大的特点是**零拷贝（Zero-Copy）**，即数据在写入后直接在二进制 Buffer 中定位，读取时不需要像 JSON 或 Protobuf 那样反序列化整个对象，非常适合高性能、低延迟的嵌入式控制端与电控数据流记录。
* **Protobuf (视频数据使用)**：同样由 Google 开源。它能更紧凑地压缩和包装数据，适合网络传输与流媒体等大小多变、对自描述性要求高的场景。本项目的视频帧采用了 `foxglove.CompressedImage` 的 Protobuf 标准规范进行封装。

---

## 二、 整体架构

本项目包含两个核心执行线程：
1. **`dart_control` 线程**：电控主循环。高速读取 BMI055 传感器，进行姿态解算，并定时更新电控数据。
2. **`dart_vision` 线程**：视觉检测主循环。抓取 OV5647 图像帧，进行 ROI 识别提取目标像素点。

为了将它们写入同一个 `.mcap` 包，引入了唯一的 `Capturer`（捕获器）单例：

```
                 +-------------------+
                 |   Camera (Image)  |
                 +---------+---------+
                           |
                           v (640x480 Bayer Frame)
                 +---------+---------+
                 |  dart_vision 线程 | <------+
                 +---------+---------+        | 
                           |                  |
    (JPEG Encode)          |                  | Target Coordinate
    异步入队列              |                  | & Status (g_vision_data)
                           v                  |
                    +------+------+           |
                    |   Capturer  |           |
                    +------+------+           |
                           ^                  |
                           |                  |
           电控数据同步写入 |                  |
                           |                  |
                 +---------+---------+        |
                 |  dart_control 线程| --------+
                 +-------------------+
                       (BMI055)
```

---

## 三、 Schema 与 Channel 的设计与定义

### 1. 电控通道：电解数据 (Flatbuffers)
* **Topic 名称**：`Dart`
* **Schema 结构 (CarData.fbs)**：
  定义了车辆的 FRD 坐标系下的加速度、陀螺仪三轴数据、解算出的 Roll/Pitch/Yaw 姿态角、IMU 处理帧数/FPS 和 CPU 占用率。
  ```fbs
  namespace foxglove;

  table CarData {
    acc_frd_x:float;
    acc_frd_y:float;
    acc_frd_z:float;
    gyro_frd_x:float;
    gyro_frd_y:float;
    gyro_frd_z:float;
    roll:float;
    yaw:float;
    pitch:float;
    imu_index:int;
    imu_fps:int;
    cpu_usage:int;
  }
  ```
  在 C++ 中，我们利用 Flatbuffers 编译器预先生成二进制描述文件（`.bfbs`）作为 Schema 并注册进 MCAP 中。

### 2. 视觉检测通道：视觉数据 (Flatbuffers)
* **Topic 名称**：`vision/detect`
* **Schema 结构 (VisionData.fbs)**：
  定义了图像中目标的二维像素坐标（`target_pixel_x`, `target_pixel_y`）、目标当前状态（`target_status`，如 FOUND/LOST）、视觉处理帧间隔 `frame_dt_ms` 以及实时视频帧率 `video_fps`。
  ```fbs
  namespace foxglove;

  table Vision {
    target_pixel_x:int;
    target_pixel_y:int;
    target_status:int;
    frame_dt_ms:int;
    video_fps:int;
  }
  ```

### 3. 视频通道：压缩视频帧 (Protobuf)
为了能够直接在 Foxglove Studio 中播放视频，我们采用了 Foxglove 官方支持的标准格式：`foxglove.CompressedImage`。
* **Topic 名称**：`camera/image`
* **Protobuf 定义**：
  ```protobuf
  message CompressedImage {
    google.protobuf.Timestamp timestamp = 1;
    string frame_id = 4;
    bytes  data     = 2; // JPEG 编码的二进制流
    string format   = 3; // "jpeg"
  }
  ```

---

## 四、 C++ 异步写入的线程安全实现

由于电控线程刷新率极高（高达几百或上千赫兹），而相机采集与 JPEG 编码/写入在树莓派单核 CPU 上较为耗时（200 FPS 下，进行 JPEG 编码和磁盘 I/O 容易阻塞控制环路），因此 `Capturer` 采用了 **“同步写入低开销数据 + 异步队列后台处理高开销视频”** 的混合写入架构。

### 1. 电控与视觉数据：同步锁写入（轻量快速）
由于 FlatBuffers 数据的序列化与 Buffer 指针直接复制极快，耗时在微秒级，因此直接采用互斥锁 `m_writer_mutex` 保护 `m_mcap_writer`，并在各自的工作线程同步写入。
* **`update_car_data(...)`**：在 `dart_control` 线程执行。
* **`write_vision_data(...)`**：在 `dart_vision` 线程执行。

### 2. 视频帧写入：异步编码队列（背压与丢帧策略）
由于 JPEG 图像压缩极其消耗 CPU 且磁盘 I/O 较慢，如果直接在相机回调线程中做 `cv::imencode` 和 MCAP 写入，会导致严重的卡顿与丢包。
设计方案如下：
1. **生产线程（`dart_vision`）只做深拷贝**：
   在 `write_video_frame` 中，直接通过 `cv::Mat::clone()` 克隆图像（仅内存复制，不进行任何压缩编码），然后记录当前纳秒时间戳并作为 `EncodedFrame` 对象推入 `m_write_queue` 队列中。
2. **背压与限流策略**：
   为避免内存溢出，将队列最大容量设为 3（`kMaxQueueSize = 3`）。如果队列已满且新帧到达，会直接丢弃最旧的帧（`pop_front`），确保系统实时性。
3. **后台消费线程（`writer_thread_loop`）**：
   后台线程通过 `std::condition_variable` 被唤醒，从队列取出原始图像帧，进行以下高开销操作：
   * **图像 JPEG 压缩**：调用 `cv::imencode(".jpg", ...)` 编码为 JPEG 字节数组（写入 `m_jpeg_buf`）。
   * **手动 Protobuf 序列化**：通过 `foxglove::serialize_compressed_image` 手动序列化为 `foxglove.CompressedImage` Protobuf 二进制 Buffer（写入 `m_pb_buf`）。
   * **MCAP 磁盘写入**：通过 `mcap::McapWriter::write` 序列化后的数据，使用 `m_writer_mutex` 锁定，写入 MCAP 文件中。

---

## 五、 Foxglove Studio 可视化联合播放

将电控、视觉、视频写入同一个 mcap 文件并保存完毕后，使用 Foxglove Studio 可以打开该文件：
1. **Image 视图**：订阅 `/camera/image` 通道，可直接看到摄像头拍下的实时画面。
2. **Data Inspection 视图**：订阅 `/Dart`，能以波形图的形式同步观测电控姿态、陀螺仪读数与 CPU 负载的波动。
3. **Plot 视图**：可同步比对视觉识别的目标像素位置（`/vision/detect` 中的 `target_pixel_x/y`）与电控解算姿态在时域上的动态配合。

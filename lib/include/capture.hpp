#pragma once
#include <string>
#include <memory>
#include <string_view>
#include <mutex>
#include <vector>
#include <deque>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <flatbuffers/flatbuffers.h>
#include <opencv2/core/mat.hpp>
#include "mcap/writer.hpp"

class CarData;

/// @brief 待编码写入的原始视频帧（轻量，仅用于队列传输）
struct EncodedFrame
{
    cv::Mat               raw_frame;      // 原始帧（深拷贝副本，后台线程对其编码）
    int                   jpeg_quality = 70;
    mcap::ChannelId       channel_id = 0;
    uint32_t              sequence = 0;
    uint64_t              log_time = 0;
    uint64_t              publish_time = 0;
};

class Capturer
{
public:
    Capturer();
    ~Capturer();
    bool init();
    bool update_car_data(CarData& car_data);

    /// @brief 异步写入一帧 JPEG 压缩视频帧到 mcap（非阻塞，入队后立即返回）
    bool write_video_frame(const cv::Mat& frame, int jpeg_quality = 70);

    /// @brief 写入一帧视觉检测数据到 mcap（线程安全）
    bool write_vision_data(int target_pixel_x, int target_pixel_y,
                           int target_status, int frame_dt_ms, int video_fps);

    void finish();

    // 返回本次运行生成的 MCAP 文件路径。
    const std::string& mcap_path() const { return m_mcap_path; }

private:
    static constexpr size_t kMaxQueueSize = 3;  // 队列上限（满则丢旧帧）

    // ── 文件路径 ──
    std::string m_mcap_path{};

    // ── 电控数据 channel ──
    std::string m_CarData_schema_name = "foxglove.CarData";
    std::string m_CarData_schema_data;
    std::unique_ptr<mcap::Schema> m_CarData_schema;
    std::string m_topic_name = "Dart";
    std::unique_ptr<mcap::Channel> m_CarData_channel;
    uint32_t m_sequence = 0;
    flatbuffers::FlatBufferBuilder m_builder;

    // ── 视频 channel ──
    std::string m_video_schema_name = "foxglove.CompressedImage";
    std::string m_video_schema_data;
    std::unique_ptr<mcap::Schema> m_video_schema;
    std::string m_video_topic_name = "camera/image";
    std::unique_ptr<mcap::Channel> m_video_channel;
    uint32_t m_video_sequence = 0;
    std::vector<uint8_t> m_jpeg_buf;     // JPEG 压缩缓冲区（复用）
    std::vector<uint8_t> m_pb_buf;       // protobuf 序列化缓冲区（复用）

    // ── 视觉检测 channel ──
    std::string m_vision_schema_name = "foxglove.Vision";
    std::string m_vision_schema_data;
    std::unique_ptr<mcap::Schema> m_vision_schema;
    std::string m_vision_topic_name = "vision/detect";
    std::unique_ptr<mcap::Channel> m_vision_channel;
    uint32_t m_vision_sequence = 0;
    flatbuffers::FlatBufferBuilder m_vision_builder;

    // ── 异步写入（后台线程 + 队列）──
    std::thread              m_writer_thread;
    std::deque<EncodedFrame> m_write_queue;
    std::mutex               m_queue_mutex;
    std::condition_variable  m_queue_cv;
    std::atomic<bool>        m_writer_running{false};

    // ── 状态 ──
    bool m_opened = false;

    // ── 线程安全 ──
    std::mutex m_writer_mutex;

    mcap::McapWriter m_mcap_writer;

    void try_to_create_target_file(std::string_view path);
    std::string get_file_Contents(std::string_view path);
    std::string get_local_time_now();
    void writer_thread_loop();  // 后台写入线程主循环

};

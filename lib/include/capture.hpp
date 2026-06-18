#pragma once
#include <string>
#include <memory>
#include <string_view>
#include <mutex>
#include <vector>
#include <flatbuffers/flatbuffers.h>
#include <opencv2/core/mat.hpp>
#include "mcap/writer.hpp"

class CarData;

class Capturer
{
public:
    Capturer();
    ~Capturer();
    bool init();
    bool update_car_data(CarData& car_data);

    /// @brief 写入一帧 JPEG 压缩视频帧到 mcap（线程安全）
    bool write_video_frame(const cv::Mat& frame, int jpeg_quality = 70);

    void finish();

private:
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

    // ── 状态 ──
    bool m_opened = false;

    // ── 线程安全 ──
    std::mutex m_writer_mutex;

    mcap::McapWriter m_mcap_writer;

    void try_to_create_target_file(std::string_view path);
    std::string get_file_Contents(std::string_view path);
    std::string get_local_time_now();

};
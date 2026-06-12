#pragma once
#include <string>
#include <memory>
#include <string_view>
#include <flatbuffers/flatbuffers.h>
#include "mcap/writer.hpp"

class CarData;

class Capture
{
public:
    Capture();
    ~Capture();
    bool init();
    bool update(CarData& car_data);
    void finish();

private:
    std::string m_mcap_path{};
    std::string m_CarData_schema_name = "foxglove.CarData";
    std::string m_CarData_schema_data;
    std::unique_ptr<mcap::Schema> m_CarData_schema;
    std::string m_topic_name = "Dart";
    std::unique_ptr<mcap::Channel> m_CarData_channel;
    uint32_t m_sequence = 0;
    bool m_opened = false;
    flatbuffers::FlatBufferBuilder m_builder;

    mcap::McapWriter m_mcap_writer;

    void try_to_create_target_file(std::string_view path);
    std::string get_file_Contents(std::string_view path);
    std::string get_local_time_now();

};
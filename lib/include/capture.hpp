#pragma once
#include <iostream>
#include <flatbuffers/flatbuffers.h>
#include "../include/mcap/writer.hpp"
#include "../include/data.hpp"


class Capture
{
public:
    Capture();
    ~Capture() = default;
    bool init();
    bool update(CarData& car_data);
    void finish();

private:
    std::string m_mcap_path = " ";
    std::string m_CarData_schema_name = "foxglove.CarData";
    std::string m_CarData_schema_data;
    std::unique_ptr<mcap::Schema> m_CarData_schema;
    std::string m_topic_name = "Dart";
    std::unique_ptr<mcap::Channel> m_CarData_channel;
    flatbuffers::FlatBufferBuilder m_builer;

    mcap::McapWriter m_mcap_writer;

    void try_to_create_aim_file(std::string_view path);
    std::string get_file_Contents(std::string_view path);
    std::string get_local_time_now();

};
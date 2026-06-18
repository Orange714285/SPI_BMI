#pragma once
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace pb {

// ── Varint encoding ──────────────────────────────────────

inline void write_varint(std::vector<uint8_t>& buf, uint64_t value) {
    while (value >= 0x80) {
        buf.push_back(static_cast<uint8_t>(value | 0x80));
        value >>= 7;
    }
    buf.push_back(static_cast<uint8_t>(value));
}

inline size_t varint_size(uint64_t value) {
    size_t n = 1;
    while (value >= 0x80) { value >>= 7; ++n; }
    return n;
}

// ── Wire type helpers ────────────────────────────────────
// tag = (field_number << 3) | wire_type
// wire_type: 0=varint, 2=length-delimited

constexpr uint8_t wire_varint = 0;
constexpr uint8_t wire_ldelim = 2;

inline void write_tag(std::vector<uint8_t>& buf, uint32_t field_number, uint8_t wire_type) {
    write_varint(buf, (field_number << 3) | wire_type);
}

inline void write_uint64(std::vector<uint8_t>& buf, uint32_t field, uint64_t value) {
    write_tag(buf, field, wire_varint);
    write_varint(buf, value);
}

inline void write_length_delimited(std::vector<uint8_t>& buf, uint32_t field,
                                    const uint8_t* data, size_t len) {
    write_tag(buf, field, wire_ldelim);
    write_varint(buf, len);
    buf.insert(buf.end(), data, data + len);
}

inline void write_len_str(std::vector<uint8_t>& buf, uint32_t field, const std::string& s) {
    write_length_delimited(buf, field,
        reinterpret_cast<const uint8_t*>(s.data()), s.size());
}

// ── Sub-message encoding ─────────────────────────────────
// A nested message is just length-delimited with wire type 2

inline void write_submsg_begin(std::vector<uint8_t>& buf, uint32_t field,
                                std::vector<uint8_t>& sub) {
    (void)buf; (void)field;
    sub.clear();
}
inline void write_submsg_end(std::vector<uint8_t>& buf, uint32_t field,
                              const std::vector<uint8_t>& sub) {
    write_tag(buf, field, wire_ldelim);
    write_varint(buf, sub.size());
    buf.insert(buf.end(), sub.begin(), sub.end());
}

}  // namespace pb

// ── foxglove.CompressedImage manual serializer ───────────
//
// proto definition:
//   message CompressedImage {
//     google.protobuf.Timestamp timestamp = 1;
//     string frame_id = 4;
//     bytes  data     = 2;
//     string format   = 3;
//   }
//   google.protobuf.Timestamp:
//     int64 seconds = 1;
//     int32 nanos   = 2;

namespace foxglove {

inline std::vector<uint8_t> serialize_compressed_image(
    int64_t seconds, int32_t nanos,
    const std::string& frame_id,
    const uint8_t* jpeg_data, size_t jpeg_size,
    const std::string& format = "jpeg")
{
    std::vector<uint8_t> buf;

    // field 1: Timestamp (sub-message)
    {
        std::vector<uint8_t> ts;
        pb::write_uint64(ts, 1, static_cast<uint64_t>(seconds));   // seconds
        pb::write_uint64(ts, 2, static_cast<uint64_t>(nanos));     // nanos
        pb::write_submsg_end(buf, 1, ts);
    }

    // field 4: frame_id
    pb::write_len_str(buf, 4, frame_id);

    // field 2: data (JPEG bytes)
    pb::write_length_delimited(buf, 2, jpeg_data, jpeg_size);

    // field 3: format ("jpeg")
    pb::write_len_str(buf, 3, format);

    return buf;
}

}  // namespace foxglove

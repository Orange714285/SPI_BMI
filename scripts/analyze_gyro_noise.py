#!/usr/bin/env python3
"""
提取 MCAP 文件中 2s~4s 的陀螺仪原始观测数据，
计算三个轴的观测噪声方差 (rad^2/s^2)。
"""

import struct
import sys
import numpy as np
from mcap.reader import make_reader

MCAP_PATH = "/home/orange/pi-workspace/DartControl/bag/2026_7_14_21_46_10.mcap"

VT = {
    "acc_frd_x_mg": 4, "acc_frd_y_mg": 6, "acc_frd_z_mg": 8,
    "gyro_frd_x_dps": 10, "gyro_frd_y_dps": 12, "gyro_frd_z_dps": 14,
    "gyro_raw_frd_x_dps": 16, "gyro_raw_frd_y_dps": 18, "gyro_raw_frd_z_dps": 20,
}

def parse_car_data_message(data: bytes):
    if len(data) < 8:
        return None
    root_offset = struct.unpack_from('<I', data, 0)[0]
    table_start = root_offset  # root_offset 是从 buffer 起点到 root table 的偏移
    if table_start + 4 > len(data):
        return None
    # vtable 偏移: table 第一字段是 soffset_t, 指向 table 之前的 vtable
    vtable_off_signed = struct.unpack_from('<i', data, table_start)[0]
    vtable_start = table_start - vtable_off_signed
    if vtable_start < 0 or vtable_start + 4 > len(data):
        return None
    table_size = struct.unpack_from('<H', data, vtable_start + 2)[0]

    result = {}
    for field_name, vt_value in VT.items():
        field_offset_pos = vtable_start + vt_value
        if field_offset_pos + 2 > len(data):
            continue
        field_off = struct.unpack_from('<H', data, field_offset_pos)[0]
        if field_off == 0 or field_off >= table_size:
            result[field_name] = 0.0
            continue
        field_addr = table_start + field_off
        if field_addr + 4 > len(data):
            continue
        result[field_name] = struct.unpack_from('<f', data, field_addr)[0]

    int_fields = {"IMU_index": 46, "IMU_fps": 48, "m_cpu_usage": 50, "m_state": 52}
    for field_name, vt_value in int_fields.items():
        field_offset_pos = vtable_start + vt_value
        if field_offset_pos + 2 > len(data):
            continue
        field_off = struct.unpack_from('<H', data, field_offset_pos)[0]
        if field_off == 0 or field_off >= table_size:
            result[field_name] = 0
            continue
        field_addr = table_start + field_off
        if field_addr + 4 > len(data):
            continue
        result[field_name] = struct.unpack_from('<i', data, field_addr)[0]
    return result


def main():
    print(f"[INFO] Opening MCAP: {MCAP_PATH}", file=sys.stderr)

    gyro_raw_data = []
    dart_channel_id = None

    with open(MCAP_PATH, 'rb') as f:
        reader = make_reader(f)
        first_log_time = None

        for schema, channel, msg in reader.iter_messages():
            # 首次遇到 Dart channel 记录其 id
            if channel.topic == "Dart" and dart_channel_id is None:
                dart_channel_id = channel.id

            if dart_channel_id is None or channel.id != dart_channel_id:
                continue

            parsed = parse_car_data_message(msg.data)
            if parsed is None:
                continue
            if first_log_time is None:
                first_log_time = msg.log_time

            t_sec = (msg.log_time - first_log_time) / 1e9
            gx = parsed.get("gyro_raw_frd_x_dps", 0.0)
            gy = parsed.get("gyro_raw_frd_y_dps", 0.0)
            gz = parsed.get("gyro_raw_frd_z_dps", 0.0)
            gyro_raw_data.append((t_sec, gx, gy, gz))

    print(f"[INFO] Total Dart messages: {len(gyro_raw_data)}", file=sys.stderr)
    if len(gyro_raw_data) == 0:
        print("[ERROR] No Dart messages found!", file=sys.stderr)
        sys.exit(1)

    print(f"[INFO] Time range: {gyro_raw_data[0][0]:.2f}s ~ {gyro_raw_data[-1][0]:.2f}s",
          file=sys.stderr)

    # 打印前三帧
    print("--- 前三帧原始数据 (°/s) ---")
    for i in range(min(3, len(gyro_raw_data))):
        t, gx, gy, gz = gyro_raw_data[i]
        print(f"  t={t:.4f}s: gyro_raw=({gx:+.4f}, {gy:+.4f}, {gz:+.4f}) °/s")

    t_start, t_end = 2.0, 4.0
    window_data = [(t, gx, gy, gz) for t, gx, gy, gz in gyro_raw_data
                   if t_start <= t < t_end]

    print(f"[INFO] Data points in [{t_start}s, {t_end}s): {len(window_data)}",
          file=sys.stderr)

    if len(window_data) < 10:
        print(f"[WARNING] Too few samples ({len(window_data)}), using all data",
              file=sys.stderr)
        window_data = gyro_raw_data

    data_dps = np.array([(gx, gy, gz) for _, gx, gy, gz in window_data])
    data_radps = data_dps * (np.pi / 180.0)

    mean_radps = np.mean(data_radps, axis=0)
    var_radps2 = np.var(data_radps, axis=0, ddof=1)
    std_radps = np.std(data_radps, axis=0, ddof=1)

    print()
    print("=" * 75)
    print("  陀螺仪观测噪声分析 (rad^2/s^2)")
    print(f"  数据区间: {t_start}s ~ {t_end}s, {len(window_data)} 个样本")
    print("=" * 75)
    print(f"  {'轴':>8s}  {'均值 (rad/s)':>16s}  {'方差 (rad²/s²)':>16s}"
          f"  {'标准差 (rad/s)':>16s}  {'均值 (°/s)':>12s}")
    print(f"  {'-'*8}  {'-'*16}  {'-'*16}  {'-'*16}  {'-'*12}")
    for i, name in enumerate(['X (FRD Roll)', 'Y (FRD Pitch)', 'Z (FRD Yaw)']):
        print(f"  {name:>14s}  {mean_radps[i]:>16.6e}  {var_radps2[i]:>16.6e}"
              f"  {std_radps[i]:>16.6e}  {data_dps[:, i].mean():>12.4f}")

    print()
    print("  残余零偏漂移:")
    for i, name in enumerate(['X', 'Y', 'Z']):
        drift_dps = data_dps[:, i].mean()
        drift_abs = abs(drift_dps)
        if drift_abs > 1e-6:
            sec_per_deg = 1.0 / drift_abs
            print(f"    {name}轴: {drift_dps:+.4f} °/s  →  每 {sec_per_deg:.1f}s 漂移 1°")
        else:
            print(f"    {name}轴: {drift_dps:+.4f} °/s  →  无显著漂移")


if __name__ == "__main__":
    main()

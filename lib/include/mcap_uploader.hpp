#pragma once

#include <string>

// 将已经关闭的 MCAP 文件上传到 config 中指定的电脑目录。
bool upload_mcap(const std::string& local_path);

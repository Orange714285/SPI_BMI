#pragma once

#include <string>
#include <vector>

// 将已经关闭的 MCAP 文件（以及同批产物，如配置快照）上传到 config 中指定的电脑目录。
// 多个文件共用一次 scp 传输。
bool upload_mcap(const std::vector<std::string>& local_paths);

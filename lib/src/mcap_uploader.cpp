#include "mcap_uploader.hpp"

#include "config.hpp"

#include <iostream>
#include <string>
#include <cstdio>      // std::remove
#include <sys/wait.h>
#include <unistd.h>

bool upload_mcap(const std::vector<std::string>& local_paths)
{
    if (local_paths.empty())
    {
        std::cerr << "[ERROR] No local file to upload." << std::endl;
        return false;
    }

    // 组成 scp 使用的目标地址：用户名@电脑IP:目标目录/。
    const std::string destination =
        std::string(config::MCAP_UPLOAD_USER) + "@" +
        config::MCAP_UPLOAD_HOST + ":" +
        config::MCAP_UPLOAD_DIRECTORY + "/";

    // 全部 argv 都在 fork 之前组装好：子进程里不再做任何可能分配内存的操作，
    // 避免多线程 fork 后撞上其它线程持有的分配器锁。
    std::vector<std::string> args;
    args.reserve(local_paths.size() + 2);
    args.push_back("scp");
    for (const auto& path : local_paths)
    {
        args.push_back(path);
    }
    args.push_back(destination);

    std::vector<char*> argv;
    argv.reserve(args.size() + 1);
    for (auto& arg : args)
    {
        argv.push_back(arg.data());  // C++17：data() 返回非 const 的 char*
    }
    argv.push_back(nullptr);

    std::cout << "[INFO] Uploading " << local_paths.size()
              << " file(s) to " << destination << std::endl;
    for (const auto& path : local_paths)
    {
        std::cout << "        " << path << std::endl;
    }

    // 创建子进程执行 scp，并等待上传结束。
    const pid_t pid = fork();
    if (pid == 0)
    {
        execvp(argv[0], argv.data());
        _exit(127);
    }
    if (pid < 0)
    {
        std::cerr << "[ERROR] Failed to start scp." << std::endl;
        return false;
    }

    int status = 0;
    if (waitpid(pid, &status, 0) < 0)
    {
        std::cerr << "[ERROR] Failed to wait for scp." << std::endl;
        return false;
    }

    if (WIFEXITED(status) && WEXITSTATUS(status) == 0)
    {
        std::cout << "[INFO] Upload succeeded." << std::endl;

        // 回传成功后删除本地副本，避免树莓派 bag/ 被逐发数据撑满。
        // ① 只有 scp 正常退出（exit 0）才删；② 路径必须位于 bag/ 下，防止误删。
        if (config::MCAP_DELETE_LOCAL_AFTER_UPLOAD)
        {
            for (const auto& path : local_paths)
            {
                if (path.find("/bag/") == std::string::npos)
                {
                    std::cerr << "[WARN] Refuse to delete (not under bag/): " << path << std::endl;
                    continue;
                }
                if (std::remove(path.c_str()) == 0)
                {
                    std::cout << "[INFO] Removed local file: " << path << std::endl;
                }
                else
                {
                    std::cerr << "[WARN] Failed to remove local file: " << path << std::endl;
                }
            }
        }
        return true;
    }

    std::cerr << "[ERROR] Upload failed." << std::endl;
    return false;
}

#include "mcap_uploader.hpp"

#include "config.hpp"

#include <iostream>
#include <string>
#include <sys/wait.h>
#include <unistd.h>

bool upload_mcap(const std::string& local_path)
{
    // 组成 scp 使用的目标地址：用户名@电脑IP:目标目录/。
    const std::string destination =
        std::string(config::MCAP_UPLOAD_USER) + "@" +
        config::MCAP_UPLOAD_HOST + ":" +
        config::MCAP_UPLOAD_DIRECTORY + "/";

    std::cout << "[INFO] Uploading MCAP to " << destination << std::endl;

    // 创建子进程执行 scp，并等待上传结束。
    const pid_t pid = fork();
    if (pid == 0)
    {
        execlp("scp", "scp", local_path.c_str(), destination.c_str(),
               static_cast<char*>(nullptr));
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
        std::cout << "[INFO] MCAP upload succeeded." << std::endl;
        return true;
    }

    std::cerr << "[ERROR] MCAP upload failed." << std::endl;
    return false;
}

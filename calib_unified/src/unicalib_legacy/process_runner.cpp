/**
 * UniCalib C++ — 外部进程调用实现 (Linux fork/exec + pipe)
 */
#include "unicalib/process_runner.hpp"
#include "unicalib/logger.hpp"
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <array>
#include <chrono>
#include <thread>
#include <filesystem>

#if defined(__linux__) || defined(__APPLE__)
#include <unistd.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cerrno>
#endif

namespace unicalib {

namespace {

constexpr size_t kPipeBufSize = 65536;

#if defined(__linux__) || defined(__APPLE__)
// 从 fd 读到字符串，非阻塞读直到 EOF 或出错
std::string read_pipe_until_eof(int fd) {
  std::string out;
  std::array<char, 4096> buf{};
  for (;;) {
    ssize_t n = read(fd, buf.data(), buf.size());
    if (n > 0)
      out.append(buf.data(), static_cast<size_t>(n));
    else if (n == 0)
      break;
    else if (errno != EINTR)
      break;
  }
  return out;
}

int run_process_impl(
  const std::vector<std::string>& argv,
  const std::unordered_map<std::string, std::string>& env,
  const std::string& cwd,
  int timeout_seconds,
  ProcessResult& out) {
  if (argv.empty()) {
    out.error_message = "run_process: argv empty";
    return -1;
  }

  int stdout_pipe[2], stderr_pipe[2];
  if (pipe(stdout_pipe) != 0 || pipe(stderr_pipe) != 0) {
    out.error_message = "pipe() failed";
    return -1;
  }

  pid_t pid = fork();
  if (pid < 0) {
    out.error_message = "fork() failed";
    close(stdout_pipe[0]); close(stdout_pipe[1]);
    close(stderr_pipe[0]); close(stderr_pipe[1]);
    return -1;
  }

  if (pid == 0) {
    // child
    close(stdout_pipe[0]);
    close(stderr_pipe[0]);
    dup2(stdout_pipe[1], STDOUT_FILENO);
    dup2(stderr_pipe[1], STDERR_FILENO);
    close(stdout_pipe[1]);
    close(stderr_pipe[1]);

    if (!cwd.empty()) {
      if (chdir(cwd.c_str()) != 0) {
        _exit(127);
      }
    }

    std::vector<char*> argv_ptrs;
    for (const auto& a : argv)
      argv_ptrs.push_back(const_cast<char*>(a.c_str()));
    argv_ptrs.push_back(nullptr);

    for (const auto& [k, v] : env) {
      setenv(k.c_str(), v.c_str(), 1);
    }
    execvp(argv[0].c_str(), argv_ptrs.data());
    _exit(127);
  }

  // parent
  close(stdout_pipe[1]);
  close(stderr_pipe[1]);
  out.stdout_output = read_pipe_until_eof(stdout_pipe[0]);
  out.stderr_output = read_pipe_until_eof(stderr_pipe[0]);
  close(stdout_pipe[0]);
  close(stderr_pipe[0]);

  int status = 0;
  if (timeout_seconds > 0) {
    int waited = 0;
    while (waited < timeout_seconds * 10) {
      pid_t r = waitpid(pid, &status, WNOHANG);
      if (r == pid) break;
      if (r < 0) break;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      ++waited;
    }
    if (waitpid(pid, &status, WNOHANG) == 0) {
      kill(pid, SIGKILL);
      waitpid(pid, &status, 0);
      out.timed_out = true;
      out.exit_code = -1;
      return -1;
    }
  } else {
    waitpid(pid, &status, 0);
  }

  if (WIFEXITED(status))
    out.exit_code = WEXITSTATUS(status);
  else
    out.exit_code = -1;
  return out.exit_code;
}
#endif

}  // namespace

ProcessResult run_process(
  const std::vector<std::string>& argv,
  const std::unordered_map<std::string, std::string>& env,
  int timeout_seconds,
  const std::string& cwd) {
  ProcessResult out;
#if defined(__linux__) || defined(__APPLE__)
  run_process_impl(argv, env, cwd, timeout_seconds, out);
#else
  (void)argv;
  (void)env;
  (void)timeout_seconds;
  (void)cwd;
  out.error_message = "run_process not implemented on this platform (Linux/macOS only)";
  out.exit_code = -1;
#endif
  return out;
}

bool check_executable(const std::string& path) {
  if (path.empty()) return false;
  std::error_code ec;
  auto st = std::filesystem::status(path, ec);
  if (ec || !std::filesystem::exists(st)) return false;
  if (!std::filesystem::is_regular_file(st)) return false;
  std::filesystem::perms p = st.permissions();
  return (p & std::filesystem::perms::owner_exec) != std::filesystem::perms::none ||
         (p & std::filesystem::perms::others_read) != std::filesystem::perms::none;
}

std::string find_python3() {
  const char* candidates[] = {"python3", "python"};
  for (const char* name : candidates) {
    ProcessResult r = run_process({name, "--version"}, {}, 2, "");
    if (r.exit_code == 0) return name;
  }
  return "python3";
}

}  // namespace unicalib

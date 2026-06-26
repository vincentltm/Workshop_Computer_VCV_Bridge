/**
 * SerialPort.hpp  —  Cross-platform serial port helper
 *
 * Thin RAII wrapper around OS serial APIs.
 * No external library dependency.
 *
 * Usage:
 *   SerialPort port;
 *   if (port.open("/dev/cu.usbmodem101")) {
 *       port.write(buf, len);
 *       int n = port.read(buf, max);
 *   }
 *   port.close();
 */

#pragma once
#include <string>
#include <vector>
#include <cstring>

#ifdef _WIN32
  #ifndef WIN32_LEAN_AND_MEAN
  #define WIN32_LEAN_AND_MEAN
  #endif
  #include <windows.h>
#else
  #include <fcntl.h>
  #include <unistd.h>
  #include <termios.h>
  #include <sys/select.h>
  #include <dirent.h>
  #include <errno.h>
#endif

class SerialPort {
public:
#ifdef _WIN32
    using Handle = HANDLE;
    static constexpr Handle INVALID = INVALID_HANDLE_VALUE;
#else
    using Handle = int;
    static constexpr Handle INVALID = -1;
#endif

    SerialPort()  : fd_(INVALID) {}
    ~SerialPort() { close(); }

    // Non-copyable
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& path) {
        close();
#ifdef _WIN32
        std::string wpath = "\\\\.\\" + path;
        fd_ = CreateFileA(wpath.c_str(), GENERIC_READ | GENERIC_WRITE,
                          0, NULL, OPEN_EXISTING, 0, NULL);
        if (fd_ == INVALID) return false;

        DCB dcb = {};
        dcb.DCBlength = sizeof(dcb);
        GetCommState(fd_, &dcb);
        dcb.BaudRate = CBR_115200;  // Baud doesn't matter for USB CDC — OS ignores it
        dcb.ByteSize = 8;
        dcb.StopBits = ONESTOPBIT;
        dcb.Parity   = NOPARITY;
        SetCommState(fd_, &dcb);

        COMMTIMEOUTS to = {};
        to.ReadIntervalTimeout         = 1;
        to.ReadTotalTimeoutMultiplier  = 0;
        to.ReadTotalTimeoutConstant    = 2;
        to.WriteTotalTimeoutMultiplier = 0;
        to.WriteTotalTimeoutConstant   = 100;
        SetCommTimeouts(fd_, &to);

        path_ = path;
        return true;
#else
        fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) { fd_ = INVALID; return false; }

        struct termios tty = {};
        tcgetattr(fd_, &tty);

        // Raw mode — baud rate irrelevant for USB CDC but set it anyway
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);
        cfmakeraw(&tty);
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 0;  // non-blocking read

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) { ::close(fd_); fd_ = INVALID; return false; }

        path_ = path;
        return true;
#endif
    }

    void close() {
        if (fd_ == INVALID) return;
#ifdef _WIN32
        CloseHandle(fd_);
#else
        ::close(fd_);
#endif
        fd_ = INVALID;
        path_.clear();
    }

    bool is_open() const { return fd_ != INVALID; }

    const std::string& path() const { return path_; }

    // Write all bytes; returns false on error
    bool write(const uint8_t* data, size_t len) {
        if (fd_ == INVALID) return false;
        size_t written = 0;
        while (written < len) {
#ifdef _WIN32
            DWORD n = 0;
            if (!WriteFile(fd_, data + written, (DWORD)(len - written), &n, NULL)) return false;
            written += n;
#else
            ssize_t n = ::write(fd_, data + written, len - written);
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) { usleep(100); continue; }
                return false;
            }
            written += (size_t)n;
#endif
        }
        return true;
    }

    // Non-blocking read; returns bytes read (0 if none available, <0 on error)
    int read(uint8_t* buf, size_t max_len) {
        if (fd_ == INVALID) return -1;
#ifdef _WIN32
        DWORD n = 0;
        if (!ReadFile(fd_, buf, (DWORD)max_len, &n, NULL)) return -1;
        return (int)n;
#else
        ssize_t n = ::read(fd_, buf, max_len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
            return -1;
        }
        return (int)n;
#endif
    }

    // ── Port enumeration ──────────────────────────────────────────────────────
    // Returns paths of likely Workshop Computer ports on this platform.

    static std::vector<std::string> enumerate() {
        std::vector<std::string> ports;
#ifdef _WIN32
        for (int i = 1; i <= 256; i++) {
            std::string name = "COM" + std::to_string(i);
            std::string path = "\\\\.\\" + name;
            HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE,
                                   0, NULL, OPEN_EXISTING, 0, NULL);
            if (h != INVALID_HANDLE_VALUE) {
                CloseHandle(h);
                ports.push_back(name);
            }
        }
#elif defined(__APPLE__)
        DIR* d = opendir("/dev");
        if (d) {
            struct dirent* e;
            while ((e = readdir(d))) {
                std::string n = e->d_name;
                // macOS CDC devices appear as cu.usbmodem*
                if (n.find("cu.usbmodem") == 0)
                    ports.push_back("/dev/" + n);
            }
            closedir(d);
        }
#else
        // Linux: ttyACM* for CDC ACM devices
        DIR* d = opendir("/dev");
        if (d) {
            struct dirent* e;
            while ((e = readdir(d))) {
                std::string n = e->d_name;
                if (n.find("ttyACM") == 0)
                    ports.push_back("/dev/" + n);
            }
            closedir(d);
        }
#endif
        return ports;
    }

private:
    Handle fd_;
    std::string path_;
};

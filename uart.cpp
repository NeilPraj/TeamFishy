#include "uart.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cerrno>
#include <cstring>
#include <string>

namespace uart {

static int g_fd = -1;

static speed_t to_speed(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: default: return B115200;
    }
}

bool open(const std::string& device, int baud) {
    if (g_fd >= 0) ::close(g_fd), g_fd = -1;

    int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY);
    if (fd < 0) return false;

    termios tio{};
    if (tcgetattr(fd, &tio) < 0) { ::close(fd); return false; }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);   // ignore modem lines, enable RX
    tio.c_cflag &= ~CRTSCTS;           // no HW flow control
    tio.c_cflag &= ~PARENB;            // no parity
    tio.c_cflag &= ~CSTOPB;            // 1 stop
    tio.c_cflag &= ~CSIZE;  tio.c_cflag |= CS8; // 8 data bits

    speed_t spd = to_speed(baud);
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);

    // Non-blocking-ish reads: return after timeout if no data.
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0; // we’ll handle timeouts with select()

    if (tcsetattr(fd, TCSANOW, &tio) < 0) { ::close(fd); return false; }
    tcflush(fd, TCIOFLUSH);

    g_fd = fd;
    return true;
}

bool is_open() { return g_fd >= 0; }

void close() {
    if (g_fd >= 0) { ::close(g_fd); g_fd = -1; }
}

bool send(std::string_view data) {
    if (g_fd < 0) return false;
    const char* p = data.data();
    size_t left = data.size();
    while (left) {
        ssize_t n = ::write(g_fd, p, left);
        if (n < 0) {
            if (errno == EINTR) continue;
            return false;
        }
        p += n; left -= static_cast<size_t>(n);
    }
    return true;
}

std::string recv(size_t max_bytes, int timeout_ms) {
    std::string out;
    if (g_fd < 0 || max_bytes == 0) return out;

    // Wait for readability up to timeout_ms
    fd_set rfds; FD_ZERO(&rfds); FD_SET(g_fd, &rfds);
    timeval tv{};
    if (timeout_ms < 0) timeout_ms = 0;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int r = ::select(g_fd + 1, &rfds, nullptr, nullptr, &tv);
    if (r <= 0) return out; // timeout or error -> return ""

    out.resize(max_bytes);
    ssize_t n = ::read(g_fd, out.data(), max_bytes);
    if (n <= 0) { out.clear(); return out; }
    out.resize(static_cast<size_t>(n));
    return out;
}

bool flush_rx() {
    if (g_fd < 0) return false;
    return (::tcflush(g_fd, TCIFLUSH) == 0);
}

} // namespace uart

#pragma once
#include <string>
#include <string_view>

namespace uart {

// Open a serial port. Defaults: /dev/ttyS2, 115200 baud, 8-N-1, raw.
bool open(const std::string& device = "/dev/ttyS2", int baud = 115200);

// True if port is open.
bool is_open();

// Close the port (safe to call if not open).
void close();

// Send bytes (returns true on success; sends all or returns false).
bool send(std::string_view data);

// Receive up to max_bytes; waits up to timeout_ms (returns "" on timeout or error).
std::string recv(size_t max_bytes = 256, int timeout_ms = 1000);

// Flush RX (input) buffer (discard unread bytes). Returns true on success.
bool flush_rx();

} // namespace uart

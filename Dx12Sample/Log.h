#pragma once

#ifdef USE_LOGGER

#include <vector>
#include <sstream>
#include <mutex>
enum class LogLevel {
    Log,
    Warning,
    Error
};

struct LogEntry {
    std::string message;
    LogLevel level;
};

class Log {
public:
    static Log& Info();
    static Log& Warn();
    static Log& Error();

    template<typename T>
    Log& operator<<(const T& value) {
        buffer_ << value;
        return *this;
    }

    Log& operator<<(std::ostream& (*manip)(std::ostream&));

    static const std::vector<LogEntry>& GetEntries();
    static void Clear();

private:
    Log(LogLevel level);

    void flush();

    static std::vector<LogEntry> logEntries_;
    static std::mutex mutex_;
    static size_t maxEntries_;

    std::ostringstream buffer_;
    LogLevel level_;
};

#endif
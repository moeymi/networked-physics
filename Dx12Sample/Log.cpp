#ifdef USE_LOGGER
#include "Log.h"

std::vector<LogEntry> Log::logEntries_;
std::mutex Log::mutex_;
size_t Log::maxEntries_ = 100;

Log::Log(LogLevel level) : level_(level) {}

Log& Log::Info() {
    static Log instance(LogLevel::Log);
    instance.level_ = LogLevel::Log;
    instance.buffer_.str("");
    return instance;
}

Log& Log::Warn() {
    static Log instance(LogLevel::Warning);
    instance.level_ = LogLevel::Warning;
    instance.buffer_.str("");
    return instance;
}

Log& Log::Error() {
    static Log instance(LogLevel::Error);
    instance.level_ = LogLevel::Error;
    instance.buffer_.str("");
    return instance;
}

Log& Log::operator<<(std::ostream& (*manip)(std::ostream&)) {
    if (manip == static_cast<std::ostream & (*)(std::ostream&)>(std::endl)) {
        flush();
    }
    else {
        manip(buffer_);
    }
    return *this;
}

void Log::flush() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (logEntries_.size() >= maxEntries_) {
        logEntries_.erase(logEntries_.begin());
    }
    logEntries_.push_back({ buffer_.str(), level_ });
    buffer_.str("");
    buffer_.clear();
}

const std::vector<LogEntry>& Log::GetEntries() {
    return logEntries_;
}

void Log::Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    logEntries_.clear();
}
#endif
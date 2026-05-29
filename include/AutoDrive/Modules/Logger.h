#pragma once
#include <spdlog/spdlog.h>
#include <string>

namespace Logger
{
	// Initializes the global default logger: rotating file + colored console.
	// Total retained log size ~= (rotatedFiles + 1) * fileSizeKB (current + backups).
	// Call once at program start (before other logging). Returns false on failure.
	bool InitLogger(const std::string &filePath, int fileSizeKB, int rotatedFiles);
}

// Convenience macros. Use the default logger and capture source file:line (for debugging).
// Format is fmt-style: LOG_INFO("value = {}", x);
#define LOG_TRACE(...) SPDLOG_TRACE(__VA_ARGS__)
#define LOG_DEBUG(...) SPDLOG_DEBUG(__VA_ARGS__)
#define LOG_INFO(...) SPDLOG_INFO(__VA_ARGS__)
#define LOG_WARN(...) SPDLOG_WARN(__VA_ARGS__)
#define LOG_ERROR(...) SPDLOG_ERROR(__VA_ARGS__)
#define LOG_CRITICAL(...) SPDLOG_CRITICAL(__VA_ARGS__)

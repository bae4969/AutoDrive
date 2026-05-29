#include "Logger.h"
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <memory>
#include <vector>
#include <chrono>

namespace Logger
{
	using namespace std;

	bool InitLogger(const string &filePath, int fileSizeKB, int rotatedFiles)
	{
		// spdlog keeps (rotatedFiles + 1) files of maxBytes each (current + backups);
		// total retained ~= (rotatedFiles + 1) * fileSizeKB. Parent dir auto-created by spdlog.
		size_t maxBytes = static_cast<size_t>(fileSizeKB > 0 ? fileSizeKB : 1024) * 1024;
		size_t maxFiles = static_cast<size_t>(rotatedFiles >= 0 ? rotatedFiles : 1);
		try
		{
			auto fileSink = make_shared<spdlog::sinks::rotating_file_sink_mt>(filePath, maxBytes, maxFiles);
			auto consoleSink = make_shared<spdlog::sinks::stdout_color_sink_mt>();

			vector<spdlog::sink_ptr> sinks{fileSink, consoleSink};
			auto logger = make_shared<spdlog::logger>("autodrive", sinks.begin(), sinks.end());

			logger->set_level(spdlog::level::debug);
			logger->flush_on(spdlog::level::warn);
			logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [%s:%#] %v");

			spdlog::set_default_logger(logger);
			spdlog::flush_every(chrono::seconds(3));
		}
		catch (const spdlog::spdlog_ex &ex)
		{
			printf("Fail to init logger: %s\n", ex.what());
			return false;
		}

		return true;
	}
}

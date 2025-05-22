//
// Created by User on 19.04.2025.
//

#include "Logger.h"

#include <chrono>
#include <unordered_map>

using namespace details;

namespace
{
constexpr auto LOG_FORMAT = "[{}] [{}] [{}::{}] at line {}. Message: {}";

std::unordered_map<LogLevel, std::string> LOG_LEVEL_TO_STRING = {
	{ LogLevel::Trace, "Trace" },
	{ LogLevel::Debug, "Debug" },
	{ LogLevel::Info, "Info" },
	{ LogLevel::Warning, "Warning" },
	{ LogLevel::Error, "Error" },
	{ LogLevel::Critical, "Critical" },
};

std::string LevelToString(LogLevel level)
{
	if (auto it = LOG_LEVEL_TO_STRING.find(level); it != LOG_LEVEL_TO_STRING.end())
	{
		return it->second;
	}
	throw std::invalid_argument("Invalid log level");
}

std::string FormatTime(std::chrono::system_clock::time_point time)
{
	auto seconds = std::chrono::system_clock::to_time_t(time);
	std::tm tmBuffer;

#ifdef _WIN32
	localtime_s(&tmBuffer, &seconds);
#else
	localtime_r(&seconds, &tmBuffer);
#endif

	std::stringstream ss;
	ss << std::put_time(&tmBuffer, "%Y-%m-%d %H:%M:%S");
	return ss.str();
}

std::string FormatMessage(LogLevel level, const std::string& message, const char* file, int line, const char* function)
{
	auto now = std::chrono::system_clock::now();
	std::string levelStr = LevelToString(level);
	std::string timeStr = FormatTime(now);

	if (file && function && line != -1)
	{
		return std::format(LOG_FORMAT, timeStr, levelStr, file, function, line, message);
	}
	return std::format("[{}] Message: {}", levelStr, message);
}
} // namespace

Logger& Logger::Instance()
{
	static Logger instance;

	return instance;
}

void Logger::AddAppender(std::unique_ptr<ILogAppender> appender)
{
	std::lock_guard lock(m_appendersMutex);
	m_appenders.emplace_back(std::move(appender));
}

void Logger::Log(LogLevel level, const std::string& message, const char* file, int line, const char* function)
{
	std::string formattedMessage = FormatMessage(level, message, file, line, function);

	std::lock_guard lock(m_appendersMutex);
	for (auto& appender : m_appenders)
	{
		appender->Append(level, formattedMessage);
	}
}

Logger::Logger()
{
#ifdef _DEBUG
	AddAppender(std::make_unique<ConsoleAppender>());
#endif
	AddAppender(std::make_unique<FileAppender>("log.txt"));
}
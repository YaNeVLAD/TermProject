//
// Created by User on 19.04.2025.
//

#ifndef LOGGER_H
#define LOGGER_H

#include <memory>
#include <vector>

#include "DefaultAppenders.h"

#define LOG(level, msg) Logger::Instance().Log(level, msg, __FILE__, __LINE__, __FUNCTION__);
#define LOG_TRACE(msg) LOG(details::LogLevel::Trace, msg)
#define LOG_DEBUG(msg) LOG(details::LogLevel::Debug, msg)
#define LOG_INFO(msg) LOG(details::LogLevel::Info, msg)
#define LOG_WARN(msg) LOG(details::LogLevel::Warning, msg)
#define LOG_ERROR(msg) LOG(details::LogLevel::Error, msg)
#define LOG_CRITICAL(msg) LOG(details::LogLevel::Critical, msg)

class Logger
{
public:
	Logger(Logger&&) = delete;
	Logger(const Logger&) = delete;
	Logger& operator=(Logger&&) = delete;
	Logger& operator=(const Logger&) = delete;

	static Logger& Instance();

	void AddAppender(std::unique_ptr<details::ILogAppender> appender);

	void Log(details::LogLevel level, const std::string& message,
		const char* file = nullptr, int line = -1, const char* function = nullptr);

private:
	std::mutex m_appendersMutex;
	std::vector<std::unique_ptr<details::ILogAppender>> m_appenders;

	Logger();
};

#endif // LOGGER_H

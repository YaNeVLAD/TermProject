//
// Created by User on 20.04.2025.
//

#ifndef LOG_APPENDER_H
#define LOG_APPENDER_H

#include <string>

namespace details
{
enum class LogLevel
{
	Trace = 0,
	Debug,
	Info,
	Warning,
	Error,
	Critical,
};

class ILogAppender
{
public:
	virtual ~ILogAppender() = default;
	virtual void Append(LogLevel level, const std::string& message) = 0;
};
} // namespace details

#endif // LOG_APPENDER_H
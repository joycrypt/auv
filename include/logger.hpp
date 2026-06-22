#pragma once
#include <iostream>
#include <iomanip>
#include <string>

namespace auv {

enum class LogLevel { DEBUG, INFO, WARN, ERROR };

class Logger {
public:
    static void setLevel(LogLevel l) { level_ = l; }

    static void debug(const std::string& m) { log(LogLevel::DEBUG, m); }
    static void info (const std::string& m) { log(LogLevel::INFO,  m); }
    static void warn (const std::string& m) { log(LogLevel::WARN,  m); }
    static void error(const std::string& m) { log(LogLevel::ERROR, m); }

    static void section(const std::string& title) {
        std::cout << "\n\033[1;36m"
                    << "╔══════════════════════════════════════════════════════╗\n"
                    << "║  " << std::left << std::setw(51) << title << "║\n"
                    << "╚══════════════════════════════════════════════════════╝"
                    << "\033[0m\n";
    }

    static void kv(const std::string& k, const std::string& v) {
        std::cout << "  \033[1;37m" << std::left << std::setw(32) << k
                    << "\033[0;37m" << v << "\033[0m\n";
    }

private:
    static LogLevel level_;
    static void log(LogLevel l, const std::string& m) {
        if (l < level_) return;
        const char* col = "\033[0m", *tag = "";
        switch(l) {
            case LogLevel::DEBUG: col="\033[0;37m"; tag="[DEBUG]"; break;
            case LogLevel::INFO:  col="\033[0;32m"; tag="[INFO ]"; break;
            case LogLevel::WARN:  col="\033[0;33m"; tag="[WARN ]"; break;
            case LogLevel::ERROR: col="\033[0;31m"; tag="[ERROR]"; break;
        }
        std::cout << col << tag << " " << m << "\033[0m\n";
    }
};

} 
    
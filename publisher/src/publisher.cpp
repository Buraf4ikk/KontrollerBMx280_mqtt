#include <ctime>
#include <iostream>

#include "general.h"
#include "publisher.h"

Publisher::Publisher(const char *id, const char *host, int port)
    : mosquittopp(id)
{
    std::cout << app::AppTag << "Connecting to MQTT Broker with address "
              << host << " and port " << port << std::endl;

    const int keepAlive = 60;
    connect(host, port, keepAlive);
}

void Publisher::on_publish(int)
{
    std::cout << app::AppTag << "Publication succeeded." << std::endl;
}

void Publisher::send_log_message(std::string message)
{
    std::cout << app::AppTag << message;
    publish(nullptr, "log", static_cast<int>(message.size()), message.c_str());
}

void Publisher::send_log_calib(std::string message)
{
    std::cout << app::AppTag << message;
    publish(nullptr, "log_calib", static_cast<int>(message.size()),
            message.c_str());
}

void Publisher::send_temperature(float temperature)
{
    std::cout << app::AppTag << "Temperature is " << temperature << std::endl;
    const std::string message = std::to_string(temperature);
    publish(nullptr, "temperature", static_cast<int>(message.size()),
            message.c_str());
}

void Publisher::send_humidity(float humidity)
{
    std::cout << app::AppTag << "Humidity is " << humidity << std::endl;
    const std::string message = std::to_string(humidity);
    publish(nullptr, "humidity", static_cast<int>(message.size()),
            message.c_str());
}

void Publisher::send_pressure(float pressure)
{
    std::cout << app::AppTag << "Pressure is " << pressure << std::endl;
    const std::string message = std::to_string(pressure);
    publish(nullptr, "pressure", static_cast<int>(message.size()),
            message.c_str());
}

/* © 2021 AO Kaspersky Lab */
#ifndef _MOSQUITTO_PUBLISHER_H
#define _MOSQUITTO_PUBLISHER_H

#include <mosquittopp.h>

class Publisher : public mosqpp::mosquittopp
{
public:
    Publisher(const char *id, const char *host, int port);
    ~Publisher() {};

    void on_publish(int mid) override;
    void send_temperature(float temperature);
    void send_humidity(float humidity);
    void send_pressure(float pressure);
    void send_log_message(std::string message);
    void send_log_calib(std::string message);
};

#endif // _MOSQUITTO_PUBLISHER_H

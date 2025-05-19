#pragma once
#include "pch.h"
#include <string>

class IPAddress {
public:
    void initialize(const std::string& address);
    void initializeLocal();

    const std::string get();

private:
    std::string m_Address;
};

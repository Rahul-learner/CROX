#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>

int main() {
    std::cout << "GS Monitor Starting..." << std::endl;
    std::cout << "Listening on /dev/ttyACM0 (Make sure you have read permissions!)" << std::endl;

    // The Pico creates a USB CDC Virtual COM port. 
    // On Linux, this acts just like a regular file, and ignores baud rates.
    std::ifstream serial("/dev/ttyACM0");
    
    if (!serial.is_open()) {
        std::cerr << "Failed to open serial port." << std::endl;
        std::cerr << "Try running: sudo chmod a+rw /dev/ttyACM0" << std::endl;
        return 1;
    }

    std::string line;
    // Read lines from the serial port infinitely
    while (std::getline(serial, line)) {
        // Here you can parse the string just like your debug.py regex does!
        std::cout << "Data: " << line << std::endl;
    }

    return 0;
}

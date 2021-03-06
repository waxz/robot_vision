//
// Created by waxz on 18-7-18.
//

#ifndef LOCATE_REFLECTION_TIME_H
#define LOCATE_REFLECTION_TIME_H

#include <chrono>

namespace time_util {

    class Timer {
    public:
        void start() {
            m_StartTime = std::chrono::system_clock::now();
            m_bRunning = true;
        }

        void stop() {
            m_EndTime = std::chrono::system_clock::now();
            m_bRunning = false;
        }

        double elapsedMicroseconds() {
            std::chrono::time_point<std::chrono::system_clock> endTime;

            if (m_bRunning) {
                endTime = std::chrono::system_clock::now();
            } else {
                endTime = m_EndTime;
            }

            return std::chrono::duration_cast<std::chrono::microseconds>(endTime - m_StartTime).count();
        }

        double elapsedSeconds() {
            return elapsedMicroseconds() / 1000000.0;
        }

    private:
        std::chrono::time_point<std::chrono::system_clock> m_StartTime;
        std::chrono::time_point<std::chrono::system_clock> m_EndTime;
        bool m_bRunning = false;
    };

}
#endif //LOCATE_REFLECTION_TIME_H

/*
     Timer timer;
    timer.start();
    int counter = 0;
    double test, test2;
    while(timer.elapsedSeconds() < 10.0)
    {
        counter++;
        test = std::cos(counter / M_PI);
        test2 = std::sin(counter / M_PI);
    }
    timer.stop();

    std::cout << counter << std::endl;
    std::cout << "Seconds: " << timer.elapsedSeconds() << std::endl;
std::cout << "Milliseconds: " << timer.elapsedMilliseconds() << std::endl





 */
#ifndef PERFORMANCETESTER
#define PERFORMANCETESTER

// Call as follows: PerformanceTester tester(&DEBUG_SERIAL, "Whatever you want to write");


class PerformanceTester
{
private:
    unsigned long startTime{0}, endTime{0}, elapsedTime{0};
    HardwareSerial* m_debugMonitor;

public:
    PerformanceTester(HardwareSerial* debugMonitor, String functionName);
    ~PerformanceTester();
};

PerformanceTester::PerformanceTester(HardwareSerial* debugMonitor, String functionName) : m_debugMonitor(debugMonitor)
{
    m_debugMonitor->print(functionName);
    m_debugMonitor->print(": ");
    startTime = millis();
}

PerformanceTester::~PerformanceTester()
{
    endTime = millis();
    elapsedTime = endTime - startTime;
    //m_debugMonitor->print("Function time: ");
    m_debugMonitor->print(elapsedTime);
    m_debugMonitor->print(" ms");
    m_debugMonitor->print("\n");
}

#endif

#ifndef _TimerClock_hpp_
#define _TimerClock_hpp_

#include <iostream>
#include <chrono>

using namespace std;
using namespace std::chrono;

class TimerClock
{
public:
 TimerClock()
 {
  update();
 }

 ~TimerClock()
 {
 }

 void update()
 {
  _start = high_resolution_clock::now();
 }
 //获取秒
 double getTimerSecond()
 {
  return getTimerMicroSec() * 0.000001;
 }
 //获取毫秒
 double getTimerMilliSec()
 {
  return getTimerMicroSec()*0.001;
 }
 //获取微妙
 long long getTimerMicroSec()
 {
  //当前时钟减去开始时钟的count
  return duration_cast<microseconds>(high_resolution_clock::now() - _start).count();
 }
 // 获取系统时间(绝对值)
 string getCurrentSystemTime()
 {
     auto tt = system_clock::to_time_t(system_clock::now());
     struct tm* ptm = localtime(&tt);
     char date[60] = {0};
     sprintf(date, "%d-%02d-%02d-%02d-%02d-%02d", (int)ptm->tm_year + 1900, (int)ptm->tm_mon + 1, (int)ptm->tm_mday, (int)ptm->tm_hour, (int)ptm->tm_min, (int)ptm->tm_sec);
     return string(date);
 }
private:
 time_point<high_resolution_clock>_start;
};

#endif
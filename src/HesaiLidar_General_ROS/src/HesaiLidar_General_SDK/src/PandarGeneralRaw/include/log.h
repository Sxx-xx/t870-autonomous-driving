/******************************************************************************
 * Copyright 2018 The Hesai Technology Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <stdio.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include <ctime>

class TranceFunc
{
public:
    TranceFunc(const char* file, const char* func){
        m_cFile = file;
        m_cFunc = func;
        auto now = std::chrono::system_clock::now();
        auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
        auto epoch = now_ms.time_since_epoch();
        auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
        long duration = value.count();
        time_t rawtime = duration / 1000;
        struct tm *ptm = localtime(&rawtime);
        int milliseconds = duration % 1000;
        printf("[T] %02d:%02d:%02d.%03d pid:%d tid:%10ld ->[File:%s Function:%s ]\n", 
            ptm->tm_hour, ptm->tm_min, ptm->tm_sec, milliseconds, getpid(), 
            (long int)std::hash<std::thread::id>()(std::this_thread::get_id()), m_cFile, m_cFunc);
    }
    ~TranceFunc(){
        auto now = std::chrono::system_clock::now();
        auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
        auto epoch = now_ms.time_since_epoch();
        auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
        long duration = value.count();
        time_t rawtime = duration / 1000;
        struct tm *ptm = localtime(&rawtime);
        int milliseconds = duration % 1000;
        printf("[T] %02d:%02d:%02d.%03d pid:%d tid:%10ld <-[File:%s Function:%s ]\n", 
            ptm->tm_hour, ptm->tm_min, ptm->tm_sec, milliseconds, getpid(), 
            (long int)std::hash<std::thread::id>()(std::this_thread::get_id()), m_cFile, m_cFunc);
    }
    const char* m_cFile;
    const char* m_cFunc;
};

#define LOG_D(format,...) {\
    auto now = std::chrono::system_clock::now();\
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);\
    auto epoch = now_ms.time_since_epoch();\
    auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);\
    long duration = value.count();\
    time_t rawtime = duration / 1000;\
    struct tm *ptm = localtime(&rawtime);\
    int milliseconds = duration % 1000;\
    printf("[D] %02d:%02d:%02d.%03d pid:%d tid:%10ld File:%s Function:%s Line:%d " format"\n", \
        ptm->tm_hour, ptm->tm_min, ptm->tm_sec, milliseconds, getpid(), \
        (long int)std::hash<std::thread::id>()(std::this_thread::get_id()), __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__);\
}

#define LOG_E(format,...) {\
    auto now = std::chrono::system_clock::now();\
    auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);\
    auto epoch = now_ms.time_since_epoch();\
    auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);\
    long duration = value.count();\
    time_t rawtime = duration / 1000;\
    struct tm *ptm = localtime(&rawtime);\
    int milliseconds = duration % 1000;\
    printf("[E] %02d:%02d:%02d.%03d pid:%d tid:%10ld File:%s Function:%s Line:%d " format"\n", \
        ptm->tm_hour, ptm->tm_min, ptm->tm_sec, milliseconds, getpid(), \
        (long int)std::hash<std::thread::id>()(std::this_thread::get_id()), __FILE__, __FUNCTION__, __LINE__, ##__VA_ARGS__);\
}

#define LOG_FUNC() TranceFunc tf( __FILE__, __FUNCTION__)


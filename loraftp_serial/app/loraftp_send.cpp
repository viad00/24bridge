// Copyright (c) 2020, Christopher A. Taylor.  All rights reserved.

/*
    Puts the radio into transmit mode.
    Sends data until canceled.

    There is no feedback from the receiver.
*/

#include "loraftp.hpp"
using namespace lora;

#include <thread>
#include <chrono>
#include <time.h>
using namespace std;


//------------------------------------------------------------------------------
// Signal

#include <csignal>
#include <atomic>

atomic<bool> Terminated = ATOMIC_VAR_INIT(false);

void SignalHandler(int)
{
    Terminated = true;
}


//------------------------------------------------------------------------------
// Entrypoint

int main(int argc, char* argv[])
{
    SetupAsyncDiskLog("sender.log", false/*enable debug logs?*/);

    spdlog::info("loraftp_send V{} starting...", kVersion);

    if (argc < 4) {
        spdlog::info("Usage: {} <file to send> <time to send sec> <interval usec>", argv[0]);
        return -1;
    }

    const char* file_name = argv[1];
    const char* time_to_send_c = argv[2];
    const char* interval_usec_c = argv[3];
    uint32_t time_to_send = atoi(time_to_send_c);
    uint32_t interval_usec = atoi(interval_usec_c);

    MappedReadOnlySmallFile mmf;
    if (!mmf.Read(file_name)) {
        spdlog::error("Failed to open file: {}", file_name);
        return false;
    }

    FileSender sender;
    ScopedFunction sender_scope([&]() {
        sender.Shutdown();
    });

    if (!sender.Initialize(file_name, mmf.GetData(), mmf.GetDataBytes(), interval_usec)) {
        spdlog::error("sender.Initialize failed");
        return -1;
    }

    signal(SIGINT, SignalHandler);

    long int target_time = time(NULL) + time_to_send;

    while (!Terminated && !sender.IsTerminated()) {
        if (time(NULL) > target_time) {
            Terminated = true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    return 0;
}

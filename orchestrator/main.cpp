/*
 * Camera Orchestrator — standalone daemon.
 *
 * Opens cameras, captures frames in background threads,
 * serves them via POSIX shared memory for zero-copy IPC.
 *
 * Other processes read frames from /dev/shm/drone_cam_<name>.
 *
 * Usage:
 *   ./camera_orchestrator \
 *       --add /dev/cam_usb2,640,480,30 \
 *       --add /dev/cam_usb3,640,480,20
 */

#include "camera_orchestrator.hpp"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <csignal>
#include <unistd.h>

static volatile bool g_running = true;
extern "C" void sigint_handler(int) { g_running = false; }

struct DeviceSpec {
    std::string device;
    int width = 640;
    int height = 480;
    int fps = 30;
};

static DeviceSpec parse_spec(const std::string& spec) {
    DeviceSpec d;
    auto p1 = spec.find(',');
    auto p2 = spec.find(',', p1 + 1);
    auto p3 = spec.find(',', p2 + 1);

    d.device = spec.substr(0, p1);
    if (p1 != std::string::npos)
        d.width = std::atoi(spec.substr(p1 + 1, p2 - p1 - 1).c_str());
    if (p2 != std::string::npos)
        d.height = std::atoi(spec.substr(p2 + 1, p3 - p2 - 1).c_str());
    if (p3 != std::string::npos)
        d.fps = std::atoi(spec.substr(p3 + 1).c_str());
    return d;
}

static void print_status_loop(CameraOrchestrator& orch) {
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(10));
        std::printf("[ORCH] Running — cameras: 2\n");
    }
}

int main(int argc, char** argv) {
    std::vector<DeviceSpec> devices;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--add" && i + 1 < argc) {
            devices.push_back(parse_spec(argv[++i]));
        } else if (arg == "--help") {
            std::printf("Usage: %s --add DEVICE,W,H,FPS [--add ...]\n", argv[0]);
            std::printf("  --add /dev/cam_usb2,640,480,30   forward camera\n");
            std::printf("  --add /dev/cam_usb3,640,480,20   downward camera\n");
            std::printf("\nFrames available via shared memory:\n");
            std::printf("  /drone_cam_cam_usb2  (forward)\n");
            std::printf("  /drone_cam_cam_usb3  (downward)\n");
            return 0;
        }
    }

    if (devices.empty()) {
        devices.push_back({"/dev/cam_usb2", 640, 480, 30});
        devices.push_back({"/dev/cam_usb3", 640, 480, 20});
        std::printf("[ORCH] No cameras specified, using defaults\n");
    }

    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    CameraOrchestrator& orch = CameraOrchestrator::instance();

    for (const auto& d : devices) {
        orch.add_camera({d.device, d.width, d.height, d.fps, true});
    }

    if (!orch.start()) {
        std::fprintf(stderr, "[ORCH] Failed to start\n");
        return 1;
    }

    std::printf("[ORCH] Camera Orchestrator running\n");
    std::printf("[ORCH] Cameras:\n");
    for (const auto& d : devices) {
        std::printf("  %s → /drone_cam_%s (%dx%d @ %dfps)\n",
                    d.device.c_str(),
                    d.device.c_str() + 1,  // skip leading /
                    d.width, d.height, d.fps);
    }
    std::printf("[ORCH] Press Ctrl+C to stop\n");

    std::thread status_thread(print_status_loop, std::ref(orch));
    status_thread.detach();

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    orch.stop();
    std::printf("\n[ORCH] Stopped\n");
    return 0;
}

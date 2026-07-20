#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <unordered_map>

struct ShmHeader {
    volatile uint64_t frame_id;
    volatile uint64_t timestamp_us;
    uint32_t width;
    uint32_t height;
    uint32_t channels;
    uint32_t data_size;
    volatile uint32_t ready;
    uint8_t padding[28];
};

class CameraOrchestrator {
public:
    static CameraOrchestrator& instance();

    CameraOrchestrator(const CameraOrchestrator&) = delete;
    CameraOrchestrator& operator=(const CameraOrchestrator&) = delete;

    struct CameraConfig {
        std::string device;
        int width = 640;
        int height = 480;
        int fps = 30;
        bool enable_shm = true;
    };

    bool add_camera(const CameraConfig& config);
    bool remove_camera(const std::string& device);
    bool start();
    void stop();
    bool is_running() const { return _running.load(); }

    bool get_frame(const std::string& device, cv::Mat& frame, uint64_t* timestamp = nullptr);

    using FrameCallback = std::function<void(const std::string& device, const cv::Mat& frame, uint64_t timestamp)>;
    int subscribe(const std::string& device, FrameCallback callback);
    void unsubscribe(int id);

    static void read_shm_frame(const std::string& shm_name, cv::Mat& frame,
                               uint64_t* timestamp = nullptr, uint64_t* frame_id = nullptr);

private:
    CameraOrchestrator() = default;
    ~CameraOrchestrator() { stop(); }

    struct CameraDevice {
        cv::VideoCapture cap;
        std::thread thread;
        std::mutex mutex;
        cv::Mat latest_frame;
        uint64_t latest_timestamp = 0;
        uint64_t frame_count = 0;
        std::atomic<bool> running{false};
        CameraConfig config;

        int shm_fd = -1;
        void* shm_ptr = nullptr;
        size_t shm_size = 0;

        std::vector<std::pair<int, FrameCallback>> callbacks;
        int next_cb_id = 0;
    };

    std::unordered_map<std::string, std::unique_ptr<CameraDevice>> _cameras;
    std::mutex _mutex;
    std::atomic<bool> _running{false};

    std::string _device_to_shm_name(const std::string& device) const;
    bool _setup_shm(CameraDevice* cam);
    void _capture_loop(CameraDevice* cam, const std::string& device);
};

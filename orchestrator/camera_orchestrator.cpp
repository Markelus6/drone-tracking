#include "camera_orchestrator.hpp"
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

CameraOrchestrator& CameraOrchestrator::instance() {
    static CameraOrchestrator inst;
    return inst;
}

std::string CameraOrchestrator::_device_to_shm_name(const std::string& device) const {
    std::string name = device;
    for (char& c : name) {
        if (c == '/') c = '_';
    }
    return "/drone_cam" + name;
}

bool CameraOrchestrator::_setup_shm(CameraDevice* cam) {
    std::string shm_name = _device_to_shm_name(cam->config.device);

    cam->shm_fd = shm_open(shm_name.c_str(), O_CREAT | O_RDWR, 0666);
    if (cam->shm_fd < 0) {
        std::perror(("shm_open " + shm_name).c_str());
        return false;
    }

    size_t data_size = static_cast<size_t>(cam->config.width) *
                       static_cast<size_t>(cam->config.height) * 3;
    cam->shm_size = sizeof(ShmHeader) + data_size;

    if (ftruncate(cam->shm_fd, static_cast<off_t>(cam->shm_size)) < 0) {
        std::perror("ftruncate");
        close(cam->shm_fd);
        cam->shm_fd = -1;
        return false;
    }

    cam->shm_ptr = mmap(nullptr, cam->shm_size, PROT_READ | PROT_WRITE,
                        MAP_SHARED, cam->shm_fd, 0);
    if (cam->shm_ptr == MAP_FAILED) {
        std::perror("mmap");
        close(cam->shm_fd);
        cam->shm_fd = -1;
        cam->shm_ptr = nullptr;
        return false;
    }

    ShmHeader* hdr = static_cast<ShmHeader*>(cam->shm_ptr);
    std::memset(hdr, 0, sizeof(ShmHeader));
    hdr->width = static_cast<uint32_t>(cam->config.width);
    hdr->height = static_cast<uint32_t>(cam->config.height);
    hdr->channels = 3;
    hdr->data_size = static_cast<uint32_t>(data_size);

    return true;
}

bool CameraOrchestrator::add_camera(const CameraConfig& config) {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_cameras.find(config.device) != _cameras.end()) {
        std::fprintf(stderr, "[ORCH] Camera %s already added\n", config.device.c_str());
        return false;
    }

    auto cam = std::make_unique<CameraDevice>();
    cam->config = config;

    if (config.enable_shm) {
        _setup_shm(cam.get());
    }

    _cameras[config.device] = std::move(cam);
    std::printf("[ORCH] Camera added: %s (%dx%d @ %dfps)\n",
                config.device.c_str(), config.width, config.height, config.fps);
    return true;
}

bool CameraOrchestrator::remove_camera(const std::string& device) {
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _cameras.find(device);
    if (it == _cameras.end()) return false;

    it->second->running = false;
    if (it->second->thread.joinable()) {
        it->second->thread.join();
    }

    if (it->second->shm_ptr) {
        munmap(it->second->shm_ptr, it->second->shm_size);
    }
    if (it->second->shm_fd >= 0) {
        close(it->second->shm_fd);
        shm_unlink(_device_to_shm_name(device).c_str());
    }

    _cameras.erase(it);
    return true;
}

bool CameraOrchestrator::start() {
    std::lock_guard<std::mutex> lock(_mutex);
    if (_running.load()) return true;

    _running.store(true);

    for (auto& [device, cam] : _cameras) {
        cam->cap.open(device, cv::CAP_V4L2);
        if (!cam->cap.isOpened()) {
            std::fprintf(stderr, "[ORCH] Failed to open camera: %s\n", device.c_str());
            continue;
        }

        cam->cap.set(cv::CAP_PROP_FRAME_WIDTH, cam->config.width);
        cam->cap.set(cv::CAP_PROP_FRAME_HEIGHT, cam->config.height);
        cam->cap.set(cv::CAP_PROP_FPS, cam->config.fps);

        int actual_w = static_cast<int>(cam->cap.get(cv::CAP_PROP_FRAME_WIDTH));
        int actual_h = static_cast<int>(cam->cap.get(cv::CAP_PROP_FRAME_HEIGHT));

        std::printf("[ORCH] Camera opened: %s -> %dx%d\n",
                    device.c_str(), actual_w, actual_h);

        cam->running.store(true);
        cam->thread = std::thread(&CameraOrchestrator::_capture_loop, this, cam.get(), device);
    }

    return true;
}

void CameraOrchestrator::stop() {
    _running.store(false);

    std::lock_guard<std::mutex> lock(_mutex);
    for (auto& [device, cam] : _cameras) {
        cam->running.store(false);
    }

    for (auto& [device, cam] : _cameras) {
        if (cam->thread.joinable()) {
            cam->thread.join();
        }
        if (cam->cap.isOpened()) {
            cam->cap.release();
        }
        if (cam->shm_ptr) {
            munmap(cam->shm_ptr, cam->shm_size);
            cam->shm_ptr = nullptr;
        }
        if (cam->shm_fd >= 0) {
            close(cam->shm_fd);
            shm_unlink(_device_to_shm_name(device).c_str());
            cam->shm_fd = -1;
        }
    }
    _cameras.clear();
}

void CameraOrchestrator::_capture_loop(CameraDevice* cam, const std::string& device) {
    cv::Mat frame;

    while (cam->running.load() && _running.load()) {
        if (!cam->cap.read(frame)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        uint64_t now_us = static_cast<uint64_t>(
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch()
            ).count()
        );

        uint64_t frame_id = ++cam->frame_count;

        // Update latest frame (for in-process consumers)
        {
            std::lock_guard<std::mutex> lock(cam->mutex);
            frame.copyTo(cam->latest_frame);
            cam->latest_timestamp = now_us;
        }

        // Write to shared memory (for external processes)
        if (cam->shm_ptr) {
            ShmHeader* hdr = static_cast<ShmHeader*>(cam->shm_ptr);
            hdr->ready = 0;
            __sync_synchronize();

            size_t data_size = static_cast<size_t>(frame.total()) * frame.elemSize();
            if (data_size <= cam->shm_size - sizeof(ShmHeader)) {
                std::memcpy(hdr + 1, frame.data, data_size);
                hdr->frame_id = frame_id;
                hdr->timestamp_us = now_us;
                __sync_synchronize();
                hdr->ready = 1;
            }
        }

        // Callbacks
        {
            std::lock_guard<std::mutex> lock(cam->mutex);
            for (const auto& [id, cb] : cam->callbacks) {
                cb(device, frame, now_us);
            }
        }
    }
}

bool CameraOrchestrator::get_frame(const std::string& device,
                                    cv::Mat& frame,
                                    uint64_t* timestamp) {
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _cameras.find(device);
    if (it == _cameras.end()) return false;

    std::lock_guard<std::mutex> frame_lock(it->second->mutex);
    if (it->second->latest_frame.empty()) return false;

    it->second->latest_frame.copyTo(frame);
    if (timestamp) *timestamp = it->second->latest_timestamp;
    return true;
}

int CameraOrchestrator::subscribe(const std::string& device, FrameCallback callback) {
    std::lock_guard<std::mutex> lock(_mutex);
    auto it = _cameras.find(device);
    if (it == _cameras.end()) return -1;
    int id = it->second->next_cb_id++;
    it->second->callbacks.emplace_back(id, std::move(callback));
    return id;
}

void CameraOrchestrator::unsubscribe(int id) {
    std::lock_guard<std::mutex> lock(_mutex);
    for (auto& [device, cam] : _cameras) {
        auto& cbs = cam->callbacks;
        cbs.erase(std::remove_if(cbs.begin(), cbs.end(),
                                 [id](const auto& p) { return p.first == id; }),
                  cbs.end());
    }
}

void CameraOrchestrator::read_shm_frame(const std::string& shm_name,
                                         cv::Mat& frame,
                                         uint64_t* timestamp,
                                         uint64_t* frame_id) {
    int fd = shm_open(shm_name.c_str(), O_RDONLY, 0666);
    if (fd < 0) {
        frame = cv::Mat();
        return;
    }

    struct stat sb;
    if (fstat(fd, &sb) < 0) {
        close(fd);
        frame = cv::Mat();
        return;
    }

    void* ptr = mmap(nullptr, static_cast<size_t>(sb.st_size),
                     PROT_READ, MAP_SHARED, fd, 0);
    close(fd);

    if (ptr == MAP_FAILED) {
        frame = cv::Mat();
        return;
    }

    const ShmHeader* hdr = static_cast<const ShmHeader*>(ptr);
    __sync_synchronize();

    if (hdr->ready && hdr->width > 0 && hdr->height > 0) {
        const uint8_t* data = static_cast<const uint8_t*>(ptr) + sizeof(ShmHeader);
        cv::Mat(hdr->height, hdr->width,
                hdr->channels == 1 ? CV_8UC1 : CV_8UC3,
                const_cast<uint8_t*>(data)).copyTo(frame);
        if (timestamp) *timestamp = hdr->timestamp_us;
        if (frame_id) *frame_id = hdr->frame_id;
    } else {
        frame = cv::Mat();
    }

    munmap(ptr, static_cast<size_t>(sb.st_size));
}

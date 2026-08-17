#include <memory>
#include <string>
#include <opencv2/opencv.hpp>
#include "MbufPoolManager.h"
#include "MbufWrapper.h"
#include "FrameInfo.h"
#include <QImage>

struct ImageSaveInfo {
    std::string imgPath;
    std::string heightPath; 
    std::string copyPath;
    mtrx::SharedMilID imgBuf = nullptr;   
    mtrx::SharedMilID heightBuf = nullptr; 
    MIL_ID mbuf = M_NULL;
    QImage qimg;
};

class ImageSavingThread {
public:
    static ImageSavingThread& instance();

    void enqueue(ImageSaveInfo task);
    void enqueue(std::string filename, QImage qimg);
    void enqueue(std::string filename, MIL_ID mbuf);
    void enqueue(std::string root, FrameInfo frame, std::string copyPath = "none", std::string extension = "jpg");
    void enqueue(std::string filename, mtrx::SharedMilID buffer, std::string copyPath = "none");
    int size();

private:
    ImageSavingThread();
    ~ImageSavingThread();
    ImageSavingThread(const ImageSavingThread&) = delete;
    ImageSavingThread& operator=(const ImageSavingThread&) = delete;

    void workerLoop();

    std::vector<std::thread> m_workers;
    std::queue<ImageSaveInfo> m_queue;

    std::mutex m_mutex;
    std::condition_variable m_cv;
    std::atomic<bool> m_running{ true };
};

using IST = ImageSavingThread;
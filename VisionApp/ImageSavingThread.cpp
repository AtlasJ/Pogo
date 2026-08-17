#include "ImageSavingThread.h"
#include <mil.h>
#include <filesystem>
#include <QFile>
#include "Utilities.h"

ImageSavingThread& ImageSavingThread::instance()
{
    static ImageSavingThread inst;
    return inst;
}

ImageSavingThread::ImageSavingThread()
{
    for (size_t i = 0; i < 2; i++) {
        m_workers.emplace_back(&ImageSavingThread::workerLoop, this);
    }
}

ImageSavingThread::~ImageSavingThread()
{
    m_running = false;
    m_cv.notify_all();

    for (auto& w : m_workers)
        if (w.joinable()) w.join();
}

void ImageSavingThread::enqueue(ImageSaveInfo task)
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.push(task);
    }
    m_cv.notify_one();
}

void ImageSavingThread::enqueue(std::string filename, mtrx::SharedMilID buffer, std::string copyPath)
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        ImageSaveInfo task;
        task.imgBuf = buffer;
        task.imgPath = filename;
        task.copyPath = copyPath;

        m_queue.push(task);
    }
    m_cv.notify_one();
}

int ImageSavingThread::size()
{
    return m_queue.size();
}

void ImageSavingThread::enqueue(std::string root, FrameInfo frame, std::string copyPath, std::string extension)
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        ImageSaveInfo task;
        task.copyPath = copyPath;

        if (frame.type == ct::s_height_map) {
            auto iid = frame.viewID + "_IMap";
            auto hid = frame.viewID + "_HeightMap_" + frame.opticID;

            task.heightBuf = frame.pHeightMap;
            task.heightPath = root + hid.toStdString() + ".tiff";

            task.imgBuf = frame.pImage;
            task.imgPath = root + iid.toStdString() + "." + extension;
        }
        else {
            auto cid = util::combineID(frame.viewID, frame.opticID).toStdString();

            task.imgBuf = frame.pImage;
            task.imgPath = root + cid + "." + extension;
        }

        m_queue.push(task);
    }
    m_cv.notify_one();
}

void ImageSavingThread::enqueue(std::string filename, QImage qimg)
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        ImageSaveInfo task;
        task.imgPath = filename;
        task.qimg = qimg;

        m_queue.push(task);
    }

    m_cv.notify_one();
}

void ImageSavingThread::enqueue(std::string filename, MIL_ID mbuf)
{
    {
        std::lock_guard<std::mutex> lock(m_mutex);

        ImageSaveInfo task;
        task.imgPath = filename;
        task.mbuf = mbuf;

        m_queue.push(task);
    }

    m_cv.notify_one();
}

void ImageSavingThread::workerLoop()
{
    while (m_running) {
        ImageSaveInfo task;

        {
            std::unique_lock<std::mutex> lock(m_mutex);

            m_cv.wait(lock, [&] { return !m_queue.empty() || !m_running; });
            if (!m_running && m_queue.empty()) break;

            task = std::move(m_queue.front());
            m_queue.pop();
        }

        if (!task.qimg.isNull()) {
            if (!task.qimg.save(task.imgPath.c_str())) {
                ct::logger::error("[ImageSavingThread] Failed to save image: %s", task.imgPath.c_str());
            }
            return;
        }

        if (task.mbuf != M_NULL) {
            MbufSaveA(task.imgPath.c_str(), task.mbuf);
            return;
        }

        // Save image
        if (task.imgBuf) {
            MIL_ID milImg = task.imgBuf->id();
            
            if (milImg == M_NULL) {
                ct::logger::error("[ImageSavingThread] Failed to save image, buffer is null: %s", task.imgPath.c_str());
                return;
            }

            int type = MbufInquire(milImg, M_TYPE, M_NULL);
            if (type == 16) {
                auto w = mtrx::get_width(milImg);
                auto h = mtrx::get_height(milImg);
                auto channel = mtrx::get_band(milImg);

                int depth = CV_16U;

                cv::Mat image(h, w, CV_MAKETYPE(depth, channel));

                if (channel == 1) {
                    MbufGet2d(milImg, 0, 0, w, h, image.data);
                }
                else {
                    MbufGetColor2d(milImg, M_PACKED + M_BGR24, M_ALL_BANDS, 0, 0, w, h, image.data);
                }

                cv::imwrite(task.imgPath, image);
            }
            else if (type == 8)
            {
                QFileInfo fileInfo(task.imgPath.c_str());
                QString extension = fileInfo.suffix();  // returns "bmp"

                if (extension == "bmp")
                {
                    MbufExportA(task.imgPath.c_str(), M_BMP, milImg);
                }
                else if (extension == "jpg" || extension == "jpeg")
                {
                    MbufExportA(task.imgPath.c_str(), M_JPEG_LOSSY, milImg);
                    if (task.copyPath != "none" && !task.copyPath.empty() && !task.heightBuf)
                    {
                        QFile::copy(task.imgPath.c_str(), task.copyPath.c_str());
                    }
                }
            }
            else {
                ct::logger::error("[ImageSavingThread] Failed to save image, unsupported bit %d: %s", type, task.imgPath.c_str());
            }
        }

        // Save height map
        if (task.heightBuf) {
            MIL_ID hid = task.heightBuf->id();

            if (hid == M_NULL) {
                ct::logger::error("[ImageSavingThread] Failed to save height image, buffer is null: %s", task.heightPath.c_str());
                return;
            }

            cv::Mat height;
            util::Mil_to_cv(hid, height);
            cv::imwrite(task.heightPath, height);
            if (task.copyPath != "none" && !task.copyPath.empty())
            {
                QFile::copy(task.heightPath.c_str(), task.copyPath.c_str());
            }
        }
    }
}
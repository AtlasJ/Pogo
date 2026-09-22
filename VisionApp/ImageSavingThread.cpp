#include "ImageSavingThread.h"
#include <mil.h>
#include <filesystem>
#include <fstream>
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

/*
* Point-cloud export of a height map, written binary little-endian: the same scan as ASCII
* would be tens of MB per unit, and every viewer (CloudCompare, MeshLab, Open3D) reads
* binary. Geometry is real millimetres so a cloud can be measured directly:
*   x = column * xPitchMm,  y = row * yPitchMm,  z = (grey - 32768) * zPitchUm / 1000
* Grey 0 is the driver's "no data" marker (manual p.40) and those pixels are dropped, so the
* vertex count has to be counted before the header is written.
*/
void ImageSavingThread::savePly(const ImageSaveInfo& task, const cv::Mat& height)
{
    if (height.empty() || height.type() != CV_16U) {
        ct::logger::error("[ImageSavingThread] PLY skipped, height map is not 16-bit: %s",
            task.plyPath.c_str());
        return;
    }

    if (task.xPitchMm <= 0.0 || task.yPitchMm <= 0.0 || task.zPitchUm <= 0.0) {
        //a cloud in the wrong units is worse than no cloud - it measures wrong silently
        ct::logger::error("[ImageSavingThread] PLY skipped, scales unknown (x=%.4f y=%.4f mm, z=%.4f um): %s",
            task.xPitchMm, task.yPitchMm, task.zPitchUm, task.plyPath.c_str());
        return;
    }

    size_t valid = 0;
    for (int r = 0; r < height.rows; r++) {
        const ushort* row = height.ptr<ushort>(r);
        for (int c = 0; c < height.cols; c++) if (row[c] != 0) valid++;
    }

    std::ofstream ofs(task.plyPath, std::ios::binary);
    if (!ofs.is_open()) {
        ct::logger::error("[ImageSavingThread] Failed to open PLY: %s", task.plyPath.c_str());
        return;
    }

    ofs << "ply\n";
    ofs << "format binary_little_endian 1.0\n";
    ofs << "comment Pogo 3D scan, units millimetres\n";
    ofs << "element vertex " << valid << "\n";
    ofs << "property float x\nproperty float y\nproperty float z\n";
    ofs << "end_header\n";

    const double zScaleMm = task.zPitchUm / 1000.0;
    for (int r = 0; r < height.rows; r++) {
        const ushort* row = height.ptr<ushort>(r);
        for (int c = 0; c < height.cols; c++) {
            if (row[c] == 0) continue;
            const float xyz[3] = {
                static_cast<float>(c * task.xPitchMm),
                static_cast<float>(r * task.yPitchMm),
                static_cast<float>((static_cast<int>(row[c]) - 32768) * zScaleMm)
            };
            ofs.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
        }
    }

    ofs.close();
    ct::logger::info("[ImageSavingThread] Saved PLY: %s (%zu points)", task.plyPath.c_str(), valid);
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

            if (!task.plyPath.empty()) savePly(task, height);
        }
    }
}
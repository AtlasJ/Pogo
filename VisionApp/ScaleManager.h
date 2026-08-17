#pragma once
#include <shared_mutex>
#include "WorldCoordinate.h"
#include "Box2D.h"
#include "QLineScan.h"
#include "QView.h"

#include <QPointF>
#include <QRectF>
#include <QJsonObject>

class ScaleManager {
private:
    ScaleManager() = default;
    ~ScaleManager() = default;
    ScaleManager(const ScaleManager&) = delete;
    ScaleManager& operator=(const ScaleManager&) = delete;

    const double allowable_double_error = 0.00001;

    dat::WorldEnvironment m_worldEnv;
    double m_worldScale = 1.0;
    double m_um_per_px = 1.0; //horizontal + vertical scale

    double m_laserFov_mm = 14.0;

    mutable std::shared_mutex mutex_h;
    mutable std::shared_mutex mutex_v;
    mutable std::shared_mutex mutex_worldEnv;
    mutable std::shared_mutex mutex_worldScale;

    void update_um_per_px();

public:
    static ScaleManager& instance() {
        static ScaleManager instance;
        return instance;
    }

    //getter setter
    void set_horizontal_um_per_px(double um_per_px);
    void set_vertical_um_per_px(double um_per_px);
    void set_world_scale(double world_scale);
    void set_world_env(const dat::WorldEnvironment& env);

    double horizontal_um_per_px() const;
    double vertical_um_per_px() const;
    double um_per_px() const;
    double world_scale() const;
    dat::WorldEnvironment world_env() const;

    void set_laser_fov_mm(double fov_mm);

    //=> Conversion
    double mm_to_px(double mm);
    double um_to_px(double um);
    double to_mm(double px);
    double to_um(double px);

    double mm_to_horizontal_px(double mm);
    double um_to_horizontal_px(double um);
    double mm_to_vertical_px(double mm);
    double um_to_vertical_px(double um);

    double to_horizontal_mm(double px);
    double to_horizontal_um(double px);
    double to_vertical_mm(double px);
    double to_vertical_um(double px);


    //conversion between world coordinates in mm and px format
    QPointF to_world_px(QPointF world_mm);
    QPointF to_world_mm(QPointF world_px);

    //conversion between world scale and fov scale (original scale)
    QRectF world_to_fov(const QRectF& rect);
    QRectF fov_to_world(const QRectF& rect);

    double world_to_fov(const double& px);
    double fov_to_world(const double& px);

    ct::Box2D world_to_fov(const ct::Box2D& box);
    ct::Box2D fov_to_world(const ct::Box2D& box);

    //conversion from world coordinates to world px and fov px
    QPointF to_fov_px(QLineScan linescan_mm);
    QPointF to_world_px(QLineScan linescan_mm);
    QPointF to_fov_px(QView view_mm);
    QPointF to_world_px(QView view_mm);


    //json
    bool extract_json_object(const QJsonObject& obj);
    QJsonObject json_object();
};
#include "myslam/Param.h"

namespace myslam
{
    // ORBextractor
    float Param::scale_ = 0.0;
    int Param::nlevels_ = 0;
    int Param::iniThFast_ = 0;
    int Param::minThFast_ = 0;

    // Odometry
    int Param::calc_method_ = 0;
    int Param::max_lost_ = 0;

    // DirectTracker
    int Param::pyramids_ = 0;
    float Param::pyramid_scale_ = 0.0;
    int Param::half_patch_ = 0;

    // Mapping
    int Param::map_size_ = 0;

    // System
    int Param::model_type_ = 0;

    // Detector
    string Param::detector_names_ = "";
    string Param::detector_config_ = "";
    string Param::detector_weight_ = "";
    float Param::detector_thr_ = 0.0;
    float Param::detector_nmsthr_ = 0.0;
    int Param::detector_inpwidth_ = 0;
    int Param::detector_inpheight_ = 0;

    // Segment
    string Param::segment_names_ = "";
    string Param::segment_config_ = "";
    string Param::segment_weight_ = "";
    float Param::segment_thr_ = 0.0;
    float Param::segment_nmsthr_ = 0.0;
    float Param::segment_maskthr_ = 0.0;

    // MotionClassifier
    float Param::min_area_ratio_ = 0.0;
    float Param::numpts_area_ratio_ = 0.0;
    int Param::halfpatch_size_ = 0;
    float Param::dyna_patch_ncc_ = 0.0;
    float Param::dyna_patch_std_ = 0.0;
    float Param::dyna_pos_change_ = 0.0;
    float Param::keeppts_ratio_ = 0.0;
    float Param::match_IOU_ = 0.0;
    bool Param::drawMatchObj_ = false;
    bool Param::drawLocateObj_ = false;
    bool Param::drawMissDetect_ = false;
    bool Param::drawGeometry_ = false;
    bool Param::drawAppearance_ = false;
    bool Param::drawPosition_ = false;

} // namespace myslam

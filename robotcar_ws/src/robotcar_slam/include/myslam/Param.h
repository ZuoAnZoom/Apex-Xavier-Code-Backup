#pragma once

#ifndef PARAM_H
#define PARAM_H

#include "myslam/CommonInclude.h"

namespace myslam
{
    class Param
    {
    public:
        
        // ORBextractor
        static float scale_;
        static int nlevels_;
        static int iniThFast_;
        static int minThFast_;

        // Odometry
        static int calc_method_;
        static int max_lost_;

        // DirectTracker
        static int pyramids_;
        static float pyramid_scale_;
        static int half_patch_;

        // Mapping
        static int map_size_;

        // System
        static int model_type_;
        
        // Detector
        static string detector_names_;
        static string detector_config_;
        static string detector_weight_;
        static float detector_thr_;
        static float detector_nmsthr_;
        static int detector_inpwidth_;
        static int detector_inpheight_;

        // Segment
        static string segment_names_;
        static string segment_config_;
        static string segment_weight_;
        static float segment_thr_;
        static float segment_nmsthr_;
        static float segment_maskthr_;

        // MotionClassifier
        static float min_area_ratio_;
        static float numpts_area_ratio_;
        static int halfpatch_size_;
        static float dyna_patch_ncc_;
        static float dyna_patch_std_;
        static float dyna_pos_change_;
        static float keeppts_ratio_;
        static float match_IOU_;
        static bool drawMatchObj_;
        static bool drawLocateObj_;
        static bool drawMissDetect_;
        static bool drawGeometry_;
        static bool drawAppearance_;
        static bool drawPosition_;
    };
} // namespace myslam
#endif

#include "myslam/Frame.h"
#include "myslam/ORBextractor_mod.h"

namespace myslam
{
    int ORBextractor::TH_LOW = 50;
    int ORBextractor::TH_HIGH = 100;
    const int PATCH_SIZE = 31;
    const int HALF_PATCH_SIZE = 15;
    const int EDGE_THRESHOLD = 19;
    int ORBextractor::nfeatures = 0;
    int ORBextractor::HISTO_LENGTH = 30;
    float ORBextractor::mfGridElementWidthInv, ORBextractor::mfGridElementHeightInv;
    std::vector<int> ORBextractor::mnFeaturesPerLevel;
    std::vector<float> ORBextractor::mvScaleFactor;
    std::vector<float> ORBextractor::mvInvScaleFactor;
    std::vector<float> ORBextractor::mvLevelSigma2;
    std::vector<float> ORBextractor::mvInvLevelSigma2;
    std::vector<int> ORBextractor::umax;
    /**
 *
 * @param image
 * @param pt
 * @param u_max:用于存储计算特征方向时，图像每个v坐标对应最大的u坐标
 * @return
 */
    static float IC_Angle(const Mat &image, Point2f pt, const vector<int> &u_max)
    {
        int m_01 = 0, m_10 = 0;

        const uchar *center = &image.at<uchar>(cvRound(pt.y), cvRound(pt.x));

        // Treat the center line differently, v=0
        for (int u = -HALF_PATCH_SIZE; u <= HALF_PATCH_SIZE; ++u)
            m_10 += u * center[u];

        // Go line by line in the circuI853lar patch
        int step = (int)image.step1();
        for (int v = 1; v <= HALF_PATCH_SIZE; ++v)
        {
            // Proceed over the two lines
            int v_sum = 0;
            int d = u_max[v];
            for (int u = -d; u <= d; ++u)
            {
                int val_plus = center[u + v * step], val_minus = center[u - v * step];
                v_sum += (val_plus - val_minus);
                m_10 += u * (val_plus + val_minus);
            }
            m_01 += v * v_sum;
        }

        return fastAtan2((float)m_01, (float)m_10);
    }

    //角度转弧度
    const float factorPI = (float)(CV_PI / 180.f);

    static void computeOrbDescriptor(const KeyPoint &kpt,
                                     const Mat &img, const Point *pattern,
                                     uchar *desc)
    {
        float angle = (float)kpt.angle * factorPI;
        float a = (float)cos(angle), b = (float)sin(angle);

        const uchar *center = &img.at<uchar>(cvRound(kpt.pt.y), cvRound(kpt.pt.x));
        const int step = (int)img.step; //一行的数据量，以字节为单位

#define GET_VALUE(idx)                                               \
    center[cvRound(pattern[idx].x * b + pattern[idx].y * a) * step + \
           cvRound(pattern[idx].x * a - pattern[idx].y * b)] //取旋转后一个像素点的值

        for (int i = 0; i < 32; ++i, pattern += 16) //每个特征点描述子长度为32字节
        {
            int t0, t1, val;
            t0 = GET_VALUE(0);
            t1 = GET_VALUE(1);
            val = t0 < t1;
            t0 = GET_VALUE(2);
            t1 = GET_VALUE(3);
            val |= (t0 < t1) << 1; //c|=2等同于c=c|2
            t0 = GET_VALUE(4);
            t1 = GET_VALUE(5);
            val |= (t0 < t1) << 2;
            t0 = GET_VALUE(6);
            t1 = GET_VALUE(7);
            val |= (t0 < t1) << 3;
            t0 = GET_VALUE(8);
            t1 = GET_VALUE(9);
            val |= (t0 < t1) << 4;
            t0 = GET_VALUE(10);
            t1 = GET_VALUE(11);
            val |= (t0 < t1) << 5;
            t0 = GET_VALUE(12);
            t1 = GET_VALUE(13);
            val |= (t0 < t1) << 6;
            t0 = GET_VALUE(14);
            t1 = GET_VALUE(15);
            val |= (t0 < t1) << 7;

            desc[i] = (uchar)val;
        }

#undef GET_VALUE
    }

    static int bit_pattern_31_[256 * 4] =
        {
            8, -3, 9, 5 /*mean (0), correlation (0)*/,
            4, 2, 7, -12 /*mean (1.12461e-05), correlation (0.0437584)*/,
            -11, 9, -8, 2 /*mean (3.37382e-05), correlation (0.0617409)*/,
            7, -12, 12, -13 /*mean (5.62303e-05), correlation (0.0636977)*/,
            2, -13, 2, 12 /*mean (0.000134953), correlation (0.085099)*/,
            1, -7, 1, 6 /*mean (0.000528565), correlation (0.0857175)*/,
            -2, -10, -2, -4 /*mean (0.0188821), correlation (0.0985774)*/,
            -13, -13, -11, -8 /*mean (0.0363135), correlation (0.0899616)*/,
            -13, -3, -12, -9 /*mean (0.121806), correlation (0.099849)*/,
            10, 4, 11, 9 /*mean (0.122065), correlation (0.093285)*/,
            -13, -8, -8, -9 /*mean (0.162787), correlation (0.0942748)*/,
            -11, 7, -9, 12 /*mean (0.21561), correlation (0.0974438)*/,
            7, 7, 12, 6 /*mean (0.160583), correlation (0.130064)*/,
            -4, -5, -3, 0 /*mean (0.228171), correlation (0.132998)*/,
            -13, 2, -12, -3 /*mean (0.00997526), correlation (0.145926)*/,
            -9, 0, -7, 5 /*mean (0.198234), correlation (0.143636)*/,
            12, -6, 12, -1 /*mean (0.0676226), correlation (0.16689)*/,
            -3, 6, -2, 12 /*mean (0.166847), correlation (0.171682)*/,
            -6, -13, -4, -8 /*mean (0.101215), correlation (0.179716)*/,
            11, -13, 12, -8 /*mean (0.200641), correlation (0.192279)*/,
            4, 7, 5, 1 /*mean (0.205106), correlation (0.186848)*/,
            5, -3, 10, -3 /*mean (0.234908), correlation (0.192319)*/,
            3, -7, 6, 12 /*mean (0.0709964), correlation (0.210872)*/,
            -8, -7, -6, -2 /*mean (0.0939834), correlation (0.212589)*/,
            -2, 11, -1, -10 /*mean (0.127778), correlation (0.20866)*/,
            -13, 12, -8, 10 /*mean (0.14783), correlation (0.206356)*/,
            -7, 3, -5, -3 /*mean (0.182141), correlation (0.198942)*/,
            -4, 2, -3, 7 /*mean (0.188237), correlation (0.21384)*/,
            -10, -12, -6, 11 /*mean (0.14865), correlation (0.23571)*/,
            5, -12, 6, -7 /*mean (0.222312), correlation (0.23324)*/,
            5, -6, 7, -1 /*mean (0.229082), correlation (0.23389)*/,
            1, 0, 4, -5 /*mean (0.241577), correlation (0.215286)*/,
            9, 11, 11, -13 /*mean (0.00338507), correlation (0.251373)*/,
            4, 7, 4, 12 /*mean (0.131005), correlation (0.257622)*/,
            2, -1, 4, 4 /*mean (0.152755), correlation (0.255205)*/,
            -4, -12, -2, 7 /*mean (0.182771), correlation (0.244867)*/,
            -8, -5, -7, -10 /*mean (0.186898), correlation (0.23901)*/,
            4, 11, 9, 12 /*mean (0.226226), correlation (0.258255)*/,
            0, -8, 1, -13 /*mean (0.0897886), correlation (0.274827)*/,
            -13, -2, -8, 2 /*mean (0.148774), correlation (0.28065)*/,
            -3, -2, -2, 3 /*mean (0.153048), correlation (0.283063)*/,
            -6, 9, -4, -9 /*mean (0.169523), correlation (0.278248)*/,
            8, 12, 10, 7 /*mean (0.225337), correlation (0.282851)*/,
            0, 9, 1, 3 /*mean (0.226687), correlation (0.278734)*/,
            7, -5, 11, -10 /*mean (0.00693882), correlation (0.305161)*/,
            -13, -6, -11, 0 /*mean (0.0227283), correlation (0.300181)*/,
            10, 7, 12, 1 /*mean (0.125517), correlation (0.31089)*/,
            -6, -3, -6, 12 /*mean (0.131748), correlation (0.312779)*/,
            10, -9, 12, -4 /*mean (0.144827), correlation (0.292797)*/,
            -13, 8, -8, -12 /*mean (0.149202), correlation (0.308918)*/,
            -13, 0, -8, -4 /*mean (0.160909), correlation (0.310013)*/,
            3, 3, 7, 8 /*mean (0.177755), correlation (0.309394)*/,
            5, 7, 10, -7 /*mean (0.212337), correlation (0.310315)*/,
            -1, 7, 1, -12 /*mean (0.214429), correlation (0.311933)*/,
            3, -10, 5, 6 /*mean (0.235807), correlation (0.313104)*/,
            2, -4, 3, -10 /*mean (0.00494827), correlation (0.344948)*/,
            -13, 0, -13, 5 /*mean (0.0549145), correlation (0.344675)*/,
            -13, -7, -12, 12 /*mean (0.103385), correlation (0.342715)*/,
            -13, 3, -11, 8 /*mean (0.134222), correlation (0.322922)*/,
            -7, 12, -4, 7 /*mean (0.153284), correlation (0.337061)*/,
            6, -10, 12, 8 /*mean (0.154881), correlation (0.329257)*/,
            -9, -1, -7, -6 /*mean (0.200967), correlation (0.33312)*/,
            -2, -5, 0, 12 /*mean (0.201518), correlation (0.340635)*/,
            -12, 5, -7, 5 /*mean (0.207805), correlation (0.335631)*/,
            3, -10, 8, -13 /*mean (0.224438), correlation (0.34504)*/,
            -7, -7, -4, 5 /*mean (0.239361), correlation (0.338053)*/,
            -3, -2, -1, -7 /*mean (0.240744), correlation (0.344322)*/,
            2, 9, 5, -11 /*mean (0.242949), correlation (0.34145)*/,
            -11, -13, -5, -13 /*mean (0.244028), correlation (0.336861)*/,
            -1, 6, 0, -1 /*mean (0.247571), correlation (0.343684)*/,
            5, -3, 5, 2 /*mean (0.000697256), correlation (0.357265)*/,
            -4, -13, -4, 12 /*mean (0.00213675), correlation (0.373827)*/,
            -9, -6, -9, 6 /*mean (0.0126856), correlation (0.373938)*/,
            -12, -10, -8, -4 /*mean (0.0152497), correlation (0.364237)*/,
            10, 2, 12, -3 /*mean (0.0299933), correlation (0.345292)*/,
            7, 12, 12, 12 /*mean (0.0307242), correlation (0.366299)*/,
            -7, -13, -6, 5 /*mean (0.0534975), correlation (0.368357)*/,
            -4, 9, -3, 4 /*mean (0.099865), correlation (0.372276)*/,
            7, -1, 12, 2 /*mean (0.117083), correlation (0.364529)*/,
            -7, 6, -5, 1 /*mean (0.126125), correlation (0.369606)*/,
            -13, 11, -12, 5 /*mean (0.130364), correlation (0.358502)*/,
            -3, 7, -2, -6 /*mean (0.131691), correlation (0.375531)*/,
            7, -8, 12, -7 /*mean (0.160166), correlation (0.379508)*/,
            -13, -7, -11, -12 /*mean (0.167848), correlation (0.353343)*/,
            1, -3, 12, 12 /*mean (0.183378), correlation (0.371916)*/,
            2, -6, 3, 0 /*mean (0.228711), correlation (0.371761)*/,
            -4, 3, -2, -13 /*mean (0.247211), correlation (0.364063)*/,
            -1, -13, 1, 9 /*mean (0.249325), correlation (0.378139)*/,
            7, 1, 8, -6 /*mean (0.000652272), correlation (0.411682)*/,
            1, -1, 3, 12 /*mean (0.00248538), correlation (0.392988)*/,
            9, 1, 12, 6 /*mean (0.0206815), correlation (0.386106)*/,
            -1, -9, -1, 3 /*mean (0.0364485), correlation (0.410752)*/,
            -13, -13, -10, 5 /*mean (0.0376068), correlation (0.398374)*/,
            7, 7, 10, 12 /*mean (0.0424202), correlation (0.405663)*/,
            12, -5, 12, 9 /*mean (0.0942645), correlation (0.410422)*/,
            6, 3, 7, 11 /*mean (0.1074), correlation (0.413224)*/,
            5, -13, 6, 10 /*mean (0.109256), correlation (0.408646)*/,
            2, -12, 2, 3 /*mean (0.131691), correlation (0.416076)*/,
            3, 8, 4, -6 /*mean (0.165081), correlation (0.417569)*/,
            2, 6, 12, -13 /*mean (0.171874), correlation (0.408471)*/,
            9, -12, 10, 3 /*mean (0.175146), correlation (0.41296)*/,
            -8, 4, -7, 9 /*mean (0.183682), correlation (0.402956)*/,
            -11, 12, -4, -6 /*mean (0.184672), correlation (0.416125)*/,
            1, 12, 2, -8 /*mean (0.191487), correlation (0.386696)*/,
            6, -9, 7, -4 /*mean (0.192668), correlation (0.394771)*/,
            2, 3, 3, -2 /*mean (0.200157), correlation (0.408303)*/,
            6, 3, 11, 0 /*mean (0.204588), correlation (0.411762)*/,
            3, -3, 8, -8 /*mean (0.205904), correlation (0.416294)*/,
            7, 8, 9, 3 /*mean (0.213237), correlation (0.409306)*/,
            -11, -5, -6, -4 /*mean (0.243444), correlation (0.395069)*/,
            -10, 11, -5, 10 /*mean (0.247672), correlation (0.413392)*/,
            -5, -8, -3, 12 /*mean (0.24774), correlation (0.411416)*/,
            -10, 5, -9, 0 /*mean (0.00213675), correlation (0.454003)*/,
            8, -1, 12, -6 /*mean (0.0293635), correlation (0.455368)*/,
            4, -6, 6, -11 /*mean (0.0404971), correlation (0.457393)*/,
            -10, 12, -8, 7 /*mean (0.0481107), correlation (0.448364)*/,
            4, -2, 6, 7 /*mean (0.050641), correlation (0.455019)*/,
            -2, 0, -2, 12 /*mean (0.0525978), correlation (0.44338)*/,
            -5, -8, -5, 2 /*mean (0.0629667), correlation (0.457096)*/,
            7, -6, 10, 12 /*mean (0.0653846), correlation (0.445623)*/,
            -9, -13, -8, -8 /*mean (0.0858749), correlation (0.449789)*/,
            -5, -13, -5, -2 /*mean (0.122402), correlation (0.450201)*/,
            8, -8, 9, -13 /*mean (0.125416), correlation (0.453224)*/,
            -9, -11, -9, 0 /*mean (0.130128), correlation (0.458724)*/,
            1, -8, 1, -2 /*mean (0.132467), correlation (0.440133)*/,
            7, -4, 9, 1 /*mean (0.132692), correlation (0.454)*/,
            -2, 1, -1, -4 /*mean (0.135695), correlation (0.455739)*/,
            11, -6, 12, -11 /*mean (0.142904), correlation (0.446114)*/,
            -12, -9, -6, 4 /*mean (0.146165), correlation (0.451473)*/,
            3, 7, 7, 12 /*mean (0.147627), correlation (0.456643)*/,
            5, 5, 10, 8 /*mean (0.152901), correlation (0.455036)*/,
            0, -4, 2, 8 /*mean (0.167083), correlation (0.459315)*/,
            -9, 12, -5, -13 /*mean (0.173234), correlation (0.454706)*/,
            0, 7, 2, 12 /*mean (0.18312), correlation (0.433855)*/,
            -1, 2, 1, 7 /*mean (0.185504), correlation (0.443838)*/,
            5, 11, 7, -9 /*mean (0.185706), correlation (0.451123)*/,
            3, 5, 6, -8 /*mean (0.188968), correlation (0.455808)*/,
            -13, -4, -8, 9 /*mean (0.191667), correlation (0.459128)*/,
            -5, 9, -3, -3 /*mean (0.193196), correlation (0.458364)*/,
            -4, -7, -3, -12 /*mean (0.196536), correlation (0.455782)*/,
            6, 5, 8, 0 /*mean (0.1972), correlation (0.450481)*/,
            -7, 6, -6, 12 /*mean (0.199438), correlation (0.458156)*/,
            -13, 6, -5, -2 /*mean (0.211224), correlation (0.449548)*/,
            1, -10, 3, 10 /*mean (0.211718), correlation (0.440606)*/,
            4, 1, 8, -4 /*mean (0.213034), correlation (0.443177)*/,
            -2, -2, 2, -13 /*mean (0.234334), correlation (0.455304)*/,
            2, -12, 12, 12 /*mean (0.235684), correlation (0.443436)*/,
            -2, -13, 0, -6 /*mean (0.237674), correlation (0.452525)*/,
            4, 1, 9, 3 /*mean (0.23962), correlation (0.444824)*/,
            -6, -10, -3, -5 /*mean (0.248459), correlation (0.439621)*/,
            -3, -13, -1, 1 /*mean (0.249505), correlation (0.456666)*/,
            7, 5, 12, -11 /*mean (0.00119208), correlation (0.495466)*/,
            4, -2, 5, -7 /*mean (0.00372245), correlation (0.484214)*/,
            -13, 9, -9, -5 /*mean (0.00741116), correlation (0.499854)*/,
            7, 1, 8, 6 /*mean (0.0208952), correlation (0.499773)*/,
            7, -8, 7, 6 /*mean (0.0220085), correlation (0.501609)*/,
            -7, -4, -7, 1 /*mean (0.0233806), correlation (0.496568)*/,
            -8, 11, -7, -8 /*mean (0.0236505), correlation (0.489719)*/,
            -13, 6, -12, -8 /*mean (0.0268781), correlation (0.503487)*/,
            2, 4, 3, 9 /*mean (0.0323324), correlation (0.501938)*/,
            10, -5, 12, 3 /*mean (0.0399235), correlation (0.494029)*/,
            -6, -5, -6, 7 /*mean (0.0420153), correlation (0.486579)*/,
            8, -3, 9, -8 /*mean (0.0548021), correlation (0.484237)*/,
            2, -12, 2, 8 /*mean (0.0616622), correlation (0.496642)*/,
            -11, -2, -10, 3 /*mean (0.0627755), correlation (0.498563)*/,
            -12, -13, -7, -9 /*mean (0.0829622), correlation (0.495491)*/,
            -11, 0, -10, -5 /*mean (0.0843342), correlation (0.487146)*/,
            5, -3, 11, 8 /*mean (0.0929937), correlation (0.502315)*/,
            -2, -13, -1, 12 /*mean (0.113327), correlation (0.48941)*/,
            -1, -8, 0, 9 /*mean (0.132119), correlation (0.467268)*/,
            -13, -11, -12, -5 /*mean (0.136269), correlation (0.498771)*/,
            -10, -2, -10, 11 /*mean (0.142173), correlation (0.498714)*/,
            -3, 9, -2, -13 /*mean (0.144141), correlation (0.491973)*/,
            2, -3, 3, 2 /*mean (0.14892), correlation (0.500782)*/,
            -9, -13, -4, 0 /*mean (0.150371), correlation (0.498211)*/,
            -4, 6, -3, -10 /*mean (0.152159), correlation (0.495547)*/,
            -4, 12, -2, -7 /*mean (0.156152), correlation (0.496925)*/,
            -6, -11, -4, 9 /*mean (0.15749), correlation (0.499222)*/,
            6, -3, 6, 11 /*mean (0.159211), correlation (0.503821)*/,
            -13, 11, -5, 5 /*mean (0.162427), correlation (0.501907)*/,
            11, 11, 12, 6 /*mean (0.16652), correlation (0.497632)*/,
            7, -5, 12, -2 /*mean (0.169141), correlation (0.484474)*/,
            -1, 12, 0, 7 /*mean (0.169456), correlation (0.495339)*/,
            -4, -8, -3, -2 /*mean (0.171457), correlation (0.487251)*/,
            -7, 1, -6, 7 /*mean (0.175), correlation (0.500024)*/,
            -13, -12, -8, -13 /*mean (0.175866), correlation (0.497523)*/,
            -7, -2, -6, -8 /*mean (0.178273), correlation (0.501854)*/,
            -8, 5, -6, -9 /*mean (0.181107), correlation (0.494888)*/,
            -5, -1, -4, 5 /*mean (0.190227), correlation (0.482557)*/,
            -13, 7, -8, 10 /*mean (0.196739), correlation (0.496503)*/,
            1, 5, 5, -13 /*mean (0.19973), correlation (0.499759)*/,
            1, 0, 10, -13 /*mean (0.204465), correlation (0.49873)*/,
            9, 12, 10, -1 /*mean (0.209334), correlation (0.49063)*/,
            5, -8, 10, -9 /*mean (0.211134), correlation (0.503011)*/,
            -1, 11, 1, -13 /*mean (0.212), correlation (0.499414)*/,
            -9, -3, -6, 2 /*mean (0.212168), correlation (0.480739)*/,
            -1, -10, 1, 12 /*mean (0.212731), correlation (0.502523)*/,
            -13, 1, -8, -10 /*mean (0.21327), correlation (0.489786)*/,
            8, -11, 10, -6 /*mean (0.214159), correlation (0.488246)*/,
            2, -13, 3, -6 /*mean (0.216993), correlation (0.50287)*/,
            7, -13, 12, -9 /*mean (0.223639), correlation (0.470502)*/,
            -10, -10, -5, -7 /*mean (0.224089), correlation (0.500852)*/,
            -10, -8, -8, -13 /*mean (0.228666), correlation (0.502629)*/,
            4, -6, 8, 5 /*mean (0.22906), correlation (0.498305)*/,
            3, 12, 8, -13 /*mean (0.233378), correlation (0.503825)*/,
            -4, 2, -3, -3 /*mean (0.234323), correlation (0.476692)*/,
            5, -13, 10, -12 /*mean (0.236392), correlation (0.475462)*/,
            4, -13, 5, -1 /*mean (0.236842), correlation (0.504132)*/,
            -9, 9, -4, 3 /*mean (0.236977), correlation (0.497739)*/,
            0, 3, 3, -9 /*mean (0.24314), correlation (0.499398)*/,
            -12, 1, -6, 1 /*mean (0.243297), correlation (0.489447)*/,
            3, 2, 4, -8 /*mean (0.00155196), correlation (0.553496)*/,
            -10, -10, -10, 9 /*mean (0.00239541), correlation (0.54297)*/,
            8, -13, 12, 12 /*mean (0.0034413), correlation (0.544361)*/,
            -8, -12, -6, -5 /*mean (0.003565), correlation (0.551225)*/,
            2, 2, 3, 7 /*mean (0.00835583), correlation (0.55285)*/,
            10, 6, 11, -8 /*mean (0.00885065), correlation (0.540913)*/,
            6, 8, 8, -12 /*mean (0.0101552), correlation (0.551085)*/,
            -7, 10, -6, 5 /*mean (0.0102227), correlation (0.533635)*/,
            -3, -9, -3, 9 /*mean (0.0110211), correlation (0.543121)*/,
            -1, -13, -1, 5 /*mean (0.0113473), correlation (0.550173)*/,
            -3, -7, -3, 4 /*mean (0.0140913), correlation (0.554774)*/,
            -8, -2, -8, 3 /*mean (0.017049), correlation (0.55461)*/,
            4, 2, 12, 12 /*mean (0.01778), correlation (0.546921)*/,
            2, -5, 3, 11 /*mean (0.0224022), correlation (0.549667)*/,
            6, -9, 11, -13 /*mean (0.029161), correlation (0.546295)*/,
            3, -1, 7, 12 /*mean (0.0303081), correlation (0.548599)*/,
            11, -1, 12, 4 /*mean (0.0355151), correlation (0.523943)*/,
            -3, 0, -3, 6 /*mean (0.0417904), correlation (0.543395)*/,
            4, -11, 4, 12 /*mean (0.0487292), correlation (0.542818)*/,
            2, -4, 2, 1 /*mean (0.0575124), correlation (0.554888)*/,
            -10, -6, -8, 1 /*mean (0.0594242), correlation (0.544026)*/,
            -13, 7, -11, 1 /*mean (0.0597391), correlation (0.550524)*/,
            -13, 12, -11, -13 /*mean (0.0608974), correlation (0.55383)*/,
            6, 0, 11, -13 /*mean (0.065126), correlation (0.552006)*/,
            0, -1, 1, 4 /*mean (0.074224), correlation (0.546372)*/,
            -13, 3, -9, -2 /*mean (0.0808592), correlation (0.554875)*/,
            -9, 8, -6, -3 /*mean (0.0883378), correlation (0.551178)*/,
            -13, -6, -8, -2 /*mean (0.0901035), correlation (0.548446)*/,
            5, -9, 8, 10 /*mean (0.0949843), correlation (0.554694)*/,
            2, 7, 3, -9 /*mean (0.0994152), correlation (0.550979)*/,
            -1, -6, -1, -1 /*mean (0.10045), correlation (0.552714)*/,
            9, 5, 11, -2 /*mean (0.100686), correlation (0.552594)*/,
            11, -3, 12, -8 /*mean (0.101091), correlation (0.532394)*/,
            3, 0, 3, 5 /*mean (0.101147), correlation (0.525576)*/,
            -1, 4, 0, 10 /*mean (0.105263), correlation (0.531498)*/,
            3, -6, 4, 5 /*mean (0.110785), correlation (0.540491)*/,
            -13, 0, -10, 5 /*mean (0.112798), correlation (0.536582)*/,
            5, 8, 12, 11 /*mean (0.114181), correlation (0.555793)*/,
            8, 9, 9, -6 /*mean (0.117431), correlation (0.553763)*/,
            7, -4, 8, -12 /*mean (0.118522), correlation (0.553452)*/,
            -10, 4, -10, 9 /*mean (0.12094), correlation (0.554785)*/,
            7, 3, 12, 4 /*mean (0.122582), correlation (0.555825)*/,
            9, -7, 10, -2 /*mean (0.124978), correlation (0.549846)*/,
            7, 0, 12, -2 /*mean (0.127002), correlation (0.537452)*/,
            -1, -6, 0, -11 /*mean (0.127148), correlation (0.547401)*/
    };

    /**
 * @brief ORBextractor的构造函数
 * 
 * 下面这些参数从配置文件中读取
 * @param _nfeatures 
 * @param _scaleFactor 
 * @param _nlevels 
 * @param _iniThFAST 
 * @param _minThFAST 
*/
    ORBextractor::ORBextractor()
    {
        mvImagePyramid.resize(Param::nlevels_);
        const int npoints = 512;
        const Point *pattern0 = (const Point *)bit_pattern_31_;
        std::copy(pattern0, pattern0 + npoints, std::back_inserter(pattern));

        //This is for orientation
        // pre-compute the end of a row in a circular patch
    }

    ORBextractor::~ORBextractor() {}

    static void computeOrientation(const Mat &image, vector<KeyPoint> &keypoints, const vector<int> &umax)
    {
        for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                                        keypointEnd = keypoints.end();
             keypoint != keypointEnd; ++keypoint)
        {
            keypoint->angle = IC_Angle(image, keypoint->pt, umax);
        }
    }

    void ExtractorNode::DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4)
    {
        const int halfX = ceil(static_cast<float>(UR.x - UL.x) / 2);
        const int halfY = ceil(static_cast<float>(BR.y - UL.y) / 2);

        //Define boundaries of childs
        n1.UL = UL;
        n1.UR = cv::Point2i(UL.x + halfX, UL.y);
        n1.BL = cv::Point2i(UL.x, UL.y + halfY);
        n1.BR = cv::Point2i(UL.x + halfX, UL.y + halfY);
        n1.vKeys.reserve(vKeys.size());

        n2.UL = n1.UR;
        n2.UR = UR;
        n2.BL = n1.BR;
        n2.BR = cv::Point2i(UR.x, UL.y + halfY);
        n2.vKeys.reserve(vKeys.size());

        n3.UL = n1.BL;
        n3.UR = n1.BR;
        n3.BL = BL;
        n3.BR = cv::Point2i(n1.BR.x, BL.y);
        n3.vKeys.reserve(vKeys.size());

        n4.UL = n3.UR;
        n4.UR = n2.BR;
        n4.BL = n3.BR;
        n4.BR = BR;
        n4.vKeys.reserve(vKeys.size());

        //Associate points to childs
        for (size_t i = 0; i < vKeys.size(); i++)
        {
            const cv::KeyPoint &kp = vKeys[i];
            if (kp.pt.x < n1.UR.x)
            {
                if (kp.pt.y < n1.BR.y)
                    n1.vKeys.push_back(kp);
                else
                    n3.vKeys.push_back(kp);
            }
            else if (kp.pt.y < n1.BR.y)
                n2.vKeys.push_back(kp);
            else
                n4.vKeys.push_back(kp);
        }

        if (n1.vKeys.size() == 1)
            n1.bNoMore = true;
        if (n2.vKeys.size() == 1)
            n2.bNoMore = true;
        if (n3.vKeys.size() == 1)
            n3.bNoMore = true;
        if (n4.vKeys.size() == 1)
            n4.bNoMore = true;
    }

    vector<cv::KeyPoint> ORBextractor::DistributeOctTree(const vector<cv::KeyPoint> &vToDistributeKeys, const int &minX,
                                                         const int &maxX, const int &minY, const int &maxY, const int &N, const int &level)
    {
        // Compute how many initial nodes
        const int nIni = round(static_cast<float>(maxX - minX) / (maxY - minY));

        const float hX = static_cast<float>(maxX - minX) / nIni;

        list<ExtractorNode> lNodes;

        vector<ExtractorNode *> vpIniNodes;
        vpIniNodes.resize(nIni);

        for (int i = 0; i < nIni; i++)
        {
            ExtractorNode ni;
            ni.UL = cv::Point2i(hX * static_cast<float>(i), 0);
            ni.UR = cv::Point2i(hX * static_cast<float>(i + 1), 0);
            ni.BL = cv::Point2i(ni.UL.x, maxY - minY);
            ni.BR = cv::Point2i(ni.UR.x, maxY - minY);
            ni.vKeys.reserve(vToDistributeKeys.size());

            lNodes.push_back(ni);
            vpIniNodes[i] = &lNodes.back();
        }

        //Associate points to childs
        for (size_t i = 0; i < vToDistributeKeys.size(); i++)
        {
            const cv::KeyPoint &kp = vToDistributeKeys[i];
            vpIniNodes[kp.pt.x / hX]->vKeys.push_back(kp);
        }

        list<ExtractorNode>::iterator lit = lNodes.begin();

        while (lit != lNodes.end())
        {
            if (lit->vKeys.size() == 1)
            {
                lit->bNoMore = true;
                lit++;
            }
            else if (lit->vKeys.empty())
                lit = lNodes.erase(lit);
            else
                lit++;
        }

        bool bFinish = false;

        int iteration = 0;

        vector<pair<int, ExtractorNode *>> vSizeAndPointerToNode;
        vSizeAndPointerToNode.reserve(lNodes.size() * 4);

        // 根据兴趣点分布,利用N叉树方法对图像进行划分区域
        while (!bFinish)
        {
            iteration++;

            int prevSize = lNodes.size();

            lit = lNodes.begin();

            int nToExpand = 0;

            vSizeAndPointerToNode.clear();

            // 将目前的子区域经行划分
            while (lit != lNodes.end())
            {
                if (lit->bNoMore)
                {
                    // If node only contains one point do not subdivide and continue
                    lit++;
                    continue;
                }
                else
                {
                    // If more than one point, subdivide
                    ExtractorNode n1, n2, n3, n4;
                    lit->DivideNode(n1, n2, n3, n4); // 再细分成四个子区域

                    // Add childs if they contain points
                    if (n1.vKeys.size() > 0)
                    {
                        lNodes.push_front(n1);
                        if (n1.vKeys.size() > 1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n2.vKeys.size() > 0)
                    {
                        lNodes.push_front(n2);
                        if (n2.vKeys.size() > 1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n3.vKeys.size() > 0)
                    {
                        lNodes.push_front(n3);
                        if (n3.vKeys.size() > 1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if (n4.vKeys.size() > 0)
                    {
                        lNodes.push_front(n4);
                        if (n4.vKeys.size() > 1)
                        {
                            nToExpand++;
                            vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(), &lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    lit = lNodes.erase(lit);
                    continue;
                }
            }

            // Finish if there are more nodes than required features
            // or all nodes contain just one point
            if ((int)lNodes.size() >= N || (int)lNodes.size() == prevSize)
            {
                bFinish = true;
            }
            // 当再划分之后所有的Node数大于要求数目时
            else if (((int)lNodes.size() + nToExpand * 3) > N)
            {

                while (!bFinish)
                {

                    prevSize = lNodes.size();

                    vector<pair<int, ExtractorNode *>> vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                    vSizeAndPointerToNode.clear();

                    // 对需要划分的部分进行排序, 即对兴趣点数较多的区域进行划分
                    sort(vPrevSizeAndPointerToNode.begin(), vPrevSizeAndPointerToNode.end());
                    for (int j = vPrevSizeAndPointerToNode.size() - 1; j >= 0; j--)
                    {
                        ExtractorNode n1, n2, n3, n4;
                        vPrevSizeAndPointerToNode[j].second->DivideNode(n1, n2, n3, n4);

                        // Add childs if they contain points
                        if (n1.vKeys.size() > 0)
                        {
                            lNodes.push_front(n1);
                            if (n1.vKeys.size() > 1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n2.vKeys.size() > 0)
                        {
                            lNodes.push_front(n2);
                            if (n2.vKeys.size() > 1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n3.vKeys.size() > 0)
                        {
                            lNodes.push_front(n3);
                            if (n3.vKeys.size() > 1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }
                        if (n4.vKeys.size() > 0)
                        {
                            lNodes.push_front(n4);
                            if (n4.vKeys.size() > 1)
                            {
                                vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(), &lNodes.front()));
                                lNodes.front().lit = lNodes.begin();
                            }
                        }

                        lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                        if ((int)lNodes.size() >= N)
                            break;
                    }

                    if ((int)lNodes.size() >= N || (int)lNodes.size() == prevSize)
                        bFinish = true;
                }
            }
        }

        // Retain the best point in each node
        // 保留每个区域响应值最大的一个兴趣点
        vector<cv::KeyPoint> vResultKeys;
        vResultKeys.reserve(nfeatures);
        for (list<ExtractorNode>::iterator lit = lNodes.begin(); lit != lNodes.end(); lit++)
        {
            vector<cv::KeyPoint> &vNodeKeys = lit->vKeys;
            cv::KeyPoint *pKP = &vNodeKeys[0];
            float maxResponse = pKP->response;

            for (size_t k = 1; k < vNodeKeys.size(); k++)
            {
                if (vNodeKeys[k].response > maxResponse)
                {
                    pKP = &vNodeKeys[k];
                    maxResponse = vNodeKeys[k].response;
                }
            }

            vResultKeys.push_back(*pKP);
        }

        return vResultKeys;
    }
    void ORBextractor::ComputeKeyPointsOctTree(vector<vector<KeyPoint>> &allKeypoints)
    {
        allKeypoints.resize(Param::nlevels_);

        const float W = 30;

        // 对每一层图像做处理
        for (int level = 0; level < Param::nlevels_; ++level)
        {
            const int minBorderX = EDGE_THRESHOLD - 3;
            const int minBorderY = minBorderX;
            const int maxBorderX = mvImagePyramid[level].cols - EDGE_THRESHOLD + 3;
            const int maxBorderY = mvImagePyramid[level].rows - EDGE_THRESHOLD + 3;

            vector<cv::KeyPoint> vToDistributeKeys;
            vToDistributeKeys.reserve(nfeatures * 10);

            const float width = (maxBorderX - minBorderX);
            const float height = (maxBorderY - minBorderY);

            const int nCols = width / W;
            const int nRows = height / W;
            const int wCell = ceil(width / nCols);
            const int hCell = ceil(height / nRows);

            for (int i = 0; i < nRows; i++)
            {
                const float iniY = minBorderY + i * hCell;
                float maxY = iniY + hCell + 6;

                if (iniY >= maxBorderY - 3)
                    continue;
                if (maxY > maxBorderY)
                    maxY = maxBorderY;

                for (int j = 0; j < nCols; j++)
                {
                    const float iniX = minBorderX + j * wCell;
                    float maxX = iniX + wCell + 6;
                    if (iniX >= maxBorderX - 6)
                        continue;
                    if (maxX > maxBorderX)
                        maxX = maxBorderX;

                    // FAST提取兴趣点, 自适应阈值
                    vector<cv::KeyPoint> vKeysCell;
                    cv::FAST(mvImagePyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX),
                             vKeysCell, Param::iniThFast_, true);

                    if (vKeysCell.empty())
                    {
                        FAST(mvImagePyramid[level].rowRange(iniY, maxY).colRange(iniX, maxX),
                             vKeysCell, Param::minThFast_, true);
                    }

                    if (!vKeysCell.empty())
                    {
                        for (vector<cv::KeyPoint>::iterator vit = vKeysCell.begin(); vit != vKeysCell.end(); vit++)
                        {
                            (*vit).pt.x += j * wCell;
                            (*vit).pt.y += i * hCell;
                            vToDistributeKeys.push_back(*vit);
                        }
                    }
                }
            }

            vector<KeyPoint> &keypoints = allKeypoints[level];
            keypoints.reserve(nfeatures);

            // 根据mnFeaturesPerLevel,即该层的兴趣点数,对特征点进行剔除
            keypoints = DistributeOctTree(vToDistributeKeys, minBorderX, maxBorderX,
                                          minBorderY, maxBorderY, mnFeaturesPerLevel[level], level);

            const int scaledPatchSize = PATCH_SIZE * mvScaleFactor[level];

            // Add border to coordinates and scale information
            const int nkps = keypoints.size();
            for (int i = 0; i < nkps; i++)
            {
                keypoints[i].pt.x += minBorderX;
                keypoints[i].pt.y += minBorderY;
                keypoints[i].octave = level;
                keypoints[i].size = scaledPatchSize;
            }
        }

        // compute orientations
        for (int level = 0; level < Param::nlevels_; ++level)
            computeOrientation(mvImagePyramid[level], allKeypoints[level], umax);
    }

    static void computeDescriptors(const Mat &image, vector<KeyPoint> &keypoints, Mat &descriptors, const vector<Point> &pattern)
    {
        descriptors = Mat::zeros((int)keypoints.size(), 32, CV_8UC1);

        for (size_t i = 0; i < keypoints.size(); i++)
            computeOrbDescriptor(keypoints[i], image, &pattern[0], descriptors.ptr((int)i));
    }

    void ORBextractor::extractORB(cv::Mat _image, vector<KeyPoint> &_keypoints, cv::Mat &_descriptors)
    {
        if (_image.empty())
            return;

        ComputePyramid(_image);

        // 计算每层图像的兴趣点
        vector<vector<KeyPoint>> allKeypoints;
        ComputeKeyPointsOctTree(allKeypoints);

        //Mat descriptors;

        int nkeypoints = 0;
        for (int level = 0; level < Param::nlevels_; ++level)
            nkeypoints += (int)allKeypoints[level].size();
        if (nkeypoints == 0)
            _descriptors.release();
        else
        {
            _descriptors.create(nkeypoints, 32, CV_8U);
            //descriptors = _descriptors.getMat();
        }

        _keypoints.clear();
        _keypoints.reserve(nkeypoints);

        int offset = 0;
        for (int level = 0; level < Param::nlevels_; ++level)
        {
            vector<KeyPoint> &keypoints = allKeypoints[level];
            int nkeypointsLevel = (int)keypoints.size();

            if (nkeypointsLevel == 0)
                continue;

            // preprocess the resized image 对图像进行高斯模糊
            Mat workingMat = mvImagePyramid[level].clone();
            GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);

            // Compute the descriptors 计算描述子
            Mat desc = _descriptors.rowRange(offset, offset + nkeypointsLevel);
            computeDescriptors(workingMat, keypoints, desc, pattern);
            _descriptors.rowRange(offset, offset + nkeypointsLevel) = desc;

            offset += nkeypointsLevel;

            // Scale keypoint coordinates
            if (level != 0)
            {
                float scale = mvScaleFactor[level]; //getScale(level, firstLevel, scaleFactor);
                for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
                                                keypointEnd = keypoints.end();
                     keypoint != keypointEnd; ++keypoint)
                    keypoint->pt *= scale;
            }
            // And add the keypoints to the output
            _keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());
        }
    }

    /*
void ORBextractor::calcDescriptors(const vector<KeyPoint> &_keypoints,
                                 cv::Mat &_descriptors)
{
    int nkeypoints = _keypoints.size();
    vector<cv::KeyPoint> allkeypoints[nlevels];
    for (int i = 0; i < nlevels; i++)
    {
        allkeypoints[i].reserve(nkeypoints);
    }
    for (int n = 0; n < nkeypoints; n++)
    {
        allkeypoints[_keypoints[n].octave].push_back(_keypoints[n]);
    }
    _descriptors.create(nkeypoints, 32, CV_8U);
    int offset = 0;
    for (int level = 0; level < nlevels; ++level)
    {
        vector<KeyPoint> &keypoints = allkeypoints[level];
        keypoints.reserve(nkeypoints);
        int nkeypointsLevel = keypoints.size();

        if (nkeypointsLevel == 0)
            continue;

        // preprocess the resized image 对图像进行高斯模糊
        Mat workingMat = mvImagePyramid[level].clone();
        GaussianBlur(workingMat, workingMat, Size(7, 7), 2, 2, BORDER_REFLECT_101);
        Mat desc = _descriptors.rowRange(offset, offset + nkeypointsLevel);
        computeDescriptors(workingMat, keypoints, desc, pattern);
        desc.copyTo(_descriptors.rowRange(offset, offset + nkeypointsLevel));
        offset += nkeypointsLevel;
    }
}
*/

    void ORBextractor::ComputePyramid(cv::Mat image)
    {
        for (int level = 0; level < Param::nlevels_; ++level)
        {
            float scale = mvInvScaleFactor[level];
            Size sz(cvRound((float)image.cols * scale), cvRound((float)image.rows * scale));
            Size wholeSize(sz.width + EDGE_THRESHOLD * 2, sz.height + EDGE_THRESHOLD * 2);
            Mat temp(wholeSize, image.type()), masktemp;
            mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

            // Compute the resized image
            if (level != 0)
            {
                resize(mvImagePyramid[level - 1], mvImagePyramid[level], sz, 0, 0, cv::INTER_LINEAR);

                copyMakeBorder(mvImagePyramid[level], temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                               BORDER_REFLECT_101 + BORDER_ISOLATED);
            }
            else
            {
                copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD, EDGE_THRESHOLD,
                               BORDER_REFLECT_101);
            }
        }
    }
    void matchLeftRight(const vector<cv::KeyPoint> &keypoints_l_, const vector<cv::KeyPoint> &keypoints_r_, vector<float> &left_to_right_,
                        const Mat &descriptors_l_, const Mat &descriptors_r_, ORBextractor::Ptr ORBextractorLeft, ORBextractor::Ptr ORBextractorRight)
    {

        int N = keypoints_l_.size();
        left_to_right_ = vector<float>(N, 0.0f);

        const int thOrbDist = (ORBextractor::TH_HIGH + ORBextractor::TH_LOW) / 2;

        const int nRows = ORBextractorLeft->mvImagePyramid[0].rows;

        vector<vector<size_t>> vRowIndices(nRows, vector<size_t>());

        for (int i = 0; i < nRows; i++)
            vRowIndices[i].reserve(200);

        const int Nr = keypoints_r_.size();

        for (int iR = 0; iR < Nr; iR++)
        {

            const cv::KeyPoint &kp = keypoints_r_[iR];
            const float &kpY = kp.pt.y;
            vector<float> vScaleFactors = ORBextractorLeft->mvScaleFactor;
            const float r = 2.0f * vScaleFactors[keypoints_r_[iR].octave];

            const int maxr = ceil(kpY + r);
            const int minr = floor(kpY - r);

            for (int yi = minr; yi <= maxr; yi++)
                vRowIndices[yi].push_back(iR);
        }

        // Set limits for search
        const float minZ = Camera::base_fx_ / Camera::fx_;
        const float minD = 0;
        const float maxD = Camera::fx_; // TODO：这个参数存疑

        // For each left keypoint search a match in the right image
        vector<pair<int, int>> vDistIdx;
        vDistIdx.reserve(N);

        for (int iL = 0; iL < N; iL++)
        {
            const cv::KeyPoint &kpL = keypoints_l_[iL];
            const int &levelL = kpL.octave;
            const float &vL = kpL.pt.y;
            const float &uL = kpL.pt.x;
            vector<float> invScaleFactors = ORBextractorLeft->mvInvScaleFactor;
            vector<float> vScaleFactors = ORBextractorLeft->mvScaleFactor;
            const vector<size_t> &vCandidates = vRowIndices[vL];

            if (vCandidates.empty())
                continue;

            const float minU = uL - maxD;
            const float maxU = uL - minD;

            if (maxU < 0)
                continue;

            // 粗匹配
            int bestDist = ORBextractor::TH_HIGH;
            size_t bestIdxR = 0;
            const cv::Mat &dL = descriptors_l_.row(iL);
            for (size_t iC = 0; iC < vCandidates.size(); iC++)
            {
                const size_t iR = vCandidates[iC];
                const cv::KeyPoint &kpR = keypoints_r_[iR];

                if (kpR.octave < levelL - 1 || kpR.octave > levelL + 1)
                    continue;

                const float &uR = kpR.pt.x;
                if (uR >= minU && uR <= maxU)
                {
                    const cv::Mat &dR = descriptors_r_.row(iR);
                    const int dist = ORBextractor::DescriptorDistance(dL, dR);

                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        bestIdxR = iR;
                    }
                }
            }

            // 匹配结果精细化
            if (bestDist < thOrbDist)
            {
                const float uR0 = keypoints_r_[bestIdxR].pt.x;

                const float invscaleFactor = invScaleFactors[kpL.octave];
                const float scaleduL = round(kpL.pt.x * invscaleFactor);
                const float scaledvL = round(kpL.pt.y * invscaleFactor);
                const float scaleduR0 = round(uR0 * invscaleFactor);

                int w = 5;
                int itcols = ORBextractorLeft->mvImagePyramid[kpL.octave].cols;
                int itrows = ORBextractorLeft->mvImagePyramid[kpL.octave].rows;
                if (scaleduL + w + 1 > itcols)
                    w = itcols - scaleduL - 1;
                if (scaledvL + w + 1 > itrows)
                    w = itrows - scaleduL - 1;
                if (w <= 1)
                    continue;
                if (w > 5)
                    continue;

                // 块匹配
                cv::Mat IL = ORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduL - w, scaleduL + w + 1);
                IL.convertTo(IL, CV_32F);
                IL = IL - IL.at<float>(w, w) * cv::Mat::ones(IL.rows, IL.cols, CV_32F);
                int bestDist = INT_MAX;
                int bestincR = 0;
                const int L = w;
                vector<float> vDists;
                vDists.resize(2 * L + 1);
                const float iniu = scaleduR0 - L - w;
                const float endu = scaleduR0 + L + w + 1;
                if (iniu < 0 || endu >= ORBextractorRight->mvImagePyramid[kpL.octave].cols)
                    continue;

                for (int incR = -L; incR <= +L; incR++)
                {

                    cv::Mat IR = ORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL - w, scaledvL + w + 1).colRange(scaleduR0 + incR - w, scaleduR0 + incR + w + 1);
                    IR.convertTo(IR, CV_32F);
                    IR = IR - IR.at<float>(w, w) * cv::Mat::ones(IR.rows, IR.cols, CV_32F);
                    float dist = cv::norm(IL, IR, cv::NORM_L1);
                    if (dist < bestDist)
                    {
                        bestDist = dist;
                        bestincR = incR;
                    }

                    vDists[L + incR] = dist;
                }

                // 二次曲线拟合得到亚像素匹配
                float deltaR;
                if (bestincR != -L & bestincR != L)
                {
                    const float dist1 = vDists[L + bestincR - 1];
                    const float dist2 = vDists[L + bestincR];
                    const float dist3 = vDists[L + bestincR + 1];
                    deltaR = (dist1 - dist3) / (2.0f * (dist1 + dist3 - 2.0f * dist2));
                }
                else
                    continue;

                if (deltaR < -1 || deltaR > 1)
                    continue;

                float bestuR = vScaleFactors[kpL.octave] * ((float)scaleduR0 + (float)bestincR + deltaR);

                float disparity = uL - bestuR;

                if (disparity >= minD && disparity < maxD)
                {
                    if (disparity <= 0)
                        bestuR = uL - 0.01;
                    left_to_right_[iL] = bestuR;
                    vDistIdx.push_back(pair<int, int>(bestDist, iL));
                }
            }
        }
        sort(vDistIdx.begin(), vDistIdx.end());
        const float median = vDistIdx[vDistIdx.size() / 2].first;
        const float thDist = 1.5f * 1.4f * median;

        for (int i = vDistIdx.size() - 1; i >= 0; i--)
        {
            if (vDistIdx[i].first < thDist)
                break;
            else
            {
                left_to_right_[vDistIdx[i].second] = 0;
            }
        }
    }
    void ORBextractor::AssignfeatoGrid(vector<cv::KeyPoint> &keypoints_l_, ORBextractor::Ptr ORBextractorLeft)
    {
        int CN = keypoints_l_.size();
        int nReserve = 0.5f * CN / (FRAME_GRID_COLS * FRAME_GRID_ROWS);
        for (unsigned int i = 0; i < FRAME_GRID_COLS; i++)
            for (unsigned int j = 0; j < FRAME_GRID_ROWS; j++)
                ORBextractorLeft->mGrid[i][j].reserve(nReserve);
        // 在mGrid中记录了各特征点
        for (int i = 0; i < CN; i++)
        {
            const cv::KeyPoint &kp = keypoints_l_[i];
            int nGridPosX, nGridPosY;
            if (ORBextractor::PosInGrid(kp, nGridPosX, nGridPosY))
                ORBextractorLeft->mGrid[nGridPosX][nGridPosY].push_back(i);
        }
    }
    bool ORBextractor::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
    {
        mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / (Camera::max_X_ - Camera::min_X_);
        mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / (Camera::max_Y_ - Camera::min_Y_);
        posX = round((kp.pt.x - Camera::min_X_) * mfGridElementWidthInv);
        posY = round((kp.pt.y - Camera::min_Y_) * mfGridElementHeightInv);

        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if (posX < 0 || posX >= FRAME_GRID_COLS || posY < 0 || posY >= FRAME_GRID_ROWS)
            return false;

        return true;
    }
    vector<size_t> ORBextractor::GetFeaturesInArea(const vector<cv::KeyPoint> &keypoints_l_, const float &x, const float &y, const float &r, const int minLevel, const int maxLevel)
    {

        vector<size_t> vIndices;
        int N = keypoints_l_.size();
        vIndices.reserve(N);
        const int nMinCellX = max(0, (int)floor((x - Camera::min_X_ - r) * mfGridElementWidthInv));
        if (nMinCellX >= FRAME_GRID_COLS)
            return vIndices;
        const int nMaxCellX = min((int)FRAME_GRID_COLS - 1, (int)ceil((x - Camera::min_X_ + r) * mfGridElementWidthInv));
        if (nMaxCellX < 0)
            return vIndices;

        const int nMinCellY = max(0, (int)floor((y - Camera::min_Y_ - r) * mfGridElementHeightInv));
        if (nMinCellY >= FRAME_GRID_ROWS)
            return vIndices;

        const int nMaxCellY = min((int)FRAME_GRID_ROWS - 1, (int)ceil((y - Camera::min_Y_ + r) * mfGridElementHeightInv));
        if (nMaxCellY < 0)
            return vIndices;

        const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
        {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
            {
                const vector<size_t> vCell = mGrid[ix][iy];
                if (vCell.empty())
                    continue;

                for (size_t j = 0, jend = vCell.size(); j < jend; j++)
                {
                    const cv::KeyPoint &kpUn = keypoints_l_[vCell[j]];
                    if (bCheckLevels)
                    {
                        if (kpUn.octave < minLevel)
                            continue;
                        if (maxLevel >= 0)
                            if (kpUn.octave > maxLevel)
                                continue;
                    }

                    const float distx = kpUn.pt.x - x;
                    const float disty = kpUn.pt.y - y;

                    if (fabs(distx) < r && fabs(disty) < r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }
    // vector<size_t> ORBextractor::GetFeaturesInArea(const ORBextractor::Ptr ORBextractorLeft, const vector<cv::KeyPoint> &keypoints_l_, const float &x, const float &y, const float &r, const int minLevel, const int maxLevel)
    // {

    //     vector<size_t> vIndices;
    //     int N = keypoints_l_.size();
    //     vIndices.reserve(N);
    //     const int nMinCellX = max(0, (int)floor((x - Camera::min_X_ - r) * mfGridElementWidthInv));
    //     if (nMinCellX >= FRAME_GRID_COLS)
    //         return vIndices;
    //     const int nMaxCellX = min((int)FRAME_GRID_COLS - 1, (int)ceil((x - Camera::min_X_ + r) * mfGridElementWidthInv));
    //     if (nMaxCellX < 0)
    //         return vIndices;

    //     const int nMinCellY = max(0, (int)floor((y - Camera::min_Y_ - r) * mfGridElementHeightInv));
    //     if (nMinCellY >= FRAME_GRID_ROWS)
    //         return vIndices;

    //     const int nMaxCellY = min((int)FRAME_GRID_ROWS - 1, (int)ceil((y - Camera::min_Y_ + r) * mfGridElementHeightInv));
    //     if (nMaxCellY < 0)
    //         return vIndices;

    //     const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

    //     for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
    //     {
    //         for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
    //         {
    //             const vector<size_t> vCell = ORBextractorLeft->mGrid[ix][iy];
    //             if (vCell.empty())
    //                 continue;

    //             for (size_t j = 0, jend = vCell.size(); j < jend; j++)
    //             {
    //                 const cv::KeyPoint &kpUn = keypoints_l_[vCell[j]];
    //                 if (bCheckLevels)
    //                 {
    //                     if (kpUn.octave < minLevel)
    //                         continue;
    //                     if (maxLevel >= 0)
    //                         if (kpUn.octave > maxLevel)
    //                             continue;
    //                 }

    //                 const float distx = kpUn.pt.x - x;
    //                 const float disty = kpUn.pt.y - y;

    //                 if (fabs(distx) < r && fabs(disty) < r)
    //                     vIndices.push_back(vCell[j]);
    //             }
    //         }
    //     }

    //     return vIndices;
    // }

    // 取出直方图中值最大的三个index
    void ORBextractor::ComputeThreeMaxima(vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3)
    {
        int max1 = 0;
        int max2 = 0;
        int max3 = 0;

        for (int i = 0; i < L; i++)
        {
            const int s = histo[i].size();
            if (s > max1)
            {
                max3 = max2;
                max2 = max1;
                max1 = s;
                ind3 = ind2;
                ind2 = ind1;
                ind1 = i;
            }
            else if (s > max2)
            {
                max3 = max2;
                max2 = s;
                ind3 = ind2;
                ind2 = i;
            }
            else if (s > max3)
            {
                max3 = s;
                ind3 = i;
            }
        }

        if (max2 < 0.1f * (float)max1)
        {
            ind2 = -1;
            ind3 = -1;
        }
        else if (max3 < 0.1f * (float)max1)
        {
            ind3 = -1;
        }
    }
    /**
 * @brief     二进制向量之间 相似度匹配距离
 * @param  a  二进制向量
 * @param  b  二进制向量
 * @return
 */
    int ORBextractor::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
    {
        const int *pa = a.ptr<int32_t>();
        const int *pb = b.ptr<int32_t>();

        int dist = 0;

        for (int i = 0; i < 8; i++, pa++, pb++)
        {
            unsigned int v = *pa ^ *pb;

            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }

        return dist;
    }

    void ORBextractor::initStaticParam()
    {
        // 自适应调整特征点提取数量
        nfeatures = Camera::width_ * Camera::height_ / 400.0;

        mvScaleFactor.resize(Param::nlevels_);
        mvLevelSigma2.resize(Param::nlevels_);
        mvScaleFactor[0] = 1.0f;
        mvLevelSigma2[0] = 1.0f;
        for (int i = 1; i < Param::nlevels_; i++)
        {
            mvScaleFactor[i] = mvScaleFactor[i - 1] * Param::scale_;
            mvLevelSigma2[i] = mvScaleFactor[i] * mvScaleFactor[i];
        }

        mvInvScaleFactor.resize(Param::nlevels_);
        mvInvLevelSigma2.resize(Param::nlevels_);
        for (int i = 0; i < Param::nlevels_; i++)
        {
            mvInvScaleFactor[i] = 1.0f / mvScaleFactor[i];
            mvInvLevelSigma2[i] = 1.0f / mvLevelSigma2[i];
        }

        mnFeaturesPerLevel.resize(Param::nlevels_);
        float factor = 1.0f / Param::scale_;
        float nDesiredFeaturesPerScale = nfeatures * (1 - factor) / (1 - (float)pow((double)factor, (double)Param::nlevels_));

        int sumFeatures = 0;
        for (int level = 0; level < Param::nlevels_ - 1; level++)
        {
            mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale);
            sumFeatures += mnFeaturesPerLevel[level];
            nDesiredFeaturesPerScale *= factor;
        }
        mnFeaturesPerLevel[Param::nlevels_ - 1] = std::max(nfeatures - sumFeatures, 0);

        umax.resize(HALF_PATCH_SIZE + 1);

        int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1);
        int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2);
        const double hp2 = HALF_PATCH_SIZE * HALF_PATCH_SIZE;
        for (v = 0; v <= vmax; ++v)
            umax[v] = cvRound(sqrt(hp2 - v * v));

        // Make sure we are symmetric
        for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v)
        {
            while (umax[v0] == umax[v0 + 1])
                ++v0;
            umax[v] = v0;
            ++v0;
        }
    }
} // namespace myslam
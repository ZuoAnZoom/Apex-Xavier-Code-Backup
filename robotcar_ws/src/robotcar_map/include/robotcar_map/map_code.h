#ifndef MAP_CODE_H
#define MAP_CODE_H

/*** 读取 ***/
// 地图话题输入(仅含三元占用值)，用以解译话题输入
#define FREE_TOPIC               0
#define OBSTACLE_TOPIC          100
#define NO_INFO_TOPIC           -1
// 地图文件输入(RGB三通道)，用以解译文件输入
#define FREE_R                  255
#define FREE_G                  255
#define FREE_B                  255
#define OBSTACLE_R               0
#define OBSTACLE_G               0
#define OBSTACLE_B               0
#define LANE_R                   0
#define LANE_G                  180
#define LANE_B                   0
#define LANE_G_RANGE            10      // (0, 180~190, 0)可表示不同车道线

/*** 存储 ***/
// 程序维护的地图信息
#define NO_INFO                 255
#define OBSTACLE                254
#define INFLATED_OBSTACLE       253
#define INFLATED_RANGE          100     // (253-153)可表示障碍物膨胀
#define FREE                     0
#define LANE                    20
#define LANE_RANGE              10      // (20~30)可表示不同车道线

/** 可视化 **/
#define LIGHT_GREEN             -1
#define WHITE                    0
#define BLACK                   100     // 0-100: GRAY
#define GREEN                   110
#define RED                     140
#define ORANGE                  200
#define YELLOW                  230
#define LIGHT_YELLOW            250

#endif /* MAP_CODE_H */
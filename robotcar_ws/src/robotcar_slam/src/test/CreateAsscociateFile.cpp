#include <iostream>
#include <fstream>
#include <iomanip>
#include <string.h>
using namespace std;

/*
这个脚本用于从Kiiti数据集中的times.txt文档生成相应的associate.txt
*/
int main()
{
    /*kitti*/
    fstream file_read("/home/lihua/Document/kitti_dataset/02/times.txt", ios::in);     // 修改路径
    fstream file_write("/home/lihua/Document/kitti_dataset/02/全程/associate.txt", ios::out); // 修改路径
    string fline;
    int num = 0;
    while (!file_read.eof())
    {
        getline(file_read, fline);

        file_write
            << fline << " image_0/" << setw(6) << setfill('0') << num << ".png"
            << " image_1/" << setw(6) << setfill('0') << num << ".png" << endl;
        fline.clear();
        num++;
    }

    /*EuRoc*/
    // fstream cam_read("/home/lihua/Document/data/euroc_MH_01_easy/cam0/data.csv", ios::in);    // 修改路径
    // fstream file_write("/home/lihua/Document/data/euroc_MH_01_easy/associate.txt", ios::out); // 修改路径
    // string fline;
    // getline(cam_read, fline);
    // while (!cam_read.eof())
    // {
    //     getline(cam_read, fline);
    //     if (fline.empty())
    //         break;
    //     fline = fline.substr(0, 19);
    //     file_write << fline << " cam0/data/" << fline << ".png"
    //                << " cam1/data/" << fline << ".png" << endl;
    //     fline.clear();
    // }
    // ifstream file("/home/lihua/Document/data/euroc_MH_01_easy/associate_all.txt");
    // ofstream file2("/home/lihua/Document/data/euroc_MH_01_easy/time_stamp.txt");
    // string curr_fline;
    // // 去除头部
    // getline(file, curr_fline);
    // while (curr_fline[0] == '#')
    // {
    //     getline(file, curr_fline);
    // }
    // cout << curr_fline << endl;
    // while (!file.eof())
    // {
    //     string time_stamp = curr_fline.substr(0, 20);
    //     cout << time_stamp << endl;
    //     file2 << time_stamp << endl;
    //     getline(file, curr_fline);
    // }

    // /*TUM*/
    // fstream file_read("/home/lihua/Document/TUM_dataset/associations/fr1_desk.txt", ios::in);
    // fstream file_write("/home/lihua/Document/TUM_dataset/rgbd_dataset_freiburg1_desk/association/associate.txt", ios::out);
    // char fline[100];
    // while (!file_read.eof())
    // {
    //     file_read.getline(fline, 100);
    //     // for (int i = 0; i < 100; i++)
    //     //     cout << fline[i];
    //     // cout << endl;
    //     char *p;
    //     p = strtok(fline, " ");
    //     int num = 1;
    //     while (p)
    //     {
    //         // cout << p << endl;
    //         if (num != 3)
    //         {
    //             file_write <<" "<< p;
    //         }
    //         p = strtok(NULL, " ");
    //         num++;
    //     }
    //     file_write << endl;
    // }

    return 0;
}

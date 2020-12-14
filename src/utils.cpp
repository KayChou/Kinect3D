#include "utils.h"

void saveKRT(Context *context)
{
    FILE *f = fopen("cameras.config", "w");
    fprintf(f, "numKinects: %d\n", numKinects);
    for(int i=0; i<numKinects; i++) {
        fprintf(f, "camera %d\n", i);
        fprintf(f, "R\n\t%f %f %f\n", context->R[i][0][0], context->R[i][0][1], context->R[i][0][2]);
        fprintf(f, "\t%f %f %f\n", context->R[i][1][0], context->R[i][1][1], context->R[i][1][2]);
        fprintf(f, "\t%f %f %f\n", context->R[i][2][0], context->R[i][2][1], context->R[i][2][2]);
        fprintf(f, "T\n\t%f %f %f\n", context->T[i][0], context->T[i][1], context->T[i][2]);
    }
    fclose(f);
}


void savePacket2Bin(framePacket *input, const char* filename){
    std::ofstream binFile;
    binFile.open(filename, std::ios::out |  std::ios::trunc | std::ios::binary);

    float tmpFloat;
    char tmpChar;

    for(int i=0; i<input->width_d * input->height_d; i++){
        tmpFloat = static_cast<float>(input->data_d[i]);
        binFile.write(reinterpret_cast<const char*>(&tmpFloat), sizeof(float));
    }

    for(int i=0; i<input->width_d * input->height_d; i++){
        tmpFloat = static_cast<float>(input->vertices[i].X);
        binFile.write(reinterpret_cast<const char*>(&tmpFloat), sizeof(float));
        tmpFloat = static_cast<float>(input->vertices[i].Y);
        binFile.write(reinterpret_cast<const char*>(&tmpFloat), sizeof(float));
        tmpFloat = static_cast<float>(input->vertices[i].Z);
        binFile.write(reinterpret_cast<const char*>(&tmpFloat), sizeof(float));
        tmpChar = static_cast<char>(input->vertices[i].R);
        binFile.write(reinterpret_cast<const char*>(&tmpChar),sizeof(char));
        tmpChar = static_cast<char>(input->vertices[i].G);
        binFile.write(reinterpret_cast<const char*>(&tmpChar),sizeof(char));
        tmpChar = static_cast<char>(input->vertices[i].B);
        binFile.write(reinterpret_cast<const char*>(&tmpChar),sizeof(char));
    }

    binFile.close();
}


bool saveRGBDFIFO2Image(FIFO<framePacket> **input, int numOfKinects){
    cv::Mat color, depth;
    
    int *cnt = new int[numOfKinects];
    for(int i=0; i<numOfKinects; i++){
        cnt[i] = 0;
    }

    for(int i=0; i<numOfKinects; i++){
        while(cnt[i]++ < framemax){
            framePacket *packet = input[i]->get();
            if( packet == NULL ) {
                break;
            }
            cv::Mat(packet->height_c, packet->width_c, CV_8UC4, packet->data_c).copyTo(color);
            cv::Mat(packet->height_d, packet->width_d, CV_32FC1, packet->data_d).copyTo(depth);

            cv::Mat point(packet->height_d, packet->width_d, CV_32FC3);
            cv::Mat RGB(packet->height_d, packet->width_d, CV_8UC3);

            for(int i=0; i<packet->height_d; i++){
                for(int j=0; j<packet->width_d; j++){
                    //std::printf("%d %d\n", i, j);
                    point.at<cv::Vec3f>(i, j)[0] = packet->vertices[i*packet->width_d + j].X;
                    point.at<cv::Vec3f>(i, j)[1] = packet->vertices[i*packet->width_d + j].Y;
                    point.at<cv::Vec3f>(i, j)[2] = packet->vertices[i*packet->width_d + j].Z;
                    // std::printf("i: %d , j: %d | %f %f %f\n", i, j, point.at<cv::Vec3f>(i, j)[0],
                    //  point.at<cv::Vec3f>(i, j)[1], point.at<cv::Vec3f>(i, j)[2]);
                    RGB.at<cv::Vec3b>(i, j)[0] = packet->vertices[i*packet->width_d + j].B;
                    RGB.at<cv::Vec3b>(i, j)[1] = packet->vertices[i*packet->width_d + j].G;
                    RGB.at<cv::Vec3b>(i, j)[2] = packet->vertices[i*packet->width_d + j].R;
                }
            }
            //cv::normalize(point_save, point_save, 0, 1.0, cv::NORM_MINMAX);

            cv::imwrite("./datas/color" + std::to_string(i) + std::to_string(cnt[i]) + ".png", color);
            cv::imwrite("./datas/depth" + std::to_string(i) + std::to_string(cnt[i]) + ".png", depth);
            cv::imwrite("./datas/point" + std::to_string(i) + std::to_string(cnt[i]) + ".tif", point);
            //cv::ImwriteFlags;
            cv::imwrite("./datas/rgb" + std::to_string(i) + std::to_string(cnt[i]) + ".png", RGB);

            packet->destroy();
        }
    }
    return true;
}


bool saveRGBDFIFO2PLY(FIFO<framePacket> **input, int numOfKinects)
{
    int *cnt = new int[numOfKinects];
    for(int i=0; i<numOfKinects; i++){
        cnt[i] = 0;
    }

    std::vector<Point3f> vertices;
	std::vector<RGB> colors;
	RGB tempColor;
    Point3f temp;

    for(int i=0; i<numOfKinects; i++){
        while(cnt[i]++ < framemax){
            framePacket *packet = input[i]->get();
            if( packet == NULL ) {
                break;
            }

            cv::Mat point(packet->height_d, packet->width_d, CV_32FC3);
            cv::Mat RGB(packet->height_d, packet->width_d, CV_8UC3);

            for(int i=0; i<packet->height_d; i++){
                for(int j=0; j<packet->width_d; j++){
                    //std::printf("%d %d\n", i, j);
                    temp.X = packet->vertices[i*packet->width_d + j].X;
                    temp.Y = packet->vertices[i*packet->width_d + j].Y;
                    temp.Z = packet->vertices[i*packet->width_d + j].Z;
                    // std::printf("i: %d , j: %d | %f %f %f\n", i, j, point.at<cv::Vec3f>(i, j)[0],
                    //  point.at<cv::Vec3f>(i, j)[1], point.at<cv::Vec3f>(i, j)[2]);
                    tempColor.B = packet->vertices[i*packet->width_d + j].B;
                    tempColor.G = packet->vertices[i*packet->width_d + j].G;
                    tempColor.R = packet->vertices[i*packet->width_d + j].R;

                    vertices.push_back(temp);
                    colors.push_back(tempColor);
                }
            }
            std::string filename = "./datas/pointcloud_" + std::to_string(i) + "_" + std::to_string(cnt[i]) + ".ply";
            savePlyFile(filename.c_str(), vertices, false, colors);
            vertices.clear();
            colors.clear();
            packet->destroy();
        }
    }
    return true;
}
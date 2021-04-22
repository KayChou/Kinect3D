//   Copyright (C) 2015  Marek Kowalski (M.Kowalski@ire.pw.edu.pl), Jacek Naruniec (J.Naruniec@ire.pw.edu.pl)
//   License: MIT Software License   See LICENSE.txt for the full license.

//   If you use this software in your research, then please use the following citation:

//    Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 3D Data
//    Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 2015 International Conference on, Lyon, France, 2015

//    @INPROCEEDINGS{Kowalski15,
//        author={Kowalski, M. and Naruniec, J. and Daniluk, M.},
//        booktitle={3D Vision (3DV), 2015 International Conference on},
//        title={LiveScan3D: A Fast and Inexpensive 3D Data Acquisition System for Multiple Kinect v2 Sensors},
//        year={2015},
//    }
#include "ICP.h"


void FindClosestPointForEach(PointCloud &sourceCloud, cv::Mat &destPoints, vector<float> &distances, vector<size_t> &indices)
{
    int nVerts2 = destPoints.rows;
    typedef nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<float, PointCloud>, PointCloud, 3> kdTree;
    kdTree tree(3, sourceCloud);
    tree.buildIndex();

#pragma omp parallel for
    for (int i = 0; i < nVerts2; i++) {
        nanoflann::KNNResultSet<float> resultSet(1);
        resultSet.init(&indices[i], &distances[i]);
        tree.findNeighbors(resultSet, (float*)destPoints.row(i).data, nanoflann::SearchParams());
    }
}


float GetStandardDeviation(vector<float> &data)
{
    float mean = 0;
    for (size_t i = 0; i < data.size(); i++) {
        mean += data[i];
    }
    mean /= data.size();

    float std = 0;

    for (size_t i = 0; i < data.size(); i++) {
        std += pow(data[i] - mean, 2);
    }

    std /= data.size();
    std = sqrt(std);

    return std;
}


void RejectOutlierMatches(vector<Point3f> &matches1, vector<Point3f> &matches2, vector<float> &matchDistances, float maxStdDev)
{
    float distanceStandardDev = GetStandardDeviation(matchDistances);

    vector<Point3f> filteredMatches1;
    vector<Point3f> filteredMatches2;
    for (size_t i = 0; i < matches1.size(); i++){
        if (matchDistances[i] > maxStdDev * distanceStandardDev)
            continue;

        filteredMatches1.push_back(matches1[i]);
        filteredMatches2.push_back(matches2[i]);
    }

    matches1 = filteredMatches1;
    matches2 = filteredMatches2;
}


void RejectOutlierMatches(vector<Point3fRGB> &matches1, vector<Point3fRGB> &matches2, vector<float> &matchDistances, float maxStdDev)
{
    float distanceStandardDev = GetStandardDeviation(matchDistances);

    vector<Point3fRGB> filteredMatches1;
    vector<Point3fRGB> filteredMatches2;
    for (size_t i = 0; i < matches1.size(); i++){
        if (matchDistances[i] > maxStdDev * distanceStandardDev)
            continue;

        filteredMatches1.push_back(matches1[i]);
        filteredMatches2.push_back(matches2[i]);
    }

    matches1 = filteredMatches1;
    matches2 = filteredMatches2;
}


float ICP_p2p(Point3f *verts1, Point3f *verts2, int nVerts1, int nVerts2, float *R, float *t, int maxIter)
{
    PointCloud cloud1;
    cloud1.pts = vector<Point3f>(verts1, verts1 + nVerts1);

    cv::Mat matR(3, 3, CV_32F, R);
    cv::Mat matT(1, 3, CV_32F, t);

    cv::Mat verts2Mat(nVerts2, 3, CV_32F, (float*)verts2);

    float error = 1;

    for (int iter = 0; iter < maxIter; iter++) {
        vector<Point3f> matched1, matched2;

        vector<float> distances(nVerts2);
        vector<size_t> indices(nVerts2);
        FindClosestPointForEach(cloud1, verts2Mat, distances, indices);

        vector<float> matchDistances;
        vector<int> matchIdxs(nVerts1, -1);
        for (int i = 0; i < nVerts2; i++) {
            int pos = matchIdxs[indices[i]];

            if (pos != -1) {
                if (matchDistances[pos] < distances[i])
                    continue;
            }

            Point3f temp;
            temp.X = verts2Mat.at<float>(i, 0);
            temp.Y = verts2Mat.at<float>(i, 1);
            temp.Z = verts2Mat.at<float>(i, 2);

            if (pos == -1) {
                matched1.push_back(verts1[indices[i]]);
                matched2.push_back(temp);

                matchDistances.push_back(distances[i]);

                matchIdxs[indices[i]] = matched1.size() - 1;
            }
            else {
                matched2[pos] = temp;
                matchDistances[pos] = distances[i];
            }
        }

        RejectOutlierMatches(matched1, matched2, matchDistances, 2.5);

        //error = 0;
        //for (int i = 0; i < matchDistances.size(); i++)
        //{
        //    error += sqrt(matchDistances[i]);
        //}
        //error /= matchDistances.size();
        //cout << error << endl;
        cv::Mat matched1MatCv(matched1.size(), 3, CV_32F, matched1.data());
        cv::Mat matched2MatCv(matched2.size(), 3, CV_32F, matched2.data());
        cv::Mat tempT;
        cv::reduce(matched1MatCv - matched2MatCv, tempT, 0, CV_REDUCE_AVG);

        for (int i = 0; i < verts2Mat.rows; i++) {
            verts2Mat.row(i) += tempT;
        }
        for (int i = 0; i < matched2MatCv.rows; i++) {
            matched2MatCv.row(i) += tempT;
        }

        cv::Mat M = matched2MatCv.t() * matched1MatCv;
        cv::SVD svd;
        svd(M);
        cv::Mat tempR = svd.u * svd.vt;

        double det = cv::determinant(tempR);
        
        if (det < 0) {
            cv::Mat temp = cv::Mat::eye(3, 3, CV_32F);
            temp.at<float>(2, 2) = -1;
            tempR = svd.u * temp * svd.vt;
        }
        

        verts2Mat = verts2Mat * tempR;

        matT += tempT * matR.t();
        matR = matR * tempR;
    }

    memcpy(verts2, verts2Mat.data, verts2Mat.rows * sizeof(float) * 3);

    memcpy(R, matR.data, 9 * sizeof(float));
    memcpy(t, matT.data, 3 * sizeof(float));

    return error;
}



void get_matched_points(Point3fRGB *verts1_RGB, Point3fRGB *verts2_RGB) 
{
    std::printf("try to get matched points\n");
    Point3f *verts1 = new Point3f[Width_depth_HR * Height_depth_HR];
    Point3f *verts2 = new Point3f[Width_depth_HR * Height_depth_HR];
    Point3fRGB *verts1_rgb = new Point3fRGB[Width_depth_HR * Height_depth_HR];
    Point3fRGB *verts2_rgb = new Point3fRGB[Width_depth_HR * Height_depth_HR];
    int nVerts1 = 0;
    int nVerts2 = 0;

    float l, a, b, X, Y, Z;

    for(int k=0; k<Width_depth_HR * Height_depth_HR; k++) {
        if( verts1_RGB[k].X != 0 && verts1_RGB[k].Y != 0 && verts1_RGB[k].Z != 0 && 
            verts1_RGB[k].X > x_bbox_min && verts1_RGB[k].X < x_bbox_max && 
            verts1_RGB[k].Y > y_bbox_min && verts1_RGB[k].Y < y_bbox_max && 
            verts1_RGB[k].Z > z_bbox_min && verts1_RGB[k].Z < z_bbox_max) {
            verts1[nVerts1].X = verts1_RGB[k].X;
            verts1[nVerts1].Y = verts1_RGB[k].Y;
            verts1[nVerts1].Z = verts1_RGB[k].Z;
            verts1_rgb[nVerts1].X = verts1_RGB[k].X;
            verts1_rgb[nVerts1].Y = verts1_RGB[k].Y;
            verts1_rgb[nVerts1].Z = verts1_RGB[k].Z;
            verts1_rgb[nVerts1].R = verts1_RGB[k].R;
            verts1_rgb[nVerts1].G = verts1_RGB[k].G;
            verts1_rgb[nVerts1].B = verts1_RGB[k].B;
            nVerts1++;
        }
        if( verts2_RGB[k].X != 0 && verts2_RGB[k].Y != 0 && verts2_RGB[k].Z != 0 && 
            verts2_RGB[k].X > x_bbox_min && verts2_RGB[k].X < x_bbox_max && 
            verts2_RGB[k].Y > y_bbox_min && verts2_RGB[k].Y < y_bbox_max && 
            verts2_RGB[k].Z > z_bbox_min && verts2_RGB[k].Z < z_bbox_max) {
            verts2[nVerts2].X = verts2_RGB[k].X;
            verts2[nVerts2].Y = verts2_RGB[k].Y;
            verts2[nVerts2].Z = verts2_RGB[k].Z;
            verts2_rgb[nVerts2].X = verts2_RGB[k].X;
            verts2_rgb[nVerts2].Y = verts2_RGB[k].Y;
            verts2_rgb[nVerts2].Z = verts2_RGB[k].Z;
            verts2_rgb[nVerts2].R = verts2_RGB[k].R;
            verts2_rgb[nVerts2].G = verts2_RGB[k].G;
            verts2_rgb[nVerts2].B = verts2_RGB[k].B;
            nVerts2++;
        }
    }

    PointCloud cloud1;
    cloud1.pts = vector<Point3f>(verts1, verts1 + nVerts1);

    cv::Mat verts2Mat(nVerts2, 3, CV_32F, (float*)verts2);

    float error = 1;

    vector<Point3f> matched1, matched2;
    vector<Point3fRGB> matched1_rgb, matched2_rgb;
    vector<Point3f> matched1_lab, matched2_lab;

    vector<float> distances(nVerts2);
    vector<size_t> indices(nVerts2);
    FindClosestPointForEach(cloud1, verts2Mat, distances, indices);

    vector<float> matchDistances;
    vector<int> matchIdxs(nVerts1, -1);
    for (int i = 0; i < nVerts2; i++) {
        int pos = matchIdxs[indices[i]];

        if (pos != -1) {
            if (matchDistances[pos] < distances[i])
                continue;
        }

        Point3f temp;
        Point3f temp_lab;
        temp.X = verts2Mat.at<float>(i, 0);
        temp.Y = verts2Mat.at<float>(i, 1);
        temp.Z = verts2Mat.at<float>(i, 2);

        if (pos == -1) {
            matched1.push_back(verts1[indices[i]]);
            matched2.push_back(temp);
            matched1_rgb.push_back(verts1_rgb[indices[i]]);
            matched2_rgb.push_back(verts2_rgb[i]);

            RGB2Lab(verts1_rgb[indices[i]].R, verts1_rgb[indices[i]].G, verts1_rgb[indices[i]].B, &temp_lab.X, &temp_lab.Y, &temp_lab.Z);
            matched1_lab.push_back(temp_lab);

            RGB2Lab(verts2_rgb[indices[i]].R, verts2_rgb[indices[i]].G, verts2_rgb[indices[i]].B, &temp_lab.X, &temp_lab.Y, &temp_lab.Z);
            matched2_lab.push_back(temp_lab);

            matchDistances.push_back(distances[i]);

            matchIdxs[indices[i]] = matched1.size() - 1;
        }
        else {
            matched2[pos] = temp;
            matchDistances[pos] = distances[i];
        }
    }

    RejectOutlierMatches(matched1_rgb, matched2_rgb, matchDistances, 0.002);

    std::printf("save to local file\n");
    FILE *f = fopen("matched_points_rgb.csv", "w");
    FILE *f0 = fopen("matched_points.csv", "w");
    FILE *f_lab = fopen("matched_points_lab.csv", "w");
    for(int i = 0; i < matched1_rgb.size(); i++) {
        fprintf(f0, "%0.5f,%0.5f,%0.5f,%u,%u,%u,%0.5f,%0.5f,%0.5f,%u,%u,%u\n", 
                matched1_rgb[i].X, matched1_rgb[i].Y, matched1_rgb[i].Z, matched1_rgb[i].R, matched1_rgb[i].G, matched1_rgb[i].B,
                matched2_rgb[i].X, matched2_rgb[i].Y, matched2_rgb[i].Z, matched2_rgb[i].R, matched2_rgb[i].G, matched2_rgb[i].B);
        if(matched2_rgb[i].R !=0 && matched2_rgb[i].G != 0 && matched2_rgb[i].B != 0 && matched1_rgb[i].R !=0 && matched1_rgb[i].G != 0 && matched1_rgb[i].B != 0) {
            fprintf(f, "%u,%u,%u,%u,%u,%u\n", 
                   matched1_rgb[i].R, matched1_rgb[i].G, matched1_rgb[i].B, 
                   matched2_rgb[i].R, matched2_rgb[i].G, matched2_rgb[i].B);
        }

        if(matched1_lab[i].X > 0 && matched1_lab[i].X <100 && matched2_lab[i].X > 0 && matched2_lab[i].X <100 &&
           matched1_lab[i].Y > -128 && matched1_lab[i].Y <128 && matched2_lab[i].Y > -128 && matched2_lab[i].Y <128 &&
           matched1_lab[i].Z > -128 && matched1_lab[i].Z <128 && matched2_lab[i].Z > -128 && matched2_lab[i].Z <128 ) {
            fprintf(f_lab, "%f,%f,%f,%f,%f,%f\n", matched1_lab[i].X, matched1_lab[i].Y, matched1_lab[i].Z, matched2_lab[i].X, matched2_lab[i].Y, matched2_lab[i].Z);
        }
    }
    fclose(f);
    fclose(f0);
    fclose(f_lab);

    delete [] verts1;
    delete [] verts2;
    delete [] verts1_rgb;
    delete [] verts2_rgb;
    
    std::printf("save finish\n");

}


void ICP::init(Context *ctx) {
    this->ctx = ctx;
}


void ICP::loop() {
    bool data_ready = false;
    int nVerts1 = 0, nVerts2 = 0;
    Point3f *verts1 = new Point3f[ctx->depth_w * ctx->depth_h];
    Point3f *verts2 = new Point3f[ctx->depth_w * ctx->depth_h * (numKinects - 1)];
    Point3fRGB *vertices;

    float *R = new float[9];
    float *T = new float[3];

    float *tempR = new float[9];
    float *tempT = new float[3];

    while(true) {
        // if button "refine" not pressed 
        if(!ctx->b_Refine) {
            usleep(1000);
            continue;
        }
        // if button "refine" is pressed but data not ready
        data_ready = true;
        for(int i=0; i<numKinects; i++) {
            if(!ctx->b_refined_data_ready[i]) {
                data_ready = false;
            }
        }
        if(!data_ready) {
            usleep(1000);
            continue;
        }
        
        for(int i=0; i<numKinects; i++) {
            nVerts1 = 0;
            nVerts2 = 0;

            for(int i = 0; i < 9; i++) {
                R[i] = 0;
            }
            for (int i = 0; i < 3; i++) {
                T[i] = 0;
                R[i + i * 3] = 1;
            }

            // get current camera's data
            vertices = ctx->frame_to_be_refined[i].vertices;
            for(int k=0; k<ctx->depth_w * ctx->depth_h; k++) {
                if( vertices[k].X != 0 && vertices[k].Y != 0 && vertices[k].Z != 0 && 
                    vertices[k].X > x_bbox_min && vertices[k].X < x_bbox_max && 
                    vertices[k].Y > y_bbox_min && vertices[k].Y < y_bbox_max && 
                    vertices[k].Z > z_bbox_min && vertices[k].Z < z_bbox_max)
                {
                    verts1[nVerts1].X = vertices[k].X;
                    verts1[nVerts1].Y = vertices[k].Y;
                    verts1[nVerts1].Z = vertices[k].Z;
                    nVerts1++;
                }
            }
            // get other camera's data
            for(int j=0; j<numKinects; j++) {
                if(j != i) {
                    vertices = ctx->frame_to_be_refined[j].vertices;
                    for(int k=0; k<ctx->depth_w * ctx->depth_h; k++) {
                        if( vertices[k].X != 0 && vertices[k].Y != 0 && vertices[k].Z != 0 && 
                            vertices[k].X > x_bbox_min && vertices[k].X < x_bbox_max && 
                            vertices[k].Y > y_bbox_min && vertices[k].Y < y_bbox_max && 
                            vertices[k].Z > z_bbox_min && vertices[k].Z < z_bbox_max)
                        {
                            verts2[nVerts2].X = vertices[k].X;
                            verts2[nVerts2].Y = vertices[k].Y;
                            verts2[nVerts2].Z = vertices[k].Z;
                            nVerts2++;
                        }
                    }
                }
            }
            // savePlyFile("verts1.ply", verts1, nVerts1);
            // savePlyFile("verts2.ply", verts2, nVerts2);
            get_matched_points(ctx->frame_to_be_refined[0].vertices, ctx->frame_to_be_refined[1].vertices);
            ICP_p2p(verts1, verts2, nVerts1, nVerts2, R, T, 5);
            
            tempT[0] = ctx->R[i][0][0] * T[0] + ctx->R[i][1][0] * T[1] + ctx->R[i][2][0] * T[2];
            tempT[1] = ctx->R[i][0][1] * T[0] + ctx->R[i][1][1] * T[1] + ctx->R[i][2][1] * T[2];
            tempT[2] = ctx->R[i][0][2] * T[0] + ctx->R[i][1][2] * T[1] + ctx->R[i][2][2] * T[2];
            ctx->T[i][0] -= tempT[0];
            ctx->T[i][1] -= tempT[1];
            ctx->T[i][2] -= tempT[2];

            tempR[0] = ctx->R[i][0][0] * R[0] + ctx->R[i][1][0] * R[1] + ctx->R[i][2][0] * R[2];
            tempR[1] = ctx->R[i][0][0] * R[3] + ctx->R[i][1][0] * R[4] + ctx->R[i][2][0] * R[5];
            tempR[2] = ctx->R[i][0][0] * R[6] + ctx->R[i][1][0] * R[7] + ctx->R[i][2][0] * R[8];
            tempR[3] = ctx->R[i][0][1] * R[0] + ctx->R[i][1][1] * R[1] + ctx->R[i][2][1] * R[2];
            tempR[4] = ctx->R[i][0][1] * R[3] + ctx->R[i][1][1] * R[4] + ctx->R[i][2][1] * R[5];
            tempR[5] = ctx->R[i][0][1] * R[6] + ctx->R[i][1][1] * R[7] + ctx->R[i][2][1] * R[8];
            tempR[6] = ctx->R[i][0][2] * R[0] + ctx->R[i][1][2] * R[1] + ctx->R[i][2][2] * R[2];
            tempR[7] = ctx->R[i][0][2] * R[3] + ctx->R[i][1][2] * R[4] + ctx->R[i][2][2] * R[5];
            tempR[8] = ctx->R[i][0][2] * R[6] + ctx->R[i][1][2] * R[7] + ctx->R[i][2][2] * R[8];
            ctx->R[i][0][0] = tempR[0];
            ctx->R[i][1][0] = tempR[1];
            ctx->R[i][2][0] = tempR[2];
            ctx->R[i][0][1] = tempR[3];
            ctx->R[i][1][1] = tempR[4];
            ctx->R[i][2][1] = tempR[5];
            ctx->R[i][0][2] = tempR[6];
            ctx->R[i][1][2] = tempR[7];
            ctx->R[i][2][2] = tempR[8];
        }
        
        ctx->b_Refine = false;
        for(int i=0; i<numKinects; i++) {
            ctx->b_refined_data_ready[i] = false;
        }   
    }
}
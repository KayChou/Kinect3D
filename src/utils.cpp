#include "utils.h"


void load_params_ir_color(char *filename, ColorCameraParams &color_param, IrCameraParams &ir_param) {
    FILE *f = fopen(filename, "r");
    if(f == NULL) {
        printf("Failed to read params: %s", filename);
        return;
    }
    else {
        printf("Load param: %s\n", filename);
    }

    char str[100];
    
    fscanf(f, "%s %s", str, str);
    fscanf(f, "%s %s %s %s", str, str, str, str);
    fscanf(f, "%s %s %s %s %s", str, str, str, str, str);

    fscanf(f, "%f %f %f %f", &ir_param.fx, &ir_param.fy, &ir_param.cx, &ir_param.cy);
    fscanf(f, "%f %f %f %f %f", &ir_param.k1, 
                                &ir_param.k2, 
                                &ir_param.k3, 
                                &ir_param.p1, 
                                &ir_param.p2);

    // printf("%f %f %f %f\n", ir_param.fx, ir_param.fy, ir_param.cx, ir_param.cy);
    fscanf(f, "%s %s", str, str);
    fscanf(f, "%s %s %s %s %s", str, str, str, str, str);
    fscanf(f, "%f %f %f %f", &color_param.fx, &color_param.fy, &color_param.cx, &color_param.cy);

    fscanf(f, "%s %s %s", str, str, str);
    fscanf(f, "%s %s", str, str);

    fscanf(f, "%s %s %s %s %s %s %s %s %s %s %s", str, str, str, str, str, str, str, str, str, str, str);
    fscanf(f, "%s %s %s %s %s %s %s %s %s %s %s", str, str, str, str, str, str, str, str, str, str, str);
    
    fscanf(f, "%f %f", &color_param.shift_d, &color_param.shift_m);
    fscanf(f, "%f %f %f %f %f %f %f %f %f %f", 
                                &color_param.mx_x3y0,
                                &color_param.mx_x0y3,
                                &color_param.mx_x2y1,
                                &color_param.mx_x1y2,
                                &color_param.mx_x2y0,
                                &color_param.mx_x0y2,
                                &color_param.mx_x1y1,
                                &color_param.mx_x1y0,
                                &color_param.mx_x0y1,
                                &color_param.mx_x0y0);
    fscanf(f, "%f %f %f %f %f %f %f %f %f %f", 
                                &color_param.my_x3y0,
                                &color_param.my_x0y3,
                                &color_param.my_x2y1,
                                &color_param.my_x1y2,
                                &color_param.my_x2y0,
                                &color_param.my_x0y2,
                                &color_param.my_x1y1,
                                &color_param.my_x1y0,
                                &color_param.my_x0y1,
                                &color_param.my_x0y0);
}
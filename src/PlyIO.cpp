#include "PlyIO.h"

template <class T>
void bigLittleEndianSwap (T * v, unsigned int numOfElements) {
    char * tmp = (char*)v;
    for (unsigned int j = 0; j < numOfElements; j++){
        unsigned int offset = 4*j;
        char c = tmp[offset];
        tmp[offset] =  tmp[offset+3];
        tmp[offset+3] = c;
        c = tmp[offset+1];
        tmp[offset+1] = tmp[offset+2];
        tmp[offset+2] = c;
    }
}


//==========================================================================
// parase plyFile header
//==========================================================================
bool parasePlyHeader(std::ifstream &in, const char* filename, unsigned int &vn, unsigned int &fn, 
                   PLYFormat &format, unsigned int &numVertProperties, bool &hasColor, int &headerSize){
    vn = 0;
    fn = 0;
    numVertProperties = 0;
    headerSize = 0;
    hasColor = false;
    std::string current, currentelement;

    in >> current;
    if (current != "ply"){
        std::cerr << "(PLY) not a PLY file" << std::endl;
        return 0;
    }

    in >> current;
    int lid = 0;
    while (current != "end_header") {
        if (current == "format") {
            in >> current;
            if (current == "binary_big_endian") {
                in >> current;
                if (current == "1.0")
                    format = BINARY_BIG_ENDIAN_1;
                else{
                    std::cerr << "(PLY) error parsing header - bad binary big endian version" << std::endl;
                    return 0;
                }
            } 
            else if (current == "binary_little_endian") {
                in >> current;
                if (current == "1.0")
                    format = BINARY_LITTLE_ENDIAN_1;
                else{
                    std::cerr << "(PLY) error parsing header - bad binary little endian version" << std::endl;
                    return 0;
                }
            } 
            else if (current == "ascii") {
                in >> current;
                if (current == "1.0")
                    format = ASCII_1;
                else{
                    std::cerr << "(PLY) error parsing header - bad ascii version" << std::endl;
                    return 0;
                }
            } 
            else {
                std::cerr << "(PLY) error parsing header (format)" << std::endl;
                return 0;
            }
        } 
        else if (current == "element") {
            in >> current;
            if (current == "vertex"){
                currentelement = current;
                in >> vn;
            }
            else if (current == "face"){
                currentelement = current;
                in >> fn;
            }
            else{
                std::cerr << "(PLY) ignoring unknown element " << current << std::endl;
                currentelement = "";
            }
        } 
        else if (currentelement != "" && current == "property") {
            in >> current;
            if (current == "float" || current == "double") {
                numVertProperties++;
                in >> current;
            }
            else if (current == "uchar") { // color
                numVertProperties++;
                hasColor = true;
                in >> current;
            }
            else if (current == "list") {
                in >> current;
                in >> current;
                in >> current;
            } 
            else {
                std::cerr << "(PLY) error parsing header (property)" << std::endl;
                return 0;
            }
        } 
        else if ( (current == "comment") || (current.find("obj_info") != std::string::npos) ) {
            char comment[MAX_COMMENT_SIZE];
            in.getline (comment, MAX_COMMENT_SIZE);
        } 
        else {}
        in >> current;
        lid++;
    }
    headerSize = in.tellg();
    return true;
}


//==========================================================================
// read plyFile: parase header and read datas
//==========================================================================
bool readPlyFile(const char* filename, std::vector<Point3f> &verts, std::vector<RGB> &colors){
    
    unsigned int vn=0, fn=0, numVertProperties=0;
    int headerSize=0;
    PLYFormat format;
    bool hasColor = false;
    std::string current, currentelement;

    // open file
    std::ifstream in(filename);
    if (!in){
        std::cerr << "(PLY) error opening file" << std::endl;
        return false;
    }

    //parase plyFile header
    std::printf("parase header...\n");
    if(parasePlyHeader(in, filename, vn, fn, format, numVertProperties, hasColor, headerSize)){
        std::printf("\tverts: %d \n\tfaces: %d \n\tProperties: %d\n", vn, fn, numVertProperties);
    }
    else{
        std::cerr << "Parase header failed\n";
        return false;
    }
    in.close();
    
    // start to read data
    verts.clear();
    colors.clear();
    Point3f tempV, tempNormal;
    RGB tempColor;
    float curvature;
    int cnt = 0;

    if(format == BINARY_BIG_ENDIAN_1){
        FILE * in = fopen (filename, "r");
        if (!in){
            std::cerr << "(PLY) error opening file" << std::endl;
            return false;
        }
        char c;
        for(int i=0; i<=headerSize; i++){ fread(&c, 1, 1, in); } // skip the header

        int R, G, B, A;
        if(numVertProperties == 10){ // x, y, z, R, G, B, nx, ny, nz, curvature
            for(unsigned int i=0; i<vn; i++){
                fread(&tempV, sizeof(float), 3, in);
                fread(&tempNormal, sizeof(float), 3, in);
                fread(&tempColor, sizeof(unsigned char), 3, in);
                fread(&A, sizeof(unsigned char), 1, in);

                bigLittleEndianSwap(&tempV, 3);
                verts.push_back(tempV);
                colors.push_back(tempColor);
            }
            std::printf("\nreading data...\n");
            std::printf("\tverts: %ld \n\tcolors: %ld\n", verts.size(), colors.size());
        }
        fclose(in);
    }

    else if(format == BINARY_LITTLE_ENDIAN_1){
        FILE *in = fopen (filename, "r");
        if (!in){
            std::cerr << "(PLY) error opening file" << std::endl;
            return false;
        }
        char c;
        for(int i=0; i<=headerSize; i++){ fread(&c, 1, 1, in); } // skip the header

        int R, G, B, A;
        if(numVertProperties == 10){ // x, y, z, nx, ny, nz, R, G, B, A
            for(unsigned int i=0; i<vn; i++){
                fread(&tempV, sizeof(float), 3, in);
                fread(&tempNormal, sizeof(float), 3, in);
                fread(&tempColor, sizeof(unsigned char), 3, in);
                fread(&A, sizeof(unsigned char), 1, in);

                verts.push_back(tempV);
                colors.push_back(tempColor);
            }
            std::printf("\nreading data...\n");
            std::printf("\tverts: %ld \n\tcolors: %ld\n", verts.size(), colors.size());
        }
        
        if(numVertProperties == 7 && hasColor){ // x, y, z, R, G, B, A
            for(unsigned int i=0; i<vn; i++){
                fread(&tempV, sizeof(float), 3, in);
                fread(&tempColor, sizeof(unsigned char), 3, in);
                fread(&A, sizeof(unsigned char), 1, in);

                verts.push_back(tempV);
                colors.push_back(tempColor);
            }
            std::printf("\nreading data...\n");
            std::printf("\tverts: %ld \n\tcolors: %ld\n", verts.size(), colors.size());
        }

        if(numVertProperties == 3){ // x, y, z
            for(unsigned int i=0; i<vn; i++){
                fread(&tempV, sizeof(float), 3, in);
                verts.push_back(tempV);
            }
            std::printf("\nreading data...\n");
            std::printf("\tverts: %ld \n\tcolors: %ld\n", verts.size(), colors.size());
        }
        fclose(in);
    }

    else if(format == ASCII_1){
        FILE * in = fopen(filename, "r");
        if (!in){
            std::cerr << "(PLY) error opening file" << std::endl;
            return false;
        }
        char c;
        for(int i=0; i<headerSize; i++){ fread(&c, 1, 1, in); } // skip the header

        int R, G, B, A;
        if(numVertProperties == 10){ // x, y, z, R, G, B, nx, ny, nz, curvature
            for(unsigned int i=0; i<vn; i++){ 
                fscanf(in, "%f %f %f", &tempV.X, &tempV.Y, &tempV.Z);
                fscanf(in, "%i %i %i", &R, &G, &B);
                fscanf(in, "%f %f %f %f", &tempNormal.X, &tempNormal.Y, &tempNormal.Z, &curvature);
                tempColor.R = R;
                tempColor.G = G;
                tempColor.B = B;
                verts.push_back(tempV);
                colors.push_back(tempColor);
            }
            std::printf("\nreading data...\n");
            std::printf("\tverts: %ld \n\tcolors: %ld\n", verts.size(), colors.size());
        }
        else if(numVertProperties == 7){ // x, y, z, R, G, B, A
            for(unsigned int i=0; i<vn; i++){
                fscanf(in, "%f %f %f", &tempV.X, &tempV.Y, &tempV.Z);
                fscanf(in, "%i %i %i %i", &R, &G, &B, &A);
                tempColor.R = R;
                tempColor.G = G;
                tempColor.B = B;
                verts.push_back(tempV);
                colors.push_back(tempColor);
            }   
            std::printf("\nreading data...\n");
            std::printf("\tverts: %ld \n\tcolors: %ld\n", verts.size(), colors.size());
            return true;
        }

        else if(numVertProperties == 6 && hasColor){ // x, y, z, R, G, B
            for(unsigned int i=0; i<vn; i++){
                fscanf(in, "%f %f %f", &tempV.X, &tempV.Y, &tempV.Z);
                fscanf(in, "%i %i %i", &R, &G, &B);
                tempColor.R = R;
                tempColor.G = G;
                tempColor.B = B;
                verts.push_back(tempV);
                colors.push_back(tempColor);
            }   
            std::printf("\nreading data...\n");
            std::printf("\tverts: %ld \n\tcolors: %ld\n", verts.size(), colors.size());
            return true;
        }

        
        else if(numVertProperties == 6){ // x, y, z, nx, ny, nz
            for(unsigned int i=0; i<vn; i++){
                fscanf(in, "%f %f %f", &tempV.X, &tempV.Y, &tempV.Z);
                fscanf(in, "%f %f %f", &tempNormal.X, &tempNormal.Y, &tempNormal.Z);
                verts.push_back(tempV);
            }   
            std::printf("\nreading data...\n");
            std::printf("\tverts: %ld \n\tcolors: %ld\n", verts.size(), colors.size());
            return true;
        }
        else if(numVertProperties == 3){ // x, y, z
            for(unsigned int i=0; i<vn; i++){
                fscanf(in, "%f %f %f", &tempV.X, &tempV.Y, &tempV.Z);
                verts.push_back(tempV);
            }   
            std::printf("\nreading data...\n");
            std::printf("\tverts: %ld \n\tcolors: %ld\n", verts.size(), colors.size());
            return true;
        }
        fclose(in);
    }
    else{
        std::cerr << "(PLY) Unknowed format\n" << std::endl;
        return false;
    }

    return true;
}


//==========================================================================
// read plyFile: parase header and read datas
//==========================================================================
void savePlyFile(const char* filename, std::vector<Point3f> vertices, bool BINARY, std::vector<RGB> colors) {
    bool hasColor = (colors.size() == vertices.size()) ? true : false;

    unsigned int nVertices = vertices.size();
    if(BINARY){
        std::ofstream plyFile;
        plyFile.open(filename, std::ios::out |  std::ios::trunc | std::ios::binary);
        //write header
        plyFile << "ply" << std::endl;
        plyFile << "format binary_little_endian 1.0" << std::endl;
        plyFile << "element vertex " << vertices.size() << std::endl;
        plyFile << "property float x" << std::endl;
        plyFile << "property float y" << std::endl;
        plyFile << "property float z" << std::endl;
        if(hasColor){
            plyFile << "property uchar red" << std::endl;
            plyFile << "property uchar green" << std::endl;
            plyFile << "property uchar blue" << std::endl;
        }
        plyFile << "end_header" << std::endl;

        float tmpFloat;
        char tmpChar;
        
        for(int i=0; i<vertices.size(); i++){
            tmpFloat = static_cast<float>(vertices[i].X);
            plyFile.write(reinterpret_cast<const char*>(&tmpFloat), sizeof(float));
            tmpFloat = static_cast<float>(vertices[i].Y);
            plyFile.write(reinterpret_cast<const char*>(&tmpFloat), sizeof(float));
            tmpFloat = static_cast<float>(vertices[i].Z);
            plyFile.write(reinterpret_cast<const char*>(&tmpFloat), sizeof(float));
            if(hasColor){
                tmpChar = static_cast<char>( colors[i].R );
                plyFile.write(reinterpret_cast<const char*>(&tmpChar),sizeof(char));
                tmpChar = static_cast<char>( colors[i].G );
                plyFile.write(reinterpret_cast<const char*>(&tmpChar),sizeof(char));
                tmpChar = static_cast<char>( colors[i].B );
                plyFile.write(reinterpret_cast<const char*>(&tmpChar),sizeof(char));
            }
        }
        plyFile.close();
    }
    else{
        FILE *f = fopen(filename, "w");
        fprintf(f, "ply\nformat ascii 1.0\n");
        fprintf(f, "element vertex %d\n", nVertices);
        fprintf(f, "property float x\nproperty float y\nproperty float z\n");
        if(hasColor){
            fprintf(f, "property uchar red\nproperty uchar green\nproperty uchar blue\n");;
        }
        fprintf(f, "end_header\n");
        for (int i = 0; i < nVertices; i++) {
            if(hasColor){
                fprintf(f, "%f %f %f %d %d %d\n", vertices[i].X, vertices[i].Y, vertices[i].Z, colors[i].R, colors[i].G, colors[i].B);
            }
            else{
                fprintf(f, "%f %f %f\n", vertices[i].X, vertices[i].Y, vertices[i].Z);
            }       
        }
        fclose(f);
    }
}


void savePlyFile(const char* filename, float* vertices, int* faces, int Vn, int Fn){ 
    std::printf(filename); fflush(stdout);
    FILE *f = fopen(filename, "w");
    fprintf(f, "ply\nformat ascii 1.0\n");
    fprintf(f, "element vertex %d\n", Vn);
    fprintf(f, "property float x\nproperty float y\nproperty float z\n");
    fprintf(f, "property uchar red\nproperty uchar green\nproperty uchar blue\n");
    fprintf(f, "element face %d\n", Fn);
    fprintf(f, "property list uchar int vertex_indices\n");

    fprintf(f, "end_header\n");
    for (int i = 0; i < Vn; i++) {
        fprintf(f, "%f %f %f %d %d %d\n", vertices[6*i], vertices[6*i+1], vertices[6*i+2], 
                                        (int)(255.0f*vertices[6*i+3]), (int)(255.0f*vertices[6*i+4]), (int)(255.0f*vertices[6*i+5]));
    }
    for(int i=0; i<Fn; i++){
        fprintf(f, "3 %d %d %d\n", faces[3*i], faces[3*i+1], faces[3*i+2]);
    }
    fclose(f);
}


void savePlyFile(const char* filename, Point3f* vertices, int Vn) {
    std::printf(filename); fflush(stdout);
    FILE *f = fopen(filename, "w");
    fprintf(f, "ply\nformat ascii 1.0\n");
    fprintf(f, "element vertex %d\n", Vn);
    fprintf(f, "property float x\nproperty float y\nproperty float z\n");

    fprintf(f, "end_header\n");

    for (int i = 0; i < Vn; i++) {
        fprintf(f, "%f %f %f\n", vertices[i].X, vertices[i].Y, vertices[i].Z);
    }
    fclose(f);
}
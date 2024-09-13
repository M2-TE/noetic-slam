#pragma once

int parseLine(char* line) {
    // This assumes that a digit will be found and the line ends in " Kb".
    int i = strlen(line);
    const char* p = line;
    while (*p <'0' || *p > '9') p++;
    line[i-3] = '\0';
    i = atoi(p);
    return i;
}
int read_phys_mem_kb() {
    FILE* file = fopen("/proc/self/status", "r");
    int result = -1;
    char line[128];
    while (fgets(line, 128, file) != NULL){
        if (strncmp(line, "VmRSS:", 6) == 0){
            result = parseLine(line);
            break;
        }
    }
    fclose(file);
    return result;
}

struct Point {
    Point(): data{0.f, 0.f, 0.f, 1.f} {}

    PCL_ADD_POINT4D;
    float intensity; // intensity
    union {
        std::uint32_t t; // time since beginning of scan in nanoseconds
        float time; // time since beginning of scan in seconds
        double timestamp; // absolute timestamp in seconds
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}
EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (float, time, time)
    (double, timestamp, timestamp))

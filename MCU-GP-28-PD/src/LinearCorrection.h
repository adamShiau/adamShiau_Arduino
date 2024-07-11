#ifndef LINEAR_CORRECTION_H
#define LINEAR_CORRECTION_H

class LinearCorrection {
private:
    int window_size;
    int count;
    float sum[3];
    float bias[3];

public:
    LinearCorrection(const int &ws);
    void update(float (&data)[3]);
    void reset();
};

#endif // LINEAR_CORRECTION_H

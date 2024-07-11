#include "LinearCorrection.h"

LinearCorrection::LinearCorrection(const int &ws) : window_size(ws) , count(0){
    for(int i=0;i<3;i++){
        sum[i] = 0;
        bias[i] = 0;
    }
    
}

void LinearCorrection::update(float (&data)[3]) {
    if (count < window_size) {
        for(int i=0;i<3;i++){
            sum[i] += data[i];
            data[i] = 0;
        }
        count++;
    }
    
    else if (count == window_size && bias[0] == 0) {
        float n = static_cast<float>(window_size);
        for(int i=0;i<3;i++){
            bias[i] = sum[i] / n;
        }
    }
    if (bias[0] != 0){
        for(int i=0; i<3; i++){
        data[i] -= bias[i];
        }
    }
    
    
}

void LinearCorrection::reset() {
    for(int i=0;i<3;i++){
        sum[i] = 0;
        bias[i] = 0;
    }
    count = 0;
}



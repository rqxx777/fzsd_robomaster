#ifndef RM_CPP_AVGFILTER_HPP
#define RM_CPP_AVGFILTER_HPP
#include <vector>

// 滑动平均滤波器: 用于对弹丸初速度进行平滑滤波
class avgFilter {
private:
    std::vector<double> buffer;  // 数据缓冲区
    double avg=0;                // 当前平均值
public:
    size_t Size;                 // 窗口大小 (需在使用前设置)

    // 更新滤波器，输入新的数据点
    void update(double data) {
        if(buffer.size()==Size){
            // 缓冲区满: 移除最老的数据点，使用增量更新平均值
            buffer.erase(buffer.begin());
            buffer.emplace_back(data);
            avg = avg - buffer[0]/Size;
            avg = avg + buffer[Size-1]/Size;
        }
        else{
            // 缓冲区未满: 直接追加，使用累计平均值公式
            buffer.emplace_back(data);
            avg = avg*(int)(buffer.size()-1)/(int)buffer.size()
                + buffer[buffer.size()-1]/(int)buffer.size();
        }
    }

    // 获取当前滑动平均值
    void get_avg(double &data) const {
        data = avg;
    }
};

#endif //RM_CPP_AVGFILTER_HPP
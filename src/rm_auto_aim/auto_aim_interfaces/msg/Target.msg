std_msgs/Header header
# 跟踪目标完整状态: 位置/速度/角度/半径/高度差
bool tracking
string id
int32 armors_num
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64 yaw
float64 v_yaw
float64 radius_1
float64 radius_2
float64 dz

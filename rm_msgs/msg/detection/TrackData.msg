std_msgs/Header header
uint8 id
bool tracking #是否需要跟随
uint8 armors_num #装甲板数量
geometry_msgs/Point position #预测的车中心的位置
float64 yaw #装甲板相对于我方机器人的朝向yaw
geometry_msgs/Vector3 velocity #预测车中心的速度
float64 v_yaw #装甲板转动的角速度
float64 radius_1 #半径1
float64 radius_2 #半径2
float64 dz #装甲板距离车中心的距离
float64 accel #加速度

# target_pos=position
# target_vel=velocity
# header不能为空
# 调用tf2官方接口把position和velocity转换到odom坐标系
# 将position减掉现在的等于error（还要考虑底盘运动的值）
# solve_success是弹道解算是否成功的标志位
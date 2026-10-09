# 单个信标的厂家坐标经安装外参转换后的结果，不重新滤波。
std_msgs/Header header
uint32 target_id
geometry_msgs/Point position
uint8 confidence
# 新无效样本也发布以立即撤销运动；断流由订阅方独立检查采集年龄。
bool valid
int8 state

# 本演示及外部观察节点使用相同 DDS 域和图像传输配置。
source "$HOME/go2_sim/setup.bash"
# 原生速度评分插件只是运行依赖，缺失时安装脚本会建立它。
if [[ -f "$HOME/go2_sim/follow_native/share/ament_index/resource_index/packages/go2_follow_mppi_critics" ]]; then
    export AMENT_PREFIX_PATH="$HOME/go2_sim/follow_native:${AMENT_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="$HOME/go2_sim/follow_native/lib:${LD_LIBRARY_PATH:-}"
fi
export ROS_DOMAIN_ID=42
export CYCLONEDDS_URI="file://$HOME/go2_sim/follow_demo/cyclonedds.xml"
# 小矩阵投影与多个 ROS 进程并行时，限制数值库线程，避免线程争抢拖慢控制回调。
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1

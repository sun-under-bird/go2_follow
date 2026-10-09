# 真实数据接入保留调用者的 DDS 域和网卡；不加载仿真 setup.bash。
source /opt/ros/humble/setup.bash
if [[ -f "$HOME/go2_sim/venv/bin/activate" ]]; then
    source "$HOME/go2_sim/venv/bin/activate"
fi
if [[ -f "$HOME/go2_sim/follow_native/share/ament_index/resource_index/packages/go2_follow_mppi_critics" ]]; then
    export AMENT_PREFIX_PATH="$HOME/go2_sim/follow_native:${AMENT_PREFIX_PATH:-}"
    export LD_LIBRARY_PATH="$HOME/go2_sim/follow_native/lib:${LD_LIBRARY_PATH:-}"
fi
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1

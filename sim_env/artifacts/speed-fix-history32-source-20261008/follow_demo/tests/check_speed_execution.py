"""隔离导航和相机，测量当前锁定步态模型的中高速线速度响应，不改变生产控制器。"""
import argparse
import sys

from follow_demo.tests import check_execution


def main():
    """复用独占资源、模型摘要与停车检查，比较直行和转弯时的命令响应。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', default='speed-chain-execution-20261006')
    args = parser.parse_args()
    # 只测用户关心的中高速区，不增加低速死区、起步或速度补偿。
    # velocity直行对照用于检查rate角速度反馈是否影响线速度响应。
    check_execution.CASES = {
        'rate-straight-0.4': ('rate', .4, 0.),
        'rate-straight-0.6': ('rate', .6, 0.),
        'rate-straight-0.8': ('rate', .8, 0.),
        'velocity-straight-0.8': ('velocity', .8, 0.),
        'rate-moving-turn-left': ('rate', .8, .3),
        'rate-moving-turn-right': ('rate', .8, -.3),
    }
    sys.argv = [sys.argv[0], '--prefix', args.prefix]
    return check_execution.main()


if __name__ == '__main__':
    raise SystemExit(main())

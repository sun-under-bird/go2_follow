"""将已经审计的实验源码和原生二进制保存成独立快照，不修改运行目录。"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def digest(path):
    """读取实际文件字节计算版本身份，不用路径或修改时间替代内容。"""
    return hashlib.sha256(path.read_bytes()).hexdigest()


def copy_verified(source, destination, expected=None):
    """核对来源与副本一致，拒绝覆盖已有的不同版本证据。"""
    value=digest(source)
    if expected is not None and value!=expected:
        raise RuntimeError(f'来源版本与验收不一致：{source}')
    if destination.exists() and digest(destination)!=value:
        raise RuntimeError(f'已有快照版本不同，须使用新目录：{destination}')
    destination.parent.mkdir(parents=True,exist_ok=True)
    shutil.copy2(source,destination)
    if digest(destination)!=value:
        raise RuntimeError(f'复制后版本不一致：{destination}')
    return value


def main():
    """从最终审计引用的哈希保存运行源码、原生库、检查入口和复现说明。"""
    parser=argparse.ArgumentParser()
    parser.add_argument('--prefix',required=True)
    parser.add_argument('--folder',required=True)
    args=parser.parse_args()
    artifacts=Path(__file__).resolve().parent
    project=artifacts.parent
    # 目录名仅允许本地证据标签，防止快照被写到项目之外。
    if not args.folder or any(c not in 'abcdefghijklmnopqrstuvwxyz0123456789-' for c in args.folder):
        raise ValueError('快照目录只能使用小写字母、数字和短横线')
    audit=json.loads((artifacts/f'{args.prefix}-final-audit.json').read_text(encoding='utf-8-sig'))
    if not audit['evidence_valid'] or not audit['cleanup']['verified']:
        raise RuntimeError('必须先完成版本及退出审计，才能冻结最终证据')
    target=artifacts/args.folder
    reference=audit['scenarios'][0]
    hashes={}
    for name,value in reference['source_sha256'].items():
        relative=Path('follow_demo')/name
        hashes[relative.as_posix()]=copy_verified(project/relative,target/relative,value)
    linux=Path(r'\\wsl.localhost\Ubuntu-22.04\home\chy\go2_sim')
    for name,value in reference['native_plugin_sha256'].items():
        relative=Path('follow_native')/name
        hashes[relative.as_posix()]=copy_verified(linux/relative,target/relative,value)
    # 原生源码与逻辑检查也保留，忽略运行生成的Python缓存。
    for directory in ('native/go2_follow_mppi_critics','follow_demo/tests'):
        for source in (project/directory).rglob('*'):
            if source.is_file() and '__pycache__' not in source.parts:
                relative=source.relative_to(project)
                hashes[relative.as_posix()]=copy_verified(source,target/relative)
    for name in ('README.md','follow_demo/README.md','native/README.md','Start-FollowDemo.ps1',
                 'Stop-FollowDemo.ps1','Check-NavigationDemo.ps1','scripts/install_follow_demo.sh'):
        hashes[name]=copy_verified(project/name,target/name)
    manifest=dict(report_prefix=args.prefix,source_file_count=len(reference['source_sha256']),
                  passed_scenes=audit['passed_scenes'],files_sha256=hashes,
                  native_architecture='Ubuntu-22.04 x86_64 ROS Humble 1.1.20',
                  note='原生二进制只用于锁定本机证据；其他架构必须按锁定源码和依赖重新构建。')
    (target/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(dict(snapshot=str(target),files=len(hashes)),ensure_ascii=False))


if __name__=='__main__':
    main()

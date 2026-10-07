# 独占启动左右看向验收，始终关闭本脚本启动的实例，不复用用户正在操作的仿真。
[CmdletBinding()]
param(
    [ValidatePattern('^[a-z0-9][a-z0-9-]{0,63}$')][string]$ReportPrefix = 'heading-rectangle'
)
$ErrorActionPreference = 'Stop'
$existing = $null
try { $existing = Invoke-RestMethod 'http://127.0.0.1:8765/api/state' -TimeoutSec 2 } catch { }
if ($null -ne $existing) { throw '已有服务运行，请先结束人工验收后再做独占角度检查。' }
& wsl.exe -d Ubuntu-22.04 -u chy --exec pgrep -af '^python -m follow_demo[.]app( |$)'
if ($LASTEXITCODE -ne 1) { throw '存在已有实例或无法检查进程，未启动角度检查。' }
$installer = & wsl.exe -d Ubuntu-22.04 -u chy --exec wslpath -a (Join-Path $PSScriptRoot 'scripts/install_follow_demo.sh')
& wsl.exe -d Ubuntu-22.04 -u chy --exec bash $installer.Trim()
if ($LASTEXITCODE -ne 0) { throw '源码安装失败。' }
$result = 1
try {
    & (Join-Path $PSScriptRoot 'Start-FollowDemo.ps1') -NoBrowser
    $command = 'source ~/go2_sim/follow_demo/setup.bash; cd ~/go2_sim; python -m follow_demo.tests.check_heading_demo --prefix ' + $ReportPrefix
    & wsl.exe -d Ubuntu-22.04 -u chy --exec bash -lc $command
    $result = $LASTEXITCODE
} finally {
    & (Join-Path $PSScriptRoot 'Stop-FollowDemo.ps1')
    Copy-Item -LiteralPath ('\\wsl.localhost\Ubuntu-22.04\home\chy\go2_sim\artifacts\'+$ReportPrefix+'.json') -Destination (Join-Path $PSScriptRoot 'artifacts') -Force
}
if ($result -ne 0) { throw '角度验收未通过，失败数据已保存，测试实例已关闭。' }
exit 0

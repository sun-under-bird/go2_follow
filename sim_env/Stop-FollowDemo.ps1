# 仅关闭本实验台并核对退出状态，不结束整台 WSL 或其他仿真任务。
[CmdletBinding()]
param()
$ErrorActionPreference = 'Stop'
$address = 'http://127.0.0.1:8765'

function Get-FollowDemoState {
    <# 读取服务身份；拒绝把其他占用同一端口的程序当作演示关闭。 #>
    try { $result = Invoke-RestMethod "$address/api/state" -TimeoutSec 2 } catch { return $null }
    if ($result.app_id -ne 'go2_follow_demo') { throw '8765 端口不是 Go2 跟随实验台，未操作。' }
    return $result
}

if ($null -ne (Get-FollowDemoState)) {
    Invoke-RestMethod "$address/api/shutdown" -Method Post -TimeoutSec 3 | Out-Null
    for ($attempt = 0; $attempt -lt 40; $attempt++) {
        Start-Sleep -Milliseconds 250
        if ($null -eq (Get-FollowDemoState)) { break }
    }
}
if ($null -ne (Get-FollowDemoState)) { throw '服务尚未退出，请查看演示错误日志。' }

# 精确匹配本启动入口的 Python 命令，只检查进程，不用宽泛的批量终止。
for ($attempt = 0; $attempt -lt 40; $attempt++) {
    $remaining = & wsl.exe -d Ubuntu-22.04 -u chy --exec pgrep -af '^python -m follow_demo[.]app( |$)'
    if ($LASTEXITCODE -eq 1) { break }
    if ($LASTEXITCODE -ne 0) { throw '无法完成 WSL 进程退出检查。' }
    Start-Sleep -Milliseconds 250
}
if ($LASTEXITCODE -eq 0) { throw "HTTP 已不可达，但仿真进程尚存：$remaining" }
# 正常退出由应用回收 Nav2；异常退出时按 PID、启动时刻和命令核对专属子进程。
& wsl.exe -d Ubuntu-22.04 -u chy --exec bash -lc 'source ~/go2_sim/follow_demo/setup.bash; cd ~/go2_sim; python -m follow_demo.mppi_runtime --stop-owned'
if ($LASTEXITCODE -ne 0) { throw 'Nav2 子进程清理未通过。' }
$nav2Remaining = & wsl.exe -d Ubuntu-22.04 -u chy --exec pgrep -af '^/opt/ros/humble/lib/nav2_(controller/controller_server|lifecycle_manager/lifecycle_manager).*__ns:=/go2_follow_mppi( |$)'
if ($LASTEXITCODE -ne 1) { throw "MPPI 子进程尚存或无法核实：$nav2Remaining" }
Write-Host 'Go2 跟随实验台已关闭：HTTP、仿真、Nav2 MPPI 子进程均已退出。'

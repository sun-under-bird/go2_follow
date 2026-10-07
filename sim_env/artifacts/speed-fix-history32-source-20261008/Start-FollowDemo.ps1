# 启动持久的 WSL 演示进程并打开浏览器，不依赖临时安装终端存活。
[CmdletBinding()]
param(
    [switch]$NoBrowser,
    # 默认采用角速度反馈；请求上限1.0不代表当前步态模型能准确跟踪；heading 保留为旧执行接口的对照。
    [ValidateSet('heading', 'velocity', 'rate')][string]$ExecutionMode = 'rate',
    # trail 只改变跟随终点语义，目标历史不能证明相机盲区可通行。
    [ValidateSet('annulus', 'trail')][string]$PlanningMode = 'trail',
    # camera 用真实 CameraInfo/TF 预测下一段包络的可观察性。
    [ValidateSet('cone', 'camera')][string]$ObservationMode = 'camera',
    # 显式仿真安装俯角，向下为正；0度保留水平对照，不能代替实测外参。
    [ValidateRange(0, 40)][double]$CameraPitchDeg = 0.0,
    # steady 仅定期唤醒 ROS 等待，不更新传感器时间或放宽停车门槛。
    [ValidateSet('native', 'steady')][string]$ExecutorWakeMode = 'steady',
    # 独立进程隔离规划的 Python/GIL 等待；thread 保留为同地图对照。
    [ValidateSet('thread', 'process')][string]$SearchExecutionMode = 'process'
)
$ErrorActionPreference = 'Stop'
$address = 'http://127.0.0.1:8765'
$existing = $null
try { $existing = Invoke-RestMethod "$address/api/state" -TimeoutSec 2 } catch { }
if ($null -ne $existing -and $existing.app_id -ne 'go2_follow_demo') { throw '8765 端口被其他服务占用。' }
if ($null -ne $existing -and $existing.execution_mode -ne $ExecutionMode) {
    throw "已有实例执行模式为 $($existing.execution_mode)，请求为 $ExecutionMode。请先自行停止该实例，再启动所需模式。"
}
if ($null -ne $existing -and $existing.planning_mode -ne $PlanningMode) {
    throw "已有实例规划模式为 $($existing.planning_mode)，请求为 $PlanningMode。请先停止该实例，再启动所需模式。"
}
if ($null -ne $existing -and $existing.observation_mode -ne $ObservationMode) {
    throw "已有实例观察模式为 $($existing.observation_mode)，请求为 $ObservationMode。请先停止该实例，再启动所需模式。"
}
if ($null -ne $existing -and ($null -eq $existing.camera_pitch_deg -or [Math]::Abs($existing.camera_pitch_deg - $CameraPitchDeg) -gt 1e-6)) {
    throw '已有实例相机俯角与请求不符或未回报身份，请先停止该实例。'
}
if ($null -ne $existing -and $existing.executor_wake_mode -ne $ExecutorWakeMode) {
    throw "已有实例执行器唤醒模式为 $($existing.executor_wake_mode)，请求为 $ExecutorWakeMode。请先停止该实例，再启动所需模式。"
}
if ($null -ne $existing -and $existing.search_execution_mode -ne $SearchExecutionMode) {
    throw "已有实例搜索运行模式为 $($existing.search_execution_mode)，请求为 $SearchExecutionMode。请先停止该实例，再启动所需模式。"
}
if ($null -eq $existing) {
    $logDirectory = Join-Path $PSScriptRoot 'logs'
    New-Item -ItemType Directory -Force -Path $logDirectory | Out-Null
    $process = Start-Process -FilePath 'wsl.exe' -ArgumentList @('-d', 'Ubuntu-22.04', '-u', 'chy',
        '--exec', '/home/chy/go2_sim/bin/go2-follow', '--execution-mode', $ExecutionMode,
        '--planning-mode', $PlanningMode, '--observation-mode', $ObservationMode,
        '--executor-wake-mode', $ExecutorWakeMode,
        '--search-execution-mode', $SearchExecutionMode,
        '--camera-pitch-deg', $CameraPitchDeg.ToString([Globalization.CultureInfo]::InvariantCulture)) -WindowStyle Hidden -PassThru `
        -RedirectStandardOutput (Join-Path $logDirectory 'follow-demo-stdout.log') `
        -RedirectStandardError (Join-Path $logDirectory 'follow-demo-stderr.log')
    $process.Id | Set-Content -LiteralPath (Join-Path $logDirectory 'follow-demo-wsl.pid')
    $ready = $false
    for ($attempt = 0; $attempt -lt 60; $attempt++) {
        Start-Sleep -Milliseconds 500
        if ($process.HasExited) { throw '演示进程已退出，请查看 logs/follow-demo-stderr.log。' }
        $state = $null
        try {
            $state = Invoke-RestMethod "$address/api/state" -TimeoutSec 1
        } catch { }
        if ($null -ne $state -and $state.app_id -eq 'go2_follow_demo') {
            if ($state.execution_mode -ne $ExecutionMode) { throw '新实例未回报请求的执行模式，请查看日志。' }
            if ($state.planning_mode -ne $PlanningMode) { throw '新实例未回报请求的规划模式，请查看日志。' }
            if ($state.observation_mode -ne $ObservationMode) { throw '新实例未回报请求的观察模式，请查看日志。' }
            if ($null -eq $state.camera_pitch_deg -or [Math]::Abs($state.camera_pitch_deg - $CameraPitchDeg) -gt 1e-6) { throw '新实例未回报请求的相机俯角，请查看日志。' }
            if ($state.executor_wake_mode -ne $ExecutorWakeMode) { throw '新实例未回报请求的执行器唤醒模式，请查看日志。' }
            if ($state.search_execution_mode -ne $SearchExecutionMode) { throw '新实例未回报请求的搜索运行模式，请查看日志。' }
            $ready = $true
            break
        }
    }
    if (-not $ready) { throw '启动尚未就绪，请查看 logs 目录；未重复启动第二个实例。' }
}
if (-not $NoBrowser) { Start-Process $address }
Write-Host "跟随演示已启动：$address"
Write-Host "执行模式：$ExecutionMode"
Write-Host "规划模式：$PlanningMode"
Write-Host "观察模式：$ObservationMode"
Write-Host "相机安装俯角：$CameraPitchDeg 度（显式仿真配置，向下为正）"
Write-Host "执行器唤醒模式：$ExecutorWakeMode"
Write-Host "搜索运行模式：$SearchExecutionMode"
Write-Host '机身矩形：长0.70 m / 宽0.32 m；前进上限0.8 m/s / 转速上限1.0 rad/s（指令）。'
Write-Host '姿态就绪后先确认起始周围 1.2 m 净空，再开始跟随并让目标沿场景路线行走。'

# 顺序验收本脚本专门启动的基础与循环场景；失败也保存结果并关闭本次实例。
[CmdletBinding()]
param(
    [ValidateSet('open', 'long_wall', 'consecutive', 'corner', 'blocked', 'square_loop', 'slalom_loop', 'wall_loop')]
    [string[]]$Scenarios = @('open', 'long_wall', 'consecutive', 'corner', 'blocked'),
    [ValidateRange(0.05, 0.80)][double]$Speed = 0.8,
    [ValidateRange(1, 20)][int]$Laps = 2,
    # 两种执行接口分别运行同一场景和门槛，不能用恒定转向基准替代闭环验收。
    [ValidateSet('heading', 'velocity', 'rate')][string]$ExecutionMode = 'rate',
    [ValidateSet('annulus', 'trail')][string]$PlanningMode = 'trail',
    [ValidateSet('cone', 'camera')][string]$ObservationMode = 'camera',
    [ValidateRange(0, 40)][double]$CameraPitchDeg = 0.0,
    [ValidateSet('native', 'steady')][string]$ExecutorWakeMode = 'steady',
    [ValidateSet('thread', 'process')][string]$SearchExecutionMode = 'process',
    # 每轮使用独立证据前缀，保留上一轮报告，名称不能包含路径或 shell 字符。
    [ValidatePattern('^[a-z0-9][a-z0-9-]{0,63}$')][string]$ReportPrefix = 'navigation-v2'
)
$ErrorActionPreference = 'Stop'

function Assert-NoExistingDemo {
    <# 不复用人工正在验收的实例，防止自动检查重置或关闭用户的仿真。 #>
    $existing = $null
    try { $existing = Invoke-RestMethod 'http://127.0.0.1:8765/api/state' -TimeoutSec 2 } catch { }
    if ($null -ne $existing) { throw '8765 端口已有服务。请先结束人工验收，再运行自动检查。' }
    & wsl.exe -d Ubuntu-22.04 -u chy --exec pgrep -af '^python -m follow_demo[.]app( |$)'
    if ($LASTEXITCODE -eq 0) { throw '发现已有跟随实验台进程，未启动检查。' }
    if ($LASTEXITCODE -ne 1) { throw '无法检查 WSL 进程，未启动检查。' }
}

Assert-NoExistingDemo
$installer = Join-Path $PSScriptRoot 'scripts\install_follow_demo.sh'
$linuxInstaller = & wsl.exe -d Ubuntu-22.04 -u chy --exec wslpath -a $installer
if ($LASTEXITCODE -ne 0) { throw '无法转换安装脚本路径。' }
& wsl.exe -d Ubuntu-22.04 -u chy --exec bash $linuxInstaller.Trim()
if ($LASTEXITCODE -ne 0) { throw '源码安装失败。' }

$durations = @{open=30; long_wall=65; consecutive=70; corner=90; blocked=60}
$results = @()
foreach ($scenario in $Scenarios) {
    try {
        & (Join-Path $PSScriptRoot 'Start-FollowDemo.ps1') -NoBrowser -ExecutionMode $ExecutionMode -PlanningMode $PlanningMode -ObservationMode $ObservationMode -ExecutorWakeMode $ExecutorWakeMode -SearchExecutionMode $SearchExecutionMode -CameraPitchDeg $CameraPitchDeg
        # Python按实际折线长度计算圈时长，速度改变后仍覆盖指定圈数，另留10秒给跟随者跨过接缝。
        if ($scenario.EndsWith('_loop')) {
            $durationCommand = 'source ~/go2_sim/follow_demo/setup.bash; cd ~/go2_sim; python -m follow_demo.scenario_route --scenario ' + $scenario + ' --laps ' + $Laps + ' --speed ' + $Speed.ToString([Globalization.CultureInfo]::InvariantCulture)
            $duration = & wsl.exe -d Ubuntu-22.04 -u chy --exec bash -lc $durationCommand
            if ($LASTEXITCODE -ne 0) { throw '无法计算闭环场景时长。' }
            $duration = [int]$duration.Trim()
        } elseif ($scenario -eq 'open') {
            # 开阔直行目标会在x=18 m触及活动边界；高速检查提前结束，避免边界停车污染跟随成绩。
            $duration = [Math]::Min($durations[$scenario], [Math]::Floor(16.0 / $Speed) - 1)
        } else { $duration = $durations[$scenario] }
        $command = 'source ~/go2_sim/follow_demo/setup.bash; cd ~/go2_sim; python -m follow_demo.tests.check_navigation_demo --scenario ' + $scenario + ' --duration ' + $duration + ' --laps ' + $Laps + ' --speed ' + $Speed.ToString([Globalization.CultureInfo]::InvariantCulture) + ' --prefix ' + $ReportPrefix + ' --execution-mode ' + $ExecutionMode + ' --planning-mode ' + $PlanningMode + ' --observation-mode ' + $ObservationMode + ' --executor-wake-mode ' + $ExecutorWakeMode + ' --search-execution-mode ' + $SearchExecutionMode
        $command += ' --camera-pitch-deg ' + $CameraPitchDeg.ToString([Globalization.CultureInfo]::InvariantCulture)
        & wsl.exe -d Ubuntu-22.04 -u chy --exec bash -lc $command
        $results += [pscustomobject]@{scenario=$scenario; execution_mode=$ExecutionMode; planning_mode=$PlanningMode; observation_mode=$ObservationMode; camera_pitch_deg=$CameraPitchDeg; executor_wake_mode=$ExecutorWakeMode; search_execution_mode=$SearchExecutionMode; exit_code=$LASTEXITCODE}
    } finally {
        # Python 检查会请求正常退出；此处再核对 HTTP 和精确匹配的仿真进程。
        & (Join-Path $PSScriptRoot 'Stop-FollowDemo.ps1')
        # 下一场启动会覆盖运行日志，关闭后立即按场景保存，便于追查动作中止等根因。
        $logDirectory = Join-Path $PSScriptRoot 'artifacts'
        New-Item -ItemType Directory -Force -Path $logDirectory | Out-Null
        foreach ($node in @('controller_server', 'lifecycle_manager')) {
            $logSource = '\\wsl.localhost\Ubuntu-22.04\home\chy\go2_sim\logs\mppi-' + $node + '.log'
            if (Test-Path -LiteralPath $logSource) {
                Copy-Item -LiteralPath $logSource -Destination (Join-Path $logDirectory ($ReportPrefix + '-' + $scenario + '-' + $node + '.log')) -Force
            }
        }
    }
    $outputDirectory = Join-Path $PSScriptRoot 'artifacts'
    New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null
    foreach ($suffix in @('.json', '-samples.json')) {
        $source = '\\wsl.localhost\Ubuntu-22.04\home\chy\go2_sim\artifacts\' + $ReportPrefix + '-' + $scenario + $suffix
        Copy-Item -LiteralPath $source -Destination $outputDirectory -Force
    }
}
$results | ConvertTo-Json | Set-Content -LiteralPath (Join-Path $PSScriptRoot ('artifacts\' + $ReportPrefix + '-suite-exit-codes.json')) -Encoding UTF8
$results | Format-Table
if (@($results | Where-Object { $_.exit_code -ne 0 }).Count) {
    throw '存在未通过场景；已保存报告并关闭测试实例。安全停车不等于绕障完成。'
}
# 停止脚本用 pgrep 的 1 表示进程不存在；不能把它误当成整个验收的失败退出码。
exit 0

# 在 Windows PowerShell 中创建或继续配置 Ubuntu 22.04；不会删除其他发行版。
[CmdletBinding()]
param(
    [string]$Distribution = 'Ubuntu-22.04',
    [string]$InstallLocation = 'D:\WSL\Ubuntu-22.04'
)
$ErrorActionPreference = 'Stop'
$logDirectory = Join-Path $PSScriptRoot 'logs'
New-Item -ItemType Directory -Force -Path $logDirectory | Out-Null

function Invoke-WslStep {
    <# 执行一个 WSL 安装步骤并保存日志；失败时立即停止，保留现场供重试。 #>
    param([string]$Name, [string[]]$WslArguments)
    & wsl.exe @WslArguments 2>&1 | Tee-Object -FilePath (Join-Path $logDirectory "$Name.log")
    if ($LASTEXITCODE -ne 0) { throw "$Name 失败，退出码 $LASTEXITCODE；请查看 logs 目录。" }
}

if (-not (Get-Command wsl.exe -ErrorAction SilentlyContinue)) {
    throw '请先以管理员身份运行 wsl --install --no-distribution，重启 Windows 后再运行本脚本。'
}
$distributions = @(& wsl.exe --list --quiet) | ForEach-Object { ($_ -replace "`0", '').Trim() }
if ($Distribution -notin $distributions) {
    Invoke-WslStep -Name 'wsl-install' -WslArguments @('--install', '--distribution', $Distribution,
        '--web-download', '--no-launch', '--location', $InstallLocation)
}

$linuxSource = (& wsl.exe -d $Distribution -u root --exec wslpath -a -u $PSScriptRoot).Trim()
if ($LASTEXITCODE -ne 0 -or -not $linuxSource.StartsWith('/')) { throw '无法转换安装脚本路径。' }
# 运行前复制脚本快照，避免安装时编辑 Windows 源文件导致 Bash 读取位置发生变化。
Invoke-WslStep -Name 'prepare-directory' -WslArguments @('-d', $Distribution, '-u', 'root',
    '--exec', 'mkdir', '-p', '/opt/go2-env-setup')
Invoke-WslStep -Name 'copy-bundle' -WslArguments @('-d', $Distribution, '-u', 'root', '--exec', 'cp', '-r',
    "$linuxSource/scripts", "$linuxSource/locks", "$linuxSource/config", "$linuxSource/follow_demo",
    "$linuxSource/native", '/opt/go2-env-setup/')
Invoke-WslStep -Name 'ubuntu-ros-bootstrap' -WslArguments @('-d', $Distribution, '-u', 'root',
    '--exec', 'bash', '/opt/go2-env-setup/scripts/bootstrap_ubuntu.sh')
Invoke-WslStep -Name 'go2-stack-install' -WslArguments @('-d', $Distribution, '-u', 'chy',
    '--exec', 'bash', '/opt/go2-env-setup/scripts/install_go2_stack.sh')
Invoke-WslStep -Name 'environment-check' -WslArguments @('-d', $Distribution, '-u', 'chy',
    '--exec', 'bash', '/home/chy/go2_sim/bin/go2-check')
Invoke-WslStep -Name 'set-default' -WslArguments @('--set-default', $Distribution)
Write-Host '配置完成。进入 WSL 后运行 source ~/go2_sim/setup.bash，再执行 go2-sim。'
Write-Host '新建用户的默认登录设置会在该发行版下次重启后生效；本脚本不会中断其他 WSL 工作。'

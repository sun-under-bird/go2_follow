"""将验收报告生成可直接双击的中文离线回放，不启动仿真或 HTTP 服务。"""
import argparse
import hashlib
import html
import json
import math
from pathlib import Path

from follow_demo.scenarios import SCENARIOS


TITLES = {name: scene['name'] for name, scene in SCENARIOS.items()}
SAMPLE_KEYS = ('t', 'x', 'y', 'yaw', 'vx', 'wz', 'target', 'distance', 'command',
               'target_speed', 'state', 'code', 'target_mode', 'depth_age', 'frames', 'path',
               'heading', 'facing', 'command_limits', 'target_route')
REPORT_KEYS = ('acceptance_schema_version', 'started_at_utc', 'requested_duration_s', 'target_speed_mps',
               'passed_basic_checks', 'stimulus_valid', 'scenario_objective_met', 'passed_fluency',
               'passed_navigation', 'execution_mode', 'expected_execution_mode', 'execution_mode_verified',
               'planning_mode', 'observation_mode', 'camera_pitch_deg', 'camera_pitch_verified',
               'executor_wake_mode', 'search_execution_mode',
               'validation_mode', 'checks', 'fluency', 'task_completion', 'final_code', 'final_distance_m',
               'steady_actual_vx', 'target_moving_mean_actual_vx', 'target_moving_robot_slow_ratio',
               'target_moving_max_distance', 'target_moving_longest_slow_seconds', 'waiting_seconds',
               'plan_ms_max', 'angle_metrics', 'error')


def clean_json(value):
    """把非有限浮点转成空值，确保内嵌数据是浏览器可解析的标准 JSON。"""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: clean_json(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean_json(item) for item in value]
    return value


def load_scene(directory, prefix, name, scene, geometry_hash):
    """读取一场真实记录；缺文件或缺样本显式保留原因，不生成通过或替代轨迹。"""
    report_path = directory / f'{prefix}-{name}.json'
    samples_path = directory / f'{prefix}-{name}-samples.json'
    result = dict(id=name, title=TITLES[name], available=False, report={}, samples=[], boxes=[],
                  status='未运行', missing=[], geometry_verified=False)
    for path in (report_path, samples_path):
        if not path.exists():
            result['missing'].append(path.name)
    if result['missing']:
        result['status'] = '未运行' if len(result['missing']) == 2 else '记录不完整'
        return result
    try:
        report = json.loads(report_path.read_text(encoding='utf-8-sig'))
        samples = json.loads(samples_path.read_text(encoding='utf-8-sig'))
        if not isinstance(report, dict) or not isinstance(samples, list) or not samples:
            result['status'] = '记录不完整'
            result['missing'] = ['报告类型无效或采样为空']
            return result
        # 不内嵌最终地图：最终已知区域不能伪装成过去每一帧的历史地图。
        result['report'] = {key: report.get(key) for key in REPORT_KEYS if key in report}
        valid = []
        for row in samples:
            if not isinstance(row, dict):
                continue
            if not all(isinstance(row.get(key), (int, float)) and math.isfinite(row[key])
                       for key in ('t', 'x', 'y')):
                continue
            valid.append({key: row.get(key) for key in SAMPLE_KEYS if key in row})
        result['samples'] = sorted(valid, key=lambda row: row['t'])
        if not result['samples']:
            result['status'], result['missing'] = '记录不完整', ['没有有效实际位姿样本']
            return result
        # 障碍只用于报告解释；来源哈希不一致时不显示可能已经变化的场景几何。
        embedded_scene = report.get('scenario_specification')
        embedded_valid = (isinstance(embedded_scene, dict)
                          and hashlib.sha256(json.dumps(embedded_scene, ensure_ascii=False, sort_keys=True).encode()).hexdigest()
                          == report.get('scenario_spec_sha256'))
        result['geometry_verified'] = embedded_valid or report.get('source_sha256', {}).get('scenarios.py') == geometry_hash
        if result['geometry_verified']:
            result['boxes'] = embedded_scene['boxes'] if embedded_valid else scene['boxes']
            if embedded_valid:
                result['title'] = embedded_scene.get('name', result['title'])
        result['geometry_source'] = 'embedded' if embedded_valid else 'source_hash'
        result['available'] = True
        verdict = report.get('passed_navigation')
        result['status'] = '通过' if verdict is True else '未通过' if verdict is False else '缺少整体判定'
    except (OSError, ValueError, TypeError) as error:
        result['status'], result['missing'] = '记录不完整', [str(error)]
    return clean_json(result)


PAGE = r'''<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="light">
<title>Go2 离线验收回放 · __TITLE__</title>
<style>
:root{--ink:#203344;--muted:#66788b;--line:#e1e8ed;--green:#168d83;--amber:#d28b27;--red:#be4248;--blue:#6285ad;--card:#fff}
*{box-sizing:border-box}body{margin:0;background:#f3f6f8;color:var(--ink);font:14px/1.55 "Microsoft YaHei","PingFang SC",Arial,sans-serif}
button,select,input{font:inherit}button,select{border:1px solid #d6e0e6;background:white;color:var(--ink);border-radius:9px;cursor:pointer}
button{padding:9px 15px}button:hover{border-color:#8fb9b4}button:focus-visible,select:focus-visible,input:focus-visible{outline:3px solid #a1d8cf;outline-offset:2px}
.shell{max-width:1420px;margin:auto;padding:26px 32px 30px}.eyebrow{color:var(--green);font-size:12px;font-weight:700;letter-spacing:1px}
header{display:flex;align-items:center;justify-content:space-between;gap:20px;margin-bottom:22px}h1{font-size:26px;line-height:1.25;margin:5px 0 7px;font-weight:700}
.intro{color:var(--muted);margin:0}.prefix{font:12px/1.5 Consolas,monospace;background:#e7eef2;padding:8px 12px;border-radius:8px;overflow-wrap:anywhere}
.scenes{display:grid;grid-template-columns:repeat(4,minmax(0,1fr));gap:10px;margin-bottom:18px}.scene{display:flex;justify-content:space-between;align-items:center;gap:7px;text-align:left;padding:13px 14px}
.scene[aria-pressed="true"]{border-color:var(--green);background:#eef9f6;box-shadow:0 0 0 1px var(--green)}.badge{font-size:11px;font-weight:600;padding:3px 8px;border-radius:20px;white-space:nowrap}
.ok{color:#146d5b;background:#dff4eb}.fail{color:#a12c36;background:#ffe8e8}.missing{color:#627283;background:#e9edf1}.neutral{color:#36658c;background:#e7f0f8}
.layout{display:grid;grid-template-columns:minmax(0,2.15fr) minmax(275px,1fr);gap:18px}.card{background:var(--card);border:1px solid var(--line);border-radius:14px;overflow:hidden;box-shadow:0 4px 18px #20334405}
.card-head{padding:17px 20px 11px;display:flex;justify-content:space-between;align-items:center;gap:8px}.card-head h2{font-size:17px;margin:0}.muted{color:var(--muted);font-size:12px}
.legend{padding:0 20px 8px;color:var(--muted);font-size:12px;display:flex;gap:17px;flex-wrap:wrap}.legend span:before{content:"";display:inline-block;width:13px;height:3px;background:var(--green);margin:0 6px 3px 0}
.legend .target:before{background:var(--amber)}.legend .obstacle:before{height:9px;background:#8b99a3;margin-bottom:0}.legend .command:before{background:var(--blue)}
.canvas-wrap{position:relative;padding:0 8px}canvas{display:block;width:100%}#map{height:410px}#speedChart{height:175px}.empty{position:absolute;inset:0;display:flex;flex-direction:column;justify-content:center;align-items:center;text-align:center;padding:35px;color:var(--muted);background:#f8fafbd9}
.empty strong{font-size:19px;color:var(--ink);margin-bottom:8px}.empty[hidden]{display:none}.controls{padding:15px 20px;border-top:1px solid var(--line);display:grid;grid-template-columns:auto 1fr auto;gap:11px;align-items:center}
#play{min-width:84px;background:var(--green);border-color:var(--green);color:white}#play:disabled{background:#98a9ad;border-color:#98a9ad;cursor:default}#scrub{width:100%;accent-color:var(--green);cursor:pointer}
.time-row{font-variant-numeric:tabular-nums;display:flex;justify-content:space-between;color:var(--muted);font-size:12px}.speed-select{padding:7px 6px}.micro{padding:0 20px 13px;color:var(--muted);font-size:11px}
.side{display:flex;flex-direction:column;gap:15px}.metrics{padding:0 20px 17px;display:grid;grid-template-columns:1fr 1fr;gap:15px}.metric small{display:block;color:var(--muted);font-size:11px}.metric b{font-size:24px;font-weight:650;font-variant-numeric:tabular-nums}.unit{font-size:11px;color:var(--muted);margin-left:4px}
.status-box{border-top:1px solid var(--line);padding:14px 20px;overflow-wrap:anywhere}.status-line{display:flex;justify-content:space-between;gap:12px;font-size:12px;margin:6px 0}.mono{font:11px/1.5 Consolas,"Microsoft YaHei",monospace}
.facts{padding:0 20px 15px}.fact{display:flex;align-items:center;justify-content:space-between;border-top:1px solid #edf1f4;padding:9px 0;font-size:12px;gap:10px}.fact strong{font-variant-numeric:tabular-nums}
.failed-list{font-size:11px;line-height:1.8;color:#a23c45;margin:8px 0 0;overflow-wrap:anywhere}.note{background:#fff8e9;color:#80611b;border:1px solid #ebdfbe;border-radius:10px;padding:13px 17px;margin-top:16px;font-size:12px}
.foot{margin-top:14px;color:var(--muted);font-size:11px;display:flex;justify-content:space-between;gap:12px;flex-wrap:wrap}.chart-card{margin-top:18px}.chart-card .card-head{padding-bottom:6px}.show-truth{display:flex;align-items:center;gap:5px;font-size:11px;color:var(--muted);cursor:pointer}.show-truth input{accent-color:#698691}
@media(max-width:950px){.shell{padding:18px}.layout{grid-template-columns:1fr}.side{display:grid;grid-template-columns:1fr 1fr}.scene{display:block;padding:10px}.scene .badge{display:inline-block;margin-top:4px}header{align-items:flex-start;flex-direction:column}.prefix{max-width:100%}}
@media(max-width:580px){.side{display:flex}.scenes{grid-template-columns:repeat(2,1fr)}.scenes .scene:last-child{grid-column:span 2}h1{font-size:23px}#map{height:330px}.controls{padding:12px;gap:7px}.card-head{padding:14px} .legend{padding-left:14px}.note{padding:11px}.metrics{padding:0 15px 15px}}
</style>
</head>
<body>
<main class="shell">
<header><div><div class="eyebrow">实际记录 · 离线回放</div><h1>Go2 跟随与避障验收</h1><p class="intro">拖动时间看真实运动、停车与跟随间距。双击本文件即可播放。</p></div><div class="prefix">__TITLE__</div></header>
<nav class="scenes" id="scenes" aria-label="验收场景"></nav>
<div class="layout">
<section><div class="card"><div class="card-head"><h2 id="sceneTitle">实际轨迹</h2><label class="show-truth"><input type="checkbox" id="truth" checked>显示辅助障碍</label></div>
<div class="legend"><span>机器狗实际路线</span><span class="target">目标实际路线</span><span class="obstacle">场景障碍真值</span></div>
<div class="canvas-wrap"><canvas id="map" aria-label="机器人和目标真实轨迹"></canvas><div class="empty" id="empty" hidden></div></div>
<div class="controls"><button id="play" type="button">▶ 播放</button><div><input id="scrub" type="range" min="0" max="1" step="0.01" value="0" aria-label="回放时间"><div class="time-row"><span id="timeNow">0.00 s</span><span id="timeEnd">0.00 s</span></div></div><select class="speed-select" id="rate" aria-label="回放速度"><option value="0.5">0.5 倍速</option><option value="1" selected>1 倍速</option><option value="2">2 倍速</option><option value="4">4 倍速</option></select></div>
<div class="micro"><span id="footprintHint"></span>指标读取最近真实样本；点间位姿插值仅改善播放连续性。</div></div>
<div class="card chart-card"><div class="card-head"><h2>实际速度与命令</h2><span class="muted">机体系前进速度 · m/s</span></div><div class="legend"><span>实际 vx</span><span class="target">目标速度</span><span class="command">最终前进命令</span></div><div class="canvas-wrap"><canvas id="speedChart" aria-label="实际速度、目标速度和前进命令时间曲线"></canvas></div><div class="micro">曲线包含停车和减速时段，点击图表也可跳到相应时间。</div></div></section>
<aside class="side"><section class="card"><div class="card-head"><h2>当前采样</h2><span class="muted" id="sampleTime">t = —</span></div><div class="metrics">
<div class="metric"><small>机器人实际 vx</small><b id="vx">—</b><span class="unit">m/s</span></div><div class="metric"><small>目标实际速度</small><b id="targetSpeed">—</b><span class="unit">m/s</span></div>
<div class="metric"><small>跟随间距</small><b id="distance">—</b><span class="unit">m</span></div><div class="metric"><small>实际角速度</small><b id="wz">—</b><span class="unit">rad/s</span></div></div>
<div class="status-box"><div class="status-line"><span>当前状态</span><strong id="state">—</strong></div><div class="mono" id="code">—</div><div class="status-line"><span>最终前进命令</span><strong id="cmdV">—</strong></div><div class="status-line"><span>最终转向命令</span><strong id="cmdW">—</strong></div><div class="status-line"><span>深度年龄 / 帧数</span><strong id="depth">—</strong></div></div></section>
<section class="card"><div class="card-head"><h2>整场报告判定</h2><span class="badge" id="verdict">未运行</span></div><div class="facts" id="facts"></div></section></aside>
</div>
<div class="note" id="note">障碍真值只辅助解释报告，没有输入导航规划器。本页不显示历史地图：报告中的最终地图不能代表过去每一帧的可见区域。没有碰撞不等于绕障完成。</div>
<footer class="foot"><span>记录是单次验收证据，不代表重复运行成功率。缺失场景不会补成绩。</span><span id="provenance"></span></footer>
</main>
<script id="replayData" type="application/json">__DATA__</script>
<script>
'use strict';
// 数据直接嵌入文件，不使用 fetch、外部字体或库，断网与停止仿真后仍可验收。
const DATA = JSON.parse(document.getElementById('replayData').textContent);
const $ = id => document.getElementById(id);
const STATE = {FOLLOWING:'跟随',OBSERVING:'观察',FACING:'看向目标',DETOUR:'绕障',WAITING:'等待',HOLDING:'保持距离',PAUSED:'暂停',ESTOP:'急停',INITIALIZING:'姿态准备',TARGET_LOST:'目标失效',ODOM_LOST:'里程计失效',MAP_STALE:'深度过期',TOO_CLOSE:'近距离保护',INPUT_LOST:'操作输入失效'};
const CHECK = {no_physical_obstacle_contact:'无物理障碍接触',posture_ready:'姿态就绪',phase_speed_limits:'阶段限速',nav2_mppi_alive:'MPPI 存活',physical_progress:'位移进展',depth_loss_stops:'深度失效停车',uwb_loss_stops:'UWB 失效停车',moving_period_observed:'目标移动窗口',moving_follow_distance_bounded:'移动时跟随间距',slow_time_ratio_bounded:'低速时间占比',longest_slow_spell_bounded:'最长连续低速',steady_window_observed:'稳态窗口',steady_actual_speed:'稳态实际速度',steady_follow_distance_bounded:'稳态跟随间距'};
Object.assign(CHECK,{physical_progress:'实际运动进展',moving_actual_speed:'移动期间实际速度',target_completed_required_laps:'目标完成指定圈数',robot_completed_required_laps:'机器狗依次完成指定圈数',target_keeps_walking:'目标持续沿路线行走',final_follow_distance_reasonable:'末段跟随间距',final_window_observed:'末段观察时长',target_reached_preset_endpoint:'目标到达预置终点',robot_fully_on_far_side:'整机越过障碍',stopped_before_wall:'墙前稳定停车'});
let scene,rows=[],clock=0,duration=0,base=0,playing=false,lastFrame=null,bounds={minX:-1,maxX:8,minY:-3,maxY:3};
// 缺测保持空值；整体判定只接受报告中的明确布尔值，不把缺项当通过。
const fmt = (value,digits=3) => typeof value==='number' && Number.isFinite(value) ? value.toFixed(digits) : '—';
const badge = value => value===true ? ['通过','ok'] : value===false ? ['未通过','fail'] : ['缺少判定','missing'];

function selectScene(id){
  // 每个场景独立从首帧开始；未运行场景保留空状态，不复用其他场的轨迹或成功判定。
  scene=DATA.scenes.find(item=>item.id===id);rows=scene.samples;playing=false;lastFrame=null;clock=0;
  base=rows.length?rows[0].t:0;duration=rows.length?Math.max(0,rows[rows.length-1].t-base):0;
  $('sceneTitle').textContent=scene.title+' · 实际轨迹';$('play').disabled=!scene.available;$('scrub').disabled=!scene.available;
  // 滑块步长为0.01秒，向上对齐上限；否则End可能落在末样本之前，遗漏最终状态。
  $('scrub').max=String(duration?Math.ceil(duration*100)/100:1);$('scrub').value='0';$('timeEnd').textContent=fmt(duration,2)+' s';
  for(const button of $('scenes').children)button.setAttribute('aria-pressed',String(button.dataset.scene===id));
  $('empty').hidden=scene.available;$('empty').replaceChildren();
  if(!scene.available){const title=document.createElement('strong');title.textContent=scene.status;const reason=document.createElement('span');reason.textContent=scene.missing.join(' · ');$('empty').append(title,reason);}
  $('footprintHint').textContent=!scene.available?'本场未运行，没有包络版本记录。':scene.report.acceptance_schema_version>=6?'矩形包络长0.70 m、宽0.32 m，按实际朝向显示。':'本场旧记录采用半径0.48 m圆包络。';
  computeBounds();renderReport();draw();
}

function computeBounds(){
  // 视窗包含整段实际机器人和目标路线，不能裁掉远离的目标来掩盖跟随掉队。
  const xs=[-1],ys=[-2,3];for(const row of rows){xs.push(row.x);ys.push(row.y);if(Array.isArray(row.target)){if(Number.isFinite(row.target[0]))xs.push(row.target[0]);if(Number.isFinite(row.target[1]))ys.push(row.target[1]);}}
  for(const box of scene.boxes){xs.push(box[0]-box[2],box[0]+box[2]);ys.push(box[1]-box[3],box[1]+box[3]);}
  bounds={minX:Math.floor(Math.min(...xs)-.8),maxX:Math.ceil(Math.max(...xs)+.8),minY:Math.floor(Math.min(...ys)-.8),maxY:Math.ceil(Math.max(...ys)+.8)};
  if(bounds.maxX-bounds.minX<5)bounds.maxX=bounds.minX+5;
}

function addFact(label,value,className=''){
  // 安全地添加一行报告指标，所有报告文本经 textContent 展示而非执行为 HTML。
  const line=document.createElement('div');line.className='fact';const name=document.createElement('span');name.textContent=label;
  const result=document.createElement('strong');result.textContent=value;result.className=className;line.append(name,result);$('facts').append(line);
}

function renderReport(){
  // 一切成绩使用报告原判定；不根据轨迹外观或子项通过重新计算整体成功。
  const report=scene.report;$('facts').replaceChildren();const v=scene.available?badge(report.passed_navigation):[scene.status,'missing'];
  $('verdict').textContent=v[0];$('verdict').className='badge '+v[1];
  $('truth').disabled=!scene.geometry_verified;
  $('note').textContent='障碍真值只辅助解释报告，没有输入导航规划器。'+(scene.geometry_verified?(scene.geometry_source==='embedded'?'使用本场内嵌场景快照，已核对摘要。':'场景文件与本场记录哈希一致。'):'本场场景文件来源未核对一致，因此隐藏辅助障碍。')+' 本页不显示历史地图：最终地图不能代表过去每一帧的可见区域。没有碰撞不等于绕障完成。';
  if(!scene.available){addFact('记录状态',scene.status);$('provenance').textContent=DATA.prefix;return;}
  for(const [label,key] of [['基础检查','passed_basic_checks'],['刺激有效','stimulus_valid'],['任务判定','scenario_objective_met']]){const b=badge(report[key]);addFact(label,b[0],b[1]);}
  const applicable=report.fluency?.applicable!==false,fluency=badge(report.passed_fluency);
  addFact('流畅性',applicable?fluency[0]:'不适用',applicable?fluency[1]:'missing');
  addFact('执行模式',report.execution_mode||'—');addFact('规划 / 观察',(report.planning_mode||'—')+' / '+(report.observation_mode||'—'));
  addFact('相机俯角',Number.isFinite(report.camera_pitch_deg)?report.camera_pitch_deg+'°（仿真配置）':'旧记录未保存');
  addFact('搜索 / 唤醒',(report.search_execution_mode||'—')+' / '+(report.executor_wake_mode||'—'));
  addFact('目标设置速度',fmt(report.target_speed_mps)+' m/s');
  addFact(scene.id==='open'?'稳态实际 vx':'目标移动期间实际 vx',fmt(scene.id==='open'?report.steady_actual_vx:report.target_moving_mean_actual_vx)+' m/s');
  addFact('移动时最大间距',fmt(report.target_moving_max_distance)+' m');addFact('最长连续低速',fmt(report.target_moving_longest_slow_seconds,2)+' s');
  addFact('移动时低速占比',typeof report.target_moving_robot_slow_ratio==='number'?fmt(report.target_moving_robot_slow_ratio*100,1)+'%':'—');
  if(report.angle_metrics)addFact('行走路径角误差 RMS',fmt(report.angle_metrics.path_error_rms_deg,1)+'°（不含停转）');
  addFact('最终间距',fmt(report.final_distance_m)+' m');addFact('最终原因',report.final_code||report.error||'—');
  if(report.task_completion?.loop){addFact('目标完成圈数',String(report.task_completion.target_completed_laps));addFact('机器狗有序通过圈数',String(report.task_completion.robot?.completed_laps||0));addFact('要求双方完成圈数',String(report.task_completion.required_laps));}
  const failed=Object.entries(report.checks||{}).filter(([,value])=>value===false).map(([key])=>CHECK[key]||key);
  const failedFluency=applicable?Object.entries(report.fluency?.checks||{}).filter(([,value])=>value===false).map(([key])=>CHECK[key]||key):[];
  const failedTask=Object.entries(report.task_completion?.checks||{}).filter(([,value])=>value===false).map(([key])=>CHECK[key]||key);
  if(failed.length||failedFluency.length||failedTask.length){const list=document.createElement('div');list.className='failed-list';list.textContent='失败项：'+[...new Set([...failed,...failedFluency,...failedTask])].join('、');$('facts').append(list);}
  $('provenance').textContent=(report.started_at_utc||'未记录时间')+' · '+rows.length+' 个实际样本';
}

function canvasContext(id){
  // 高分屏只提高画布清晰度，不改变米制比例；两张画布均随窗口宽度更新。
  const canvas=$(id),rect=canvas.getBoundingClientRect(),ratio=window.devicePixelRatio||1;
  const width=Math.max(1,rect.width),height=Math.max(1,rect.height);
  if(canvas.width!==Math.round(width*ratio)||canvas.height!==Math.round(height*ratio)){canvas.width=Math.round(width*ratio);canvas.height=Math.round(height*ratio);}
  const ctx=canvas.getContext('2d');ctx.setTransform(ratio,0,0,ratio,0,0);ctx.clearRect(0,0,width,height);return {ctx,width,height};
}

function sampleIndex(){
  // 对时间有序的实测样本做二分定位，指标保持最近过去样本，避免插值制造新测量。
  let low=0,high=rows.length-1,target=base+clock;
  while(low<high){const middle=Math.ceil((low+high)/2);if(rows[middle].t<=target)low=middle;else high=middle-1;}
  return low;
}

function shownPose(index){
  // 只对显示位置做邻帧插值；航向跨 ±π 按最短角差连接，不改右侧实测指标。
  const row=rows[index],next=rows[Math.min(index+1,rows.length-1)];
  const ratio=next.t>row.t?Math.max(0,Math.min(1,(base+clock-row.t)/(next.t-row.t))):0;
  const interpolate=(a,b)=>Number.isFinite(a)&&Number.isFinite(b)?a+(b-a)*ratio:a;
  let yaw=row.yaw||0;if(Number.isFinite(next.yaw)){const delta=Math.atan2(Math.sin(next.yaw-yaw),Math.cos(next.yaw-yaw));yaw+=delta*ratio;}
  const target=Array.isArray(row.target)&&Array.isArray(next.target)?[interpolate(row.target[0],next.target[0]),interpolate(row.target[1],next.target[1])]:row.target;
  return {x:interpolate(row.x,next.x),y:interpolate(row.y,next.y),yaw,target};
}

function drawMap(index){
  // 按米制等比例绘制轨迹、实际朝向和报告版本对应的机身包络；辅助障碍须先通过来源核对。
  const {ctx,width,height}=canvasContext('map'),pad=36;
  const scale=Math.min((width-pad*2)/(bounds.maxX-bounds.minX),(height-pad*2)/(bounds.maxY-bounds.minY));
  const ox=(width-(bounds.maxX-bounds.minX)*scale)/2,oy=(height-(bounds.maxY-bounds.minY)*scale)/2;
  const point=(x,y)=>[ox+(x-bounds.minX)*scale,height-oy-(y-bounds.minY)*scale];
  ctx.fillStyle='#fbfcfd';ctx.fillRect(0,0,width,height);ctx.font='10px "Microsoft YaHei",sans-serif';ctx.lineWidth=1;
  const step=bounds.maxX-bounds.minX>18?2:1;
  for(let x=Math.ceil(bounds.minX);x<=bounds.maxX;x+=step){const [px]=point(x,0);ctx.strokeStyle='#e8eef2';ctx.beginPath();ctx.moveTo(px,oy);ctx.lineTo(px,height-oy);ctx.stroke();ctx.fillStyle='#8192a0';ctx.fillText(String(x),px-3,height-oy+17);}
  for(let y=Math.ceil(bounds.minY);y<=bounds.maxY;y+=step){const [,py]=point(0,y);ctx.strokeStyle='#e8eef2';ctx.beginPath();ctx.moveTo(ox,py);ctx.lineTo(width-ox,py);ctx.stroke();ctx.fillStyle='#8192a0';ctx.fillText(String(y),ox-23,py+3);}
  ctx.fillStyle='#8192a0';ctx.fillText('x / m',width-ox-27,height-oy+30);ctx.fillText('y / m',ox-22,oy-12);
  if($('truth').checked&&scene.geometry_verified){for(const [x,y,sx,sy] of scene.boxes){const [left,top]=point(x-sx,y+sy);ctx.fillStyle='#8997a1';ctx.fillRect(left,top,2*sx*scale,2*sy*scale);ctx.strokeStyle='#657885';ctx.strokeRect(left,top,2*sx*scale,2*sy*scale);}}
  if(!rows.length)return;
  const actual=shownPose(index);
  function line(target){
    // 仅绘制已经回放到的实际路线，未播放的未来轨迹不伪装成当前规划路径。
    ctx.beginPath();let started=false;for(let i=0;i<=index;i++){const row=rows[i],xy=target?row.target:[row.x,row.y];if(!Array.isArray(xy)||!Number.isFinite(xy[0])||!Number.isFinite(xy[1]))continue;const p=point(...xy);started?ctx.lineTo(...p):ctx.moveTo(...p);started=true;}
    const final=target?actual.target:[actual.x,actual.y];if(started&&Array.isArray(final))ctx.lineTo(...point(...final));ctx.stroke();
  }
  ctx.lineWidth=2.3;ctx.strokeStyle='#d28b27';ctx.setLineDash([6,5]);line(true);ctx.setLineDash([]);ctx.strokeStyle='#168d83';ctx.lineWidth=3;line(false);
  const [rx,ry]=point(actual.x,actual.y),radius=.48*scale;
  ctx.strokeStyle='#168d83';ctx.fillStyle='#168d8318';ctx.lineWidth=1.3;ctx.beginPath();
  // 新记录按实测朝向画矩形；旧版记录保留当时圆形模型，不能改变历史测试语义。
  if(scene.report.acceptance_schema_version>=6){
    const shape=scene.report.task_completion?.criteria?.robot_footprint||{length_m:.7,width_m:.32};
    const corners=[[1,1],[-1,1],[-1,-1],[1,-1]].map(([sx,sy])=>{const x=sx*shape.length_m/2,y=sy*shape.width_m/2;return point(actual.x+x*Math.cos(actual.yaw)-y*Math.sin(actual.yaw),actual.y+x*Math.sin(actual.yaw)+y*Math.cos(actual.yaw));});
    corners.forEach((p,i)=>i?ctx.lineTo(...p):ctx.moveTo(...p));ctx.closePath();
  }else{ctx.arc(rx,ry,radius,0,Math.PI*2);}ctx.fill();ctx.stroke();
  ctx.fillStyle='#168d83';ctx.beginPath();ctx.arc(rx,ry,Math.max(3,scale*.12),0,Math.PI*2);ctx.fill();
  const head=point(actual.x+.38*Math.cos(actual.yaw),actual.y+.38*Math.sin(actual.yaw));ctx.strokeStyle='#075e57';ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(rx,ry);ctx.lineTo(...head);ctx.stroke();
  if(Array.isArray(actual.target)&&actual.target.every(Number.isFinite)){const [tx,ty]=point(...actual.target);ctx.strokeStyle='#d28b2770';ctx.lineWidth=1;ctx.setLineDash([3,4]);ctx.beginPath();ctx.moveTo(rx,ry);ctx.lineTo(tx,ty);ctx.stroke();ctx.setLineDash([]);ctx.fillStyle='#d28b27';ctx.beginPath();ctx.arc(tx,ty,5,0,Math.PI*2);ctx.fill();ctx.fillStyle='#906119';ctx.fillText('目标',tx+8,ty-8);}
  ctx.fillStyle='#075e57';ctx.fillText('Go2',rx+radius+4,ry-5);
}

function drawChart(index){
  // 画完整速度记录及当前时间线，保留实际停车段和缺测断点。
  const {ctx,width,height}=canvasContext('speedChart'),left=42,right=16,top=12,bottom=30;
  ctx.fillStyle='#fff';ctx.fillRect(0,0,width,height);if(!rows.length)return;
  const values=rows.flatMap(row=>[row.vx,row.target_speed,row.command?.[0]]).filter(Number.isFinite);
  const minimum=Math.min(-.08,...values),maximum=Math.max(.8,...values),span=maximum-minimum;
  const px=time=>left+time/Math.max(duration,.001)*(width-left-right),py=value=>height-bottom-(value-minimum)/span*(height-top-bottom);
  ctx.font='10px "Microsoft YaHei",sans-serif';ctx.lineWidth=1;ctx.strokeStyle='#edf1f4';ctx.fillStyle='#8393a1';
  for(let i=0;i<=4;i++){const value=minimum+span*i/4,y=py(value);ctx.beginPath();ctx.moveTo(left,y);ctx.lineTo(width-right,y);ctx.stroke();ctx.fillText(value.toFixed(2),3,y+3);}
  for(let i=0;i<=5;i++){const time=duration*i/5;ctx.fillText(time.toFixed(0)+'s',px(time)-5,height-10);}
  for(const [get,color,dash] of [[r=>r.vx,'#168d83',[]],[r=>r.target_speed,'#d28b27',[5,4]],[r=>r.command?.[0],'#6285ad',[]]]){
    ctx.strokeStyle=color;ctx.lineWidth=1.6;ctx.setLineDash(dash);ctx.beginPath();let started=false;
    for(const row of rows){const value=get(row);if(!Number.isFinite(value)){started=false;continue;}const x=px(row.t-base),y=py(value);started?ctx.lineTo(x,y):ctx.moveTo(x,y);started=true;}ctx.stroke();
  }
  ctx.setLineDash([]);ctx.strokeStyle='#20334480';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(px(clock),top);ctx.lineTo(px(clock),height-bottom);ctx.stroke();
}

function draw(){
  // 用最近过去采样刷新数值与状态，切换未运行场景时全部清空。
  $('play').textContent=playing?'❚❚ 暂停':'▶ 播放';$('scrub').value=String(clock);$('timeNow').textContent=fmt(clock,2)+' s';
  const index=rows.length?sampleIndex():0,row=rows[index]||{};
  $('vx').textContent=fmt(row.vx);$('targetSpeed').textContent=fmt(row.target_speed);$('distance').textContent=fmt(row.distance);$('wz').textContent=fmt(row.wz);
  $('cmdV').textContent=fmt(row.command?.[0])+' m/s';$('cmdW').textContent=fmt(row.command?.[1])+' rad/s';
  $('state').textContent=STATE[row.state]||row.state||'—';$('code').textContent=row.code||'—';$('sampleTime').textContent='t = '+fmt(row.t,2)+' s';
  if(row.target_route?.loop)$('sampleTime').textContent+=' · 目标 '+row.target_route.laps+' 圈 / 路段 '+row.target_route.segment;
  $('depth').textContent=fmt(row.depth_age,2)+' s / '+(Number.isFinite(row.frames)?row.frames:'—');drawMap(index);drawChart(index);
}

function tick(timestamp){
  // 播放按浏览器真实时间乘倍速推进；末帧自动暂停，不回绕或隐去最终失败。
  if(playing){if(lastFrame!==null)clock=Math.min(duration,clock+(timestamp-lastFrame)/1000*Number($('rate').value));if(clock>=duration)playing=false;draw();}
  lastFrame=timestamp;requestAnimationFrame(tick);
}

// 显式创建全部场景入口，缺文件也保留场景按钮，避免把没有验证的场景隐藏。
for(const item of DATA.scenes){const button=document.createElement('button');button.className='scene';button.dataset.scene=item.id;button.type='button';const text=document.createElement('span');text.textContent=item.title;const tag=document.createElement('span');tag.className='badge '+(item.available?badge(item.report.passed_navigation)[1]:'missing');tag.textContent=item.status;button.append(text,tag);button.addEventListener('click',()=>selectScene(item.id));$('scenes').append(button);}
$('play').addEventListener('click',()=>{if(!scene.available)return;if(clock>=duration)clock=0;playing=!playing;lastFrame=null;draw();});
$('scrub').addEventListener('input',()=>{clock=Math.min(duration,Number($('scrub').value));lastFrame=null;draw();});
$('truth').addEventListener('change',draw);window.addEventListener('resize',draw);
$('speedChart').addEventListener('click',event=>{if(!scene.available)return;const rect=$('speedChart').getBoundingClientRect();clock=Math.max(0,Math.min(duration,(event.clientX-rect.left-42)/(rect.width-58)*duration));lastFrame=null;draw();});
selectScene((DATA.scenes.find(item=>item.available)||DATA.scenes[0]).id);requestAnimationFrame(tick);
</script>
</body></html>'''


def build_replay(directory, prefix):
    """生成一个自包含 HTML；全部场景缺失情况和原始判定均固定在本次文件中。"""
    scenarios_path = Path(__file__).resolve().parents[1] / 'scenarios.py'
    geometry_hash = hashlib.sha256(scenarios_path.read_bytes()).hexdigest()
    payload = dict(prefix=prefix, scenes=[load_scene(directory, prefix, name, scene, geometry_hash)
                                         for name, scene in SCENARIOS.items()])
    # 防止报告字符串包含 </script> 提前截断内嵌 JSON，不在页面中直接执行报告内容。
    embedded = json.dumps(payload, ensure_ascii=False, allow_nan=False, separators=(',', ':'))
    embedded = embedded.replace('<', '\\u003c').replace('>', '\\u003e').replace('&', '\\u0026')
    output = directory / f'{prefix}-验收回放.html'
    output.write_text(PAGE.replace('__TITLE__', html.escape(prefix)).replace('__DATA__', embedded), encoding='utf-8')
    return output, payload


def main():
    """读取命令行的证据目录和报告前缀，输出可离线打开的验收文件路径。"""
    parser = argparse.ArgumentParser(description='生成无需仿真和 HTTP 的中文跟随验收回放')
    parser.add_argument('--directory', type=Path, default=Path.home() / 'go2_sim/artifacts')
    parser.add_argument('--prefix', default='navigation-v2')
    args = parser.parse_args()
    if not args.directory.is_dir():
        parser.error(f'证据目录不存在：{args.directory}')
    output, payload = build_replay(args.directory, args.prefix)
    print(output.resolve())
    print('；'.join(f"{scene['title']}：{scene['status']}（{len(scene['samples'])} 帧）" for scene in payload['scenes']))


if __name__ == '__main__':
    main()

# RSView 历史 PCAP：同算法轨旁测距

2026-10-09 当前预设 `rsview-current-boundary-beta-v20`：归属不清但有当前支持的数值以橙色警告保留，旧ID不得恢复；新输入原生退出访问冲突仍未修复，先作为离线诊断测试版。见 [现场数据与验证边界](C:/Users/mqx/Documents/智能轨道检测/rsview_centerline/analysis/20261009_field_cases/FIELD_V20_NOTES.md)。

2026-10-09：共用 v19 后端已加入第二轮等价提速，原生回放生命周期检查再次通过49项。测距定义与显示点不变，见 [最新性能记录](C:/Users/mqx/Documents/智能轨道检测/rsview_centerline/analysis/20261009_latency/LATENCY_NOTES.md)。

_2026-10-08；用于在原生窗口核查当前 v19 功能，不是精度或高速验收。_

同一 v19 预设已加入等价性能修订：减少归属冗余计算，不改变测距点和归属判据。历史回放与实时共用，仍需重新启动旧窗口。见 [性能核查记录](C:/Users/mqx/Documents/智能轨道检测/rsview_centerline/analysis/20261008_latency/LATENCY_NOTES.md)。

---

## 📋 使用范围

当前配置为 `rsview-adaptive-motion-beta-v19`，保留 v18 归属保护，并增加经过检查的扫描内平移候选与局部地图复用。轮径未知不阻塞这条补偿路径，未按RPM换算线速度。首次观测可能显示橙色，关联有歧义的区域进入待确认。重复标注、整链路时延和后台运行异常仍未完全解决，详见 [v19运动核查](C:/Users/mqx/Documents/智能轨道检测/rsview_centerline/analysis/20261008_adaptive_motion/MOTION_V19_NOTES.md)与 [v18归属记录](C:/Users/mqx/Documents/智能轨道检测/rsview_centerline/analysis/20261008_instance_refinement/INSTANCE_V18_NOTES.md)。

普通 Airy PCAP（例如桌面 `333.pcap`）现在可以启用与真实雷达相同的测距配置及处理代码。输入为当前原生扫描的真实 XYZ、点时间戳与 Ring，结果直接显示在 RSView。原始 PCAP 不改写，也不生成整文件逐帧大点云导出。

| 入口 | 状态栏 | 位姿与背景来源 |
| --- | --- | --- |
| 真实 Airy＋当前菜单 | `LIVE BETA` | 当前与过去雷达扫描，无录制先验 |
| 普通 Airy PCAP＋当前菜单 | `PCAP REPLAY` | 同一相对补偿算法，无录制先验 |
| 旧专用在线脚本 | `ONLINE SIM` | 明确标注的实验录制位姿 |
| 旧专用离线脚本 | `OFFLINE PRECOMPUTED` | 保存的逐帧结果 |

当前菜单不会调用后两种路径。取消菜单后，原有专用文件会恢复原来的轨距/离线显示；普通文件恢复原始点云。

## 🔧 打开与核查

1. 使用 [当前启动入口](C:/Users/mqx/Documents/智能轨道检测/rsview_centerline/start_clearance_live.cmd) 打开新的 D 盘 RSView。旧窗口已经载入的 Python 代码不会自动刷新，先保存旧窗口中的工作。
2. 正常打开桌面 `333.pcap`，解码型号选择 Airy，保持原有正确端口与网络设置。先确认普通点云能够正常播放。
3. 勾选 `Tools → 轨旁测距（实时/回放测试）`。不必连接真实雷达，也不必使用网页或旧的专用回放脚本。
4. 等待计算。紫线是中心线；绿线与橙线仍沿用当前算法的确认/不确定标注，不代表已识别出设备名称。
5. 暂停在感兴趣的帧，等待状态栏 `input/result` 对应同一帧并出现 `CURRENT SCAN`，再人工核对。`DELAYED SNAPSHOT` 表示较早结果和它自己的点云，不是所选最新帧已经完成。
6. 可前后拖动或单步。跳段后历史关联清空，首帧可能显示 `UNCOMPENSATED`；连续向前播放几帧才可能重新获得有效雷达相对补偿。
7. 取消勾选即可退出，恢复原点云、TF 设置以及原有专用覆盖层。关闭或切换文件会释放旧计算进程，新文件需要重新勾选。

## ⚠️ 常见情况

- 菜单仍灰色：确认是新实例、原生 Airy PCAP，而非 BIN、其他型号或尚未打开的数据源。
- 提示重新打开录制：旧窗口的 TF 代理与受管理设置不一致，功能拒绝猜测原生状态；重新打开文件后再启用。
- 暂停后短暂没有线：所选帧仍在计算，或者该帧没有可靠中心线/候选。算法不补造测距值。
- 倒放时补偿不足：每次时间倒退都需要新会话，不能把未来点云作为过去帧的运动证据。
- 仍有重复、误报或漂移：本次是回放接入，不是设备识别算法升级；需要用新入口暴露并定位这些问题。

## 🔗 证据与边界

本机真实 RSView 已检查 `333` 的暂停起算、连续帧、播放中跳段、暂停保留、前后跳转、文件末尾与切换 `22222` 后恢复旧轨距。见 [回放核查目录](C:/Users/mqx/Documents/智能轨道检测/rsview_centerline/analysis/20261007_recorded_playback)。

雷达原点半径 5 m 的输入范围不等于轨道中心线横向 5 m 走廊。回放暂停后的密集计算、流畅程度与物理小车高速运行不是同一种实验。GPS/惯导、80–300 rpm、连续 5 Hz 与绝对距离精度仍未验证。重复标注、未知背景和遮挡边界等 v16 限制仍保留。

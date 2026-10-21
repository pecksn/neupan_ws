nmea_navsat_driver
===============

ROS driver to parse NMEA strings and publish standard ROS NavSat message types. Does not require the GPSD daemon to be running.

API
---

This package has no released Code API.

The ROS API documentation and other information can be found at http://ros.org/wiki/nmea_navsat_driver

本项目串口链路诊断
------------------

现有唯一串口读取路径每秒发布 ``/gnss/serial_diagnostics`` （String JSON），统计
原始读入字节/完整与部分行、报文分类、非ASCII、PVTSLNA识别/解析/CRC及Publisher
调用。``total`` 为会话累计，``window`` 为本周期，``rates_per_sec`` 使用实际单调时钟。
会话ID/序号、模块绝对路径与SHA256用于核对实际运行版本；不保存原始字节全文。

重启GPS驱动及综合日志后，完整报告写入原综合JSONL，三份录包配置也默认订阅。
launch参数 ``serial_diagnostics_print:=false`` （默认）只关闭终端汇总，不关闭话题；综合日志仍逐秒保存统计，异常警告保留。
不改设备配置、频率、原串口超时或质量门控；统计率不是设备物理输出率，Publisher
调用也不是网络交付确认。设备已确认配置10 Hz时，应先核对链路，不能凭旧日志1 Hz提频。

查看：``rostopic echo /gnss/serial_diagnostics``。具体字段见工作区的GDUT室外导航.md。

GNSS observation timing
------------------------------------

``time_mode=shadow`` is the default: ``/gnss/time_diagnostics`` reports each
GGA/PVTSLNA/RMC measurement epoch, exact serial-read ROS/monotonic timestamps,
publication timestamps and invalid-time reasons. It does NOT change navigation
message timestamps or Fixed/Float behavior. ``time_mode=host`` disables this
stream. The full outdoor system now selects ``measurement`` with
``time_clock_verified=auto``: wait for kernel clock synchronization and a
continuous two-second window of valid FINE PVTSLNA and GGA UTC epochs before
publishing measurement fixes/headings. This is an automatic consistency gate,
not proof of hardware synchronization. Standalone driver defaults remain shadow.

PVTSLNA time uses the CRC-checked N4 header (GPS/GPST + FINE, week,
milliseconds, receiver leap offset). GGA dates require a recent valid receiver
date from PVTSLNA or RMC; no host-date substitution is used by the diagnostic.
Unsupported BDST, UNKNOWN and invalid timestamps are reported, not converted.
Without independently verified common UTC clocks, only ``candidate_age_ms``
is populated; ``measurement_age_ms`` is null. ``time_clock_verified=true`` is
an operator assertion, never a clock synchronization command. A detected clock
step revokes it for the remainder of the diagnostic session.

In auto mode, startup UNKNOWN/empty UTC waits without permanently failing;
startup clock steps restart the window. After readiness, a clock step or loss
of kernel synchronization latches protection until a parked system restart.
Normal GNSS loss still follows the original LIO continuation policy.

The legacy ``use_GNSS_time`` switch is NOT full-chain time alignment; leave it
false during shadow acceptance. See ``PROJECT_ISSUES_CN.md`` section 2.2 for
the pending outdoor clock acceptance and parked, unified deployment switch.
Measurement mode preserves receiver epochs on fix/GGA and heading/quality,
suppresses repeated/out-of-order fix/heading, and never substitutes host time
for invalid measurements. It requires GGA positions and Fixed-gated PVTSLNA
headings. UNKNOWN time is not made usable by enabling this parameter.

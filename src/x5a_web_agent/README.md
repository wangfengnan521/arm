# X5A 手机语音抓取 Demo

手机当麦克风，Ubuntu 主机跑 Web + ROS 2 Task Server，复用已经验证过的 `x5a_pick_place`。

```text
手机浏览器
  → FastAPI / WebSocket
  → command_parser（规则解析）
  → ros_bridge（只发 ROS 2 Action）
  → /x5a/pick_place
  → x5a_task_server
  → x5a_pick_place.execute_pick_place()
  → MoveIt 2
  → x5a_moveit_official_adapter
  → /arm_cmd
  → ARX X5A
```

Web / ASR / 自然语言层 **不会** 发布 `/arm_cmd`，也不会直接控制关节。

## 安装 Python 依赖

使用 ROS 2 Humble 的系统 Python 3.10：

```bash
sudo apt install -y python3-pip python3-fastapi python3-uvicorn python3-websockets
# 或者
/usr/bin/python3 -m pip install --user -r ~/arx/arm/src/x5a_web_agent/requirements.txt
```

## 编译

```bash
cd ~/arx/arm
source /opt/ros/humble/setup.bash
source ~/repos/arx/ARX_X5/ROS2/X5_ws/install/setup.bash
colcon build --symlink-install \
  --packages-select x5a_task_interfaces x5a_pick_place x5a_task_server x5a_web_agent \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

完整启动、测试和排错见仓库根目录 [docs/VOICE_INTERACTION.md](../../docs/VOICE_INTERACTION.md)。

## 接到 QQ 群机器人

网页麦克风经常被 QQ/微信内置浏览器拦住。更稳的方式是：群里发「抓红色」或发一条语音，机器人回群并调用现有抓取链。

兼容 **OneBot v11**（Napcat / Lagrange / llonebot 都可以）。

1. 先让网页服务跑起来（它同时提供 QQ 回调地址）：

```bash
source /opt/ros/humble/setup.bash
source ~/arx/arm/install/setup.bash
export X5A_QQ_API=http://127.0.0.1:3000
export X5A_QQ_GROUP_IDS=你的群号
ros2 run x5a_web_agent web_agent --host 0.0.0.0 --port 8000
```

2. 在 Napcat 里登录机器人 QQ，打开 HTTP 服务（默认 `http://127.0.0.1:3000`），并把 **HTTP 上报** 填成：

```text
http://127.0.0.1:8000/api/onebot
```

如果 Napcat 和网页不在同一台电脑，把 `127.0.0.1` 换成机械臂主机 IP。

3. 进群发：

- 文字：`抓红色` / `帮我拿白色`
- 或直接发一条 **语音**

群里会回：`收到，开始抓取红色方块`，结束后再回完成/失败。

可选环境变量：

| 变量 | 含义 |
|---|---|
| `X5A_QQ_API` | Napcat HTTP 地址，默认 `http://127.0.0.1:3000` |
| `X5A_QQ_GROUP_IDS` | 允许的群号，逗号分隔。不填则所有群都听 |
| `X5A_QQ_TOKEN` | Napcat token，没有就空着 |
| `X5A_QQ_AT_ONLY=1` | 必须 @机器人 才执行 |
| `X5A_QQ_SELF_ID` | 机器人 QQ 号，配合 @ 使用 |

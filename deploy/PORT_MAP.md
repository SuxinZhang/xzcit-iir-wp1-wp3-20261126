# IIR 端口与职责映射

> 所有浏览器入口统一经 Caddy 443 HTTPS；后端只监听回环地址，避免 CORS 与端口暴露。

| 对外路径 | Caddy 转发目标 | 服务 | 职责 |
| :--- | :--- | :--- | :--- |
| `/`、`/student`、`/ws`、`/ws/video` | `127.0.0.1:8088` | `Master/Web-Next`（Rust Axum） | 管理端、学生端 H5、遥控/遥测/巡逻 WebSocket |
| `/api/inspection/*` | `127.0.0.1:8089` | `Master/Gateway`（FastAPI） | 手动/自动抓拍、历史查询、媒体元数据 |
| `/ws/inspection` | `127.0.0.1:8089` | `Master/Gateway` | 缺陷告警 WebSocket |
| `/captures/*` | `127.0.0.1:8089` | `Master/Gateway` | 原图/标注图/JSON 归档资源 |
| `/whep/*`、`/whip/*` | `127.0.0.1:8889` | MediaMTX | WebRTC 视频/音频发布与订阅 |
| `127.0.0.1:8089` | 仅本机 | `Master/Gateway` | 禁止浏览器跨端口直连 |
| `127.0.0.1:8095` | 仅本机 | 音频守护进程 | 音频会话控制，仅 Web-Next 代理访问 |
| `127.0.0.1:8088` | 仅本机 | `Master/Web-Next` | 禁止绕过 HTTPS 访问 |
| `127.0.0.1:8554/8889` | 仅本机 | MediaMTX | RTSP/WebRTC，公网只经 Caddy |

"""OneBot v11 adapter for Napcat / Lagrange / llonebot QQ group bots.

QQ group text or voice -> same pick-place command path.
Never publishes /arm_cmd.
"""
from __future__ import annotations

import json
import os
import urllib.error
import urllib.request
from typing import Any, Dict, List, Optional, Tuple


def _env(name: str, default: str = "") -> str:
    return str(os.environ.get(name, default) or "").strip()


def allowed_group_ids() -> List[int]:
    raw = _env("X5A_QQ_GROUP_IDS")
    out: List[int] = []
    for part in raw.replace("，", ",").split(","):
        part = part.strip()
        if part.isdigit():
            out.append(int(part))
    return out


def api_base() -> str:
    return _env("X5A_QQ_API", "http://127.0.0.1:3000").rstrip("/")


def api_token() -> str:
    return _env("X5A_QQ_TOKEN")


def at_only() -> bool:
    return _env("X5A_QQ_AT_ONLY", "0") in {"1", "true", "TRUE", "yes"}


def self_id() -> str:
    return _env("X5A_QQ_SELF_ID")


def _http_json(url: str, payload: Optional[Dict[str, Any]] = None, timeout: float = 20.0) -> Dict[str, Any]:
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    headers = {"Content-Type": "application/json"}
    token = api_token()
    if token:
        headers["Authorization"] = f"Bearer {token}"
    req = urllib.request.Request(url, data=data, headers=headers, method="POST" if data else "GET")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode("utf-8", errors="replace")
    except urllib.error.URLError as exc:
        raise RuntimeError(f"QQ API 调用失败: {exc}") from exc
    if not raw:
        return {}
    try:
        return json.loads(raw)
    except json.JSONDecodeError:
        return {"raw": raw}


def send_group_msg(group_id: int, text: str) -> None:
    if not text:
        return
    _http_json(f"{api_base()}/send_group_msg", {"group_id": int(group_id), "message": text})


def fetch_record_wav(file_id: str) -> Tuple[bytes, str]:
    """Ask Napcat/OneBot to convert QQ silk/amr voice to wav."""
    result = _http_json(
        f"{api_base()}/get_record",
        {"file": file_id, "out_format": "wav"},
        timeout=30.0,
    )
    data = result.get("data") or {}
    path = data.get("file") or data.get("path") or ""
    url = data.get("url") or ""
    if path and os.path.isfile(path):
        return PathBytes(path)
    if url:
        return download_bytes(url), "audio/wav"
    raise RuntimeError(f"get_record 没有返回音频: {result}")


def PathBytes(path: str) -> Tuple[bytes, str]:
    return open(path, "rb").read(), "audio/wav"


def download_bytes(url: str) -> bytes:
    req = urllib.request.Request(url)
    token = api_token()
    if token:
        req.add_header("Authorization", f"Bearer {token}")
    with urllib.request.urlopen(req, timeout=20.0) as resp:
        return resp.read()


def _as_segments(message: Any) -> List[Dict[str, Any]]:
    if isinstance(message, list):
        return [item for item in message if isinstance(item, dict)]
    if isinstance(message, str):
        return [{"type": "text", "data": {"text": message}}]
    return []


def extract_group_command(event: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    """Return None if this event should be ignored."""
    if event.get("post_type") != "message":
        return None
    if event.get("message_type") != "group":
        return None
    group_id = int(event.get("group_id") or 0)
    if not group_id:
        return None
    allowed = allowed_group_ids()
    if allowed and group_id not in allowed:
        return None

    segments = _as_segments(event.get("message"))
    mentioned = False
    bot = self_id()
    texts: List[str] = []
    record_file = ""
    record_url = ""
    for item in segments:
        kind = str(item.get("type") or "")
        data = item.get("data") or {}
        if kind == "at":
            qq = str(data.get("qq") or "")
            if bot and qq == bot:
                mentioned = True
            if qq == "all":
                mentioned = True
        elif kind == "text":
            texts.append(str(data.get("text") or ""))
        elif kind == "record":
            record_file = str(data.get("file") or data.get("file_id") or "")
            record_url = str(data.get("url") or "")

    raw = str(event.get("raw_message") or "")
    if not texts and raw and "[CQ:record" not in raw:
        texts.append(raw)

    text = " ".join(part.strip() for part in texts).strip()
    if at_only() and not mentioned and not record_file:
        return None
    if not text and not record_file and not record_url:
        return None
    return {
        "group_id": group_id,
        "user_id": int(event.get("user_id") or 0),
        "text": text,
        "record_file": record_file,
        "record_url": record_url,
        "mentioned": mentioned,
    }

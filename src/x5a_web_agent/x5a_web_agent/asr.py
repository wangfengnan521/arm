"""Speech-to-text backends.

Default is local faster-whisper on the robot host. The phone only records
audio and uploads it. Browser Web Speech API remains as an unused fallback.
"""
from __future__ import annotations

import os
import subprocess
import tempfile
import threading
from abc import ABC, abstractmethod
from pathlib import Path


def _disable_proxy_for_local_model() -> None:
    """Clash/Meta sets socks:// proxies that huggingface_hub cannot parse."""
    for key in (
        "ALL_PROXY",
        "all_proxy",
        "HTTP_PROXY",
        "http_proxy",
        "HTTPS_PROXY",
        "https_proxy",
        "SOCKS_PROXY",
        "socks_proxy",
    ):
        os.environ.pop(key, None)
    os.environ.setdefault("NO_PROXY", "*")
    os.environ.setdefault("no_proxy", "*")


class SpeechToText(ABC):
    name = "base"
    ready = True

    @abstractmethod
    def transcribe(
        self,
        audio_bytes: bytes,
        mime_type: str = "audio/webm",
        language: str = "zh",
    ) -> str:
        raise NotImplementedError


class BrowserSpeechToText(SpeechToText):
    name = "browser_web_speech"

    def transcribe(
        self,
        audio_bytes: bytes,
        mime_type: str = "audio/webm",
        language: str = "zh",
    ) -> str:
        raise RuntimeError(
            "BrowserSpeechToText runs on the phone. Send the transcript as text."
        )


class WhisperSpeechToText(SpeechToText):
    name = "faster_whisper"

    def __init__(self, model_size: str = "small") -> None:
        self.model_size = model_size
        self._model = None
        self._lock = threading.Lock()
        self.ready = False
        self.error = ""

    def preload(self) -> None:
        self._ensure_model()

    def _ensure_model(self):
        if self._model is not None:
            return self._model
        with self._lock:
            if self._model is not None:
                return self._model
            _disable_proxy_for_local_model()
            from faster_whisper import WhisperModel

            try:
                self._model = WhisperModel(
                    self.model_size,
                    device="cpu",
                    compute_type="int8",
                    local_files_only=True,
                )
            except Exception:
                self._model = WhisperModel(
                    self.model_size,
                    device="cpu",
                    compute_type="int8",
                )
            self.ready = True
            self.error = ""
            return self._model

    def transcribe(
        self,
        audio_bytes: bytes,
        mime_type: str = "audio/webm",
        language: str = "zh",
    ) -> str:
        if not audio_bytes:
            return ""
        _disable_proxy_for_local_model()
        model = self._ensure_model()
        suffix = _suffix_for_mime(mime_type)
        with tempfile.TemporaryDirectory(prefix="x5a-asr-") as tmp:
            src = Path(tmp) / f"clip{suffix}"
            wav = Path(tmp) / "clip.wav"
            src.write_bytes(audio_bytes)
            if suffix != ".wav":
                proc = subprocess.run(
                    [
                        "ffmpeg",
                        "-hide_banner",
                        "-loglevel",
                        "error",
                        "-y",
                        "-i",
                        str(src),
                        "-ac",
                        "1",
                        "-ar",
                        "16000",
                        str(wav),
                    ],
                    capture_output=True,
                    text=True,
                )
                if proc.returncode != 0 or not wav.is_file():
                    raise RuntimeError(
                        proc.stderr.strip() or "ffmpeg could not decode uploaded audio"
                    )
            else:
                wav = src
            segments, _info = model.transcribe(
                str(wav),
                language=language or "zh",
                vad_filter=True,
                beam_size=5,
            )
            return "".join(segment.text for segment in segments).strip()


def _suffix_for_mime(mime_type: str) -> str:
    name = (mime_type or "").lower()
    if "wav" in name:
        return ".wav"
    if "mp4" in name or "m4a" in name or "aac" in name:
        return ".m4a"
    if "ogg" in name or "opus" in name:
        return ".ogg"
    if "mpeg" in name or "mp3" in name:
        return ".mp3"
    return ".webm"


def create_asr(backend: str = "whisper") -> SpeechToText:
    name = (backend or "whisper").strip().lower()
    if name in {"browser", "web_speech", "browser_web_speech"}:
        return BrowserSpeechToText()
    if name in {"whisper", "faster-whisper", "faster_whisper", "local"}:
        return WhisperSpeechToText()
    raise ValueError(f"unknown ASR backend: {backend!r}")

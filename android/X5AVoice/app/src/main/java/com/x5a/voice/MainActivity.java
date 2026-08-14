package com.x5a.voice;

import android.Manifest;
import android.content.pm.PackageManager;
import android.media.MediaRecorder;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.os.SystemClock;
import android.view.MotionEvent;
import android.widget.TextView;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import org.json.JSONObject;

import java.io.File;


public class MainActivity extends AppCompatActivity {
    private static final int REQ_MIC = 17;
    private static final String[] TERMINAL = {"SUCCESS", "FAILED", "IDLE", "DRY_RUN"};

    private final RobotClient client = new RobotClient();
    private final Handler handler = new Handler(Looper.getMainLooper());

    private TextView linkBadge;
    private TextView hostText;
    private TextView busyBadge;
    private TextView stageText;
    private TextView resultText;
    private TextView transcriptText;
    private TextView parsedText;
    private TextView speechStatus;
    private TextView holdBtn;

    private MediaRecorder recorder;
    private File recordFile;
    private long holdStart;
    private boolean holding;
    private final Runnable pollTask = new Runnable() {
        @Override
        public void run() {
            client.pollState(new RobotClient.Callback() {
                @Override
                public void onOk(JSONObject json) {
                    applyState(json);
                }

                @Override
                public void onError(String message) {
                    linkBadge.setText("已断开");
                    handler.postDelayed(pollTask, 1500);
                }
            });
            handler.postDelayed(this, 800);
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        linkBadge = findViewById(R.id.linkBadge);
        hostText = findViewById(R.id.hostText);
        busyBadge = findViewById(R.id.busyBadge);
        stageText = findViewById(R.id.stageText);
        resultText = findViewById(R.id.resultText);
        transcriptText = findViewById(R.id.transcriptText);
        parsedText = findViewById(R.id.parsedText);
        speechStatus = findViewById(R.id.speechStatus);
        holdBtn = findViewById(R.id.holdBtn);

        findViewById(R.id.btnRed).setOnClickListener(v -> sendColor("red"));
        findViewById(R.id.btnWhite).setOnClickListener(v -> sendColor("white"));
        findViewById(R.id.btnOrange).setOnClickListener(v -> sendColor("orange"));
        findViewById(R.id.btnStop).setOnClickListener(v -> client.cancel(toastCb("已请求停止")));

        holdBtn.setOnTouchListener((v, event) -> {
            if (event.getAction() == MotionEvent.ACTION_DOWN) {
                v.performClick();
                startHold();
                return true;
            }
            if (event.getAction() == MotionEvent.ACTION_UP
                    || event.getAction() == MotionEvent.ACTION_CANCEL) {
                endHold();
                return true;
            }
            return false;
        });

        connect();
    }

    private void connect() {
        linkBadge.setText("连接中");
        stageText.setText("正在连接 192.168.0.50");
        client.discover(new RobotClient.Callback() {
            @Override
            public void onOk(JSONObject json) {
                String base = json.optString("base", client.baseUrl());
                hostText.setText(base);
                linkBadge.setText("已连接");
                stageText.setText(json.optBoolean("task_server", true) ? "空闲，等待指令" : "网页已连上，Task Server 未就绪");
                handler.removeCallbacks(pollTask);
                handler.post(pollTask);
            }

            @Override
            public void onError(String message) {
                linkBadge.setText("未连接");
                stageText.setText("连不上主机");
                resultText.setText(message);
                resultText.setTextColor(getColor(R.color.bad));
                handler.postDelayed(MainActivity.this::connect, 2000);
            }
        });
    }

    private void applyState(JSONObject state) {
        linkBadge.setText("已连接");
        String stage = state.optString("stage", "IDLE");
        String stageZh = state.optString("stage_zh", stage);
        if (!stageZh.isEmpty()) {
            stageText.setText(stageZh);
        }
        boolean terminal = isTerminal(stage);
        boolean busy = !terminal && (state.optBoolean("busy", false)
                || "BUSY".equals(state.optString("robot_state")));
        busyBadge.setText(busy ? "忙碌" : "空闲");
        busyBadge.setTextColor(getColor(busy ? R.color.accent : R.color.ok));
        String transcript = state.optString("transcript", "");
        if (!transcript.isEmpty() && !holding) {
            transcriptText.setText(transcript);
        }
        JSONObject parsed = state.optJSONObject("parsed");
        if (parsed != null) {
            parsedText.setText(parsed.toString());
        }
        JSONObject last = state.optJSONObject("last_result");
        if (last != null && last.has("message")) {
            boolean ok = last.optBoolean("success", false);
            resultText.setText(last.optString("message"));
            resultText.setTextColor(getColor(ok ? R.color.ok : R.color.bad));
        }
    }

    private boolean isTerminal(String stage) {
        for (String item : TERMINAL) {
            if (item.equals(stage)) {
                return true;
            }
        }
        return false;
    }

    private void startHold() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.RECORD_AUDIO)
                != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this, new String[]{Manifest.permission.RECORD_AUDIO}, REQ_MIC);
            return;
        }
        holding = true;
        holdStart = SystemClock.elapsedRealtime();
        holdBtn.setBackgroundResource(R.drawable.hold_btn_hot);
        holdBtn.setText("松开发送");
        holdBtn.setTextColor(0xFFFFFFFF);
        speechStatus.setText("正在录音…");
        speechStatus.setTextColor(getColor(R.color.bad));
        transcriptText.setText("正在录音…");
        try {
            recordFile = new File(getCacheDir(), "speech.m4a");
            if (recordFile.exists()) {
                //noinspection ResultOfMethodCallIgnored
                recordFile.delete();
            }
            recorder = new MediaRecorder();
            recorder.setAudioSource(MediaRecorder.AudioSource.MIC);
            recorder.setOutputFormat(MediaRecorder.OutputFormat.MPEG_4);
            recorder.setAudioEncoder(MediaRecorder.AudioEncoder.AAC);
            recorder.setAudioSamplingRate(16000);
            recorder.setAudioEncodingBitRate(64000);
            recorder.setOutputFile(recordFile.getAbsolutePath());
            recorder.prepare();
            recorder.start();
        } catch (Exception e) {
            holding = false;
            resetHold();
            speechStatus.setText("无法开始录音：" + e.getMessage());
        }
    }

    private void endHold() {
        if (!holding) {
            return;
        }
        holding = false;
        long elapsed = SystemClock.elapsedRealtime() - holdStart;
        try {
            if (recorder != null) {
                recorder.stop();
                recorder.release();
            }
        } catch (Exception ignored) {
        }
        recorder = null;
        resetHold();
        if (elapsed < 500 || recordFile == null || !recordFile.exists() || recordFile.length() < 800) {
            speechStatus.setText("按太短了，请按住说完整一句");
            transcriptText.setText("按住说话，松手发送");
            return;
        }
        speechStatus.setText("正在用主机模型识别…");
        transcriptText.setText("正在识别…");
        client.uploadAudio(recordFile, new RobotClient.Callback() {
            @Override
            public void onOk(JSONObject json) {
                String text = json.optString("text", "").trim();
                if (text.isEmpty()) {
                    speechStatus.setText(json.optString("message", "没有听清，请再说一次"));
                    transcriptText.setText("没有听清");
                    return;
                }
                transcriptText.setText(text);
                speechStatus.setText("识别到：" + text);
                speechStatus.setTextColor(getColor(R.color.ok));
                client.sendText(text, resultCb());
            }

            @Override
            public void onError(String message) {
                speechStatus.setText("识别失败：" + message);
            }
        });
    }

    private void resetHold() {
        holdBtn.setBackgroundResource(R.drawable.hold_btn);
        holdBtn.setText("按住 说话");
        holdBtn.setTextColor(0xFF1A1204);
        speechStatus.setTextColor(getColor(R.color.muted));
    }

    private void sendColor(String color) {
        transcriptText.setText("手动：" + color);
        client.sendColor(color, resultCb());
    }

    private RobotClient.Callback resultCb() {
        return new RobotClient.Callback() {
            @Override
            public void onOk(JSONObject json) {
                if (json.has("parsed")) {
                    parsedText.setText(String.valueOf(json.opt("parsed")));
                }
                if (json.has("message")) {
                    resultText.setText(json.optString("message"));
                    resultText.setTextColor(getColor(
                            json.optBoolean("ok", json.optBoolean("success", false))
                                    ? R.color.ok : R.color.bad));
                }
            }

            @Override
            public void onError(String message) {
                resultText.setText(message);
                resultText.setTextColor(getColor(R.color.bad));
            }
        };
    }

    private RobotClient.Callback toastCb(String fallback) {
        return new RobotClient.Callback() {
            @Override
            public void onOk(JSONObject json) {
                Toast.makeText(MainActivity.this, json.optString("message", fallback), Toast.LENGTH_SHORT).show();
            }

            @Override
            public void onError(String message) {
                Toast.makeText(MainActivity.this, message, Toast.LENGTH_SHORT).show();
            }
        };
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == REQ_MIC && grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            Toast.makeText(this, "已授权，再按住说话", Toast.LENGTH_SHORT).show();
        }
    }

    @Override
    protected void onDestroy() {
        handler.removeCallbacks(pollTask);
        if (recorder != null) {
            try {
                recorder.release();
            } catch (Exception ignored) {
            }
        }
        super.onDestroy();
    }
}

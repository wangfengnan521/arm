package com.x5a.voice;

import android.os.Handler;
import android.os.Looper;

import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.ByteArrayOutputStream;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.security.SecureRandom;
import java.security.cert.X509Certificate;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import javax.net.ssl.HostnameVerifier;
import javax.net.ssl.HttpsURLConnection;
import javax.net.ssl.SSLContext;
import javax.net.ssl.TrustManager;
import javax.net.ssl.X509TrustManager;

/**
 * Talks only to the existing FastAPI agent. Never publishes /arm_cmd.
 */
final class RobotClient {
    interface Callback {
        void onOk(JSONObject json);
        void onError(String message);
    }

    static final String[] CANDIDATES = new String[] {
            "http://192.168.0.50:8000",
            "https://192.168.0.50:8000",
            "https://192.168.0.50:8443"
    };

    private final ExecutorService io = Executors.newCachedThreadPool();
    private final Handler main = new Handler(Looper.getMainLooper());
    private String baseUrl = CANDIDATES[0];

    RobotClient() {
        trustSelfSignedForDemo();
    }

    String baseUrl() {
        return baseUrl;
    }

    void discover(Callback cb) {
        io.execute(() -> {
            Exception last = null;
            for (String candidate : CANDIDATES) {
                try {
                    JSONObject json = get(candidate + "/api/health");
                    baseUrl = candidate;
                    JSONObject out = json == null ? new JSONObject() : json;
                    out.put("base", candidate);
                    postMain(cb, out, null);
                    return;
                } catch (Exception e) {
                    last = e;
                }
            }
            postMain(cb, null, last == null ? "连不上主机" : last.getMessage());
        });
    }

    void pollState(Callback cb) {
        io.execute(() -> {
            try {
                postMain(cb, get(baseUrl + "/api/state"), null);
            } catch (Exception e) {
                postMain(cb, null, e.getMessage());
            }
        });
    }

    void sendText(String text, Callback cb) {
        io.execute(() -> {
            try {
                JSONObject body = new JSONObject();
                body.put("text", text);
                postMain(cb, postJson(baseUrl + "/api/command", body), null);
            } catch (Exception e) {
                postMain(cb, null, e.getMessage());
            }
        });
    }

    void sendColor(String color, Callback cb) {
        io.execute(() -> {
            try {
                JSONObject body = new JSONObject();
                body.put("action", "pick_place");
                body.put("target_color", color);
                postMain(cb, postJson(baseUrl + "/api/task", body), null);
            } catch (Exception e) {
                postMain(cb, null, e.getMessage());
            }
        });
    }

    void cancel(Callback cb) {
        io.execute(() -> {
            try {
                postMain(cb, postJson(baseUrl + "/api/cancel", new JSONObject()), null);
            } catch (Exception e) {
                postMain(cb, null, e.getMessage());
            }
        });
    }

    void uploadAudio(File file, Callback cb) {
        io.execute(() -> {
            try {
                postMain(cb, postMultipart(baseUrl + "/api/asr", file), null);
            } catch (Exception e) {
                postMain(cb, null, e.getMessage());
            }
        });
    }

    private void postMain(Callback cb, JSONObject json, String error) {
        main.post(() -> {
            if (error != null) {
                cb.onError(error);
            } else {
                cb.onOk(json == null ? new JSONObject() : json);
            }
        });
    }

    private JSONObject get(String url) throws Exception {
        HttpURLConnection conn = open(url);
        conn.setRequestMethod("GET");
        conn.setConnectTimeout(2500);
        conn.setReadTimeout(4000);
        return readJson(conn);
    }

    private JSONObject postJson(String url, JSONObject body) throws Exception {
        byte[] raw = body.toString().getBytes(StandardCharsets.UTF_8);
        HttpURLConnection conn = open(url);
        conn.setRequestMethod("POST");
        conn.setDoOutput(true);
        conn.setConnectTimeout(4000);
        conn.setReadTimeout(180000);
        conn.setRequestProperty("Content-Type", "application/json; charset=utf-8");
        conn.getOutputStream().write(raw);
        return readJson(conn);
    }

    private JSONObject postMultipart(String url, File file) throws Exception {
        String boundary = "----x5a" + System.currentTimeMillis();
        HttpURLConnection conn = open(url);
        conn.setRequestMethod("POST");
        conn.setDoOutput(true);
        conn.setConnectTimeout(4000);
        conn.setReadTimeout(180000);
        conn.setRequestProperty("Content-Type", "multipart/form-data; boundary=" + boundary);
        DataOutputStream out = new DataOutputStream(conn.getOutputStream());
        out.writeBytes("--" + boundary + "\r\n");
        out.writeBytes("Content-Disposition: form-data; name=\"audio\"; filename=\"speech.m4a\"\r\n");
        out.writeBytes("Content-Type: audio/mp4\r\n\r\n");
        FileInputStream in = new FileInputStream(file);
        byte[] buf = new byte[4096];
        int n;
        while ((n = in.read(buf)) > 0) {
            out.write(buf, 0, n);
        }
        in.close();
        out.writeBytes("\r\n--" + boundary + "--\r\n");
        out.flush();
        out.close();
        return readJson(conn);
    }

    private HttpURLConnection open(String url) throws Exception {
        return (HttpURLConnection) new URL(url).openConnection();
    }

    private JSONObject readJson(HttpURLConnection conn) throws Exception {
        int code = conn.getResponseCode();
        InputStream stream = code >= 400 ? conn.getErrorStream() : conn.getInputStream();
        if (stream == null) {
            throw new IllegalStateException("HTTP " + code);
        }
        BufferedReader reader = new BufferedReader(new InputStreamReader(stream, StandardCharsets.UTF_8));
        StringBuilder sb = new StringBuilder();
        String line;
        while ((line = reader.readLine()) != null) {
            sb.append(line);
        }
        reader.close();
        conn.disconnect();
        if (sb.length() == 0) {
            return new JSONObject();
        }
        return new JSONObject(sb.toString());
    }

    private static void trustSelfSignedForDemo() {
        try {
            TrustManager[] trustAll = new TrustManager[] {
                    new X509TrustManager() {
                        public void checkClientTrusted(X509Certificate[] c, String a) {}
                        public void checkServerTrusted(X509Certificate[] c, String a) {}
                        public X509Certificate[] getAcceptedIssuers() { return new X509Certificate[0]; }
                    }
            };
            SSLContext sc = SSLContext.getInstance("TLS");
            sc.init(null, trustAll, new SecureRandom());
            HttpsURLConnection.setDefaultSSLSocketFactory(sc.getSocketFactory());
            HostnameVerifier all = (h, s) -> true;
            HttpsURLConnection.setDefaultHostnameVerifier(all);
        } catch (Exception ignored) {
        }
    }
}

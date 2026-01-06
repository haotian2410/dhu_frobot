import streamlit as st
import cv2
import time
import numpy as np
import requests
from datetime import datetime
from streamlit_autorefresh import st_autorefresh

# ------------------------------
# Config
# ------------------------------
ROBOT_API = "http://127.0.0.1:8000"

def call_api(path, params=None, timeout=5):
    try:
        r = requests.get(f"{ROBOT_API}{path}", params=params, timeout=timeout)
        return r.json()
    except Exception as e:
        return {"ok": False, "error": str(e)}

# ------------------------------
# Streamlit page
# ------------------------------
st.set_page_config(page_title="Robot Flower UI", layout="wide")

# ------------------------------
# Session state init
# ------------------------------
if "vlm_logs" not in st.session_state:
    st.session_state.vlm_logs = []
if "is_running" not in st.session_state:
    st.session_state.is_running = False
if "cap" not in st.session_state:
    st.session_state.cap = None

# ------------------------------
# Auto refresh when running
# ------------------------------
if st.session_state.is_running:
    st_autorefresh(interval=1000, key="refresh")  # 每 1 秒刷新一次

# ------------------------------
# Sidebar
# ------------------------------
with st.sidebar:
    st.title("⚙️ Control Panel")

    # status query
    status_resp = call_api("/status")
    if status_resp.get("ok", True):
        busy = status_resp.get("busy", False)
        inited = status_resp.get("inited", False)
        stop_flag = status_resp.get("stop", False)
        st.write(f"✅ inited: **{inited}**")
        st.write(f"⏳ busy: **{busy}**")
        st.write(f"❓ stop: **{stop_flag}**")
    else:
        st.warning("Status API not reachable")

    st.divider()

    st.write("Device State: ", "⚡⚡ Running" if st.session_state.is_running else "⚪ Idle")

    if st.button(" Init System", use_container_width=True):
        st.session_state.vlm_logs.append(f"[{datetime.now().strftime('%H:%M:%S')}] Init request...")
        resp = call_api("/init", timeout=60)  # 初始化可能很久
        if resp.get("ok"):
            st.session_state.vlm_logs.append(f"[{datetime.now().strftime('%H:%M:%S')}] ✅ Init success")
            st.session_state.is_running = True

            # camera init
            if st.session_state.cap is None:
                st.session_state.cap = cv2.VideoCapture(0)

        else:
            st.session_state.vlm_logs.append(f"[{datetime.now().strftime('%H:%M:%S')}] ❌ Init failed: {resp.get('error')}")
            st.session_state.is_running = False

    if st.button("❓ Emergency Stop", type="primary", use_container_width=True):
        call_api("/stop")
        st.session_state.is_running = False
        st.session_state.vlm_logs.append(f"[{datetime.now().strftime('%H:%M:%S')}] ⚪⚪ Emergency stop!")
        st.error("Emergency stop triggered!")

# ------------------------------
# Main UI layout
# ------------------------------
col1, col2 = st.columns([2, 1])

# ------------------------------
# Camera view
# ------------------------------
with col1:
    st.subheader(" Live Monitor")
    video_placeholder = st.empty()

    if st.session_state.is_running:
        try:
            r = requests.get(f"{ROBOT_API}/frame", timeout=1)
            if r.status_code == 200:
                jpg = np.frombuffer(r.content, dtype=np.uint8)
                frame = cv2.imdecode(jpg, cv2.IMREAD_COLOR)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                video_placeholder.image(frame, channels="RGB", use_container_width=True)
            else:
                st.warning("Camera frame not available")
        except Exception as e:
            st.warning(f"Camera error: {e}")
    else:
        video_placeholder.image("https://via.placeholder.com/1280x800.png?text=Camera+Offline")

# ------------------------------
# Command UI
# ------------------------------
with col2:
    st.subheader(" Task Command")

    user_input = st.text_input("Input instruction (e.g. put rose into vase)", key="text_input")

    if st.button("Send Command", use_container_width=True):
        if not user_input:
            st.warning("Please input a command.")
        else:
            st.session_state.vlm_logs.append(f"[{datetime.now().strftime('%H:%M:%S')}] User: {user_input}")
            resp = call_api("/run", {"text": user_input}, timeout=5)

            if resp.get("ok"):
                st.session_state.vlm_logs.append(f"[{datetime.now().strftime('%H:%M:%S')}] ✅ Command sent")
            else:
                st.session_state.vlm_logs.append(f"[{datetime.now().strftime('%H:%M:%S')}] ❌ Send failed: {resp.get('error')}")

# ------------------------------
# Fetch logs (PopLogs: incremental)
# ------------------------------
if st.session_state.is_running:
    resp = call_api("/logs")
    logs = resp.get("logs", [])
    if isinstance(logs, list) and len(logs) > 0:
        for line in logs:
            st.session_state.vlm_logs.append(line)

# ------------------------------
# Log panel
# ------------------------------
st.divider()
st.subheader(" VLM Task Logs")

log_container = st.container(height=300)
with log_container:
    for log in reversed(st.session_state.vlm_logs[-200:]):  # 只显示最近 200 行，防止卡
        st.text(log)

# ------------------------------
# UI Styling
# ------------------------------
st.markdown("""
    <style>
    .stButton button {
        border-radius: 6px;
    }
    .stTextInput input {
        border-radius: 6px;
    }
    </style>
""", unsafe_allow_html=True)

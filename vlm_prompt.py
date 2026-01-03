import os
import json
import base64
import requests

# -----------------------------
# 统一项目根目录
# -----------------------------
BASE = os.environ.get("ROBOT_HOME", ".")
promptall_path = os.path.join(BASE, "Data", "Prompt", "promptall.txt")
order_path     = os.path.join(BASE, "Data", "Prompt", "order.txt")
quest_path     = os.path.join(BASE, "Data", "Prompt", "quest.txt")

# 你项目里拍照保存的位置（看你 grep 里是 Data/Capture/flower.jpg）
image_path     = os.path.join(BASE, "Data", "Capture", "flower.jpg")

# 输出：scan.json（你的 C++ 会读这个）
out_json_path  = os.path.join(BASE, "Result", "VLM", "scan.json")
os.makedirs(os.path.dirname(out_json_path), exist_ok=True)

# -----------------------------
# 读取 system prompt / order / quest
# -----------------------------
with open(promptall_path, "r", encoding="utf-8") as f:
    system_prompt = f.read()

with open(order_path, "r", encoding="utf-8") as t:
    system_order = t.read()

with open(quest_path, "r", encoding="utf-8") as t:
    system_quest = t.read()

# -----------------------------
# 读图转 base64 (data url)
# -----------------------------
with open(image_path, "rb") as f:
    b64 = base64.b64encode(f.read()).decode("utf-8")
image_data_url = f"data:image/jpeg;base64,{b64}"

# -----------------------------
# 调 OpenAI (HTTP)
# -----------------------------
api_key = "1c914b4d-3447-4b57-b53c-c04bf7bf9f2b"

headers = {
    "Authorization": f"Bearer {api_key}",
    "Content-Type": "application/json",
}

# 你原来怎么组织 prompt，这里给你一个合理结构：
# - system: promptall.txt
# - user: 组合 quest + order + 图片
user_text = f"{system_quest}\n\n{system_order}".strip()

payload = {
    "model": "gpt-4o-mini",
    "messages": [
        {"role": "system", "content": system_prompt},
        {"role": "user", "content": [
            {"type": "text", "text": user_text},
            {"type": "image_url", "image_url": {"url": image_data_url}},
        ]},
    ],
    "temperature": 0.2,
}

resp = requests.post(
    "https://api.openai.com/v1/chat/completions",
    headers=headers,
    data=json.dumps(payload),
    timeout=90,
)
resp.raise_for_status()

data = resp.json()
content = data["choices"][0]["message"]["content"]

# -----------------------------
# 把模型输出写成 scan.json
# -----------------------------
# 你的 C++ 里是 json::parse(file) ，所以这里必须落盘为合法 JSON
# 如果模型返回的是 JSON 字符串，这里解析一下再 dump
try:
    result = json.loads(content)
except json.JSONDecodeError:
    # 兜底：有时模型会带 ```json ... ```
    cleaned = content.strip()
    if cleaned.startswith("```"):
        cleaned = cleaned.split("\n", 1)[-1]
        if cleaned.endswith("```"):
            cleaned = cleaned.rsplit("\n", 1)[0]
    result = json.loads(cleaned)

with open(out_json_path, "w", encoding="utf-8") as f:
    json.dump(result, f, ensure_ascii=False, indent=2)

print("Wrote:", out_json_path)

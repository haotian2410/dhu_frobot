import os
import json
import base64
import requests

# =============================
# 配置
# =============================
API_KEY = "1c914b4d-3447-4b57-b53c-c04bf7bf9f2b"
BASE_URL = "https://ark.cn-beijing.volces.com/api/v3"
MODEL_ID = "doubao-seed-1-6-vision-250815"

BASE = os.environ.get("ROBOT_HOME", ".")
image_path = os.path.join(BASE, "Data", "Capture", "flower.jpg")
output_file_path = os.path.join(BASE, "Result", "VLM", "judge.txt")
os.makedirs(os.path.dirname(output_file_path), exist_ok=True)

# =============================
# 图片转 Base64
# =============================
def encode_image(path):
    with open(path, "rb") as f:
        return base64.b64encode(f.read()).decode("utf-8")

base64_image = encode_image(image_path)

# =============================
# 构造请求
# =============================
headers = {
    "Authorization": f"Bearer {API_KEY}",
    "Content-Type": "application/json",
}

payload = {
    "model": MODEL_ID,
    "messages": [
        {
            "role": "user",
            "content": [
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/jpeg;base64,{base64_image}"
                    }
                },
                {
                    "type": "text",
                    "text": "图中花枝是否在图像的中心区域？如果是则回答‘0’，偏左回答‘1’，偏右回答‘2’"
                }
            ]
        }
    ],
    "extra_body": {
        "thinking": {
            "type": "disabled"
        }
    }
}

# =============================
# 发送请求
# =============================
resp = requests.post(
    f"{BASE_URL}/chat/completions",
    headers=headers,
    data=json.dumps(payload),
    timeout=60
)

resp.raise_for_status()
data = resp.json()

# =============================
# 解析结果
# =============================
result_content = data["choices"][0]["message"]["content"]

# =============================
# 写文件
# =============================
with open(output_file_path, "w", encoding="utf-8") as f:
    f.write(result_content)

print(f"vlm识别结果已成功保存到: {output_file_path}")
print("vlm模型返回结果:")
print(result_content)

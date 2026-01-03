from openai import OpenAI
import base64
import json

client = OpenAI(
    api_key="1c914b4d-3447-4b57-b53c-c04bf7bf9f2b",
    base_url="https://ark.cn-beijing.volces.com/api/v3",
)

# --------------------------------------
# 读取 system prompt（你想要的“从文件中读”）
# --------------------------------------
with open(r"C:\Program\Project\Robot\Data\Prompt\promptall.txt", "r", encoding="utf-8") as f:
    system_prompt = f.read()

# 图片路径
image_path = r"C:\Program\Project\Robot\Data\Capture\flower.jpg"
txt_path = r"C:\Program\Project\Robot\Data\Prompt\order.txt"

with open(r"C:\Program\Project\Robot\Data\Prompt\order.txt", "r", encoding="utf-8") as t:
    system_order = t.read()

def encode_image(path):
    with open(path, "rb") as f:
        return base64.b64encode(f.read()).decode("utf-8")

base64_image = encode_image(image_path)

# --------------------------------------
# 正确格式的 messages
#  system 和 user 分开写
# --------------------------------------
messages = [
    {
        "role": "system",
        "content": [
            {"type": "text", "text": system_prompt}
        ]
    },
    {
        "role": "user",
        "content": [
            {
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/jpg;base64,{base64_image}"
                }
            },
            {"type": "text", "text": system_order}
        ]
    }
]

response = client.chat.completions.create(
    model="doubao-seed-1-6-vision-250815",
    messages=messages,
    extra_body={
        "thinking": {"type": "disabled"}
    }
)

result_content = response.choices[0].message.content

# 保存到文件
output_file_path = r"C:\Program\Project\Robot\Result\VLM\scan.json"
data = json.loads(result_content)

# 保存到 JSON 文件
try:
    with open(output_file_path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=4)
    print(f"vlm识别结果已成功保存到: {output_file_path}")
except Exception as e:
    print(f"保存文件时出错: {e}")

print("已保存到:", output_file_path)
print("vlm模型返回结果:")
print(result_content)

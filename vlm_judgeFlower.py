from openai import OpenAI
import base64

client = OpenAI(
    # 从环境变量中获取方舟 API Key
    api_key="1c914b4d-3447-4b57-b53c-c04bf7bf9f2b",
    # api_key = "54b3d8a4-3ea9-4c62-b1f1-50446fe1b83b",
    base_url = "https://ark.cn-beijing.volces.com/api/v3",
)

# 需传给大模型的图片
import os
BASE = os.environ.get("ROBOT_HOME", ".")
image_path = os.path.join(BASE, "Data", "Capture", "flower.jpg")
output_file_path = os.path.join(BASE, "Result", "VLM", "judge.txt")


# 定义方法将指定路径图片转为Base64编码
def encode_image(image_path):
  with open(image_path, "rb") as image_file:
    return base64.b64encode(image_file.read()).decode('utf-8')

# 将图片转为Base64编码
base64_image = encode_image(image_path)

response = client.chat.completions.create(
    # 指定您创建的方舟推理接入点 ID，此处已帮您修改为您的推理接入点 ID
    model="doubao-seed-1-6-vision-250815",
    messages=[
        {
            "role": "user",
            "content": [
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/jpg;base64,{base64_image}"
                    },
                },
                {"type": "text", "text": "图中花枝是否在图像的中心区域？如果是则回答‘0’，偏左回答‘1’，偏右回答‘2’"},
            ],
        }
    ],
    extra_body={
         "thinking": {
             "type": "disabled" # 不使用深度思考能力
             # "type": "enabled" # 使用深度思考能力
         }
    },
)
# 获取响应内容
result_content = response.choices[0].message.content

# 将结果保存到txt文件
try:
    with open(output_file_path, "w", encoding="utf-8") as file:
        file.write(result_content)
    print(f"vlm识别结果已成功保存到: {output_file_path}")
except Exception as e:
    print(f"保存文件时出错: {e}")

# 同时在控制台输出结果
print("vlm模型返回结果:")
print(result_content)
#!/usr/bin/env python3
import edge_tts

VOICE = "zh-CN-XiaorouNeural"
INPUT_FILE = r"..\Data\Word2Voice\word\words.txt"
OUTPUT_FILE = r"..\Result\Word2Voice\voice\voice.mp3"


def convertion(content) -> None:
    """Main function"""
    communicate = edge_tts.Communicate(content, VOICE)
    communicate.save_sync(OUTPUT_FILE)

if __name__ == "__main__":
    with open(INPUT_FILE, 'r', encoding='utf-8') as file:
        content = file.read()
    convertion(content)

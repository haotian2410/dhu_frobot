#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <locale>
#include <codecvt>
#include <SFML/Audio.hpp>
#include <SFML/Window/Keyboard.hpp>
#include <cstdlib>

using namespace std;
using namespace sf;


int mian() {
    if (!sf::SoundBufferRecorder::isAvailable())
    {
        // error: audio capture is not available on this system
    }

    cout << "按回车开始录音..." << endl;
    cin.get();

    sf::SoundBufferRecorder recorder;

    // start the capture
    recorder.start();

    cout << "正在录音，按回车停止..." << endl;
    cin.get();
    
    recorder.stop();

    // retrieve the buffer that contains the captured audio data
    const sf::SoundBuffer& buffer = recorder.getBuffer();

    //buffer.saveToFile("my_record.wav");

    sf::Sound sound(buffer);

    sound.play();
    cin.get();
    cout << buffer.saveToFile("C:/Program/Project/Robot/Data/VoiceConvertion/voice/voice.wav");

    return 0;
}

int voice2words() {
    string cmd = "C:\\Program\\Libraries\\whisper\\whisper-cli.exe "
                         "-m \"C:\\Program\\Libraries\\whisper\\ggml-medium.bin\" "
                         "\"C:\\Program\\Project\\Robot\\Data\\VoiceConvertion\\voice\\voice.mp3\" "
                         "-osrt -l Chinese";
    system(cmd.c_str());

    wifstream fin;

    // 设置UTF-8 locale
    fin.imbue(locale(fin.getloc(), new codecvt_utf8<wchar_t>));

    fin.open("C:/Program/Project/Robot/Data/VoiceConvertion/voice/voice.mp3.srt");
    wstring line;
    int count = 0;
    vector<wstring> text;

    while (getline(fin, line))
    {
        count++;
        if (count == 3)
            text.push_back(line);
        if (count == 4)
            count = 0;
    }
    fin.close();
    // 设置控制台输出为UTF-8
    locale::global(locale(""));
    wcout.imbue(locale());

    wofstream fout;
    fout.imbue(locale(fout.getloc(), new codecvt_utf8<wchar_t>));
    fout.open("C:/Program/Project/Robot/Data/VoiceConvertion/words/voice.txt");
    for (auto s : text) 
        fout << s << endl;
    fout.close();
    return 0;
}
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class SoundResponseProfile
{
    public SoundType soundType;
    public float activationThreshold = 0.3f;
    
    [Header("Visual Response")]
    public bool enableHUD = true;
    public bool enableLED = true;
    public Color ledColor = Color.white;  // 새로 추가: LED 색상
    
    [Header("Timing")]
    public float hudBlinkSpeed = 1.0f;
    public float ledBlinkSpeed = 1.0f;
    public float ledTimerSpeed = 1.0f;

    // 방향별 임계값
    public Dictionary<string, float> directionThresholds = new Dictionary<string, float>
    {
        ["behind"] = 0.4f,
        ["behind-left"] = 0.5f,
        ["behind-right"] = 0.5f,
        ["to the left"] = 0.6f,
        ["to the right"] = 0.6f,
        ["ahead"] = 0.7f,
        ["ahead-left"] = 0.7f,
        ["ahead-right"] = 0.7f
    };

    // 볼륨 레벨별 응답
    public Dictionary<string, VolumeResponseLevel> volumeLevels = new Dictionary<string, VolumeResponseLevel>
    {
        ["none"] = new VolumeResponseLevel("none", 0f, 0.2f, false, false),
        ["low"] = new VolumeResponseLevel("low", 0.2f, 0.5f, true, false),
        ["medium"] = new VolumeResponseLevel("medium", 0.5f, 0.8f, true, true),
        ["high"] = new VolumeResponseLevel("high", 0.8f, 1.0f, true, true)
    };

    // HUD 매핑
    public Dictionary<string, string[]> hudMapping = new Dictionary<string, string[]>();

    public SoundResponseProfile()
    {
        // 기본값들은 위에서 설정
    }

    public SoundResponseProfile(SoundType type, Color color)
    {
        soundType = type;
        ledColor = color;
    }
}

[System.Serializable]
public class VolumeResponseLevel
{
    public string name;
    public float minVolume;
    public float maxVolume;
    public bool showHUD;
    public bool activateLED;

    public VolumeResponseLevel(string name, float minVolume, float maxVolume, bool showHUD, bool activateLED)
    {
        this.name = name;
        this.minVolume = minVolume;
        this.maxVolume = maxVolume;
        this.showHUD = showHUD;
        this.activateLED = activateLED;
    }

    public bool IsInRange(float volume)
    {
        return volume >= minVolume && volume <= maxVolume;
    }
}
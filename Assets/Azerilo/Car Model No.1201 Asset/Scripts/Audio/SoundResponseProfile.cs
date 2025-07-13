using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class SoundResponseProfile
{
  [Header("Basic Settings")]
  public SoundType soundType = SoundType.Ambulance;
  public float activationThreshold = 0.3f;

  [Header("Visual Response Settings")]
  public bool enableHUD = true;
  public bool enableLED = true;
  public float hudBlinkSpeed = 1.0f;
  public float ledBlinkSpeed = 1.0f;
  public float ledTimerSpeed = 1.0f;

  [Header("Direction-based Thresholds")]
  public Dictionary<string, float> directionThresholds;

  [Header("Volume Response Levels")]
  public Dictionary<string, VolumeResponseLevel> volumeLevels;

  [Header("HUD Mapping")]
  public Dictionary<string, string[]> hudMapping;

  public SoundResponseProfile()
  {
    InitializeDefaults();
  }

  private void InitializeDefaults()
  {
    // 방향별 임계값 초기화
    directionThresholds = new Dictionary<string, float>
    {
      ["behind"] = 0.85f,
      ["behind-left"] = 0.90f,
      ["behind-right"] = 0.90f,
      ["to the left"] = 0.90f,
      ["to the right"] = 0.90f,
      ["ahead"] = 0.95f,
      ["ahead-left"] = 0.95f,
      ["ahead-right"] = 0.95f
    };

    // 볼륨 레벨 초기화
    volumeLevels = new Dictionary<string, VolumeResponseLevel>
    {
      ["none"] = new VolumeResponseLevel("None", 0.0f, 0.3f, false, false),
      ["low"] = new VolumeResponseLevel("Low", 0.3f, 0.6f, true, false),
      ["medium"] = new VolumeResponseLevel("Medium", 0.6f, 0.85f, true, true),
      ["high"] = new VolumeResponseLevel("High", 0.85f, 0.95f, true, true),
      ["critical"] = new VolumeResponseLevel("Critical", 0.95f, 1.0f, true, true)
    };
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

  public float GetIntensityRatio(float volume)
  {
    if (!IsInRange(volume)) return 0f;
    return (volume - minVolume) / (maxVolume - minVolume);
  }
}

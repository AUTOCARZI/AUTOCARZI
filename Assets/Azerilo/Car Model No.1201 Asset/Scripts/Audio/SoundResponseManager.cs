using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class SoundResponseManager
{
  // 메인 프로필 맵
  private Dictionary<SoundType, SoundResponseProfile> soundProfiles;

  public void Initialize()
  {
    soundProfiles = new Dictionary<SoundType, SoundResponseProfile>();

    // 앰뷸런스 프로필
    var ambulanceProfile = CreateAmbulanceProfile();
    RegisterProfile(ambulanceProfile);

    // 경적 프로필
    var hornProfile = CreateCarHornProfile();
    RegisterProfile(hornProfile);

    Debug.Log($"[SoundResponseManager] Initialized {soundProfiles.Count} sound profiles");
  }

  private SoundResponseProfile CreateAmbulanceProfile()
  {
    var profile = new SoundResponseProfile()
    {
      soundType = SoundType.Ambulance,
      activationThreshold = 0.3f,
      enableHUD = true,
      enableLED = true,
      hudBlinkSpeed = 1.0f,
      ledBlinkSpeed = 1.0f,
      ledTimerSpeed = 1.0f
    };

    // HUD 매핑 초기화
    profile.hudMapping = new Dictionary<string, string[]>
    {
      ["behind-left"] = new[] { "ambulance-left" },
      ["behind-right"] = new[] { "ambulance-right" },
      ["behind"] = new[] { "ambulance-left", "ambulance-right" },
      ["to the left"] = new[] { "ambulance-left" },
      ["to the right"] = new[] { "ambulance-right" },
      ["ahead"] = new string[0],
      ["ahead-left"] = new string[0],
      ["ahead-right"] = new string[0]
    };

    return profile;
  }

  private SoundResponseProfile CreateCarHornProfile()
  {
    var profile = new SoundResponseProfile()
    {
      soundType = SoundType.CarHorn,
      activationThreshold = 0.5f,
      enableHUD = true,
      enableLED = true,
      hudBlinkSpeed = 0f,
      ledBlinkSpeed = 2.0f,  // 더 빠른 깜빡임
      ledTimerSpeed = 2.0f
    };

    // 경적은 모든 방향에서 더 높은 임계값
    profile.directionThresholds = new Dictionary<string, float>
    {
      ["behind"] = 0.6f,
      ["behind-left"] = 0.7f,
      ["behind-right"] = 0.7f,
      ["to the left"] = 0.7f,
      ["to the right"] = 0.7f,
      ["ahead"] = 0.8f,
      ["ahead-left"] = 0.8f,
      ["ahead-right"] = 0.8f
    };

    // 경적용 HUD 매핑 여기에!!
    profile.hudMapping = new Dictionary<string, string[]>
    {
      ["behind-left"] = new string[0],
      ["behind-right"] = new string[0],
      ["behind"] = new string[0],
      ["to the left"] = new string[0],
      ["to the right"] = new string[0],
      ["ahead"] = new string[0],
      ["ahead-left"] = new string[0],
      ["ahead-right"] = new string[0]
    };

    return profile;
  }

  public void RegisterProfile(SoundResponseProfile profile)
  {
    soundProfiles[profile.soundType] = profile;
    Debug.Log($"[SoundResponseManager] Registered profile: {profile.soundType}");
  }


  public SoundResponseProfile GetProfile(SoundType soundType)
  {
    return soundProfiles.ContainsKey(soundType) ? soundProfiles[soundType] : null;
  }

  public VolumeResponseLevel GetVolumeLevel(SoundType soundType, float volume)
  {
    var profile = GetProfile(soundType);
    if (profile?.volumeLevels == null) return null;

    foreach (var level in profile.volumeLevels.Values)
    {
      if (level.IsInRange(volume))
      {
        return level;
      }
    }

    return profile.volumeLevels.ContainsKey("none") ? profile.volumeLevels["none"] : null;
  }

  public float GetDirectionThreshold(SoundType soundType, string direction)
  {
    var profile = GetProfile(soundType);
    return profile?.directionThresholds.ContainsKey(direction) == true
        ? profile.directionThresholds[direction]
        : 0.95f;
  }

  public string[] GetHUDsForDirection(SoundType soundType, string direction)
  {
    var profile = GetProfile(soundType);

    if (profile.hudMapping.ContainsKey(direction))
    {
      var result = profile.hudMapping[direction];
      Debug.Log($"Found HUDs: [{string.Join(", ", result)}], Count: {result.Length}");
      return result;
    }
    else
    {
      Debug.LogWarning($"Direction '{direction}' not found in mapping");
      return new string[0];
    }
  }

  // 런타임에 프로필 수정 가능
  public void UpdateDirectionThreshold(SoundType soundType, string direction, float threshold)
  {
    var profile = GetProfile(soundType);
    if (profile?.directionThresholds != null)
    {
      profile.directionThresholds[direction] = threshold;
      Debug.Log($"[SoundResponseManager] Updated {soundType} {direction} threshold to {threshold:F2}");
    }
  }

  public void UpdateActivationThreshold(SoundType soundType, float threshold)
  {
    var profile = GetProfile(soundType);
    if (profile != null)
    {
      profile.activationThreshold = threshold;
      Debug.Log($"[SoundResponseManager] Updated {soundType} activation threshold to {threshold:F2}");
    }
  }

  // 등록된 모든 사운드 타입 가져오기
  public SoundType[] GetRegisteredSoundTypes()
  {
    var types = new SoundType[soundProfiles.Count];
    soundProfiles.Keys.CopyTo(types, 0);
    return types;
  }

  // 로필 정보 출력 (디버깅용)
  public void PrintProfileInfo(SoundType soundType)
  {
    var profile = GetProfile(soundType);
    if (profile == null)
    {
      Debug.LogWarning($"[SoundResponseManager] Profile not found: {soundType}");
      return;
    }

    Debug.Log($"[SoundResponseManager] Profile Info for {soundType}:");
    Debug.Log($"  - Activation Threshold: {profile.activationThreshold:F2}");
    Debug.Log($"  - HUD Enabled: {profile.enableHUD}");
    Debug.Log($"  - LED Enabled: {profile.enableLED}");
    Debug.Log($"  - LED Blink Speed: {profile.ledBlinkSpeed:F2}");
    Debug.Log($"  - Direction Thresholds: {profile.directionThresholds.Count} entries");
    Debug.Log($"  - Volume Levels: {profile.volumeLevels.Count} entries");
    Debug.Log($"  - HUD Mappings: {profile.hudMapping.Count} entries");
  }
}

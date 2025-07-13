using UnityEngine;

// 경적 사운드 소스
public class CarHornSoundSource : ISoundSource
{
  private AudioSource hornAudioSource;
  private Transform hornTransform;
  private float maxDistance = 50f;

  public CarHornSoundSource(AudioSource audioSource, Transform transform)
  {
    hornAudioSource = audioSource;
    hornTransform = transform;
  }

  public bool IsActive() => hornAudioSource != null && hornAudioSource.isPlaying;

  public float GetPerceivedVolume(Transform listener)
  {
    if (!IsActive()) return 0f;

    float distance = Vector3.Distance(GetPosition(), listener.position);
    float volumeAttenuation = Mathf.Clamp01(1f - (distance / maxDistance));
    return hornAudioSource.volume * volumeAttenuation;
  }

  public Vector3 GetPosition() => hornTransform.position;
  public SoundType GetSoundType() => SoundType.CarHorn;
}

using UnityEngine;

public interface ISoundSource
{
  bool IsActive();
  float GetPerceivedVolume(Transform listener);
  Vector3 GetPosition();
  SoundType GetSoundType();
}

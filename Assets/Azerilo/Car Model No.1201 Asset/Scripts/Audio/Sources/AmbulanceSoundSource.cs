using UnityEngine;

// 앰뷸런스 사운드 소스
public class AmbulanceSoundSource : ISoundSource
{
  private CarControllerAmbulance ambulance;

  public AmbulanceSoundSource(CarControllerAmbulance amb)
  {
    ambulance = amb;
  }

  public bool IsActive() => ambulance != null;
  public float GetPerceivedVolume(Transform listener) => ambulance.GetPerceivedVolumeFrom(listener);
  public Vector3 GetPosition() => ambulance.transform.position;
  public SoundType GetSoundType() => SoundType.Ambulance;
}

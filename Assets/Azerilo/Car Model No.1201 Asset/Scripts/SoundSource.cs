using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

public enum SoundType
{
    Ambulance,
    CarHorn,
    FireTruck,
    Police,
    Motorcycle,
    Train
}

public interface ISoundSource
{
    bool IsActive();
    float GetPerceivedVolume(Transform listener);
    Vector3 GetPosition();
    SoundType GetSoundType();
}

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

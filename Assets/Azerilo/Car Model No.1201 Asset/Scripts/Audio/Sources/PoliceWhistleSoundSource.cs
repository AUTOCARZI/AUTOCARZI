using UnityEngine;

public class PoliceWhistleSoundSource : ISoundSource
{
    private AudioSource whistleAudioSource;
    private Transform whistleTransform;

    public PoliceWhistleSoundSource(AudioSource audioSource, Transform transform)
    {
        this.whistleAudioSource = audioSource;
        this.whistleTransform = transform;
    }

    public bool IsActive()
    {
        return whistleAudioSource != null && whistleTransform != null && whistleAudioSource.isPlaying;
    }

    public float GetPerceivedVolume(Transform listenerTransform)
    {
        if (!IsActive()) return 0f;

        float distance = Vector3.Distance(whistleTransform.position, listenerTransform.position);
        float baseVolume = whistleAudioSource.volume;

        float distanceAttenuation = Mathf.Max(0f, 1f - (distance / 200f));
        float perceivedVolume = baseVolume * distanceAttenuation;

        return perceivedVolume;
    }

    public Vector3 GetPosition()
    {
        return whistleTransform != null ? whistleTransform.position : Vector3.zero;
    }

    public SoundType GetSoundType()
    {
        return SoundType.PoliceWhistle;
    }
}
using UnityEngine;

public class PoliceOfficer : MonoBehaviour
{
    public AudioSource whistleAudioSource;
    
    void Start()
    {
        if (whistleAudioSource == null)
            whistleAudioSource = GetComponent<AudioSource>();
        
        Invoke("StartWhistling", 14f);
        
        Debug.Log("[PoliceOfficer] Whistle will start in 2 seconds...");
    }
    
    void StartWhistling()
    {
        whistleAudioSource.loop = true;
        whistleAudioSource.Play();
        
        Debug.Log("[PoliceOfficer] Whistle looping started!");
    }
    public void StopWhistling()
    {
        whistleAudioSource.loop = false;
    }
}
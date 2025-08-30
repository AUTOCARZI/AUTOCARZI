using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(InputManagerAmbulance))]
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(LightingManager))]
public class CarControllerAmbulance : MonoBehaviour
{
    public InputManagerAmbulance im;
    public LightingManager lm;
    public List<WheelCollider> throttleWheels;
    public List<GameObject> steeringWheels;
    public List<GameObject> meshes;
    public float strengthCoefficient = 10000f;
    public float maxTurn = 20f;
    public Transform CM;
    public Rigidbody rb;
    public float brakeStrength;

    [Header("Audio Settings")]
    public AudioSource sirenSound;
    public AudioClip sirenClip;
    public AudioSource hornSound;
    public AudioClip hornClip;



    void Start()
    {
        im = GetComponent<InputManagerAmbulance>();
        rb = GetComponent<Rigidbody>();
        if (CM)
        {
            rb.centerOfMass = CM.position;
        }

        SetupAudioSources();
    }

    void SetupAudioSources()
    {
        // Setup siren sound
        if (sirenSound == null)
        {
            sirenSound = gameObject.AddComponent<AudioSource>();
        }
        if (sirenClip != null)
        {
            sirenSound.clip = sirenClip;
            sirenSound.loop = true;
            sirenSound.volume = 1f;
            sirenSound.spatialBlend = 1f; // Full 3D
            sirenSound.minDistance = 20f;
            sirenSound.maxDistance = 150f;
            sirenSound.playOnAwake = false; // 시작할 때 자동 재생 안함
        }

        // Setup horn sound
        if (hornSound == null)
        {
            hornSound = gameObject.AddComponent<AudioSource>();
        }
        if (hornClip != null)
        {
            hornSound.clip = hornClip;
            hornSound.loop = false;
            hornSound.volume = 0.8f;
            hornSound.spatialBlend = 1f; // Full 3D
            hornSound.minDistance = 5f;
            hornSound.maxDistance = 50f;
            hornSound.playOnAwake = false;
        }
    }

    void Update()
    {
        // Toggle headlights
        if (im.l)
        {
            lm.ToggleHeadlights();
        }

        // Handle siren
        if (im.siren)
        {
            if (!sirenSound.isPlaying)
            {
                sirenSound.Play();
            }
        }
        else
        {
            if (sirenSound.isPlaying)
            {
                sirenSound.Stop();
            }
        }

        // Handle horn
        if (im.horn)
        {
            if (!hornSound.isPlaying)
            {
                hornSound.Play();
            }
        }
        else
        {
            if (hornSound.isPlaying)
            {
                hornSound.Stop();
            }
        }
    }

    void FixedUpdate()
    {
        foreach (WheelCollider wheel in throttleWheels)
        {
            if (im.brake)
            {
                wheel.motorTorque = 0f;
                wheel.brakeTorque = brakeStrength * Time.deltaTime;
            }
            else
            {
                wheel.motorTorque = strengthCoefficient * Time.deltaTime * im.throttle;
                wheel.brakeTorque = 0f;
            }
        }

        foreach (GameObject wheel in steeringWheels)
        {
            wheel.GetComponent<WheelCollider>().steerAngle = maxTurn * im.steer;
            wheel.transform.localEulerAngles = new Vector3(0f, im.steer * maxTurn, 0f);
        }

        foreach (GameObject mesh in meshes)
        {
            mesh.transform.Rotate(rb.linearVelocity.magnitude * (transform.InverseTransformDirection(rb.linearVelocity).z >= 0 ? 1 : -1) / (2 * Mathf.PI * 0.33f), 0f, 0f);
        }
    }

    // Method to get how loud this ambulance sounds from another position
    public float GetPerceivedVolumeFrom(Transform listener)
    {
        float totalVolume = 0f;

        // Check siren volume
        if (sirenSound && sirenSound.isPlaying)
        {
            totalVolume += CalculateVolumeFromDistance(sirenSound, listener);
        }

        // Check horn volume
        if (hornSound && hornSound.isPlaying)
        {
            totalVolume += CalculateVolumeFromDistance(hornSound, listener);
        }

        return totalVolume;
    }

    // NEW: Get separate siren volume
    public float GetSirenVolumeFrom(Transform listener)
    {
        if (sirenSound && sirenSound.isPlaying)
        {
            return CalculateVolumeFromDistance(sirenSound, listener);
        }
        return 0f;
    }

    // NEW: Get separate horn volume
    public float GetHornVolumeFrom(Transform listener)
    {
        if (hornSound && hornSound.isPlaying)
        {
            return CalculateVolumeFromDistance(hornSound, listener);
        }
        return 0f;
    }

    // NEW: Check if siren is playing
    public bool IsSirenPlaying()
    {
        return sirenSound && sirenSound.isPlaying;
    }

    // NEW: Check if horn is playing
    public bool IsHornPlaying()
    {
        return hornSound && hornSound.isPlaying;
    }

    // NEW: Get siren AudioSource
    public AudioSource GetSirenAudioSource()
    {
        return sirenSound;
    }

    // NEW: Get horn AudioSource
    public AudioSource GetHornAudioSource()
    {
        return hornSound;
    }

    private float CalculateVolumeFromDistance(AudioSource audioSource, Transform listener)
    {
        float distance = Vector3.Distance(transform.position, listener.position);
        float minDistance = audioSource.minDistance;
        float maxDistance = audioSource.maxDistance;

        if (distance <= minDistance)
        {
            return audioSource.volume;
        }
        else if (distance >= maxDistance)
        {
            return 0f;
        }
        else
        {
            float rolloff = 1f - ((distance - minDistance) / (maxDistance - minDistance));
            return audioSource.volume * rolloff;
        }
    }
}
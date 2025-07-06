using System.Collections;
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

    void Start()
    {
        im = GetComponent<InputManagerAmbulance>();
        rb = GetComponent<Rigidbody>();
        if (CM)
        {
            rb.centerOfMass = CM.position;
        }

        // Setup siren sound
        SetupSirenSound();
    }

    void SetupSirenSound()
    {
        sirenSound = GetComponent<AudioSource>();
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
            sirenSound.minDistance = 10f;
            sirenSound.maxDistance = 100f;
            sirenSound.Play();
        }
    }

    void Update()
    {
        if (im.l)
        {
            lm.ToggleHeadlights();
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
        if (!sirenSound || !sirenSound.isPlaying) return 0f;

        float distance = Vector3.Distance(transform.position, listener.position);
        float minDistance = sirenSound.minDistance;
        float maxDistance = sirenSound.maxDistance;

        if (distance <= minDistance)
        {
            return sirenSound.volume;
        }
        else if (distance >= maxDistance)
        {
            return 0f;
        }
        else
        {
            float rolloff = 1f - ((distance - minDistance) / (maxDistance - minDistance));
            return sirenSound.volume * rolloff;
        }
    }
}
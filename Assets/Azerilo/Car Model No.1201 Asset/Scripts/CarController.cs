using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(InputManager))]
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(LightingManager))]
public class CarController : MonoBehaviour
{
    public InputManager im;
    public LightingManager lm;
    public List<WheelCollider> throttleWheels;
    public List<GameObject> steeringWheels;
    public List<GameObject> meshes;
    public float strengthCoefficient = 10000f;
    public float maxTurn = 20f;
    public Transform CM;
    public Rigidbody rb;
    public float brakeStrength;

    [Header("Audio Detection")]
    public CarControllerAmbulance ambulanceToDetect;
    public float ambulanceVolumeThreshold = 0.3f;

    [Header("LED Warning System")]
    public Transform rightLEDParent;
    public Transform leftLEDParent;

    [Header("Direction-based Blink Thresholds")]
    public float behindBlinkThreshold = 0.90f;    // 뒤쪽 (behind, behind-left, behind-right)
    public float sideBlinkThreshold = 0.90f;     // 옆쪽 (to the left, to the right)
    public float frontBlinkThreshold = 0.95f;    // 앞쪽 (ahead, ahead-left, ahead-right)

    private float currentAmbulanceVolume;
    private Vector3 ambulanceDirection;
    private float ambulanceDistance;
    private string ambulanceRelativeDirection;
    private LEDNode[] allLEDs;
    private float currentBlinkThreshold; // 현재 방향에 따른 임계값

    void Start()
    {
        im = GetComponent<InputManager>();
        rb = GetComponent<Rigidbody>();
        if (CM)
        {
            rb.centerOfMass = CM.position;
        }

        if (ambulanceToDetect == null)
        {
            ambulanceToDetect = FindObjectOfType<CarControllerAmbulance>();
        }

        FindAllLEDs();
    }

    void FindAllLEDs()
    {
        List<LEDNode> allLEDsList = new List<LEDNode>();

        if (rightLEDParent != null)
        {
            LEDNode[] rightLEDs = rightLEDParent.GetComponentsInChildren<LEDNode>();
            allLEDsList.AddRange(rightLEDs);
            Debug.Log($"Found {rightLEDs.Length} RIGHT LED nodes");
        }

        if (leftLEDParent != null)
        {
            LEDNode[] leftLEDs = leftLEDParent.GetComponentsInChildren<LEDNode>();
            allLEDsList.AddRange(leftLEDs);
            Debug.Log($"Found {leftLEDs.Length} LEFT LED nodes");
        }

        allLEDs = allLEDsList.ToArray();
        Debug.Log($"Total LEDs found: {allLEDs.Length}");
    }

    void Update()
    {
        if (im.l)
        {
            lm.ToggleHeadlights();
        }

        CheckAmbulanceAudio();
        UpdateWarningLEDs();
    }

    void CheckAmbulanceAudio()
    {
        if (ambulanceToDetect != null)
        {
            currentAmbulanceVolume = ambulanceToDetect.GetPerceivedVolumeFrom(this.transform);

            Vector3 toAmbulance = ambulanceToDetect.transform.position - this.transform.position;
            ambulanceDistance = toAmbulance.magnitude;
            ambulanceDirection = toAmbulance.normalized;

            ambulanceRelativeDirection = GetRelativeDirection(ambulanceDirection);

            // 방향에 따른 임계값 설정
            currentBlinkThreshold = GetBlinkThresholdForDirection(ambulanceRelativeDirection);

            if (currentAmbulanceVolume > ambulanceVolumeThreshold)
            {
                Debug.Log($"Ambulance {ambulanceRelativeDirection}! Volume: {currentAmbulanceVolume:F2}, Blink Threshold: {currentBlinkThreshold:F2}, Distance: {ambulanceDistance:F1}m");
                ReactToAmbulanceDirection(ambulanceRelativeDirection);
            }
        }
    }

    float GetBlinkThresholdForDirection(string direction)
    {
        switch (direction)
        {
            // 뒤쪽 - 0.90 임계값
            case "behind":
            case "behind-left":
            case "behind-right":
                return behindBlinkThreshold;

            // 옆쪽 - 0.90 임계값
            case "to the left":
            case "to the right":
                return sideBlinkThreshold;

            // 앞쪽 - 0.95 임계값
            case "ahead":
            case "ahead-left":
            case "ahead-right":
                return frontBlinkThreshold;

            default:
                return frontBlinkThreshold; // 기본값은 앞쪽과 동일
        }
    }

    void UpdateWarningLEDs()
    {
        if (allLEDs != null && allLEDs.Length > 0)
        {
            if (currentAmbulanceVolume >= currentBlinkThreshold)
            {
                // Calculate blink speed based on volume
                float volumeRatio = (currentAmbulanceVolume - currentBlinkThreshold) / (1.0f - currentBlinkThreshold);
                volumeRatio = Mathf.Clamp01(volumeRatio);

                float minOffTime = 0.5f;
                float maxOffTime = 2.0f;
                float blinkSpeed = Mathf.Lerp(maxOffTime, minOffTime, volumeRatio);

                float minTimerSpeed = 2.0f;
                float maxTimerSpeed = 8.0f;
                float timerSpeed = Mathf.Lerp(minTimerSpeed, maxTimerSpeed, volumeRatio);

                // 모든 LED 초기화
                foreach (LEDNode led in allLEDs)
                {
                    led.isFirstNode = false;
                    led.prevNode = null;

                    led.SetTimingSettings(
                        0.8f,        // onTime
                        blinkSpeed,  // offTime
                        timerSpeed,  // onTimerSpeed
                        timerSpeed   // offTimerSpeed
                    );
                }

                // 체인 연결: 각 LED가 이전 LED를 참조하도록 설정
                for (int i = 0; i < allLEDs.Length; i++)
                {
                    if (i == 0)
                    {
                        // 첫 번째 LED만 firstNode로 설정
                        allLEDs[i].isFirstNode = true;
                        allLEDs[i].prevNode = null;
                    }
                    else
                    {
                        // 나머지 LED들은 이전 LED를 참조
                        allLEDs[i].isFirstNode = false;
                        allLEDs[i].prevNode = allLEDs[i - 1];
                    }
                }

                Debug.Log($"LEDs chained blinking! Direction: {ambulanceRelativeDirection}, Volume: {currentAmbulanceVolume:F2}");
            }
            else
            {
                // Volume below threshold - 모든 LED 정상상태로
                foreach (LEDNode led in allLEDs)
                {
                    led.SetTimingSettings(0.1f, 10.0f, 1.0f, 1.0f);
                    led.isFirstNode = false;
                    led.prevNode = null;
                }
            }
        }
    }

    string GetRelativeDirection(Vector3 directionToAmbulance)
    {
        Vector3 localDirection = transform.InverseTransformDirection(directionToAmbulance);
        float angle = Mathf.Atan2(localDirection.x, localDirection.z) * Mathf.Rad2Deg;

        if (angle < 0) angle += 360f;

        if (angle >= 337.5f || angle < 22.5f) return "ahead";
        else if (angle >= 22.5f && angle < 67.5f) return "ahead-right";
        else if (angle >= 67.5f && angle < 112.5f) return "to the right";
        else if (angle >= 112.5f && angle < 157.5f) return "behind-right";
        else if (angle >= 157.5f && angle < 202.5f) return "behind";
        else if (angle >= 202.5f && angle < 247.5f) return "behind-left";
        else if (angle >= 247.5f && angle < 292.5f) return "to the left";
        else return "ahead-left";
    }

    void ReactToAmbulanceDirection(string direction)
    {
        switch (direction)
        {
            case "ahead":
                Debug.Log("Ambulance is directly ahead - should slow down and move aside!");
                break;
            case "ahead-right":
                Debug.Log("Ambulance is ahead-right - should move left!");
                break;
            case "to the right":
                Debug.Log("Ambulance is to the right - should move left!");
                break;
            case "behind-right":
                Debug.Log("Ambulance is behind-right - should move left and let it pass!");
                break;
            case "behind":
                Debug.Log("Ambulance is directly behind - should move aside!");
                break;
            case "behind-left":
                Debug.Log("Ambulance is behind-left - should move right and let it pass!");
                break;
            case "to the left":
                Debug.Log("Ambulance is to the left - should move right!");
                break;
            case "ahead-left":
                Debug.Log("Ambulance is ahead-left - should move right!");
                break;
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

    public float GetCurrentAmbulanceVolume() => currentAmbulanceVolume;
    public Vector3 GetAmbulanceDirection() => ambulanceDirection;
    public float GetAmbulanceDistance() => ambulanceDistance;
    public string GetAmbulanceRelativeDirection() => ambulanceRelativeDirection;
}
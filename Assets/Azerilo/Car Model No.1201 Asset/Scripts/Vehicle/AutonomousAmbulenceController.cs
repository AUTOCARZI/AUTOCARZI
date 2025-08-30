using System.Collections;
using UnityEngine;

[RequireComponent(typeof(CarControllerAmbulance))]
[RequireComponent(typeof(InputManagerAmbulance))]
public class AutonomousAmbulanceController : MonoBehaviour
{
    [Header("Autonomous Settings")]
    public bool enableAutonomousMode = true;
    public float startDelay = 10f;
    public float forwardSpeed = 1f;
    
    [Header("Obstacle Detection")]
    public float raycastDistance = 10f;
    public float raycastHeight = 1f;
    public float raycastWidth = 2f;
    public LayerMask obstacleLayerMask = -1;
    public int numberOfRays = 3;
    
    [Header("Debug")]
    public bool showRaycastGizmos = true;
    public Color rayHitColor = Color.red;
    public Color rayMissColor = Color.green;
    
    private CarControllerAmbulance carController;
    private InputManagerAmbulance inputManager;
    private bool isAutonomousActive = false;
    private bool obstacleDetected = false;
    private bool sirenActivated = false;
    
    [Header("Emergency Brake")]
    public float emergencyBrakeForce = 5000f;
    public float emergencyDrag = 10f;
    
    private RaycastHit[] raycastResults;
    private bool[] rayHits;
    private float originalDrag;
    private bool emergencyBrakeApplied = false;
    
    void Start()
    {
        carController = GetComponent<CarControllerAmbulance>();
        inputManager = GetComponent<InputManagerAmbulance>();
        
        raycastResults = new RaycastHit[numberOfRays];
        rayHits = new bool[numberOfRays];
        
        if (carController.rb == null)
        {
            carController.rb = GetComponent<Rigidbody>();
        }

        originalDrag = carController.rb.linearDamping;
        
        if (enableAutonomousMode)
        {
            Debug.Log($"[AutonomousAmbulance] Starting autonomous mode in {startDelay} seconds");
            StartCoroutine(StartAutonomousMode());
        }
    }
    
    void Update()
    {
        if (isAutonomousActive)
        {
            DetectObstacles();
            ControlAmbulance();
        }
    }
    
    IEnumerator StartAutonomousMode()
    {
        yield return new WaitForSeconds(startDelay);
        
        isAutonomousActive = true;
        inputManager.isAutonomousMode = true;
        
        if (!sirenActivated)
        {
            inputManager.siren = true;
            sirenActivated = true;
            Debug.Log("[AutonomousAmbulance] Siren activated!");
        }
        
        Debug.Log("[AutonomousAmbulance] Autonomous mode activated!");
    }
    
    void DetectObstacles()
    {
        bool anyObstacleDetected = false;
        Vector3 origin = transform.position + Vector3.up * raycastHeight;
        Vector3 forward = transform.forward;
        
        for (int i = 0; i < numberOfRays; i++)
        {
            Vector3 rayDirection = forward;
            
            if (numberOfRays > 1)
            {
                float angle = 0f;
                if (numberOfRays == 3)
                {
                    angle = (i - 1) * 15f;
                }
                else if (numberOfRays > 3)
                {
                    angle = Mathf.Lerp(-30f, 30f, (float)i / (numberOfRays - 1));
                }
                
                rayDirection = Quaternion.AngleAxis(angle, Vector3.up) * forward;
            }
            
            Vector3 rayOrigin = origin;
            if (numberOfRays > 1 && raycastWidth > 0)
            {
                Vector3 rightOffset = transform.right * raycastWidth * ((float)i / (numberOfRays - 1) - 0.5f);
                rayOrigin += rightOffset;
            }
            
            if (Physics.Raycast(rayOrigin, rayDirection, out raycastResults[i], raycastDistance, obstacleLayerMask))
            {
                rayHits[i] = true;
                anyObstacleDetected = true;
                
                if (!obstacleDetected)
                {
                    Debug.Log($"[AutonomousAmbulance] Obstacle detected: {raycastResults[i].collider.name} at distance {raycastResults[i].distance:F2}m");
                }
            }
            else
            {
                rayHits[i] = false;
            }
        }
        
        if (obstacleDetected != anyObstacleDetected)
        {
            obstacleDetected = anyObstacleDetected;
            
            if (obstacleDetected)
            {
                Debug.Log("[AutonomousAmbulance] Stopping - obstacle ahead!");
            }
            else
            {
                Debug.Log("[AutonomousAmbulance] Path clear - resuming movement");
            }
        }
    }
    
    void ControlAmbulance()
    {
        if (!obstacleDetected)
        {
            inputManager.throttle = forwardSpeed;
            inputManager.brake = false;
            
            // 정상 주행으로 복원
            if (emergencyBrakeApplied)
            {
                carController.rb.linearDamping = originalDrag;
                emergencyBrakeApplied = false;
                Debug.Log("[AutonomousAmbulance] Resuming normal movement");
            }
        }
        else
        {
            inputManager.throttle = 0f;
            inputManager.brake = true;
            
            ApplyEmergencyBrake();
        }
        
        inputManager.steer = 0f;
        
        if (!inputManager.siren && sirenActivated)
        {
            inputManager.siren = true;
        }
    }
    
    void ApplyEmergencyBrake()
    {
        if (!emergencyBrakeApplied)
        {
            emergencyBrakeApplied = true;
            Debug.Log("[AutonomousAmbulance] Emergency brake applied!");
        }
        
        Rigidbody rb = carController.rb;
        
        // 현재 속도가 매우 낮으면 완전 정지
        if (rb.linearVelocity.magnitude < 0.1f)
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }
        else
        {
            // 역방향 제동력 적용
            Vector3 brakeForce = -rb.linearVelocity.normalized * emergencyBrakeForce;
            rb.AddForce(brakeForce, ForceMode.Force);
            
            // 저항 증가 (한 번만 설정)
            if (rb.linearDamping != emergencyDrag)
            {
                rb.linearDamping = emergencyDrag;
            }
        }
        
        // 바퀴에 강한 브레이크 적용
        foreach (WheelCollider wheel in carController.throttleWheels)
        {
            wheel.motorTorque = 0f;
            wheel.brakeTorque = carController.brakeStrength * 5f;
        }
    }
    
    void OnDrawGizmos()
    {
        if (!showRaycastGizmos || !Application.isPlaying) return;
        
        Vector3 origin = transform.position + Vector3.up * raycastHeight;
        Vector3 forward = transform.forward;
        
        for (int i = 0; i < numberOfRays; i++)
        {
            Vector3 rayDirection = forward;
            
            if (numberOfRays > 1)
            {
                float angle = 0f;
                if (numberOfRays == 3)
                {
                    angle = (i - 1) * 15f;
                }
                else if (numberOfRays > 3)
                {
                    angle = Mathf.Lerp(-30f, 30f, (float)i / (numberOfRays - 1));
                }
                
                rayDirection = Quaternion.AngleAxis(angle, Vector3.up) * forward;
            }
            
            Vector3 rayOrigin = origin;
            if (numberOfRays > 1 && raycastWidth > 0)
            {
                Vector3 rightOffset = transform.right * raycastWidth * ((float)i / (numberOfRays - 1) - 0.5f);
                rayOrigin += rightOffset;
            }
            
            Gizmos.color = rayHits != null && i < rayHits.Length && rayHits[i] ? rayHitColor : rayMissColor;
            
            Vector3 endPoint = rayOrigin + rayDirection * raycastDistance;
            if (raycastResults != null && i < raycastResults.Length && rayHits != null && i < rayHits.Length && rayHits[i])
            {
                endPoint = raycastResults[i].point;
                Gizmos.DrawWireSphere(raycastResults[i].point, 0.5f);
            }
            
            Gizmos.DrawLine(rayOrigin, endPoint);
        }
        
        Gizmos.color = obstacleDetected ? Color.red : Color.green;
        Gizmos.DrawWireCube(transform.position + transform.forward * (raycastDistance * 0.5f) + Vector3.up * raycastHeight, 
                           new Vector3(raycastWidth * 2, 1f, raycastDistance));
    }
    
    public void EnableAutonomousMode()
    {
        if (!isAutonomousActive)
        {
            enableAutonomousMode = true;
            inputManager.isAutonomousMode = true;
            StartCoroutine(StartAutonomousMode());
        }
    }
    
    public void DisableAutonomousMode()
    {
        isAutonomousActive = false;
        enableAutonomousMode = false;
        inputManager.isAutonomousMode = false;
        
        inputManager.throttle = 0f;
        inputManager.steer = 0f;
        inputManager.brake = false;
        
        Debug.Log("[AutonomousAmbulance] Autonomous mode disabled");
    }
    
    public void ToggleSiren()
    {
        inputManager.siren = !inputManager.siren;
        sirenActivated = inputManager.siren;
    }
    
    public bool IsAutonomousActive => isAutonomousActive;
    public bool IsObstacleDetected => obstacleDetected;
    public bool IsSirenActivated => sirenActivated;
}
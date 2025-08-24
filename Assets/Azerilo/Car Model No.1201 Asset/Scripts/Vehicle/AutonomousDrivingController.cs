using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(CarController))]
public class AutonomousDrivingController : MonoBehaviour
{
  [Header("Autonomous Mode")]
  public bool isAutonomousMode = false;

  [Header("Speed Control")]
  public float targetSpeed = 30f;
  public float safeFollowingDistance = 12f;
  public float emergencyBrakeDistance = 6f;
  public float speedControlSensitivity = 1.2f;
  public float accelerationSmoothness = 1.5f;
  public float maxThrottleRate = 1.0f;

  [Header("Steering Control")]
  public float steeringKp = 2.5f;
  public float steeringKi = 0.15f;
  public float steeringKd = 0.7f;
  public float maxSteeringAngle = 1f;

  [Header("Steering Override")]
  public bool allowSteeringOverride = false;
  public float steeringOverrideValue = 0f;

  [Header("Obstacle Detection")]
  public float forwardRayDistance = 30f;
  public LayerMask obstacleLayer = -1;
  public int frontRayCount = 5;
  public Transform raycastOrigin;
  public float raycastAngleRange = 10f;

  [Header("Safety")]
  public float manualOverrideDelay = 2f;
  public bool enableEmergencyBraking = true;

  private const float MIN_SPEED_THRESHOLD = 0.5f;
  private const float DECELERATION_THRESHOLD = -5f;
  private const float LATERAL_FILTER_DISTANCE = 3f;
  private const float RAYCAST_HEIGHT_OFFSET = 0.5f;
  private const float KMH_TO_MS_CONVERSION = 3.6f;
  private const float MIN_THROTTLE_WHEN_NO_LANE = 0.4f;
  private const float EMERGENCY_BRAKE_SPEED_THRESHOLD = 25f;

  private CarController carController;
  private PIDController steeringPID;
  private Rigidbody carRigidbody;

  private float frontObstacleDistance = float.MaxValue;
  private bool hasObstacle = false;
  private float lastManualInputTime = 0f;

  private bool wasManuallyOverridden = false;

  void Start()
  {
    InitializeComponents();
    InitializePIDController();
    SubscribeToEvents();
  }

  void InitializeComponents()
  {
    carController = GetComponent<CarController>();
    carRigidbody = GetComponent<Rigidbody>();

    if (raycastOrigin == null)
      raycastOrigin = transform;

    if (carController == null)
    {
      Debug.LogError("[AutonomousDriving] CarController not found!");
      enabled = false;
    }
  }

  void InitializePIDController()
  {
    steeringPID = new PIDController(steeringKp, steeringKi, steeringKd);
  }

  void SubscribeToEvents()
  {
    EventManager.Subscribe<LaneWarningEvent>(OnLaneWarningReceived);
  }

  void Update()
  {
    HandleInputs();

    if (isAutonomousMode)
    {
      PerformSensorScanning();
      ProcessAutonomousControl();
    }
  }

  void HandleInputs()
  {
    Keyboard keyboard = Keyboard.current;
    if (keyboard != null && keyboard.f1Key.wasPressedThisFrame)
    {
      ToggleAutonomousMode();
    }

    CheckForManualOverride();
  }

  void CheckForManualOverride()
  {
    Keyboard keyboard = Keyboard.current;
    if (keyboard == null) return;

    bool hasManualInput = keyboard.upArrowKey.isPressed ||
                         keyboard.downArrowKey.isPressed ||
                         keyboard.leftArrowKey.isPressed ||
                         keyboard.rightArrowKey.isPressed ||
                         keyboard.spaceKey.isPressed;

    if (hasManualInput && isAutonomousMode)
    {
      lastManualInputTime = Time.time;
      wasManuallyOverridden = true;
      SetAutonomousMode(false);
    }

    if (wasManuallyOverridden && Time.time - lastManualInputTime > manualOverrideDelay)
    {
      if (!hasManualInput)
      {
        wasManuallyOverridden = false;
      }
    }
  }

  void PerformSensorScanning()
  {
    frontObstacleDistance = float.MaxValue;
    hasObstacle = false;

    Vector3 rayStart = raycastOrigin.position + Vector3.up * RAYCAST_HEIGHT_OFFSET;

    for (int i = 0; i < frontRayCount; i++)
    {
      float angle = Mathf.Lerp(-raycastAngleRange, raycastAngleRange, (float)i / (frontRayCount - 1));
      Vector3 direction = Quaternion.Euler(0, angle, 0) * raycastOrigin.forward;

      if (Physics.Raycast(rayStart, direction, out RaycastHit hit, forwardRayDistance, obstacleLayer))
      {
        float lateralDistance = Vector3.Cross(raycastOrigin.forward, hit.point - raycastOrigin.position).magnitude;

        if (lateralDistance < LATERAL_FILTER_DISTANCE && hit.distance < frontObstacleDistance)
        {
          frontObstacleDistance = hit.distance;
          hasObstacle = true;
        }
      }

      Debug.DrawRay(rayStart, direction * forwardRayDistance, hasObstacle ? Color.red : Color.green, 0.1f);
    }
  }

  void ProcessAutonomousControl()
  {
    float throttleInput = CalculateThrottleInput();
    float steerInput = CalculateSteeringInput();
    bool brakeInput = ShouldApplyBrakes();

    SendAutonomousInput(throttleInput, steerInput, brakeInput);
  }

  private bool ShouldApplyMinimumThrottle()
  {
    return !carController.isLaneDetectionActive && !hasObstacle;
  }

  float CalculateThrottleInput()
  {
    float currentSpeedKmh = GetCurrentSpeedKmh();

    if (ShouldEmergencyBrake(currentSpeedKmh))
      return 0f;

    float effectiveTargetSpeed = CalculateEffectiveTargetSpeed();
    float speedError = effectiveTargetSpeed - currentSpeedKmh;

    return CalculateThrottleFromSpeedError(speedError, currentSpeedKmh);
  }

  private float GetCurrentSpeedKmh()
  {
    return carRigidbody.linearVelocity.magnitude * KMH_TO_MS_CONVERSION;
  }

  private bool ShouldEmergencyBrake(float currentSpeedKmh)
  {
    return hasObstacle && frontObstacleDistance < emergencyBrakeDistance;
  }

  private float CalculateEffectiveTargetSpeed()
  {
    if (!hasObstacle)
      return targetSpeed;

    float distanceRatio = Mathf.Clamp01(frontObstacleDistance / safeFollowingDistance);

    if (frontObstacleDistance <= emergencyBrakeDistance)
    {
      return 15f;
    }

    float speedMultiplier = Mathf.Pow(distanceRatio, 0.6f);
    float minSpeedRatio = 0.4f;

    return targetSpeed * Mathf.Lerp(minSpeedRatio, 1f, speedMultiplier);
  }

  private float CalculateThrottleFromSpeedError(float speedError, float currentSpeedKmh)
  {
    if (speedError > MIN_SPEED_THRESHOLD)
    {
      float speedRatio = currentSpeedKmh / targetSpeed;
      float normalizedError = Mathf.Clamp01(speedError / (targetSpeed * 0.4f));

      float obstacleMultiplier = 1f;
      if (hasObstacle && frontObstacleDistance < safeFollowingDistance)
      {
        obstacleMultiplier = Mathf.Clamp(frontObstacleDistance / safeFollowingDistance, 0.3f, 1f);
      }

      float baseThrottle = Mathf.Pow(normalizedError, 1f / accelerationSmoothness) * maxThrottleRate;
      float speedAdjustedThrottle = baseThrottle * (1f - speedRatio * 0.3f);

      float minThrottle = currentSpeedKmh < 30f ? 0.6f : 0.3f;

      return Mathf.Clamp(speedAdjustedThrottle * obstacleMultiplier, minThrottle, 1f);
    }
    else if (speedError < DECELERATION_THRESHOLD)
    {
      if (hasObstacle && frontObstacleDistance < emergencyBrakeDistance * 1.2f)
      {
        return -0.3f;
      }
      return 0f;
    }

    if (speedError > 5f)
    {
      return 0.8f;
    }
    else if (speedError > 0f)
    {
      return 0.6f;
    }
    else
    {
      return 0.3f;
    }
  }

  float CalculateSteeringInput()
  {
    float steerOutput;
    
    if (allowSteeringOverride)
    {
      // 조향 오버라이드가 활성화된 경우 수동 조향 값 사용
      steerOutput = steeringOverrideValue;
    }
    else
    {
      // 기본 자율주행 조향 (차선 유지)
      float targetOffset = 0f;
      steerOutput = steeringPID.Calculate(targetOffset, carController.currentLaneOffset, Time.deltaTime);
    }
    
    return Mathf.Clamp(steerOutput, -maxSteeringAngle, maxSteeringAngle);
  }

  bool ShouldApplyBrakes()
  {
    if (!enableEmergencyBraking) return false;

    float currentSpeedKmh = GetCurrentSpeedKmh();
    return IsEmergencyBrakingRequired(currentSpeedKmh);
  }

  private bool IsEmergencyBrakingRequired(float currentSpeedKmh)
  {
    return (hasObstacle && frontObstacleDistance < emergencyBrakeDistance) ||
           (hasObstacle && frontObstacleDistance < safeFollowingDistance && currentSpeedKmh > EMERGENCY_BRAKE_SPEED_THRESHOLD);
  }

  void SendAutonomousInput(float throttle, float steer, bool brake)
  {
    var autonomousInput = new CarInputEvent(throttle, steer, brake, false);
    EventManager.Publish(autonomousInput);
  }

  public void SetSteeringOverride(bool enabled, float steerValue = 0f)
  {
    allowSteeringOverride = enabled;
    steeringOverrideValue = steerValue;
    
    if (enabled)
    {
      Debug.Log($"[AutonomousDriving] 조향 오버라이드 활성화: {steerValue}");
    }
    else
    {
      Debug.Log("[AutonomousDriving] 조향 오버라이드 비활성화 - 차선 유지 모드");
    }
  }

  public void ToggleAutonomousMode()
  {
    if (wasManuallyOverridden)
    {
      Debug.LogWarning("[AutonomousDriving] Manual override active");
      return;
    }

    SetAutonomousMode(!isAutonomousMode);
  }

  public void SetAutonomousMode(bool enabled)
  {
    isAutonomousMode = enabled;

    if (enabled)
    {
      steeringPID.Reset();
    }
    else
    {
      SendAutonomousInput(0f, 0f, false);
    }
  }

  private void OnLaneWarningReceived(LaneWarningEvent laneEvent)
  {
    if (!isAutonomousMode) return;

    if (Mathf.Abs(laneEvent.laneOffset) > carController.laneOffsetThreshold * 1.5f)
    {
      Debug.LogWarning($"[AutonomousDriving] Lane departure: {laneEvent.laneOffset:F2}m");
    }
  }

  void OnDestroy()
  {
    EventManager.Unsubscribe<LaneWarningEvent>(OnLaneWarningReceived);
  }

  void OnDrawGizmos()
  {
    if (!Application.isPlaying || raycastOrigin == null) return;

    Vector3 rayStart = raycastOrigin.position + Vector3.up * RAYCAST_HEIGHT_OFFSET;

    for (int i = 0; i < frontRayCount; i++)
    {
      float angle = Mathf.Lerp(-raycastAngleRange, raycastAngleRange, (float)i / (frontRayCount - 1));
      Vector3 direction = Quaternion.Euler(0, angle, 0) * raycastOrigin.forward;

      Gizmos.color = hasObstacle ? Color.red : Color.green;
      Gizmos.DrawRay(rayStart, direction * forwardRayDistance);
    }

    Gizmos.color = Color.red;
    Gizmos.DrawWireSphere(raycastOrigin.position, emergencyBrakeDistance);

    Gizmos.color = Color.yellow;
    Gizmos.DrawWireSphere(raycastOrigin.position, safeFollowingDistance);
  }
}

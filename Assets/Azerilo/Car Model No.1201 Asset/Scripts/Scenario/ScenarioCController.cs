using UnityEngine;
using System.Collections;

public class ScenarioCController : MonoBehaviour
{
  [Header("Scenario Objects")]
  public CarController car;

  [Header("Movement Settings")]
  public float normalSpeed = 15;
  public float laneChangeSpeed = 15f;
  public float laneCorrectionSpeed = 30f;
  public float rightTurnSpeed = 20;

  [Header("Scenario Control")]
  public float straightDistance = 20f;
  public float laneChangeDistance = 21f;
  public float laneCorrectionDistance = 8f;
  public float rightTurnDelay = 2f;
  public float rightTurnDistance = 20f;

  private Vector3 stateStartPosition;
  private float distanceTraveled;

  private enum CarState
  {
    Normal,
    LaneChanging,
    LaneCorrecting,
    RightTurning
  }

  [Header("Movement Control")]
  public float laneChangeSteer = 0.5f;
  public float laneCorrectionSteer = -1.0f;
  public float rightTurnSteer = 5.0f;
  public float laneChangeThrottle = 0.8f;
  public float rightTurnThrottle = 1.0f;

  private AutonomousDrivingController carAutonomous;
  private HUDManager hudManager;
  private LEDManager ledManager;
  private CarState currentCarState = CarState.Normal;
  private bool isManualControlActive = false;
  private bool scenarioStarted = false;
  private bool scenarioEnded = false;

  void Start()
  {
    InitializeComponents();

    if (carAutonomous != null)
    {
      carAutonomous.isAutonomousMode = true;
      carAutonomous.targetSpeed = normalSpeed;
    }

    StartCoroutine(StartScenarioCoroutine());
  }

  void InitializeComponents()
  {
    if (car == null)
    {
      Debug.LogError("[ScenarioC] Car 참조가 설정되지 않았습니다!");
      enabled = false;
      return;
    }

    carAutonomous = car.GetComponent<AutonomousDrivingController>();
    hudManager = FindFirstObjectByType<HUDManager>();
    ledManager = FindFirstObjectByType<LEDManager>();

    if (carAutonomous == null)
    {
      Debug.LogError("[ScenarioC] AutonomousDrivingController를 찾을 수 없습니다!");
      enabled = false;
      return;
    }

    if (hudManager == null || ledManager == null )
    {
      Debug.LogError("[ScenarioC] 매니저를 찾을 수 없습니다!");
    }
  }

  IEnumerator StartScenarioCoroutine()
  {
    yield return new WaitForSeconds(0.5f);

    StartScenario();
    scenarioStarted = true;
  }

  void StartScenario()
  {
    if (carAutonomous != null)
    {
      carAutonomous.isAutonomousMode = true;
      carAutonomous.targetSpeed = normalSpeed;
      carAutonomous.enableEmergencyBraking = false;
    }

    StartCoroutine(ExecuteScenarioCoroutine());
  }

  IEnumerator ExecuteScenarioCoroutine()
  {
    yield return StartCoroutine(ExecuteDistanceBasedScenario());
    EndScenario();
  }

  IEnumerator ExecuteDistanceBasedScenario()
  {
    SetCarState(CarState.Normal);
    yield return StartCoroutine(WaitForDistance(straightDistance));

    TriggerBypassTrafficEvent();
    SetCarState(CarState.LaneChanging);
    yield return StartCoroutine(WaitForDistance(laneChangeDistance));

    SetCarState(CarState.LaneCorrecting);
    yield return StartCoroutine(WaitForDistance(laneCorrectionDistance));
    
    TriggerBypassTrafficEndEvent();
    SetCarState(CarState.Normal);
    yield return new WaitForSeconds(rightTurnDelay);

    SetCarState(CarState.RightTurning);
    yield return StartCoroutine(WaitForDistance(rightTurnDistance));

    SetCarState(CarState.Normal);
  }

  IEnumerator WaitForDistance(float targetDistance)
  {
    if (car == null) yield break;

    stateStartPosition = car.transform.position;
    distanceTraveled = 0f;

    while (distanceTraveled < targetDistance)
    {
      Vector3 currentPosition = car.transform.position;
      distanceTraveled = Vector3.Distance(stateStartPosition, currentPosition);
      yield return null;
    }
  }

  void EndScenario()
  {
    if (scenarioEnded) return;

    scenarioEnded = true;
    currentCarState = CarState.Normal;

    SetCarState(CarState.Normal);
  }

  void RestoreOriginalSettings()
  {
    if (carAutonomous != null)
    {
      carAutonomous.SetAutonomousMode(true);
      carAutonomous.targetSpeed = normalSpeed;
      carAutonomous.enableEmergencyBraking = true;
      carAutonomous.SetSteeringOverride(false);
    }
    isManualControlActive = false;
  }

  void SetCarState(CarState newState)
  {
    if (currentCarState == newState) return;

    currentCarState = newState;

    switch (newState)
    {
      case CarState.Normal:
        if (carAutonomous != null)
        {
          carAutonomous.SetAutonomousMode(true);
          carAutonomous.targetSpeed = normalSpeed;
          carAutonomous.SetSteeringOverride(false);
        }
        isManualControlActive = false;
        break;

      case CarState.LaneChanging:
        if (carAutonomous != null)
        {
          carAutonomous.SetAutonomousMode(true);
          carAutonomous.targetSpeed = laneChangeSpeed;
          carAutonomous.SetSteeringOverride(true, laneChangeSteer);
        }
        isManualControlActive = false;
        break;

      case CarState.LaneCorrecting:
        if (carAutonomous != null)
        {
          carAutonomous.SetAutonomousMode(true);
          carAutonomous.targetSpeed = laneCorrectionSpeed;
          carAutonomous.SetSteeringOverride(true, laneCorrectionSteer);
        }
        isManualControlActive = false;
        break;

      case CarState.RightTurning:
        if (carAutonomous != null)
        {
          carAutonomous.SetAutonomousMode(true);
          carAutonomous.targetSpeed = rightTurnSpeed;
          carAutonomous.SetSteeringOverride(true, rightTurnSteer);
        }
        isManualControlActive = false;
        break;
    }
  }

  void TriggerBypassTrafficEvent()
  {
    if (hudManager != null && ledManager != null )
    {
      EventManager.Publish(new HUDControlEvent("bypass-traffic", true));
      EventManager.Publish(new LEDControlEvent(1.0f, 1.0f, true, 0, "", SoundType.Default)); 
    }
  }

  void TriggerBypassTrafficEndEvent()
  {
    if (ledManager != null )
    {
      EventManager.Publish(new LEDControlEvent(0f, 0f, false, 0, "", SoundType.None)); 
    }
  }

  void Update()
  {
    if (!scenarioStarted || scenarioEnded) return;

    if (currentCarState == CarState.Normal && carAutonomous != null && carAutonomous.isAutonomousMode)
    {
      if (Mathf.Abs(carAutonomous.targetSpeed - normalSpeed) > 0.1f)
      {
        carAutonomous.targetSpeed = normalSpeed;
      }
    }

    if (isManualControlActive)
    {
      SendManualInput();
    }
  }

  void SendManualInput()
  {
    float throttle = 0f;
    float steer = 0f;
    bool brake = false;

    switch (currentCarState)
    {
      case CarState.LaneChanging:
        throttle = laneChangeThrottle;
        steer = laneChangeSteer;
        break;

      case CarState.RightTurning:
        throttle = rightTurnThrottle;
        steer = rightTurnSteer;
        break;

      case CarState.Normal:
      default:
        if (carAutonomous != null && !carAutonomous.isAutonomousMode)
        {
          carAutonomous.SetAutonomousMode(true);
          isManualControlActive = false;
        }
        return;
    }

    var inputEvent = new CarInputEvent(throttle, steer, brake, false);
    EventManager.Publish(inputEvent);
  }

  [ContextMenu("Restart Scenario")]
  public void RestartScenario()
  {
    StopAllCoroutines();

    scenarioStarted = false;
    scenarioEnded = false;
    currentCarState = CarState.Normal;

    RestoreOriginalSettings();

    StartCoroutine(StartScenarioCoroutine());
  }

  [ContextMenu("Stop Scenario")]
  public void StopScenario()
  {
    scenarioStarted = false;
    scenarioEnded = true;
    currentCarState = CarState.Normal;

    RestoreOriginalSettings();
  }
}

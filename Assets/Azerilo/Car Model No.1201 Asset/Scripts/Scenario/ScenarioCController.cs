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
  public float rightTurnSpeed = 30;

  [Header("Scenario Control")]
  public float straightDistance = 20f;
  public float laneChangeDistance = 21f;
  public float laneCorrectionDistance = 9f;
  public float beforeRightTurnDistance = 13f;
  public float rightTurnDistance = 10f;

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
  public float rightTurnSteer = 2.5f;
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
      carAutonomous.SetAutonomousMode(false);
    }

    // 즉시 수동제어 활성화
    isManualControlActive = true;
    currentCarState = CarState.Normal;
    Debug.Log("[ScenarioC] Start에서 즉시 수동제어 활성화");

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
      Debug.LogWarning("[ScenarioC] AutonomousDrivingController를 찾을 수 없습니다! Manual driving으로만 동작합니다.");
    }

    if (hudManager == null || ledManager == null )
    {
      Debug.LogError("[ScenarioC] 매니저를 찾을 수 없습니다!");
    }
  }

  IEnumerator StartScenarioCoroutine()
  {
    scenarioStarted = true;
    Debug.Log("[ScenarioC] 시나리오 시작됨 - scenarioStarted = true");
    yield return new WaitForSeconds(0.5f);

    StartScenario();
  }

  void StartScenario()
  {
    Debug.Log("[ScenarioC] StartScenario 호출됨");
    
    if (carAutonomous != null)
    {
      carAutonomous.SetAutonomousMode(false);
      carAutonomous.enableEmergencyBraking = false;
    }

    Debug.Log("[ScenarioC] ExecuteScenarioCoroutine 시작");
    StartCoroutine(ExecuteScenarioCoroutine());
  }

  IEnumerator ExecuteScenarioCoroutine()
  {
    yield return StartCoroutine(ExecuteDistanceBasedScenario());
    EndScenario();
  }

  IEnumerator ExecuteDistanceBasedScenario()
  {
    Debug.Log("[ScenarioC] ExecuteDistanceBasedScenario 시작");
    SetCarState(CarState.Normal);
    Debug.Log("[ScenarioC] SetCarState(Normal) 호출됨");
    yield return StartCoroutine(WaitForDistance(straightDistance));

    TriggerBypassTrafficEvent();
    SetCarState(CarState.LaneChanging);
    yield return StartCoroutine(WaitForDistance(laneChangeDistance));

    SetCarState(CarState.LaneCorrecting);
    yield return StartCoroutine(WaitForDistance(laneCorrectionDistance));
    
    TriggerBypassTrafficEndEvent();
    SetCarState(CarState.Normal);
    yield return StartCoroutine(WaitForDistance(beforeRightTurnDistance));

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
      carAutonomous.SetAutonomousMode(false);
      carAutonomous.enableEmergencyBraking = true;
    }
    isManualControlActive = true;
  }

  void SetCarState(CarState newState)
  {
    if (currentCarState == newState) return;

    Debug.Log($"[ScenarioC] SetCarState: {currentCarState} → {newState}");
    currentCarState = newState;

    switch (newState)
    {
      case CarState.Normal:
        if (carAutonomous != null)
        {
          carAutonomous.SetAutonomousMode(false);
        }
        isManualControlActive = true;
        Debug.Log("[ScenarioC] Normal state - isManualControlActive = true");
        break;

      case CarState.LaneChanging:
        if (carAutonomous != null)
        {
          carAutonomous.SetAutonomousMode(false);
        }
        isManualControlActive = true;
        Debug.Log("[ScenarioC] LaneChanging state - isManualControlActive = true");
        break;

      case CarState.LaneCorrecting:
        if (carAutonomous != null)
        {
          carAutonomous.SetAutonomousMode(false);
        }
        isManualControlActive = true;
        Debug.Log("[ScenarioC] LaneCorrecting state - isManualControlActive = true");
        break;

      case CarState.RightTurning:
        if (carAutonomous != null)
        {
          carAutonomous.SetAutonomousMode(false);
        }
        isManualControlActive = true;
        Debug.Log("[ScenarioC] RightTurning state - isManualControlActive = true");
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
    if (!scenarioStarted || scenarioEnded) 
    {
      Debug.Log($"[ScenarioC] Update blocked - scenarioStarted: {scenarioStarted}, scenarioEnded: {scenarioEnded}");
      return;
    }

    Debug.Log($"[ScenarioC] Update - isManualControlActive: {isManualControlActive}, currentCarState: {currentCarState}");
    
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
      case CarState.Normal:
        throttle = 0.4f;
        steer = 0f;
        break;
        
      case CarState.LaneChanging:
        throttle = 0.4f;
        steer = laneChangeSteer;
        break;
        
      case CarState.LaneCorrecting:
        throttle = 0.4f;
        steer = laneCorrectionSteer;
        break;

      case CarState.RightTurning:
        throttle = 0.4f;
        steer = rightTurnSteer;
        break;
    }

    Debug.Log($"[ScenarioC] SendManualInput - State: {currentCarState}, Throttle: {throttle}, Steer: {steer}");
    var movementEvent = new MovementControlEvent(throttle, steer, brake);
    EventManager.Publish(movementEvent);
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

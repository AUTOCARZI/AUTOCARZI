using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;

[RequireComponent(typeof(CarController))]
public class MLCarAgent : Agent
{
    [Header("Debug Info")]
    [SerializeField] private bool isAgentInitialized = false;
    [SerializeField] private bool hasDecisionRequester = false;
    [SerializeField] private string currentBehaviorType = "";
    [SerializeField] private bool isAcademyInitialized = false;
    [SerializeField] private int framesSinceLastAction = 0;

    [Header("Reset Settings")]
    private Vector3 initialPosition; // 시작 위치 자동 저장
    private Quaternion initialRotation; // 시작 회전 자동 저장

    [Header("Terrain Detection")]
    public float terrainCheckInterval = 0.1f;
    private float nextTerrainCheck = 0f;

    [Header("Centerline Detection")]
    public bool hasCenterlineViolation = false;
    public float centerlineViolationPenalty = -20f; // 강한 페널티
    
    [Header("Obstacle Detection")]
    public int rayCount = 5;
    public float rayDistance = 50f;
    public float rayAngleRange = 45f;
    public LayerMask obstacleLayer = -1;
    public Transform rayOrigin;
    
    [Header("Control Settings")]
    public float obstacleThreshold = 40f; // 이 거리보다 가까우면 ML이 제어
    public float maxSpeed = 80f;
    
    [Header("Rewards")]
    public float obstacleAvoidReward = 0.2f;
    public float laneKeepReward = 0.1f;
    public float collisionPenalty = -1f;
    public float priceReward = 5f; // price 태그 도달 보상
    public bool resetOnPriceReach = true; // price 도달 시 리셋 여부
    public float priceApproachReward = 0.1f; // 가까워질 때 보상
    public float priceDistancePenalty = -0.05f; // 멀어질 때 페널티
    public float forwardSpeedReward = 0.1f; // 전진 속도 보상
    public float stationaryPenalty = -0.1f; // 제자리에 있을 때 페널티
    public float directionAlignmentReward = 0.2f; // 목표 방향 일치 보상
    public float safeDirectionReward = 0.3f; // 안전한 방향으로 조향 시 보상
    public float dangerDirectionPenalty = -0.2f; // 위험한 방향으로 조향 시 페널티
    
    // Price 목표 추적용
    private GameObject priceTarget;
    private float lastDistanceToPrice = float.MaxValue;
    private Vector3 lastPosition;
    private float stationaryTime = 0f;
    
    // Ray 안전성 추적용
    private float[] rayAngles;
    private bool[] rayIsSafe;
    
    // Components
    private CarController carController;
    private Rigidbody carRigidbody;
    private DecisionRequester decisionRequester;
    private BehaviorParameters behaviorParameters;
    
    // State
    private float[] rayDistances;
    private bool hasObstacle;
    private float closestObstacleDistance;
    
    public override void Initialize()
    {
        carController = GetComponent<CarController>();
        carRigidbody = GetComponent<Rigidbody>();
        decisionRequester = GetComponent<DecisionRequester>();
        behaviorParameters = GetComponent<BehaviorParameters>();
        
        if (rayOrigin == null)
            rayOrigin = transform;
            
        rayDistances = new float[rayCount];
        rayAngles = new float[rayCount];
        rayIsSafe = new bool[rayCount];
        
        initialPosition = transform.position;
        initialRotation = transform.rotation;
        
        // Ray 각도 미리 계산
        for (int i = 0; i < rayCount; i++)
        {
            if (rayCount > 1)
            {
                rayAngles[i] = Mathf.Lerp(-rayAngleRange / 2f, rayAngleRange / 2f, (float)i / (rayCount - 1));
            }
            else
            {
                rayAngles[i] = 0f;
            }
        }
        
        // Price 태그 오브젝트 찾기
        priceTarget = GameObject.FindGameObjectWithTag("Price");
        if (priceTarget != null)
        {
            Debug.Log($"[ML Agent] Price target found: {priceTarget.name}");
            lastDistanceToPrice = Vector3.Distance(transform.position, priceTarget.transform.position);
        }
        else
        {
            Debug.LogWarning("[ML Agent] No object with 'price' tag found!");
        }
        
        // 디버그 정보 업데이트
        isAgentInitialized = true;
        hasDecisionRequester = decisionRequester != null;
        isAcademyInitialized = Academy.IsInitialized;
        
        if (behaviorParameters != null)
        {
            currentBehaviorType = behaviorParameters.BehaviorType.ToString();
        }
        
        Debug.Log($"[ML Agent] Initialized - Academy: {isAcademyInitialized}, DecisionRequester: {hasDecisionRequester}, BehaviorType: {currentBehaviorType}");
        
        // Decision Requester가 없으면 추가
        if (!hasDecisionRequester)
        {
            Debug.LogWarning("[ML Agent] DecisionRequester not found! Adding manually...");
            decisionRequester = gameObject.AddComponent<DecisionRequester>();
            decisionRequester.DecisionPeriod = 5;
            decisionRequester.TakeActionsBetweenDecisions = true;
            hasDecisionRequester = true;
            Debug.Log("[ML Agent] DecisionRequester added successfully");
        }
        
        // BehaviorParameters 확인
        if (behaviorParameters == null)
        {
            Debug.LogError("[ML Agent] BehaviorParameters not found!");
        }
        else
        {
            Debug.Log($"[ML Agent] Behavior Name: {behaviorParameters.BehaviorName}, Type: {behaviorParameters.BehaviorType}");
        }
    }

    public override void OnEpisodeBegin()
    {
        Debug.Log("[ML Agent] Episode Begin called");
        ResetToInitialPosition();
        framesSinceLastAction = 0;
        
        // Price 목표까지의 거리 초기화
        if (priceTarget != null)
        {
            lastDistanceToPrice = Vector3.Distance(transform.position, priceTarget.transform.position);
            Debug.Log($"[ML Agent] Initial distance to price: {lastDistanceToPrice:F1}m");
        }
        
        // 위치 추적 초기화
        lastPosition = transform.position;
        stationaryTime = 0f;
    }

    void Start()
    {
        Debug.Log("[ML Agent] Start() called");
        
        // Academy 상태 확인
        Debug.Log($"[ML Agent] Academy Initialized: {Academy.IsInitialized}");
        Debug.Log($"[ML Agent] Agent GameObject: {gameObject.name}");
        Debug.Log($"[ML Agent] Agent enabled: {enabled}");
    }

    void Update()
    {
        framesSinceLastAction++;
        
        // 디버그 정보 업데이트 (1초마다)
        if (Time.frameCount % 60 == 0)
        {
            isAcademyInitialized = Academy.IsInitialized;
            Debug.Log($"[ML Debug] Frames since last action: {framesSinceLastAction}, Academy: {isAcademyInitialized}");
            
            if (framesSinceLastAction > 300) // 5초 이상 액션이 없으면
            {
                Debug.LogWarning("[ML Agent] No actions received for 5+ seconds! Checking setup...");
                CheckMLAgentSetup();
            }
        }
        
        // 0.1초마다 Terrain 체크
        if (Time.time >= nextTerrainCheck)
        {
            nextTerrainCheck = Time.time + terrainCheckInterval;
        
            if (IsOnTerrain())
            {
                Debug.Log("[ML Agent] On Terrain detected - Resetting");
                ResetToInitialPosition();
                AddReward(collisionPenalty * 0.3f);
            }
        }
    }

    private void CheckMLAgentSetup()
    {
        Debug.Log("=== ML Agent Setup Check ===");
        Debug.Log($"Agent Enabled: {enabled}");
        Debug.Log($"Agent GameObject Active: {gameObject.activeInHierarchy}");
        Debug.Log($"Academy Initialized: {Academy.IsInitialized}");
        Debug.Log($"DecisionRequester: {GetComponent<DecisionRequester>() != null}");
        Debug.Log($"BehaviorParameters: {GetComponent<BehaviorParameters>() != null}");
        
        var behavior = GetComponent<BehaviorParameters>();
        if (behavior != null)
        {
            Debug.Log($"Behavior Name: {behavior.BehaviorName}");
            Debug.Log($"Behavior Type: {behavior.BehaviorType}");
            Debug.Log($"Model: {behavior.Model}");
        }
        
        var requester = GetComponent<DecisionRequester>();
        if (requester != null)
        {
            Debug.Log($"Decision Period: {requester.DecisionPeriod}");
            Debug.Log($"Take Actions Between Decisions: {requester.TakeActionsBetweenDecisions}");
        }
        Debug.Log("========================");
    }

    private bool IsOnTerrain()
    {
        Vector3 rayStart = transform.position + Vector3.up * 0.5f;
    
        // 아래쪽으로 Raycast
        if (Physics.Raycast(rayStart, Vector3.down, out RaycastHit hit, 2f))
        {
            // TerrainCollider 감지
            if (hit.collider.GetComponent<TerrainCollider>() != null)
            {
                Debug.DrawRay(rayStart, Vector3.down * hit.distance, Color.red, 0.1f);
                return true;
            }
        }
    
        Debug.DrawRay(rayStart, Vector3.down * 2f, Color.green, 0.1f);
        return false;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Debug.Log($"[ML Agent] CollectObservations called - Frame: {Time.frameCount}");
        
        // 1. Ray distances (정규화)
        PerformRaycast();
        for (int i = 0; i < rayCount; i++)
        {
            sensor.AddObservation(rayDistances[i] / rayDistance);
        }
        
        // 2. Ray safety information (새로 추가)
        for (int i = 0; i < rayCount; i++)
        {
            sensor.AddObservation(rayIsSafe[i] ? 1f : 0f);
        }
        
        // 3. Current speed
        float currentSpeed = carRigidbody.linearVelocity.magnitude * 3.6f;
        sensor.AddObservation(currentSpeed / maxSpeed);
        
        // 4. Lane info
        sensor.AddObservation(carController.currentLaneOffset);
        sensor.AddObservation(carController.isLaneDetectionActive ? 1f : 0f);
        
        // 5. Vehicle velocity
        Vector3 localVel = transform.InverseTransformDirection(carRigidbody.linearVelocity);
        sensor.AddObservation(localVel.x); // 횡방향
        sensor.AddObservation(localVel.z); // 전진방향

        // 6. Centerline violation (새로 추가)
        sensor.AddObservation(hasCenterlineViolation ? 1f : 0f);
        
        // 7. Obstacle direction info (새로 추가)
        if (hasObstacle)
        {
            // 가장 가까운 장애물의 각도
            float closestObstacleAngle = 0f;
            for (int i = 0; i < rayCount; i++)
            {
                if (rayDistances[i] == closestObstacleDistance)
                {
                    closestObstacleAngle = rayAngles[i];
                    break;
                }
            }
            sensor.AddObservation(closestObstacleAngle / (rayAngleRange / 2f)); // 정규화
            sensor.AddObservation(closestObstacleDistance / rayDistance); // 정규화된 거리
        }
        else
        {
            sensor.AddObservation(0f); // 장애물 각도 없음
            sensor.AddObservation(1f); // 장애물 거리 최대
        }
        
        // 8. Safe direction info (새로 추가)
        float bestSafeAngle = 0f;
        float maxSafeDistance = 0f;
        int safeRayCount = 0;
        
        for (int i = 0; i < rayCount; i++)
        {
            if (rayIsSafe[i])
            {
                safeRayCount++;
                if (rayDistances[i] > maxSafeDistance)
                {
                    maxSafeDistance = rayDistances[i];
                    bestSafeAngle = rayAngles[i];
                }
            }
        }
        
        sensor.AddObservation(bestSafeAngle / (rayAngleRange / 2f)); // 정규화된 안전한 각도
        sensor.AddObservation(safeRayCount / (float)rayCount); // 안전한 Ray 비율
        
        Debug.Log($"[ML Agent] Observations collected: {rayCount * 2 + 11} total");
        Debug.Log($"[ML Debug] Safe rays: {safeRayCount}/{rayCount}, Best safe angle: {bestSafeAngle:F1}°");
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        framesSinceLastAction = 0; // 액션 받았으므로 리셋
        
        Debug.Log($"[ML Agent] 🎉 OnActionReceived called! Frame: {Time.frameCount}");
        Debug.Log($"[ML Agent] Action values: Steer={actions.ContinuousActions[0]:F3}, Throttle={actions.ContinuousActions[1]:F3}");
        
        float mlSteer = Mathf.Clamp(actions.ContinuousActions[0], -1f, 1f);
        float mlThrottle = Mathf.Clamp(actions.ContinuousActions[1], -1f, 1f);
        
        float finalSteer, finalThrottle;
        
        Debug.Log($"[ML] CLOSESTOB Distance: {closestObstacleDistance:F1}m");

        // 간단한 하이브리드 로직
        if (hasObstacle && closestObstacleDistance < obstacleThreshold)
        {
            Debug.Log($"[ML] 🤖 ML Agent controlling! Distance: {closestObstacleDistance:F1}m");
            // ML이 장애물 회피
            finalSteer = mlSteer;
            finalThrottle = mlThrottle;
        }
        else
        {
            Debug.Log($"[ML] 🛣️ Lane following mode. Distance: {closestObstacleDistance:F1}m");
            
            // Lane detection으로 차선 유지
            if (carController.isLaneDetectionActive)
            {
                finalSteer = -carController.currentLaneOffset * 2f; // 간단한 P 제어
                finalSteer = Mathf.Clamp(finalSteer, -1f, 1f);
                Debug.Log($"[ML] Using lane detection: offset={carController.currentLaneOffset:F2}");
            }
            else
            {
                finalSteer = mlSteer; // Lane detection 없으면 ML 사용
                Debug.Log("[ML] No lane detection, using ML steering");
            }
            
            // 속도는 장애물 거리에 따라 조정
            float targetSpeed = hasObstacle ? 
                maxSpeed * (closestObstacleDistance / rayDistance) : maxSpeed;
            float currentSpeed = carRigidbody.linearVelocity.magnitude * 3.6f;
            finalThrottle = (targetSpeed - currentSpeed) / 30f; // 간단한 속도 제어
            finalThrottle = Mathf.Clamp(finalThrottle, -1f, 1f);
            
            Debug.Log($"[ML] Speed control: target={targetSpeed:F1}, current={currentSpeed:F1}");
        }
        
        Debug.Log($"[ML] Final control: steer={finalSteer:F2}, throttle={finalThrottle:F2}");
        
        // 제어 적용
        var inputEvent = new CarInputEvent(finalThrottle, finalSteer, false, false);
        EventManager.Publish(inputEvent);
        
        // 보상 계산
        CalculateRewards();
    }

    private void PerformRaycast()
    {
        hasObstacle = false;
        closestObstacleDistance = float.MaxValue;
        
        Vector3 rayStart = rayOrigin.position + Vector3.up * 0.5f;
        
        Debug.Log($"[Ray Debug] Starting raycast - rayDistance: {rayDistance}, rayCount: {rayCount}");
        
        for (int i = 0; i < rayCount; i++)
        {
            float angle = 0f;
            if (rayCount > 1)
            {
                angle = Mathf.Lerp(-rayAngleRange / 2f, rayAngleRange / 2f, (float)i / (rayCount - 1));
            }
            
            Vector3 rayDirection = Quaternion.Euler(0, angle, 0) * rayOrigin.forward;
            
            if (Physics.Raycast(rayStart, rayDirection, out RaycastHit hit, rayDistance, obstacleLayer))
            {
                rayDistances[i] = hit.distance;
                rayIsSafe[i] = hit.distance > obstacleThreshold;
                Debug.Log($"[Ray Debug] Ray {i} HIT: distance={hit.distance:F1}m, object={hit.collider.name}, layer={hit.collider.gameObject.layer}");
                
                if (hit.distance < closestObstacleDistance)
                {
                    closestObstacleDistance = hit.distance;
                    hasObstacle = true;
                }


                
                // 🔧 Hit된 경우: 빨간색으로 hit 지점까지만
                Debug.DrawRay(rayStart, rayDirection * hit.distance, Color.red, 1f);
                // 🔧 나머지 구간: 노란색 점선으로 표시
                Debug.DrawRay(rayStart + rayDirection * hit.distance, rayDirection * (rayDistance - hit.distance), Color.yellow, 1f);
            }
            else
            {
                rayIsSafe[i] = true;
                rayDistances[i] = rayDistance;
                Debug.Log($"[Ray Debug] Ray {i} NO HIT: full distance {rayDistance}m");
                
                // 🔧 Hit 안된 경우: 초록색으로 전체 거리
                Debug.DrawRay(rayStart, rayDirection * rayDistance, Color.green, 1f);
            }
        }
        
        Debug.Log($"[Ray Debug] Result - hasObstacle: {hasObstacle}, closestDistance: {closestObstacleDistance:F1}m");
    }

    private void CalculateRewards()
    {
        float reward = 0f;
        
        // 현재 속도와 위치
        float currentSpeed = carRigidbody.linearVelocity.magnitude * 3.6f;
        Vector3 currentPosition = transform.position;
        
        // 안전한 방향으로의 조향 보상/페널티
        if (hasObstacle)
        {
            float currentSteerInput = 0f; // 마지막 조향 입력 (OnActionReceived에서 가져와야 함)
            
            // 현재 조향 방향 계산 (대략적)
            Vector3 currentDirection = transform.forward;
            Vector3 leftDirection = Quaternion.Euler(0, -45, 0) * currentDirection;
            Vector3 rightDirection = Quaternion.Euler(0, 45, 0) * currentDirection;
            
            // 가장 안전한 방향 찾기
            float bestSafeAngle = 0f;
            float maxSafeDistance = 0f;
            bool foundSafeDirection = false;
            
            for (int i = 0; i < rayCount; i++)
            {
                if (rayIsSafe[i] && rayDistances[i] > maxSafeDistance)
                {
                    maxSafeDistance = rayDistances[i];
                    bestSafeAngle = rayAngles[i];
                    foundSafeDirection = true;
                }
            }
            
            if (foundSafeDirection)
            {
                // 차량의 현재 이동 방향
                Vector3 velocity = carRigidbody.linearVelocity;
                if (velocity.magnitude > 1f)
                {
                    Vector3 movementDirection = velocity.normalized;
                    Vector3 safeDirection = Quaternion.Euler(0, bestSafeAngle, 0) * transform.forward;
                    
                    // 안전한 방향으로 움직이고 있는지 확인
                    float safeAlignment = Vector3.Dot(movementDirection, safeDirection);
                    
                    if (safeAlignment > 0.5f) // 안전한 방향으로 움직임
                    {
                        reward += safeDirectionReward * safeAlignment;
                        Debug.Log($"[ML Reward] Safe direction reward: +{safeDirectionReward * safeAlignment:F3} (angle: {bestSafeAngle:F1}°, alignment: {safeAlignment:F2})");
                    }
                    else if (safeAlignment < -0.3f) // 위험한 방향으로 움직임
                    {
                        // 가장 위험한 방향 확인
                        float worstDangerDistance = rayDistance;
                        for (int i = 0; i < rayCount; i++)
                        {
                            if (!rayIsSafe[i] && rayDistances[i] < worstDangerDistance)
                            {
                                worstDangerDistance = rayDistances[i];
                            }
                        }
                        
                        if (worstDangerDistance < 20f) // 20m 이내 위험
                        {
                            reward += dangerDirectionPenalty;
                            Debug.Log($"[ML Reward] Danger direction penalty: {dangerDirectionPenalty} (moving toward obstacle at {worstDangerDistance:F1}m)");
                        }
                    }
                }
                
                Debug.Log($"[ML Debug] Best safe direction: {bestSafeAngle:F1}° (distance: {maxSafeDistance:F1}m)");
            }
            else
            {
                // 안전한 방향이 없음 - 감속 권장
                if (currentSpeed > 30f)
                {
                    reward -= 0.1f;
                    Debug.Log($"[ML Reward] No safe direction, speed too high penalty: -0.1");
                }
            }
        }
        
        // 전진 속도 보상 (강화)
        Vector3 localVel = transform.InverseTransformDirection(carRigidbody.linearVelocity);
        if (localVel.z > 5f) // 최소 5km/h 이상일 때만 보상
        {
            reward += forwardSpeedReward * (localVel.z / maxSpeed);
            Debug.Log($"[ML Reward] Forward speed reward: +{forwardSpeedReward * (localVel.z / maxSpeed):F3} (speed: {currentSpeed:F1}km/h)");
        }
        
        // 제자리에 머물기 페널티
        float movementDistance = Vector3.Distance(currentPosition, lastPosition);
        if (movementDistance < 0.5f) // 0.5m 미만 이동
        {
            stationaryTime += Time.fixedDeltaTime;
            if (stationaryTime > 2f) // 2초 이상 제자리
            {
                reward += stationaryPenalty;
                Debug.Log($"[ML Reward] Stationary penalty: {stationaryPenalty} (time: {stationaryTime:F1}s)");
            }
        }
        else
        {
            stationaryTime = 0f; // 움직이면 리셋
        }
        lastPosition = currentPosition;
        
        // Price 목표까지의 거리 보상
        if (priceTarget != null)
        {
            float currentDistanceToPrice = Vector3.Distance(transform.position, priceTarget.transform.position);
            
            // 목표 방향 일치도 보상 (강화)
            Vector3 directionToTarget = (priceTarget.transform.position - transform.position).normalized;
            Vector3 carDirection = transform.forward;
            float alignment = Vector3.Dot(carDirection, directionToTarget);
            
            if (alignment > 0.3f && currentSpeed > 10f) // 목표 방향으로 빠르게 움직일 때
            {
                reward += directionAlignmentReward * alignment * (currentSpeed / maxSpeed);
                Debug.Log($"[ML Reward] Direction alignment: +{directionAlignmentReward * alignment * (currentSpeed / maxSpeed):F3} (alignment: {alignment:F2})");
            }
            
            // 거리 변화 계산
            float distanceChange = currentDistanceToPrice - lastDistanceToPrice;
            
            if (distanceChange < -1f) // 1m 이상 가까워짐
            {
                float approachReward = Mathf.Abs(distanceChange) * priceApproachReward;
                reward += approachReward;
                Debug.Log($"[ML Reward] Approaching target! Distance: {currentDistanceToPrice:F1}m, Reward: +{approachReward:F3}");
            }
            else if (distanceChange > 1f) // 1m 이상 멀어짐
            {
                float distancePenalty = distanceChange * priceDistancePenalty;
                reward += distancePenalty;
                Debug.Log($"[ML Reward] Moving away! Distance: {currentDistanceToPrice:F1}m, Penalty: {distancePenalty:F3}");
            }
            
            lastDistanceToPrice = currentDistanceToPrice;
            
            // 목표까지 거리별 추가 보상
            if (currentDistanceToPrice < 50f)
            {
                reward += 0.05f;
                Debug.Log($"[ML Reward] Within 50m bonus: +0.05");
            }
            if (currentDistanceToPrice < 20f)
            {
                reward += 0.1f;
                Debug.Log($"[ML Reward] Within 20m bonus: +0.1");
            }
            if (currentDistanceToPrice < 10f)
            {
                reward += 0.2f;
                Debug.Log($"[ML Reward] Within 10m bonus: +0.2");
            }
            
            // 너무 멀어지면 큰 페널티
            if (currentDistanceToPrice > 300f)
            {
                reward -= 1.0f;
                Debug.LogWarning($"[ML Reward] Too far from target! Distance: {currentDistanceToPrice:F1}m, Big penalty: -1.0");
            }
        }
        
        // 너무 느린 속도 페널티
        if (currentSpeed < 5f && localVel.z >= 0) // 전진하려 하지만 너무 느림
        {
            reward -= 0.02f;
            Debug.Log($"[ML Reward] Too slow penalty: -0.02 (speed: {currentSpeed:F1}km/h)");
        }
        
        // 후진 페널티 (목표에서 멀어지는 경우만)
        if (localVel.z < -2f)
        {
            reward -= 0.05f;
            Debug.Log($"[ML Reward] Reverse penalty: -0.05");
        }
        
        // 장애물 회피 보상
        if (hasObstacle)
        {
            reward += obstacleAvoidReward * (closestObstacleDistance / rayDistance);
        }
        
        // 차선 유지 보상 (기본적으로 비활성화)
        if (carController.isLaneDetectionActive && false) // ML이 항상 제어하므로 비활성화
        {
            reward += laneKeepReward * (1f - Mathf.Abs(carController.currentLaneOffset));
        }

        if (hasCenterlineViolation)
        {
            reward += centerlineViolationPenalty;
            Debug.LogWarning("[ML Agent] CENTERLINE VIOLATION PENALTY!");
        }
        
        AddReward(reward);
        
        // 총 보상 로그 (10초마다)
        if (Time.frameCount % 600 == 0)
        {
            Debug.Log($"[ML Reward] Total reward this step: {reward:F3}, Speed: {currentSpeed:F1}km/h");
        }
    }

    void OnCollisionEnter(Collision collision)
    {
        // Price 태그 감지 (Collision)
        if (collision.gameObject.CompareTag("Price"))
        {
            Debug.Log($"[ML Agent] PRICE REACHED! +{priceReward} reward");
            AddReward(priceReward);
            
            if (resetOnPriceReach)
            {
                ResetToInitialPosition();
                EndEpisode();
            }
            return;
        }
        
        if (collision.gameObject.GetComponent<TerrainCollider>() != null)
        {
            Debug.Log("Collision with TerrainCollider!");
            ResetToInitialPosition();
        }
        // 장애물과 충돌 시 처리
        if (collision.gameObject.layer == LayerMask.NameToLayer("Obstacle") || 
            collision.gameObject.CompareTag("Obstacle"))
        {
            Debug.Log($"[ML Agent] Collision with: {collision.gameObject.name}");
            
            ResetToInitialPosition();

            AddReward(collisionPenalty);  // -1점
            EndEpisode();                 // 에피소드 종료
        }
    }

    // Trigger 감지 추가 (Box Collider가 Trigger인 경우)
    void OnTriggerEnter(Collider other)
    {
        // Price 태그 감지 (Trigger)
        if (other.CompareTag("price"))
        {
            Debug.Log($"[ML Agent] PRICE TRIGGER REACHED! +{priceReward} reward");
            AddReward(priceReward);
            
            if (resetOnPriceReach)
            {
                Debug.Log("[ML Agent] Resetting after reaching price target");
                ResetToInitialPosition();
                EndEpisode();
            }
            return;
        }
        
        // 다른 트리거 처리...
        Debug.Log($"[ML Agent] Trigger entered: {other.name}, tag: {other.tag}");
    }

    private void ResetToInitialPosition()
    {
        // 속도 먼저 0으로
        carRigidbody.linearVelocity = Vector3.zero;
        carRigidbody.angularVelocity = Vector3.zero;
    
        // 저장된 초기 위치로 복원
        transform.position = initialPosition;
        transform.rotation = initialRotation;
    
        Debug.Log($"[ML Agent] Reset to initial position: {initialPosition}");
    }

    public void OnCollisionDetected()
    {
        AddReward(collisionPenalty);
        //EndEpisode();
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        Debug.Log("[ML Agent] Heuristic called - using keyboard input");
        var actions = actionsOut.ContinuousActions;
        actions[0] = Input.GetAxis("Horizontal"); // Steering
        actions[1] = Input.GetAxis("Vertical");   // Throttle
    }
    
    // 간단한 상태 확인용
    public bool IsMLControlling() => true; // 항상 ML이 제어
    public float GetObstacleDistance() => closestObstacleDistance;
}
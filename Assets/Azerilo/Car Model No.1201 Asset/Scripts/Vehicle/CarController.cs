using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net.Sockets;
using System.Net;
using System.Threading;

[RequireComponent(typeof(InputManager))]
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(LightingManager))]
[RequireComponent(typeof(MovementProcessor))]
public class CarController : MonoBehaviour
{
    [Header("Car Physics")]
    public List<WheelCollider> throttleWheels;
    public List<GameObject> steeringWheels;
    public List<GameObject> meshes;
    public float strengthCoefficient = 10000f;
    public float maxTurn = 20f;
    public Transform CM;
    public float brakeStrength;

    [Header("Lane Detection Response")]
    public float currentLaneOffset = 0f;
    public bool isLaneDetectionActive = false;
    public float laneOffsetThreshold = 0.3f;

    [Header("UDP Video Streaming")]
    public Camera laneDetectionCamera;
    public string serverIP = "127.0.0.1";
    public int videoPort = 8888;
    public int responsePort = 8889;
    public float targetFPS = 15f; // FPS 낮춤
    public int imageWidth = 320; // 해상도 낮춤
    public int imageHeight = 240; // 해상도 낮춤  
    public int jpegQuality = 30; // 품질 더 낮춤
    public int maxPacketSize = 1400; // 안전한 크기로 변경

    // UDP 스트리밍 관련
    private UdpClient videoClient;
    private UdpClient responseClient;
    private IPEndPoint serverEndPoint;
    private Thread responseThread;
    private bool isStreaming = false;
    private bool usePNG = true;

    // 카메라 관련
    private RenderTexture renderTexture;
    private Texture2D captureTexture;
    private float captureInterval;
    private float nextCaptureTime = 0f;

    // 패킷 순번 관리
    private uint sequenceNumber = 0;

    [Header("Audio Detection")]
    public CarControllerAmbulance ambulanceToDetect;
    public float ambulanceVolumeThreshold = 0.3f;

    [Header("Car Horn Detection")]
    public AudioSource carHornSource;
    public Transform carHornTransform;
    public float carHornVolumeThreshold = 0.5f;

    [Header("Sound Response System")]
    public SoundResponseManager soundResponseManager = new SoundResponseManager();

    private InputManager im;
    private LightingManager lm;
    private Rigidbody rb;

    // Audio detection variables
    private float currentAmbulanceVolume;
    private Vector3 ambulanceDirection;
    private float ambulanceDistance;
    private string ambulanceRelativeDirection;
    private float currentBlinkThreshold;

    // HUD 상태 관리 (중복 이벤트 방지)
    private string currentActiveHUD = "";

    // 사운드 소스 맵
    private Dictionary<SoundType, ISoundSource> soundSources;

    void Start()
    {
        im = GetComponent<InputManager>();
        lm = GetComponent<LightingManager>();
        rb = GetComponent<Rigidbody>();

        if (CM)
        {
            rb.centerOfMass = CM.position;
        }

        if (ambulanceToDetect == null)
        {
            ambulanceToDetect = FindFirstObjectByType<CarControllerAmbulance>();
        }

        InitializeSoundSystem();
        InitializeLaneDetectionCamera();
        InitializeUDPStreaming();

        // 모든 이벤트 구독
        EventManager.Subscribe<SoundEvent>(OnSoundEventReceived);
        EventManager.Subscribe<CarInputEvent>(OnCarInputEventReceived);

        Debug.Log("[CarController] CarController initialized with Emergency Response System");
    }

    void OnDestroy()
    {
        // 스트리밍 중단
        isStreaming = false;

        // 스레드 정리
        if (responseThread != null && responseThread.IsAlive)
        {
            responseThread.Join(1000);
        }

        // UDP 클라이언트 정리
        if (videoClient != null)
        {
            videoClient.Close();
            videoClient.Dispose();
        }

        if (responseClient != null)
        {
            responseClient.Close();
            responseClient.Dispose();
        }

        // RenderTexture 정리 (GetTemporary 사용)
        if (renderTexture != null)
        {
            RenderTexture.ReleaseTemporary(renderTexture);
            renderTexture = null;
        }

        if (captureTexture != null)
        {
            DestroyImmediate(captureTexture);
        }

        // 이벤트 구독 해제
        EventManager.Unsubscribe<SoundEvent>(OnSoundEventReceived);
        EventManager.Unsubscribe<CarInputEvent>(OnCarInputEventReceived);
    }

    void InitializeSoundSystem()
    {
        soundResponseManager.Initialize();

        // 사운드 소스 맵 초기화
        soundSources = new Dictionary<SoundType, ISoundSource>();

        // 앰뷸런스 프로필 등록
        if (ambulanceToDetect != null)
        {
            soundSources[SoundType.Ambulance] = new AmbulanceSoundSource(ambulanceToDetect);
        }

        // 임시로 앰뷸런스를 경적 소스로 등록
        if (carHornSource == null)
        {
            carHornSource = ambulanceToDetect.GetHornAudioSource();
            Debug.Log("[CarController] Using ambulance horn as car horn source");
        }

        if (carHornTransform == null)
        {
            carHornTransform = ambulanceToDetect.transform;
            Debug.Log("[CarController] Using ambulance transform as car horn transform");
        }

        soundSources[SoundType.CarHorn] = new CarHornSoundSource(carHornSource, carHornTransform);

        Debug.Log($"[CarController] Sound Response System initialized with {soundSources.Count} sources");

        // 모든 프로필 정보 출력
        foreach (SoundType soundType in soundResponseManager.GetRegisteredSoundTypes())
        {
            soundResponseManager.PrintProfileInfo(soundType);
        }
    }

    // 모든 사운드 소스 체크
    void CheckAllAudioSources()
    {
        foreach (var kvp in soundSources)
        {
            SoundType soundType = kvp.Key;
            ISoundSource source = kvp.Value;

            if (source.IsActive())
            {
                float volume = source.GetPerceivedVolume(this.transform);
                Vector3 sourcePosition = source.GetPosition();
                CheckSoundAndRespond(soundType, volume, sourcePosition);
            }
        }
    }

    // 사운드 체크 메서드
    private void CheckSoundAndRespond(SoundType soundType, float volume, Vector3 sourcePosition)
    {
        var profile = soundResponseManager.GetProfile(soundType);
        if (profile == null) return;

        if (volume > profile.activationThreshold)
        {
            Vector3 toSource = sourcePosition - this.transform.position;
            float distance = toSource.magnitude;
            Vector3 direction = toSource.normalized;
            string relativeDirection = GetRelativeDirection(direction);

            var soundDirection = ConvertToSoundDirection(relativeDirection);
            var soundEvent = new SoundEvent(
                soundDirection,
                distance,
                volume,
                soundType,
                sourcePosition,
                this.transform.position,
                soundResponseManager.GetDirectionThreshold(soundType, relativeDirection)
            );

            EventManager.Publish(soundEvent);
            Debug.Log($"[CarController] {soundType} detected: Volume={volume:F2}, Direction={relativeDirection}");
        }
        else
        {
            StopAllEmergencyEffects();
        }
    }

    private void StopAllEmergencyEffects()
    {
        // LED 중지
        var ledStopEvent = new LEDControlEvent(0f, 0f, false, 0f, "");
        EventManager.Publish(ledStopEvent);

        // HUD 중지 - 단, OneShot/Timed 모드는 자연스럽게 종료되도록 함
        if (!string.IsNullOrEmpty(currentActiveHUD))
        {
            // 경적과 같은 단발성 이벤트는 강제로 중단하지 않음
            if (!currentActiveHUD.StartsWith("horn-"))
            {
                EventManager.Publish(new HUDControlEvent(currentActiveHUD, false));
                currentActiveHUD = "";
                Debug.Log("[CarController] Continuous HUD stopped - volume below threshold");
            }
            else
            {
                Debug.Log("[CarController] OneShot HUD (horn) will complete naturally");
            }
        }

        Debug.Log("[CarController] Emergency effects processing completed");
    }

    // 이벤트 핸들러
    private void OnSoundEventReceived(SoundEvent soundEvent)
    {
        RouteToHUD(soundEvent);
        RouteToLED(soundEvent);
    }

    private void OnCarInputEventReceived(CarInputEvent inputEvent)
    {
        RouteToMovement(inputEvent);

        if (inputEvent.headlights)
        {
            lm.ToggleHeadlights();
        }
    }

    private void RouteToHUD(SoundEvent soundEvent)
    {
        string directionStr = ConvertToDirectionString(soundEvent.direction);
        var volumeLevel = soundResponseManager.GetVolumeLevel(soundEvent.soundType, soundEvent.volume);

        if (volumeLevel == null)
        {
            Debug.LogWarning($"[CarController] No volume level found for {soundEvent.soundType}");
            return;
        }

        Debug.Log($"[CarController] Sound Response: {soundEvent.soundType} - {volumeLevel.name}");
        Debug.Log($"[CarController] RouteToHUD Debug:");
        Debug.Log($"  - Sound Type: {soundEvent.soundType}");
        Debug.Log($"  - Direction: {directionStr}");
        Debug.Log($"  - Volume: {soundEvent.volume:F2}");
        Debug.Log($"  - Volume Level: {volumeLevel.name}");
        Debug.Log($"  - Should Show HUD: {volumeLevel.showHUD}");

        // 대상 HUD 결정 -> 고치기
        string targetHUD = "";
        if (volumeLevel.showHUD)
        {
            string[] targetHUDs = soundResponseManager.GetHUDsForDirection(soundEvent.soundType, directionStr);
            if (targetHUDs.Length > 0)
            {
                targetHUD = targetHUDs[0]; // 첫 번째 HUD 사용
            }
        }

        // 상태가 변경될 때만 이벤트 발행
        if (currentActiveHUD != targetHUD)
        {
            // 이전 HUD 끄기
            if (!string.IsNullOrEmpty(currentActiveHUD))
            {
                EventManager.Publish(new HUDControlEvent(currentActiveHUD, false));
                Debug.Log($"[CarController] Deactivating HUD: {currentActiveHUD}");
            }

            // 새 HUD 켜기
            if (!string.IsNullOrEmpty(targetHUD))
            {
                EventManager.Publish(new HUDControlEvent(targetHUD, true, directionStr));
                Debug.Log($"[CarController] Activating HUD: {targetHUD} for {soundEvent.soundType}");
            }

            currentActiveHUD = targetHUD;
            Debug.Log($"[CarController] HUD state changed to: {targetHUD}");
        }
    }

    // 모듈화된 LED 라우팅
    private void RouteToLED(SoundEvent soundEvent)
    {
        string directionStr = ConvertToDirectionString(soundEvent.direction);
        var volumeLevel = soundResponseManager.GetVolumeLevel(soundEvent.soundType, soundEvent.volume);
        var profile = soundResponseManager.GetProfile(soundEvent.soundType);

        if (volumeLevel == null || profile == null)
        {
            Debug.LogWarning($"[CarController] Missing profile or volume level for {soundEvent.soundType}");
            return;
        }

        float directionThreshold = soundResponseManager.GetDirectionThreshold(soundEvent.soundType, directionStr);
        bool shouldBlink = volumeLevel.activateLED && soundEvent.volume >= directionThreshold;

        Debug.Log($"[CarController] RouteToLED Debug:");
        Debug.Log($"  - Sound Type: {soundEvent.soundType}");
        Debug.Log($"  - Direction: {directionStr}");
        Debug.Log($"  - Volume: {soundEvent.volume:F2}");
        Debug.Log($"  - Direction Threshold: {directionThreshold:F2}");
        Debug.Log($"  - Volume Level: {volumeLevel.name}");
        Debug.Log($"  - Should Blink: {shouldBlink}");

        if (shouldBlink)
        {
            // 프로필에서 깜빡임 설정 가져오기
            float blinkSpeed = profile.ledBlinkSpeed;
            float timerSpeed = profile.ledTimerSpeed;

            var ledEvent = new LEDControlEvent(blinkSpeed, timerSpeed, true, soundEvent.volume, directionStr);
            EventManager.Publish(ledEvent);

            Debug.Log($"[CarController] LED Event: Speed={blinkSpeed:F2}, Timer={timerSpeed:F2} (From {soundEvent.soundType} profile)");
        }
        else
        {
            var ledEvent = new LEDControlEvent(0f, 0f, false, soundEvent.volume, directionStr);
            EventManager.Publish(ledEvent);

            Debug.Log($"[CarController] LED Event: STOP (Level: {volumeLevel.name})");
        }
    }

    private void RouteToMovement(CarInputEvent inputEvent)
    {
        var movementEvent = new MovementControlEvent(inputEvent.throttle, inputEvent.steer, inputEvent.brake);

        EventManager.Publish(movementEvent);
    }


    private SoundEvent.Direction ConvertToSoundDirection(string directionStr)
    {
        switch (directionStr)
        {
            case "ahead": return SoundEvent.Direction.Ahead;
            case "ahead-right": return SoundEvent.Direction.AheadRight;
            case "to the right": return SoundEvent.Direction.Right;
            case "behind-right": return SoundEvent.Direction.BehindRight;
            case "behind": return SoundEvent.Direction.Behind;
            case "behind-left": return SoundEvent.Direction.BehindLeft;
            case "to the left": return SoundEvent.Direction.Left;
            case "ahead-left": return SoundEvent.Direction.AheadLeft;
            default: return SoundEvent.Direction.Ahead;
        }
    }

    private string ConvertToDirectionString(SoundEvent.Direction direction)
    {
        switch (direction)
        {
            case SoundEvent.Direction.Ahead: return "ahead";
            case SoundEvent.Direction.AheadRight: return "ahead-right";
            case SoundEvent.Direction.Right: return "to the right";
            case SoundEvent.Direction.BehindRight: return "behind-right";
            case SoundEvent.Direction.Behind: return "behind";
            case SoundEvent.Direction.BehindLeft: return "behind-left";
            case SoundEvent.Direction.Left: return "to the left";
            case SoundEvent.Direction.AheadLeft: return "ahead-left";
            default: return "ahead";
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

    void InitializeLaneDetectionCamera()
    {
        if (laneDetectionCamera == null)
        {
            GameObject cameraObj = new GameObject("LaneDetectionCamera");
            cameraObj.transform.SetParent(this.transform);
            cameraObj.transform.localPosition = new Vector3(0, 1.5f, 2f);
            cameraObj.transform.localRotation = Quaternion.Euler(15f, 0, 0);

            laneDetectionCamera = cameraObj.AddComponent<Camera>();
            laneDetectionCamera.fieldOfView = 60f;
            laneDetectionCamera.nearClipPlane = 0.1f;
            laneDetectionCamera.farClipPlane = 100f;
            laneDetectionCamera.clearFlags = CameraClearFlags.SolidColor;
            laneDetectionCamera.backgroundColor = Color.black;
            laneDetectionCamera.depthTextureMode = DepthTextureMode.None;
            laneDetectionCamera.enabled = false;

            // 선명도 개선 설정
            laneDetectionCamera.allowHDR = false;           // HDR 비활성화
            laneDetectionCamera.allowMSAA = false;          // 안티앨리어싱 비활성화
            laneDetectionCamera.allowDynamicResolution = false;
            laneDetectionCamera.useOcclusionCulling = false;

            Debug.Log("[CarController] Camera anti-aliasing and post-processing disabled for sharpness");
        }

        // 고품질 RenderTexture 생성
        RenderTextureDescriptor rtDesc = new RenderTextureDescriptor(imageWidth, imageHeight, RenderTextureFormat.RGB565, 0);
        rtDesc.sRGB = false;
        rtDesc.enableRandomWrite = false;
        rtDesc.useMipMap = false;
        rtDesc.autoGenerateMips = false;
        rtDesc.msaaSamples = 1;  // 안티앨리어싱 완전 비활성화

        renderTexture = RenderTexture.GetTemporary(rtDesc);
        renderTexture.name = "LaneDetectionRT_Sharp";
        renderTexture.filterMode = FilterMode.Point;  // 픽셀 완벽 필터링
        renderTexture.anisoLevel = 0;

        laneDetectionCamera.targetTexture = renderTexture;
        captureTexture = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);
        captureTexture.filterMode = FilterMode.Point;  // 선명한 텍스처

        captureInterval = 1f / targetFPS;

        Debug.Log($"[CarController] Sharp Lane Detection Camera initialized - {imageWidth}x{imageHeight} @ {targetFPS}fps");
    }

    void InitializeUDPStreaming()
    {
        try
        {
            // 고품질 설정 강제 적용
            if (!usePNG)
            {
                jpegQuality = Mathf.Clamp(jpegQuality, 80, 100);  // 최소 80 이상
            }
            maxPacketSize = 1400;

            // 비디오 전송용 UDP 클라이언트
            videoClient = new UdpClient();
            serverEndPoint = new IPEndPoint(IPAddress.Parse(serverIP), videoPort);

            Debug.Log($"[CarController] Created UDP client, local endpoint: {videoClient.Client.LocalEndPoint}");
            Debug.Log($"[CarController] Target server: {serverEndPoint}");

            // 응답 수신용 UDP 클라이언트
            responseClient = new UdpClient(responsePort);
            Debug.Log($"[CarController] Response client listening on: {responseClient.Client.LocalEndPoint}");

            // 응답 수신 스레드 시작
            responseThread = new Thread(ReceiveResponses);
            responseThread.IsBackground = true;
            responseThread.Start();

            isStreaming = true;
            string format = usePNG ? "PNG (lossless)" : $"JPEG (quality: {jpegQuality})";
            Debug.Log($"[CarController] Sharp UDP streaming initialized");
            Debug.Log($"[CarController] Settings: {imageWidth}x{imageHeight}, {format}, MaxPacket={maxPacketSize}");
        }
        catch (Exception e)
        {
            Debug.LogError($"[CarController] Failed to initialize UDP streaming: {e.Message}");
        }
    }

    void Update()
    {
        AutonomousDrivingController autonomousController = GetComponent<AutonomousDrivingController>();

        // 자율 주행 모드 확인
        bool isAutonomousActive = autonomousController != null && autonomousController.isAutonomousMode;

        // 자율 주행 모드가 아닐 때만 입력 처리
        if (!isAutonomousActive)
        {
            bool headlightPressed = im.l;
            var inputEvent = new CarInputEvent(im.throttle, im.steer, im.brake, headlightPressed);
            EventManager.Publish(inputEvent);
        }

        CheckAllAudioSources();
        ProcessVideoStreaming();
    }

    void ProcessVideoStreaming()
    {
        if (isStreaming && Time.time >= nextCaptureTime)
        {
            nextCaptureTime = Time.time + captureInterval;
            StartCoroutine(CaptureAndStreamFrame());
        }
    }

    IEnumerator CaptureAndStreamFrame()
    {
        if (laneDetectionCamera == null || renderTexture == null || !renderTexture.IsCreated())
        {
            yield break;
        }

        byte[] imageBytes = null;
        bool captureSuccess = false;

        // 캡처 부분 - 더 안전한 방법 사용
        try
        {
            // 카메라 수동 렌더링
            laneDetectionCamera.Render();

            // 더 안전한 픽셀 읽기 방법
            RenderTexture currentRT = RenderTexture.active;
            RenderTexture.active = renderTexture;

            captureTexture.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0, false);
            captureTexture.Apply();

            // 원래 RenderTexture로 복원
            RenderTexture.active = currentRT;

            // JPEG로 인코딩
            imageBytes = captureTexture.EncodeToJPG(jpegQuality);
            captureSuccess = (imageBytes != null && imageBytes.Length > 0);
        }
        catch (System.Exception e)
        {
            Debug.LogError($"[CarController] Frame capture error: {e.Message}");
            // 안전하게 RenderTexture.active 복원
            RenderTexture.active = null;
            captureSuccess = false;
        }

        // 전송 부분
        if (captureSuccess)
        {
            SendFrameUDP(imageBytes);
        }
        else
        {
            Debug.LogWarning("[CarController] Failed to capture or encode frame");
        }

        yield return null;
    }


    void SendFrameUDP(byte[] frameData)
    {
        if (videoClient == null || !isStreaming) return;

        try
        {
            // 헤더 정보 준비
            var header = new FrameHeader
            {
                sequenceNumber = sequenceNumber++,
                timestamp = (uint)(Time.time * 1000), // 밀리초
                frameSize = (uint)frameData.Length,
                imageWidth = (uint)imageWidth,
                imageHeight = (uint)imageHeight,
                vehicleSpeed = rb.linearVelocity.magnitude
            };

            Debug.Log($"[CarController] Sending frame {header.sequenceNumber}: {frameData.Length} bytes to {serverEndPoint}");

            // 헤더를 바이트로 직렬화
            byte[] headerBytes = StructToBytes(header);
            Debug.Log($"[CarController] Header size: {headerBytes.Length} bytes");

            // 거의 모든 이미지는 분할 전송이 필요할 것
            int totalPacketSize = frameData.Length + headerBytes.Length;

            Debug.Log($"[CarController] Frame size: {frameData.Length} bytes, Total: {totalPacketSize} bytes, Max: {maxPacketSize}");

            if (totalPacketSize > maxPacketSize)
            {
                // 분할 전송 (일반 메서드로 호출)
                SendFragmentedFrame(headerBytes, frameData);
                Debug.Log($"[CarController] Sent fragmented frame: {frameData.Length} bytes");
            }
            else
            {
                // 단일 패킷으로 전송
                byte[] packet = new byte[headerBytes.Length + frameData.Length];
                Array.Copy(headerBytes, 0, packet, 0, headerBytes.Length);
                Array.Copy(frameData, 0, packet, headerBytes.Length, frameData.Length);

                int bytesSent = videoClient.Send(packet, packet.Length, serverEndPoint);
                Debug.Log($"[CarController] Sent single packet: {packet.Length} bytes, confirmed: {bytesSent} bytes");
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"[CarController] UDP send error: {e.Message}");
            Debug.LogError($"[CarController] Frame size was: {frameData.Length} bytes");
            Debug.LogError($"[CarController] Server endpoint: {serverEndPoint}");
        }
    }

    void SendFragmentedFrame(byte[] headerBytes, byte[] frameData)
    {
        int maxDataSize = maxPacketSize - headerBytes.Length - 8;
        int totalFragments = Mathf.CeilToInt((float)frameData.Length / maxDataSize);

        Debug.Log($"[CarController] Fragmenting frame: {frameData.Length} bytes into {totalFragments} packets");
        Debug.Log($"[CarController] Max data per packet: {maxDataSize} bytes");

        for (int i = 0; i < totalFragments; i++)
        {
            int offset = i * maxDataSize;
            int fragmentSize = Mathf.Min(maxDataSize, frameData.Length - offset);

            // 분할 패킷 생성
            byte[] fragmentPacket = new byte[headerBytes.Length + 8 + fragmentSize];

            // 헤더 복사
            Array.Copy(headerBytes, 0, fragmentPacket, 0, headerBytes.Length);

            // 분할 정보 추가 (fragment index, total fragments)
            BitConverter.GetBytes(i).CopyTo(fragmentPacket, headerBytes.Length);
            BitConverter.GetBytes(totalFragments).CopyTo(fragmentPacket, headerBytes.Length + 4);

            // 프레임 데이터 조각 복사
            Array.Copy(frameData, offset, fragmentPacket, headerBytes.Length + 8, fragmentSize);

            try
            {
                int bytesSent = videoClient.Send(fragmentPacket, fragmentPacket.Length, serverEndPoint);
                if (i == 0) // 첫 번째 패킷만 로그
                    Debug.Log($"[CarController] Fragment {i + 1}/{totalFragments}: {fragmentPacket.Length} bytes sent, confirmed: {bytesSent} bytes to {serverEndPoint}");
            }
            catch (Exception e)
            {
                Debug.LogError($"[CarController] Fragment {i + 1} send error: {e.Message} (Size: {fragmentPacket.Length})");
                Debug.LogError($"[CarController] Target: {serverEndPoint}");
            }
        }

        Debug.Log($"[CarController] Completed sending {totalFragments} fragments");
    }

    void ReceiveResponses()
    {
        IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);

        while (isStreaming)
        {
            try
            {
                byte[] responseData = responseClient.Receive(ref remoteEndPoint);

                // 메인 스레드에서 처리하도록 큐에 추가
                lock (responseQueue)
                {
                    responseQueue.Enqueue(responseData);
                }
            }
            catch (Exception e)
            {
                if (isStreaming) // 스트리밍 중단이 아닌 실제 에러인 경우만 로그
                {
                    Debug.LogError($"[CarController] UDP receive error: {e.Message}");
                }
            }
        }
    }

    private Queue<byte[]> responseQueue = new Queue<byte[]>();

    void LateUpdate()
    {
        // 큐에서 응답 처리 (메인 스레드)
        lock (responseQueue)
        {
            while (responseQueue.Count > 0)
            {
                byte[] responseData = responseQueue.Dequeue();
                ProcessLaneDetectionResponse(responseData);
            }
        }
    }

    void ProcessLaneDetectionResponse(byte[] responseData)
    {
        try
        {
            // 응답 데이터 파싱
            var response = BytesToStruct<LaneDetectionResponseUDP>(responseData);

            bool success = response.success != 0;
            isLaneDetectionActive = success;

            if (success)
            {
                currentLaneOffset = response.lane_offset;

                Debug.Log($"[CarController] Lane Detection - Offset: {currentLaneOffset:F3}, " +
                         $"Confidence: {response.confidence:F2}, Latency: {Time.time * 1000 - response.timestamp:F1}ms");

                // 차선 이탈 경고
                if (Mathf.Abs(currentLaneOffset) > laneOffsetThreshold)
                {
                    var laneWarningEvent = new LaneWarningEvent(currentLaneOffset, response.confidence);
                    EventManager.Publish(laneWarningEvent);

                    Debug.LogWarning($"[CarController] Lane departure detected! Offset: {currentLaneOffset:F3}");
                }
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"[CarController] Failed to parse UDP response: {e.Message}");
        }
    }

    byte[] StructToBytes<T>(T obj) where T : struct
    {
        int size = System.Runtime.InteropServices.Marshal.SizeOf(obj);
        byte[] arr = new byte[size];
        IntPtr ptr = System.Runtime.InteropServices.Marshal.AllocHGlobal(size);
        System.Runtime.InteropServices.Marshal.StructureToPtr(obj, ptr, true);
        System.Runtime.InteropServices.Marshal.Copy(ptr, arr, 0, size);
        System.Runtime.InteropServices.Marshal.FreeHGlobal(ptr);
        return arr;
    }

    T BytesToStruct<T>(byte[] arr) where T : struct
    {
        T obj = default(T);
        int size = System.Runtime.InteropServices.Marshal.SizeOf(obj);
        IntPtr ptr = System.Runtime.InteropServices.Marshal.AllocHGlobal(size);
        System.Runtime.InteropServices.Marshal.Copy(arr, 0, ptr, size);
        obj = (T)System.Runtime.InteropServices.Marshal.PtrToStructure(ptr, obj.GetType());
        System.Runtime.InteropServices.Marshal.FreeHGlobal(ptr);
        return obj;
    }

    // ==================== PUBLIC API METHODS ====================

    // 런타임에 새 사운드 소스 추가
    public void RegisterSoundSource(SoundType soundType, ISoundSource soundSource)
    {
        if (soundSources == null)
        {
            soundSources = new Dictionary<SoundType, ISoundSource>();
        }

        soundSources[soundType] = soundSource;
        Debug.Log($"[CarController] Registered new sound source: {soundType}");
    }

    // 사운드 소스 제거
    public void UnregisterSoundSource(SoundType soundType)
    {
        if (soundSources != null && soundSources.ContainsKey(soundType))
        {
            soundSources.Remove(soundType);
            Debug.Log($"[CarController] Unregistered sound source: {soundType}");
        }
    }

    // 설정 업데이트 메서드들
    public void UpdateSoundThreshold(SoundType soundType, string direction, float threshold)
    {
        soundResponseManager.UpdateDirectionThreshold(soundType, direction, threshold);
    }

    public void UpdateSoundActivationThreshold(SoundType soundType, float threshold)
    {
        soundResponseManager.UpdateActivationThreshold(soundType, threshold);
    }

    public void TestUDPConnection()
    {
        if (videoClient == null) return;

        try
        {
            // 간단한 테스트 메시지 전송
            string testMessage = "TEST_PACKET_FROM_UNITY";
            byte[] testData = System.Text.Encoding.UTF8.GetBytes(testMessage);

            Debug.Log($"[CarController] Sending test packet: '{testMessage}' ({testData.Length} bytes)");
            Debug.Log($"[CarController] Local endpoint: {videoClient.Client.LocalEndPoint}");
            Debug.Log($"[CarController] Target endpoint: {serverEndPoint}");

            int bytesSent = videoClient.Send(testData, testData.Length, serverEndPoint);
            Debug.Log($"[CarController] Test packet sent: {bytesSent} bytes confirmed");
        }
        catch (Exception e)
        {
            Debug.LogError($"[CarController] Test packet failed: {e.Message}");
        }
    }

    public float GetCurrentAmbulanceVolume() => currentAmbulanceVolume;
    public Vector3 GetAmbulanceDirection() => ambulanceDirection;
    public float GetAmbulanceDistance() => ambulanceDistance;
    public string GetAmbulanceRelativeDirection() => ambulanceRelativeDirection;
    public SoundResponseManager GetSoundResponseManager() => soundResponseManager;
    public Dictionary<SoundType, ISoundSource> GetSoundSources() => soundSources;
}

// UDP 전송용 구조체들
[System.Runtime.InteropServices.StructLayout(System.Runtime.InteropServices.LayoutKind.Sequential, Pack = 1)]
public struct FrameHeader
{
    public uint sequenceNumber;
    public uint timestamp;
    public uint frameSize;
    public uint imageWidth;
    public uint imageHeight;
    public float vehicleSpeed;
}

[System.Runtime.InteropServices.StructLayout(System.Runtime.InteropServices.LayoutKind.Sequential, Pack = 1)]
public struct LaneDetectionResponseUDP
{
    public byte success;
    public float lane_offset;
    public float confidence;
    public uint timestamp;
    public uint sequence_number;
}
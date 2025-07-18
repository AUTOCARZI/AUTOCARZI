using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class HUDManager : MonoBehaviour
{
  [Header("HUD Elements")]
  public Canvas hudCanvas;

  [Header("Ambulance HUD Elements")]
  public RawImage hudAmbulanceBehindRightMoveLeft;     // 우측 후방 → 좌측 이동
  public RawImage hudAmbulanceBehindLeftMoveRight;     // 좌측 후방 → 우측 이동
  public RawImage hudAmbulanceBehindMoveRight;         // 후방 → 우측 이동
  public RawImage hudAmbulanceBehindMoveLeft;          // 후방 → 좌측 이동

  [Header("Horn HUD Elements")]
  public RawImage hudHornBehind;                       // 후방 경적
  public RawImage hudHornBehindLeft;                   // 후방 좌측 경적
  public RawImage hudHornBehindRight;                  // 후방 우측 경적

  [Header("Animation Settings")]
  public float hudBlinkInterval = 1f;
  public float hudFadeSpeed = 2f;

  private Dictionary<string, HUDElement> hudElements;
  private Dictionary<string, Coroutine> blinkingCoroutines;

  void Start()
  {
    Debug.Log("[HUDManager] HUDManager Starting...");

    InitializeHUD();

    Debug.Log("[HUDManager] Subscribing to HUDControlEvent...");
    EventManager.Subscribe<HUDControlEvent>(OnHUDControl);

    Debug.Log("[HUDManager] HUDManager initialization complete");

    // 테스트 이벤트 발행해보기
    Debug.Log("[HUDManager] Testing event system...");
    var testEvent = new HUDControlEvent("test", true, "test");
    EventManager.Publish(testEvent);
  }

  void OnDestroy()
  {
    Debug.Log("[HUDManager] HUDManager OnDestroy - Unsubscribing events");
    EventManager.Unsubscribe<HUDControlEvent>(OnHUDControl);
    Cleanup();
  }

  private void InitializeHUD()
  {
    hudElements = new Dictionary<string, HUDElement>();
    blinkingCoroutines = new Dictionary<string, Coroutine>();

    if (hudCanvas != null)
    {
      hudCanvas.gameObject.SetActive(true);

      // 새로운 상황별 앰뷸런스 HUD 등록 (지속형)
      RegisterHUD("ambulance-behind-right-move-left", hudAmbulanceBehindRightMoveLeft, HUDMode.Continuous);
      RegisterHUD("ambulance-behind-left-move-right", hudAmbulanceBehindLeftMoveRight, HUDMode.Continuous);
      RegisterHUD("ambulance-behind-move-right", hudAmbulanceBehindMoveRight, HUDMode.Continuous);
      RegisterHUD("ambulance-behind-move-left", hudAmbulanceBehindMoveLeft, HUDMode.Continuous);

      // 경적 HUD 등록 (단발형 - 3회 깜빡임, 최소 2초)
      RegisterHUD("horn-behind", hudHornBehind, HUDMode.OneShot, 2.0f, 3);
      RegisterHUD("horn-behind-left", hudHornBehindLeft, HUDMode.OneShot, 2.0f, 3);
      RegisterHUD("horn-behind-right", hudHornBehindRight, HUDMode.OneShot, 2.0f, 3);

      Debug.Log("[HUDManager] HUD System initialized");
    }
    else
    {
      Debug.LogError("[HUDManager] hudCanvas is null!");
    }
  }

  private void OnHUDControl(HUDControlEvent hudEvent)
  {
    Debug.Log($"[HUDManager] Received HUDControlEvent: {hudEvent.hudId}, Show: {hudEvent.shouldShow}, Direction: {hudEvent.direction}");

    if (hudEvent.shouldShow)
    {
      Debug.Log($"[HUDManager] Starting HUD display for {hudEvent.hudId}");
      StartBlinking(hudEvent.hudId);
    }
    else
    {
      Debug.Log($"[HUDManager] Stopping HUD display for {hudEvent.hudId}");
      StopBlinking(hudEvent.hudId);
    }
  }

  public void RegisterHUD(string key, RawImage rawImage, HUDMode mode = HUDMode.Continuous, float minDuration = 2.0f, int minCycles = 3)
  {
    if (rawImage != null)
    {
      HUDElement element = new HUDElement(rawImage, mode, minDuration, minCycles);
      hudElements[key] = element;

      Color color = rawImage.color;
      color.a = 0f;
      rawImage.color = color;
      rawImage.gameObject.SetActive(false);

      Debug.Log($"[HUDManager] Registered HUD: {key} (Mode: {mode})");
    }
    else
    {
      Debug.LogWarning($"[HUDManager] RawImage for {key} is null!");
    }
  }


  public void StartBlinking(string key)
  {
    if (!hudElements.ContainsKey(key))
    {
      Debug.LogWarning($"[HUDManager] HUD element '{key}' not found!");
      return;
    }

    HUDElement element = hudElements[key];

    // 이미 깜빡이고 있으면 중복 시작하지 않음
    if (blinkingCoroutines.ContainsKey(key) && blinkingCoroutines[key] != null)
    {
      Debug.Log($"[HUDManager] Already blinking: {key} - skipping");
      return;
    }

    // HUD 표시 시작
    element.StartDisplay();

    Debug.Log($"[HUDManager] Starting blink for {key} (Mode: {element.mode})");
    blinkingCoroutines[key] = StartCoroutine(BlinkCoroutine(key));
  }

  public void StopBlinking(string key)
  {
    if (!hudElements.ContainsKey(key))
    {
      return;
    }

    HUDElement element = hudElements[key];

    // 모드에 따라 중단 가능 여부 확인
    if (!element.CanStop())
    {
      Debug.Log($"[HUDManager] Cannot stop {key} yet (Mode: {element.mode}, Cycles: {element.currentBlinkCount}/{element.targetBlinkCycles})");
      return;
    }

    if (blinkingCoroutines.ContainsKey(key) && blinkingCoroutines[key] != null)
    {
      Debug.Log($"[HUDManager] Stopping blink for {key}");
      StopCoroutine(blinkingCoroutines[key]);
      blinkingCoroutines[key] = null;
    }

    if (hudElements.ContainsKey(key))
    {
      StartCoroutine(FadeOut(key));
    }
  }

  private System.Collections.IEnumerator BlinkCoroutine(string key)
  {
    HUDElement element = hudElements[key];

    while (true)
    {
      yield return FadeIn(key);
      yield return new WaitForSeconds(hudBlinkInterval * 0.3f);
      yield return FadeOut(key);
      yield return new WaitForSeconds(hudBlinkInterval * 0.7f);

      // 깜빡임 횟수 증가
      element.IncrementBlinkCount();

      // OneShot 모드의 경우 자동 종료 체크
      if (element.mode == HUDMode.OneShot && element.CanStop())
      {
        Debug.Log($"[HUDManager] OneShot HUD {key} completed {element.currentBlinkCount} cycles, stopping");
        break;
      }

      // Timed 모드의 경우 시간 기반 자동 종료 체크
      if (element.mode == HUDMode.Timed && element.CanStop())
      {
        Debug.Log($"[HUDManager] Timed HUD {key} completed minimum duration, stopping");
        break;
      }
    }

    // 코루틴 정리
    if (blinkingCoroutines.ContainsKey(key))
    {
      blinkingCoroutines[key] = null;
    }

    // 최종 페이드아웃
    yield return FadeOut(key);
  }

  private System.Collections.IEnumerator FadeIn(string key)
  {
    if (!hudElements.ContainsKey(key)) yield break;

    HUDElement element = hudElements[key];
    element.rawImage.gameObject.SetActive(true);

    yield return FadeToAlpha(element.rawImage, 1f);
  }

  private System.Collections.IEnumerator FadeOut(string key)
  {
    if (!hudElements.ContainsKey(key)) yield break;

    HUDElement element = hudElements[key];
    yield return FadeToAlpha(element.rawImage, 0f);
    element.rawImage.gameObject.SetActive(false);
  }

  private System.Collections.IEnumerator FadeToAlpha(RawImage image, float targetAlpha)
  {
    Color color = image.color;
    float startAlpha = color.a;
    float elapsed = 0f;
    float duration = 1f / hudFadeSpeed;

    while (elapsed < duration)
    {
      elapsed += Time.deltaTime;
      color.a = Mathf.Lerp(startAlpha, targetAlpha, elapsed / duration);
      image.color = color;
      yield return null;
    }

    color.a = targetAlpha;
    image.color = color;
  }

  public void Cleanup()
  {
    foreach (string key in hudElements.Keys)
    {
      StopBlinking(key);
    }
  }
}

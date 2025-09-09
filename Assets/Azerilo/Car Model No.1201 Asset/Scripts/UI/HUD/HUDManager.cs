using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class HUDManager : MonoBehaviour
{
  [Header("HUD Elements")]
  public Canvas hudCanvas;

  [Header("Ambulance HUD Elements")]
  public RawImage hudAmbulanceBehindRightMoveLeft;     // 후방 앰뷸런스
  public RawImage hudAmbulanceFront;                   // 전방 앰뷸런스

  [Header("Rain HUD Elements")]
  public RawImage hudRainSlowingDown;                   // 우천으로 인한 감속
  public RawImage hudHeavyRainSlowingDown;              // 폭우로 인한 감속

  [Header("Bypass HUD Elements")]
  public RawImage hudBypassTraffic;                    // 교통 정체로 인한 우회
  public RawImage hudBypassAccident;                   // 사고로 인한 우회

  [Header("Animation Settings")]
  public float hudBlinkInterval = 1f;
  public float hudFadeSpeed = 2f;

  [Header("Reaction Time Tracking")]
  public ReactionTimeTracker reactionTracker;

  private Dictionary<string, HUDElement> hudElements;
  private Dictionary<string, Coroutine> blinkingCoroutines;

  void Start()
  {
    Debug.Log("[HUDManager] HUDManager Starting...");

    InitializeHUD();

    Debug.Log("[HUDManager] Subscribing to HUDControlEvent...");
    EventManager.Subscribe<HUDControlEvent>(OnHUDControl);

    if (reactionTracker == null)
    {
        reactionTracker = FindObjectOfType<ReactionTimeTracker>();
        if (reactionTracker == null)
        {
            GameObject trackerObj = new GameObject("ReactionTimeTracker");
            reactionTracker = trackerObj.AddComponent<ReactionTimeTracker>();
            Debug.Log("[HUDManager] Created ReactionTimeTracker automatically");
        }
    }
    Debug.Log("[HUDManager] HUDManager initialization complete");
  }

  void OnDestroy()
  {
    Debug.Log("[HUDManager] HUDManager OnDestroy - Unsubscribing events");
    EventManager.Unsubscribe<HUDControlEvent>(OnHUDControl);

    if (gameObject.activeInHierarchy)
    {
      Cleanup();
    }
  }

  private void InitializeHUD()
  {
    hudElements = new Dictionary<string, HUDElement>();
    blinkingCoroutines = new Dictionary<string, Coroutine>();

    if (hudCanvas != null)
    {
      hudCanvas.gameObject.SetActive(true);

      // 상황별 앰뷸런스 HUD(지속형)
      RegisterHUD("ambulance-behind-right-move-left", hudAmbulanceBehindRightMoveLeft, HUDMode.Continuous);
      RegisterHUD("ambulance-front", hudAmbulanceFront, HUDMode.Static);  // 깜빡임 제거

      // 상황별 우천 HUD(정적형 - 깜빡임 없음)
      RegisterHUD("rain-slowing-down", hudRainSlowingDown, HUDMode.Static);  // 깜빡임 제거
      RegisterHUD("heavy-rain-slowing-down", hudHeavyRainSlowingDown, HUDMode.Static);  // 깜빡임 제거

      // 상황별 우회 HUD(정적 표시, 4초 지속)
      RegisterHUD("bypass-traffic", hudBypassTraffic, HUDMode.Static, 4.0f, 1);
      RegisterHUD("bypass-accident", hudBypassAccident, HUDMode.Continuous);

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

  public void RegisterHUD(string key, RawImage rawImage, HUDMode mode = HUDMode.Continuous, float minDuration = 3.0f, int minCycles = 3)
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


  private string GetHUDEventType(string hudId)
  {
    if (hudId.Contains("ambulance")) return "Ambulance";
    if (hudId.Contains("rain")) return "Rain";
    if (hudId.Contains("bypass")) return "Bypass";
    return "Unknown";
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

    if (reactionTracker != null)
    {
        string eventType = GetHUDEventType(key);
        reactionTracker.StartEvent(eventType);
    }
    
    // HUD 표시 시작
    element.StartDisplay();

    Debug.Log($"[HUDManager] Starting display for {key} (Mode: {element.mode})");
    
    // Static 모드일 때는 깜빡임 없이 바로 표시
    if (element.mode == HUDMode.Static)
    {
      blinkingCoroutines[key] = StartCoroutine(StaticDisplay(key));
    }
    else
    {
      blinkingCoroutines[key] = StartCoroutine(BlinkCoroutine(key));
    }
  }

    public void StopBlinking(string key)
    {
        if (!hudElements.ContainsKey(key))
        {
            return;
        }

        HUDElement element = hudElements[key];

        // Static 모드는 언제든지 중단 가능
        if (element.mode != HUDMode.Static && !element.CanStop())
        {
            Debug.Log($"[HUDManager] Cannot stop {key} yet (Mode: {element.mode}, Cycles: {element.currentBlinkCount}/{element.targetBlinkCycles})");
            return;
        }

        if (blinkingCoroutines.ContainsKey(key) && blinkingCoroutines[key] != null)
        {
            Debug.Log($"[HUDManager] Stopping display for {key}");
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

  // Static 모드용 - 깜빡임 없이 지속 표시
  private System.Collections.IEnumerator StaticDisplay(string key)
  {
    if (!hudElements.ContainsKey(key)) yield break;

    HUDElement element = hudElements[key];
    element.rawImage.gameObject.SetActive(true);

    // 바로 완전히 표시
    yield return FadeToAlpha(element.rawImage, 1f);

    // minimumDisplayDuration이 설정되어 있으면 해당 시간 후 자동 종료
    if (element.minimumDisplayDuration > 0)
    {
      yield return new WaitForSeconds(element.minimumDisplayDuration);
      yield return FadeOut(key);
      if (blinkingCoroutines.ContainsKey(key))
      {
        blinkingCoroutines[key] = null;
      }
    }
    else
    {
      // 시간 제한이 없으면 무한히 지속 (StopBlinking이 호출될 때까지)
      while (true)
      {
        yield return null;
      }
    }
  }

  public void Cleanup()
  {
    if (hudElements != null)
    {
      foreach (string key in hudElements.Keys)
      {
        StopBlinking(key);
      }
    }
  }
}

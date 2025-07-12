using System.Collections.Generic;
using UnityEngine;

public class HUDElement
{
    public UnityEngine.UI.RawImage rawImage;
    public bool isActive;

    public HUDElement(UnityEngine.UI.RawImage img)
    {
        rawImage = img;
        isActive = false;
    }
}

public class HUDManager : MonoBehaviour
{
    [Header("HUD Elements")]
    public Canvas hudCanvas;
    public UnityEngine.UI.RawImage hudAmbulanceLeft;
    public UnityEngine.UI.RawImage hudAmbulanceRight;
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
            RegisterHUD("ambulance-left", hudAmbulanceLeft);
            RegisterHUD("ambulance-right", hudAmbulanceRight);

            // 여기에 새로운 HUD!! 등록 -> SoundResponseManager에 대응되는 프로필 추가 필요

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

    public void RegisterHUD(string key, UnityEngine.UI.RawImage rawImage)
    {
        if (rawImage != null)
        {
            HUDElement element = new HUDElement(rawImage);
            hudElements[key] = element;

            Color color = rawImage.color;
            color.a = 0f;
            rawImage.color = color;
            rawImage.gameObject.SetActive(false);

            Debug.Log($"[HUDManager] Registered HUD: {key}");
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

        // 이미 깜빡이고 있으면 중복 시작하지 않음
        if (blinkingCoroutines.ContainsKey(key) && blinkingCoroutines[key] != null)
        {
            Debug.Log($"[HUDManager] Already blinking: {key} - skipping");
            return;
        }

        Debug.Log($"[HUDManager] Starting blink for {key}");
        blinkingCoroutines[key] = StartCoroutine(BlinkCoroutine(key));
    }
   
    public void StopBlinking(string key)
    {
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
        while (true)
        {
            yield return FadeIn(key);
            yield return new WaitForSeconds(hudBlinkInterval * 0.3f);
            yield return FadeOut(key);
            yield return new WaitForSeconds(hudBlinkInterval * 0.7f);
        }
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

    private System.Collections.IEnumerator FadeToAlpha(UnityEngine.UI.RawImage image, float targetAlpha)
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
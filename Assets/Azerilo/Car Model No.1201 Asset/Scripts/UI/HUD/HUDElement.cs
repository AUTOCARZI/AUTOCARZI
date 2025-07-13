using UnityEngine;
using UnityEngine.UI;

public class HUDElement
{
  public RawImage rawImage;
  public bool isActive;
  public HUDMode mode = HUDMode.Continuous;
  public float displayStartTime;
  public float minimumDisplayDuration = 2.0f;
  public int targetBlinkCycles = 3;
  public int currentBlinkCount = 0;
  public bool isInMinimumPeriod = false;

  public HUDElement(RawImage img)
  {
    rawImage = img;
    isActive = false;
    displayStartTime = 0f;
    currentBlinkCount = 0;
  }
  
  public HUDElement(RawImage img, HUDMode hudMode, float minDuration = 2.0f, int minCycles = 3)
  {
    rawImage = img;
    isActive = false;
    mode = hudMode;
    minimumDisplayDuration = minDuration;
    targetBlinkCycles = minCycles;
    displayStartTime = 0f;
    currentBlinkCount = 0;
  }

  public void StartDisplay()
  {
    displayStartTime = Time.time;
    currentBlinkCount = 0;
    isInMinimumPeriod = true;
    isActive = true;
  }

  public bool CanStop()
  {
    switch (mode)
    {
      case HUDMode.Continuous:
        return true; // 언제든 중단 가능
      
      case HUDMode.OneShot:
        return currentBlinkCount >= targetBlinkCycles; // 최소 깜빡임 횟수 달성
      
      case HUDMode.Timed:
        return Time.time - displayStartTime >= minimumDisplayDuration; // 최소 시간 경과
      
      default:
        return true;
    }
  }

  public void IncrementBlinkCount()
  {
    currentBlinkCount++;
  }
}

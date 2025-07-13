using UnityEngine.UI;

public class HUDElement
{
  public RawImage rawImage;
  public bool isActive;

  public HUDElement(RawImage img)
  {
    rawImage = img;
    isActive = false;
  }
}

using UnityEngine;

public class LightController : MonoBehaviour
{
    [Header("Light Settings")]
    public Light targetLight;
    public Material emissiveMaterial; // 튜브 라이트용 머티리얼
    
    [Header("Color Settings")]
    public Color redColor = Color.red;
    public Color blueColor = Color.blue;
    public Color greenColor = Color.green;
    public Color yellowColor = Color.yellow;
    public Color whiteColor = Color.white;
    
    [Header("Blinking Settings")]
    public float blinkSpeed = 2f;
    public float blinkIntensity = 3f;
    public float normalIntensity = 1f;
    
    private bool isBlinking = false;
    private Color currentColor;
    private float originalIntensity;
    
    void Start()
    {
        // Light 컴포넌트가 없으면 자동으로 찾기
        if (targetLight == null)
            targetLight = GetComponent<Light>();
            
        originalIntensity = targetLight.intensity;
        currentColor = targetLight.color;
    }
    
    void Update()
    {
        HandleInput();
        
        if (isBlinking)
        {
            BlinkLight();
        }
    }
    
    void HandleInput()
    {
        // 숫자 키로 색상 변경
        if (Input.GetKeyDown(KeyCode.Alpha1))
        {
            ChangeColor(redColor);
        }
        else if (Input.GetKeyDown(KeyCode.Alpha2))
        {
            ChangeColor(blueColor);
        }
        else if (Input.GetKeyDown(KeyCode.Alpha3))
        {
            ChangeColor(greenColor);
        }
        else if (Input.GetKeyDown(KeyCode.Alpha4))
        {
            ChangeColor(yellowColor);
        }
        else if (Input.GetKeyDown(KeyCode.Alpha5))
        {
            ChangeColor(whiteColor);
        }
        
        // 스페이스바로 깜빡임 토글
        if (Input.GetKeyDown(KeyCode.Space))
        {
            ToggleBlink();
        }
        
        // B키로 깜빡임 시작/정지
        if (Input.GetKeyDown(KeyCode.B))
        {
            ToggleBlink();
        }
        
        // R키로 랜덤 색상
        if (Input.GetKeyDown(KeyCode.R))
        {
            Color randomColor = new Color(
                Random.Range(0f, 1f),
                Random.Range(0f, 1f),
                Random.Range(0f, 1f),
                1f
            );
            ChangeColor(randomColor);
        }
    }
    
    void ChangeColor(Color newColor)
    {
        currentColor = newColor;
        targetLight.color = newColor;
        
        // 만약 Emissive Material이 있다면 함께 변경
        if (emissiveMaterial != null)
        {
            emissiveMaterial.SetColor("_EmissionColor", newColor);
        }
        
        Debug.Log($"Light color changed to: {newColor}");
    }
    
    void ToggleBlink()
    {
        isBlinking = !isBlinking;
        
        if (!isBlinking)
        {
            // 깜빡임 중지 시 원래 밝기로 복원
            targetLight.intensity = originalIntensity;
        }
        
        Debug.Log($"Blinking: {isBlinking}");
    }
    
    void BlinkLight()
    {
        // 사인파를 이용한 부드러운 깜빡임
        float blinkValue = Mathf.Sin(Time.time * blinkSpeed) * 0.5f + 0.5f;
        targetLight.intensity = Mathf.Lerp(normalIntensity, blinkIntensity, blinkValue);
    }
    
    // 외부에서 호출할 수 있는 메서드들
    public void SetLightColor(Color color)
    {
        ChangeColor(color);
    }
    
    public void StartBlinking()
    {
        isBlinking = true;
    }
    
    public void StopBlinking()
    {
        isBlinking = false;
        targetLight.intensity = originalIntensity;
    }
    
    public void SetBlinkSpeed(float speed)
    {
        blinkSpeed = speed;
    }
}
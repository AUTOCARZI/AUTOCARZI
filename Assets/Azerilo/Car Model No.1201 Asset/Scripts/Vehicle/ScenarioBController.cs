using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

public class ScenarioBController : MonoBehaviour
{
    private Rigidbody rb;
    private AutonomousDrivingController autonomousController;
    
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        autonomousController = GetComponent<AutonomousDrivingController>();
    }
    
    void OnTriggerEnter(Collider other)
    {   
        if (other.CompareTag("Wall"))
        {
            if (autonomousController != null && autonomousController.isAutonomousMode)
            {
                autonomousController.SetAutonomousMode(false);
            }
            
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            
            Destroy(other.gameObject);
        }
        else if (other.CompareTag("Finish Point"))
        {
            
            if (autonomousController != null && autonomousController.isAutonomousMode)
            {
                autonomousController.SetAutonomousMode(false);
            }
            
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            
            EndGame();
        }
    }
    
    void EndGame()
    {
        #if UNITY_EDITOR
            EditorApplication.isPlaying = false;
        #else
            Application.Quit();
        #endif
    }
}
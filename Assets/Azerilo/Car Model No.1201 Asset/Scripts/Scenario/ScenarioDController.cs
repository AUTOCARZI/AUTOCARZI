using UnityEngine;

public class ScenarioDController : MonoBehaviour
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
            autonomousController.SetAutonomousMode(false);
            
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
            
            Destroy(other.gameObject);
        } else if (other.CompareTag("Auto"))
        {
            autonomousController.SetAutonomousMode(true);
            Destroy(other.gameObject);
        }
    }
}
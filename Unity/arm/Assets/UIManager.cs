using UnityEngine;

public class UIManager : MonoBehaviour
{
    public ArmController armController;
    private float updateInterval = 1.0f; // Update every second
    private float timeSinceLastUpdate = 0f;

    void Update()
    {
        if (armController == null) return;

        timeSinceLastUpdate += Time.deltaTime;

        if (timeSinceLastUpdate >= updateInterval)
        {
            timeSinceLastUpdate = 0f;
            Debug.Log($"Arm Status Update:\n" +
                     $"Connected: {armController.IsConnected}\n" +
                     $"Calibrated: {armController.IsCalibrated}\n" +
                     $"Last Command: {armController.LastCommand}");
        }
    }
}

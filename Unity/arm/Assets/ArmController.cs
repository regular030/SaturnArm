using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.Interaction.Toolkit;
using TMPro;
using NativeWebSocket;
using System.Threading.Tasks;
using UnityEngine.UI;
using System.Collections;

public class ArmController : MonoBehaviour
{
    [Header("Controller References")]
    public Transform rightController;
    [Header("UI Elements")]
    public TextMeshProUGUI statusText;
    public Transform vrDisplay;
    public RawImage cameraDisplay;

    [Header("Network Settings")]
    public string armWebSocketUrl = "ws://<pico-ip>/ws";
    public string cameraStreamUrl = "http://<pico-ip>/camera";

    private WebSocket armWebSocket;
    private Vector3 controllerStartPos;
    private bool isCalibrated = false;
    private float armLength;
    private Texture2D cameraTexture;
    private Coroutine cameraCoroutine;

    async void Start()
    {
        statusText.text = "Press trigger to calibrate";
        Application.runInBackground = true;

        // Initialize camera texture
        cameraTexture = new Texture2D(2, 2);
        cameraDisplay.texture = cameraTexture;
    }

    void Update()
    {
#if !UNITY_WEBGL || UNITY_EDITOR
        armWebSocket?.DispatchMessageQueue();
#endif

        HandleControllerInput();
        UpdateDisplayPosition();
    }

    private void HandleControllerInput()
    {
        // Get controller input
        bool triggerPressed = Input.GetAxis("XRI_Right_Trigger") > 0.5f;
        bool gripPressed = Input.GetAxis("XRI_Right_Grip") > 0.5f;

        // Calibration
        if (triggerPressed && !isCalibrated)
        {
            CalibrateSystem();
        }

        // Reconnect
        if (gripPressed && (armWebSocket == null || armWebSocket.State != WebSocketState.Open))
        {
            ReconnectToArm();
        }

        // Send movement commands
        if (isCalibrated && armWebSocket != null && armWebSocket.State == WebSocketState.Open)
        {
            SendMovementCommand();
        }
    }

    private async void CalibrateSystem()
    {
        controllerStartPos = rightController.position;
        armLength = Vector3.Distance(Camera.main.transform.position, controllerStartPos) + 0.02f;

        await ConnectToArm();
        StartCameraStream();

        isCalibrated = true;
        statusText.text = "Calibrated! Move controller to move arm";
    }

    private async Task ConnectToArm()
    {
        if (armWebSocket != null)
        {
            await armWebSocket.Close();
        }

        armWebSocket = new WebSocket(armWebSocketUrl);

        armWebSocket.OnOpen += () =>
        {
            Debug.Log("Connected to arm!");
            statusText.text = "Connected!";
            armWebSocket.SendText("{\"cmd\":\"home\"}");
        };

        armWebSocket.OnError += (error) =>
        {
            Debug.LogError($"WebSocket Error: {error}");
            statusText.text = $"Connection failed\nPress grip to retry";
        };

        armWebSocket.OnClose += (code) =>
        {
            Debug.Log($"Connection closed: {code}");
        };

        await armWebSocket.Connect();
    }

    private void StartCameraStream()
    {
        if (cameraCoroutine != null)
        {
            StopCoroutine(cameraCoroutine);
        }
        cameraCoroutine = StartCoroutine(CameraStreamCoroutine());
    }

    private IEnumerator CameraStreamCoroutine()
    {
        while (isCalibrated)
        {
            using (WWW www = new WWW(cameraStreamUrl))
            {
                yield return www;
                if (string.IsNullOrEmpty(www.error))
                {
                    www.LoadImageIntoTexture(cameraTexture);
                }
                else
                {
                    Debug.LogError("Camera stream error: " + www.error);
                }
            }
            yield return new WaitForSeconds(0.1f); // 10 FPS
        }
    }

    public bool IsConnected => armWebSocket != null && armWebSocket.State == WebSocketState.Open;
    public bool IsCalibrated => isCalibrated;
    public string LastCommand { get; private set; } = "None";

    private void SendMovementCommand()
    {
        Vector3 relativePos = rightController.position - controllerStartPos;
        float x = relativePos.z * 20f;
        float y = relativePos.y * 20f;

        LastCommand = $"X: {x:F2}, Y: {y:F2}";
        string json = $"{{\"cmd\":\"move\",\"x\":{x:F2},\"y\":{y:F2},\"z\":0}}";
        armWebSocket.SendText(json);
    }

    private async void ReconnectToArm()
    {
        statusText.text = "Reconnecting...";
        await ConnectToArm();
        if (armWebSocket.State == WebSocketState.Open)
        {
            StartCameraStream();
        }
    }

    private void UpdateDisplayPosition()
    {
        if (!isCalibrated) return;

        // Position display 1.5m in front of camera at eye level
        Transform cameraTransform = Camera.main.transform;
        vrDisplay.position = cameraTransform.position + cameraTransform.forward * 1.5f;
        vrDisplay.rotation = Quaternion.LookRotation(vrDisplay.position - cameraTransform.position);
    }

    async void OnApplicationQuit()
    {
        if (armWebSocket != null && armWebSocket.State == WebSocketState.Open)
        {
            await armWebSocket.Close();
        }
    }
}

using UnityEngine;

public class CenterMarker : MonoBehaviour
{
    public Vector3 offset = Vector3.zero; // Public offset variable
    
    void Start()
    {
        // Create the red cube
        GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Cube);
        marker.name = "CenterMarker";
        marker.transform.localScale = Vector3.one * 0.03f;

        // Set material to red
        Material redMat = new Material(Shader.Find("Standard"));
        redMat.color = Color.red;
        marker.GetComponent<Renderer>().material = redMat;

        // Position the marker at center + offset
        marker.transform.position = CalculateCenter() + offset;
        
        // Parent to target object (optional)
        marker.transform.SetParent(transform);
    }

    Vector3 CalculateCenter()
    {
        Renderer[] renderers = GetComponentsInChildren<Renderer>();
        if (renderers.Length == 0) return transform.position;

        Bounds combinedBounds = renderers[0].bounds;
        foreach (Renderer r in renderers) 
            combinedBounds.Encapsulate(r.bounds);
        
        return combinedBounds.center;
    }
}
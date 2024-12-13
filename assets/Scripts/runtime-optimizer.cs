using UnityEngine;

public class RuntimeOptimizer : MonoBehaviour
{
    void Awake()
    {
        if (Application.isBatchMode)
        {
            // Physics optimizations
            Physics.simulationMode = SimulationMode.Script;
            Time.fixedDeltaTime = 0.01f;
            
            // Graphics optimizations
            QualitySettings.vSyncCount = 0;
            Application.targetFrameRate = -1;
            
            // Audio optimizations
            AudioListener.pause = true;
            AudioListener.volume = 0;
            
            // Display optimizations
            Screen.SetResolution(320, 200, false);
            
            // Performance optimizations
            QualitySettings.SetQualityLevel(0, true);
            QualitySettings.shadows = ShadowQuality.Disable;
            QualitySettings.shadowDistance = 0;
            QualitySettings.realtimeReflectionProbes = false;
            QualitySettings.billboardsFaceCameraPosition = false;
        }
    }
}

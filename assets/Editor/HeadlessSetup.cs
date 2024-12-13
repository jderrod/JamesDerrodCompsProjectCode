using UnityEngine;
using UnityEditor;
using System.IO;

public class HeadlessSetup : EditorWindow
{
    private int numberOfEnvironments = 4;
    private float environmentSpacing = 50f;
    
    [MenuItem("ML-Agents/Setup Headless Training", false, 1)]
    public static void ShowWindow()
    {
        GetWindow<HeadlessSetup>("Training Setup");
    }

    private void OnGUI()
    {
        GUILayout.Label("Headless Training Setup", EditorStyles.boldLabel);
        numberOfEnvironments = EditorGUILayout.IntSlider("Number of Environments", numberOfEnvironments, 1, 16);
        environmentSpacing = EditorGUILayout.FloatField("Environment Spacing", environmentSpacing);

        EditorGUILayout.Space(10);
        EditorGUILayout.HelpBox("Make sure your car has the 'Player' tag!", MessageType.Info);

        if (GUILayout.Button("1. Setup Training Environments"))
        {
            SetupTrainingEnvironments();
        }

        if (GUILayout.Button("2. Build Headless Player"))
        {
            BuildHeadlessPlayer();
        }
    }

    private void SetupTrainingEnvironments()
    {
        // Find car and track generator objects
        GameObject originalCar = GameObject.FindGameObjectWithTag("Player");
        GameObject trackGeneratorObj = GameObject.FindObjectOfType<TrackGenerator>()?.gameObject;

        // Validate findings
        if (originalCar == null)
        {
            EditorUtility.DisplayDialog("Setup Error", 
                "Could not find the car object. Make sure it has the 'Player' tag.", "OK");
            return;
        }

        if (trackGeneratorObj == null)
        {
            EditorUtility.DisplayDialog("Setup Error", 
                "Could not find the TrackGenerator object.", "OK");
            return;
        }

        Debug.Log($"Found car: {originalCar.name} and track generator: {trackGeneratorObj.name}");

        // Create parent for environments
        GameObject environmentParent = new GameObject("TrainingEnvironments");

        // Create environments
        for (int i = 0; i < numberOfEnvironments; i++)
        {
            float xOffset = (i % 4) * environmentSpacing;
            float zOffset = (i / 4) * environmentSpacing;

            GameObject newEnvironment = new GameObject($"Environment_{i}");
            newEnvironment.transform.parent = environmentParent.transform;
            newEnvironment.transform.position = new Vector3(xOffset, 0, zOffset);

            // Create new car
            GameObject newCar = Instantiate(originalCar, newEnvironment.transform);
            newCar.transform.localPosition = Vector3.zero;

            // Create new track generator
            GameObject newTrackGen = Instantiate(trackGeneratorObj, newEnvironment.transform);
            newTrackGen.transform.localPosition = Vector3.zero;
        }

        // Disable originals
        originalCar.SetActive(false);
        trackGeneratorObj.SetActive(false);

        // Add optimizer
        GameObject optimizer = new GameObject("RuntimeOptimizer");
        optimizer.AddComponent<RuntimeOptimizer>();

        GenerateTrainingConfig();

        if (EditorUtility.DisplayDialog("Setup Complete", 
            $"Created {numberOfEnvironments} training environments. Would you like to save the scene?", 
            "Save", "Don't Save"))
        {
            UnityEditor.SceneManagement.EditorSceneManager.SaveScene(
                UnityEditor.SceneManagement.EditorSceneManager.GetActiveScene());
        }
    }

    private void GenerateTrainingConfig()
    {
        string config = @"behaviors:
  car_training_config:
    trainer_type: ppo
    hyperparameters:
      batch_size: 2048
      buffer_size: 16384
      learning_rate: 3.0e-4
      beta: 5.0e-3
      epsilon: 0.2
      lambd: 0.95
      num_epoch: 3
      learning_rate_schedule: linear
    network_settings:
      normalize: true
      hidden_units: 256
      num_layers: 3
    reward_signals:
      extrinsic:
        gamma: 0.99
        strength: 1.0
      curiosity:
        strength: 0.02
        gamma: 0.99
        encoding_size: 256
    max_steps: 2000000
    time_horizon: 128
    summary_freq: 10000
    checkpoint_interval: 50000
    threaded: true";

        File.WriteAllText("Assets/training_config.yaml", config);
        AssetDatabase.Refresh();
    }

    private void BuildHeadlessPlayer()
    {
        // Ensure current scene is saved
        if (UnityEditor.SceneManagement.EditorSceneManager.SaveCurrentModifiedScenesIfUserWantsTo() == false)
        {
            Debug.LogWarning("Build canceled - Scene not saved");
            return;
        }

        // Get current scene path
        string currentScenePath = UnityEngine.SceneManagement.SceneManager.GetActiveScene().path;
        
        // Add scene to build settings if not already added
        EditorBuildSettingsScene[] buildScenes = EditorBuildSettings.scenes;
        bool sceneInBuild = false;
        foreach (var scene in buildScenes)
        {
            if (scene.path == currentScenePath)
            {
                sceneInBuild = true;
                break;
            }
        }
        
        if (!sceneInBuild)
        {
            EditorBuildSettingsScene[] newBuildScenes = new EditorBuildSettingsScene[buildScenes.Length + 1];
            System.Array.Copy(buildScenes, newBuildScenes, buildScenes.Length);
            newBuildScenes[buildScenes.Length] = new EditorBuildSettingsScene(currentScenePath, true);
            EditorBuildSettings.scenes = newBuildScenes;
        }

        string buildPath = "Builds/HeadlessTraining.exe";
        
        BuildPlayerOptions buildPlayerOptions = new BuildPlayerOptions
        {
            scenes = new[] { currentScenePath },
            locationPathName = buildPath,
            target = BuildTarget.StandaloneWindows64,
            options = BuildOptions.Development
        };

        BuildPipeline.BuildPlayer(buildPlayerOptions);
        Debug.Log($"Build completed at: {buildPath}");
    }
}
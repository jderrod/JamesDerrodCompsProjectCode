using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Unity.Mathematics;
using UnityEngine.Splines;
using System.Collections;
using System.Collections.Generic;

public class CarAgent : Agent
{
    private ActionBuffers storedActions;
    private Rigidbody2D carRigidbody;
    public Transform wheelsTransform;

    // Car control parameters
    public float accelerationFactor = 1f;
    public float brakingFactor = .5f;
    public float turnFactor = 90f;
    public float driftFactor = 0.95f;

    public float distanceToNextTurn;
    public float nextTurnPosition;

    // Track parameters
    private SplineContainer splineContainer;
    public float trackWidth = 2f;
    private float outOfBoundsPenalty = -5.0f;
    private TrackGenerator trackGenerator;

    private float timeSinceReachedPoint = 0f;
    private float progressTimeLimitPerPoint = 15f;
    private int currentPointIndex = 0;
    private Vector2[] splinePoints;
    private int targetPointIndex = 1;

    private float currentSplineDistance = 0f;
    private float previousSplineDistance = 0f;

    private GameObject carPrefab;
    private GameObject carInstance;

        private int lapsCompleted = 0;
    private float progressMultiplier = 1.0f;
    private const float LAP_COMPLETION_REWARD = 10.0f;
    private int currentTurnIndex = 0;

    private List<bool> turnsCompleted;

    private void Awake()
    {
        // Get a reference to the car game object
        carInstance = GameObject.FindGameObjectWithTag("Player");

        if (carInstance != null)
        {
            // Get the Rigidbody2D and Wheels transform
            carRigidbody = carInstance.GetComponent<Rigidbody2D>();
            wheelsTransform = carInstance.transform.Find("Wheels");
        }
        else
        {
            Debug.LogError("Car game object not found in the scene!");
        }
    }

    private bool HasCompletedLap()
    {
        // Consider a lap complete if we've made it around the track (e.g., currentSplineDistance < previousSplineDistance)
        return currentSplineDistance < previousSplineDistance && currentSplineDistance < 0.1f;
    }

    private void UseDefaultPosition()
    {
        transform.position = new Vector3(7.3f, -.32f, 0);
        transform.rotation = Quaternion.identity;
    }

    public override void Initialize()
    {
        base.Initialize();
        Debug.Log("Agent initialized.");
        trackGenerator = FindObjectOfType<TrackGenerator>();
        if (trackGenerator == null)
        {
            Debug.LogError("No TrackGenerator found in scene!");
        }
        trackGenerator = FindObjectOfType<TrackGenerator>();
        StartCoroutine(InitializeRoutine());
    }

    private void InitializeSplinePoints()
    {
        if (splineContainer == null) return;

        int totalPoints = trackGenerator.numberOfPoints;
        splinePoints = new Vector2[totalPoints];
        
        // Store points in generation order
        for (int i = 0; i < totalPoints; i++)
        {
            float t = (float)i / totalPoints;
            Vector3 point3D = splineContainer.EvaluatePosition(t);
            splinePoints[i] = new Vector2(point3D.x, point3D.y);
        }
        
        // Always start at index 0 and target index 1
        currentPointIndex = 0;
        targetPointIndex = 1;
    }

    private IEnumerator InitializeRoutine()
    {
        yield return new WaitForSeconds(0.2f);
        
        if (trackGenerator != null)
        {
            splineContainer = trackGenerator.splineContainer;
            Debug.Log($"Found TrackGenerator and SplineContainer: {splineContainer != null}");
        }
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        storedActions = actionBuffers;  // Store the actions
        // Get and clamp actions
        float steeringInput = Mathf.Clamp(actionBuffers.ContinuousActions[0], -1f, 1f);
        float accelerationInput = Mathf.Clamp(actionBuffers.ContinuousActions[1], 0f, 1f);
        float brakingInput = Mathf.Clamp(actionBuffers.ContinuousActions[2], 0f, 1f);

        // Check if we're still within track bounds
        if (!IsWithinTrackBounds(transform.position))
        {
            return; // Episode ended in IsWithinTrackBounds
        }

        // Apply movement forces
        ApplyCarControls(steeringInput, accelerationInput, brakingInput);
        
        // Calculate rewards
        CheckProgressTimer();
    }

    private void ApplyCarControls(float steering, float acceleration, float braking)
    {
        // Apply forward thrust
        Vector2 engineForceVector = transform.up * acceleration * accelerationFactor;
        carRigidbody.AddForce(engineForceVector, ForceMode2D.Force);

        // Apply braking
        if (braking > 0)
        {
            carRigidbody.velocity = Vector2.Lerp(carRigidbody.velocity, Vector2.zero, 
                braking * Time.deltaTime * brakingFactor);
        }

        // Apply steering
        float turnAmount = -steering * turnFactor * Time.deltaTime;
        float newRotation = carRigidbody.rotation + turnAmount;

        // Handle drift
        Vector2 forwardVelocity = transform.up * Vector2.Dot(carRigidbody.velocity, transform.up);
        Vector2 rightVelocity = transform.right * Vector2.Dot(carRigidbody.velocity, transform.right);
        carRigidbody.velocity = forwardVelocity + rightVelocity * driftFactor;
        
        carRigidbody.MoveRotation(newRotation);
    }


    [Header("Track Sensors")]
    public float sensorLength = 5f;  // Length of the raycasts
    public float forwardAngle = 45f;  // Angle for diagonal forward sensors
    public bool showDebugRays = true;  // For visualization

    // Store sensor readings
    private struct SensorReadings
    {
        public float forwardLeft;
        public float forward;
        public float forwardRight;
        public float left;
        public float right;
    }

    private SensorReadings GetTrackEdgeSensorReadings()
    {
        if (splineContainer == null)
            return new SensorReadings 
            { 
                forwardLeft = sensorLength,
                forward = sensorLength,
                forwardRight = sensorLength,
                left = sensorLength,
                right = sensorLength
            };

        SensorReadings readings = new SensorReadings();
        
        // Forward sensors
        readings.forwardLeft = CastSensor(Quaternion.Euler(0, 0, forwardAngle) * transform.up);
        readings.forward = CastSensor(transform.up);
        readings.forwardRight = CastSensor(Quaternion.Euler(0, 0, -forwardAngle) * transform.up);
        
        // Side sensors
        readings.left = CastSensor(-transform.right);   // Left sensor
        readings.right = CastSensor(transform.right);   // Right sensor

        return readings;
    }

    private float CastSensor(Vector2 direction)
    {
        float distance = sensorLength;
        Vector2 startPos = transform.position;
        Vector2 endPos = startPos + direction * sensorLength;

        // Sample points along the ray
        int samples = 10;
        for (int i = 0; i <= samples; i++)
        {
            float t = i / (float)samples;
            Vector2 samplePoint = Vector2.Lerp(startPos, endPos, t);

            if (!CheckPointWithinTrack(samplePoint))
            {
                distance = Vector2.Distance(startPos, samplePoint);
                break;
            }
        }

        if (showDebugRays)
        {
            Color rayColor = Color.green;
            rayColor.a = 0.5f;
            Debug.DrawRay(transform.position, direction * distance, rayColor);
            if (distance < sensorLength)
            {
                Debug.DrawRay(transform.position + (Vector3)(direction * distance), direction * 0.5f, Color.red);
            }
        }

        return distance;
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        if (splinePoints == null || splineContainer == null)
        {
            // Add dummy observations if spline isn't initialized
            for (int i = 0; i < 22; i++) 
            {
                sensor.AddObservation(0f);
            }
            return;
        }
        sensor.AddObservation(timeSinceReachedPoint / progressTimeLimitPerPoint);


        // Distance to current target point
        float distanceToTarget = Vector2.Distance((Vector2)transform.position, splinePoints[targetPointIndex]);
        sensor.AddObservation(distanceToTarget);

        // Look ahead to next spline points
        for (int i = 0; i < 5; i++)
        {
            int lookAheadIndex = (targetPointIndex + i) % splinePoints.Length;
            Vector2 nextPoint = splinePoints[lookAheadIndex];
            Vector2 directionToPoint = nextPoint - (Vector2)transform.position;
            float angle = Vector2.SignedAngle(transform.up, directionToPoint);
            sensor.AddObservation(angle / 180f);
        }

        // Current velocity
        sensor.AddObservation(carRigidbody.velocity.x / 10f);
        sensor.AddObservation(carRigidbody.velocity.y / 10f);
        
        // Angular velocity
        sensor.AddObservation(carRigidbody.angularVelocity / 360f);

        // Get all sensor readings
        var readings = GetTrackEdgeSensorReadings();
        
        // Add normalized sensor readings (5 new observations)
        sensor.AddObservation(readings.forwardLeft / sensorLength);
        sensor.AddObservation(readings.forward / sensorLength);
        sensor.AddObservation(readings.forwardRight / sensorLength);
        sensor.AddObservation(readings.left / sensorLength);
        sensor.AddObservation(readings.right / sensorLength);

        // Track position info
        float nearestT = FindNearestPointOnSpline(transform.position);
        
        // Local position relative to nearest track point
        Vector3 nearestPoint = splineContainer.EvaluatePosition(nearestT);
        Vector3 localTargetDir = transform.InverseTransformPoint(nearestPoint);
        sensor.AddObservation(localTargetDir.x / 10f);
        sensor.AddObservation(localTargetDir.y / 10f);

        // Progress along track
        sensor.AddObservation(nearestT);
        
        // turn info
        sensor.AddObservation(trackGenerator.turnPoints[currentTurnIndex]); // Vector3 
        sensor.AddObservation(Vector2.Distance(transform.position, trackGenerator.turnPoints[currentTurnIndex]) / trackWidth);  
    }

    private void CheckProgressTimer()
    {
        if (splineContainer == null || splinePoints == null || trackGenerator == null || 
            trackGenerator.turnPoints == null || trackGenerator.turnPoints.Count == 0 ||
            currentTurnIndex >= trackGenerator.turnPoints.Count)
        {
            return;
        }

        // Calculate distance to target checkpoint
        float distanceToTarget = Vector2.Distance((Vector2)transform.position, splinePoints[targetPointIndex]);

        // Check turn completion
        int turnSplineIndex = Mathf.RoundToInt(FindNearestPointOnSpline(trackGenerator.turnPoints[currentTurnIndex]) * trackGenerator.numberOfPoints);
        int currentSplineIndex = Mathf.RoundToInt(FindNearestPointOnSpline(transform.position) * trackGenerator.numberOfPoints);
        
        if (currentSplineIndex == (turnSplineIndex + 2) % trackGenerator.numberOfPoints && !turnsCompleted[currentTurnIndex])
        {
            turnsCompleted[currentTurnIndex] = true;
            currentTurnIndex = (currentTurnIndex + 1) % trackGenerator.turnPoints.Count;
            Debug.Log($"Turn completed at index {currentSplineIndex}");
            AddReward(3.0f);
        }

        // Original checkpoint logic
        if (distanceToTarget < trackWidth)
        {
            timeSinceReachedPoint = 0f;
            currentPointIndex = targetPointIndex;
            targetPointIndex = (targetPointIndex + 1) % splinePoints.Length;

            if (targetPointIndex == 1)
            {
                trackGenerator.RecordLapCompletion(true);
                lapsCompleted++;
                AddReward(LAP_COMPLETION_REWARD);
                Debug.Log($"Lap completed!");
                EndEpisode();
            }

            float pointReward = 1.0f * progressMultiplier;
            AddReward(pointReward);

            Debug.DrawLine(
                new Vector3(splinePoints[currentPointIndex].x, splinePoints[currentPointIndex].y, 0),
                new Vector3(splinePoints[currentPointIndex].x, splinePoints[currentPointIndex].y + 2f, 0),
                Color.green,
                0.5f
            );

            Debug.Log($"Reached checkpoint {currentPointIndex}. Reward: {pointReward}");
        }
        else
        {
            timeSinceReachedPoint += Time.fixedDeltaTime;
        }

        if (timeSinceReachedPoint >= progressTimeLimitPerPoint)
        {
            AddReward(-8.0f);
            trackGenerator.RecordLapCompletion(false);
            EndEpisode();
            Debug.Log($"Episode ended due to timeout.");
        }
    }
    private bool IsWithinTrackBounds(Vector3 position)
    {
        if (splineContainer == null) return true;

        var carCollider = GetComponent<Collider2D>();
        if (carCollider == null)
        {
            return CheckPointWithinTrack(position);
        }

        // Get bounds corners
        Bounds bounds = carCollider.bounds;
        Vector3[] checkPoints = new Vector3[]
        {
            position + new Vector3(-bounds.extents.x, -bounds.extents.y, 0),
            position + new Vector3(bounds.extents.x, -bounds.extents.y, 0),
            position + new Vector3(-bounds.extents.x, bounds.extents.y, 0),
            position + new Vector3(bounds.extents.x, bounds.extents.y, 0),
            position + new Vector3(0, bounds.extents.y, 0),
            position + new Vector3(0, -bounds.extents.y, 0),
            position + new Vector3(-bounds.extents.x, 0, 0),
            position + new Vector3(bounds.extents.x, 0, 0),
            position
        };

        foreach (Vector3 point in checkPoints)
        {
            if (!CheckPointWithinTrack(point))
            {
                AddReward(outOfBoundsPenalty);
                trackGenerator.RecordLapCompletion(false);
                Debug.Log("Ending Episode OOB");
                EndEpisode();
                return false;
            }
        }

        return true;
    }

    private bool CheckPointWithinTrack(Vector3 point)
    {
        float nearestT = FindNearestPointOnSpline(point);
        Vector3 nearestPoint = splineContainer.EvaluatePosition(nearestT);
        Vector3 trackDirection = splineContainer.EvaluateTangent(nearestT);
        Vector3 trackNormal = Vector3.Cross(trackDirection, Vector3.forward).normalized;
        float distanceFromCenter = Vector3.Dot(point - nearestPoint, trackNormal);
        
        return Mathf.Abs(distanceFromCenter) <= trackWidth / 2f;
    }

    private float FindNearestPointOnSpline(Vector3 position)
    {
        float nearestT = 0f;
        float nearestDistance = float.MaxValue;
        int steps = 100;

        for (int i = 0; i < steps; i++)
        {
            float t = i / (float)steps;
            Vector3 point = splineContainer.EvaluatePosition(t);
            float distance = Vector3.Distance(position, point);

            if (distance < nearestDistance)
            {
                nearestDistance = distance;
                nearestT = t;
            }
        }

        return nearestT;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;

        continuousActionsOut[0] = Input.GetAxis("Horizontal");      // Steering
        continuousActionsOut[1] = Mathf.Clamp01(Input.GetAxis("Vertical"));   // Acceleration
        continuousActionsOut[2] = Mathf.Clamp01(-Input.GetAxis("Vertical")); // Braking

        // Debug info when spacebar is pressed
        if (Input.GetKey(KeyCode.Space))
        {
            Debug.Log($"Current position: {transform.position}, Velocity: {carRigidbody.velocity}, " +
                     $"Track width: {trackWidth}, Within bounds: {IsWithinTrackBounds(transform.position)}");
        }
    }

    public override void OnEpisodeBegin()
    {
        if (trackGenerator == null)
        {
            trackGenerator = FindObjectOfType<TrackGenerator>();
        }

        if (trackGenerator != null)
        {
            StartCoroutine(GenerateTrackAndPositionCar());
        }
        
        ResetState();
    }

    private IEnumerator GenerateTrackAndPositionCar()
    {
        yield return StartCoroutine(trackGenerator.GenerateNewTrackCoroutine());
        splineContainer = trackGenerator.splineContainer;
        InitializeSplinePoints();
        
        // Initialize turns list after track is generated
        currentTurnIndex = 0;
        turnsCompleted = new List<bool>();
        for (int i = 0; i < trackGenerator.turnPoints.Count; i++)
        {
            turnsCompleted.Add(false);
        }

        // Position the car at the starting position
        if (splinePoints != null && splinePoints.Length > 0)
        {
            Vector2 startPoint = splinePoints[0];
            Vector2 nextPoint = splinePoints[1];
            Vector2 direction = (nextPoint - startPoint).normalized;

            if (carInstance != null)
            {
                // Move the car to the starting position and rotate it
                carInstance.transform.position = new Vector3(startPoint.x, startPoint.y, 0);
                carInstance.transform.up = new Vector3(direction.x, direction.y, 0);

                // Reset the car's velocity and angular velocity
                if (carRigidbody != null)
                {
                    carRigidbody.velocity = Vector2.zero;
                    carRigidbody.angularVelocity = 0f;
                }

                if (wheelsTransform != null)
                {
                    wheelsTransform.localRotation = Quaternion.identity;
                }
            }
        }
        else
        {
            Debug.LogError("Spline points not initialized!");
        }
    }

    private void ResetState()
    {
        timeSinceReachedPoint = 0f;
        currentPointIndex = 0;
        targetPointIndex = 1;
        currentTurnIndex = 0;
        turnsCompleted = new List<bool>(new bool[trackGenerator.turnPoints.Count]);
    }


    void FixedUpdate()
    {
        RequestDecision();
    }

    void OnDrawGizmos()
    {
        if (!Application.isPlaying || splineContainer == null || splinePoints == null) return;
        
        float nearestT = FindNearestPointOnSpline(transform.position);
        
        // Visualize next points
        for (int i = 0; i < 5; i++)  
        {
            int lookAheadIndex = (targetPointIndex + i) % splinePoints.Length;
            Vector2 nextPoint = splinePoints[lookAheadIndex];
            
            // Debug visualization only
            Gizmos.color = Color.yellow;
            Gizmos.DrawSphere(new Vector3(nextPoint.x, nextPoint.y, 0), 0.2f);
            Gizmos.DrawLine(transform.position, new Vector3(nextPoint.x, nextPoint.y, 0));
        }
        
    }
}
using UnityEngine;
using UnityEngine.Splines;
using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using System.Linq;

public class TrackGenerator : MonoBehaviour
{
    public SplineContainer splineContainer;
    public int numberOfPoints = 25;
    public float baseRadius = 10f;
    public float trackWidth = 2f;
    public GameObject turnMarkerPrefab;

    [Header("Difficulty Settings")]
    public int maxTurns = 6;
    public int startingTurns = 3;
    public float turnIntensity = 3f;

    [Header("Performance-Based Progression")]
    public int episodesForAverage = 100;
    public int minimumEpisodes = 20;
    public float requiredCompletionRate = 0.75f;
    
    public List<Vector3> turnPoints = new List<Vector3>();
    private Queue<bool> completionHistory = new Queue<bool>();
    private int currentTurns;
    private int lastRecordedEpisode = -1;

    void Start()
    {
        currentTurns = startingTurns;
        StartCoroutine(GenerateNewTrackCoroutine());
    }

    public void RecordLapCompletion(bool completed)
    {
        int currentEpisode = Academy.Instance.EpisodeCount;
        
        if (currentEpisode != lastRecordedEpisode)
        {
            completionHistory.Enqueue(completed);
            if (completionHistory.Count > episodesForAverage)
            {
                completionHistory.Dequeue();
            }

            if (completionHistory.Count >= minimumEpisodes)
            {
                float completionRate = completionHistory.Count(x => x) / (float)completionHistory.Count;

                if (completionRate >= requiredCompletionRate && currentTurns < maxTurns)
                {
                    currentTurns++;
                    completionHistory.Clear();
                    Debug.Log($"Increasing turns to {currentTurns}. Previous completion rate: {completionRate:F2}");
                }
            }

            lastRecordedEpisode = currentEpisode;
        }
    }

    public IEnumerator GenerateNewTrackCoroutine()
    {
        yield return new WaitForEndOfFrame();
        GenerateRandomSpline();
        GenerateTrackMesh();
        PlaceTurnMarkers();
        
        Debug.Log($"Generating new track with parameters: Points={numberOfPoints}, " +
                 $"Radius={baseRadius}, Width={trackWidth}, Turns={currentTurns}, " +
                 $"Intensity={turnIntensity}");
    }

    void GenerateRandomSpline()
    {
        if (splineContainer == null)
        {
            GameObject splineObject = new GameObject("TrackSpline");
            splineObject.transform.parent = this.transform;
            splineContainer = splineObject.AddComponent<SplineContainer>();
        }

        Spline spline = splineContainer.Spline;
        spline.Clear();
        turnPoints.Clear();

        List<BezierKnot> knots = new List<BezierKnot>();
        HashSet<int> turnIndices = GenerateRandomTurnIndices();

        for (int i = 0; i < numberOfPoints; i++)
        {
            float angle = (float)i / numberOfPoints * Mathf.PI * 2f;
            float radius = baseRadius;

            if (turnIndices.Contains(i))
            {
                radius += Random.Range(-turnIntensity, turnIntensity);
                Vector3 turnPosition = new Vector3(Mathf.Cos(angle), Mathf.Sin(angle), 0f) * radius;
                turnPoints.Add(turnPosition);
            }

            Vector3 position = new Vector3(Mathf.Cos(angle), Mathf.Sin(angle), 0f) * radius;
            BezierKnot knot = new BezierKnot(position);
            Vector3 tangentDirection = new Vector3(-Mathf.Sin(angle), Mathf.Cos(angle), 0f);
            knot.TangentIn = -tangentDirection * (radius / 3f);
            knot.TangentOut = tangentDirection * (radius / 3f);
            knots.Add(knot);
        }

        foreach (var knot in knots)
        {
            spline.Add(knot);
        }

        spline.Closed = true;
        SmoothTangents(spline);
    }

    HashSet<int> GenerateRandomTurnIndices()
    {
        HashSet<int> turnIndices = new HashSet<int>();
        int minSpacingBetweenTurns = numberOfPoints / (currentTurns * 2);
        int attempts = 0;
        int maxAttempts = 100;

        while (turnIndices.Count < currentTurns && attempts < maxAttempts)
        {
            int randomIndex = Random.Range(0, numberOfPoints);
            bool tooClose = false;

            foreach (int existingIndex in turnIndices)
            {
                int distance = Mathf.Min(
                    Mathf.Abs(randomIndex - existingIndex),
                    numberOfPoints - Mathf.Abs(randomIndex - existingIndex)
                );
                
                if (distance < minSpacingBetweenTurns)
                {
                    tooClose = true;
                    break;
                }
            }

            if (!tooClose)
            {
                turnIndices.Add(randomIndex);
            }

            attempts++;
        }

        return turnIndices;
    }

    public (Vector3, float) GetNextTurnPosition(float currentT)
    {
        float minDistance = float.MaxValue;
        Vector3 nextTurnPosition = Vector3.zero;
        
        foreach (Vector3 turnPoint in turnPoints)
        {
            float turnT = FindNearestPointOnSpline(turnPoint);
            float distanceToTurn = (turnT - currentT + 1) % 1;
            
            if (distanceToTurn < minDistance && distanceToTurn > 0.05f)
            {
                minDistance = distanceToTurn;
                nextTurnPosition = turnPoint;
            }
        }
        
        return (nextTurnPosition, minDistance);
    }
    
    public float FindNearestPointOnSpline(Vector3 position)
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
    void GenerateTrackMesh()
    {
        MeshFilter meshFilter = splineContainer.gameObject.GetComponent<MeshFilter>();
        if (meshFilter == null)
            meshFilter = splineContainer.gameObject.AddComponent<MeshFilter>();

        MeshRenderer meshRenderer = splineContainer.gameObject.GetComponent<MeshRenderer>();
        if (meshRenderer == null)
            meshRenderer = splineContainer.gameObject.AddComponent<MeshRenderer>();

        Material trackMaterial = new Material(Shader.Find("Sprites/Default"));
        trackMaterial.color = Color.gray;
        meshRenderer.material = trackMaterial;
        meshRenderer.sortingOrder = -1;

        Mesh trackMesh = new Mesh();
        int subdivisions = 200;

        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();

        for (int i = 0; i <= subdivisions; i++)
        {
            float t = (float)i / subdivisions;
            Vector3 position = splineContainer.EvaluatePosition(t);
            Vector3 tangent = splineContainer.EvaluateTangent(t);
            Vector3 normal = Vector3.Cross(tangent, Vector3.forward).normalized;

            vertices.Add(position - normal * (trackWidth / 2f));
            vertices.Add(position + normal * (trackWidth / 2f));

            if (i < subdivisions)
            {
                int baseIndex = i * 2;
                triangles.Add(baseIndex);
                triangles.Add(baseIndex + 2);
                triangles.Add(baseIndex + 1);
                triangles.Add(baseIndex + 1);
                triangles.Add(baseIndex + 2);
                triangles.Add(baseIndex + 3);
            }
        }

        trackMesh.SetVertices(vertices);
        trackMesh.SetTriangles(triangles, 0);
        trackMesh.RecalculateNormals();
        trackMesh.RecalculateBounds();

        meshFilter.mesh = trackMesh;
    }

    void SmoothTangents(Spline spline)
    {
        for (int i = 0; i < spline.Count; i++)
        {
            BezierKnot knot = spline[i];
            int prevIndex = (i - 1 + spline.Count) % spline.Count;
            int nextIndex = (i + 1) % spline.Count;

            Vector3 knotPosition = (Vector3)spline[i].Position;
            Vector3 prevPosition = (Vector3)spline[prevIndex].Position;
            Vector3 nextPosition = (Vector3)spline[nextIndex].Position;

            Vector3 forwardTangent = (nextPosition - knotPosition).normalized;
            Vector3 backwardTangent = (knotPosition - prevPosition).normalized;
            Vector3 averagedTangent = (forwardTangent + backwardTangent) * 0.5f;

            float prevDistance = Vector3.Distance(knotPosition, prevPosition);
            float nextDistance = Vector3.Distance(knotPosition, nextPosition);
            
            knot.TangentIn = -averagedTangent * (prevDistance * 0.4f);
            knot.TangentOut = averagedTangent * (nextDistance * 0.4f);

            spline.SetKnot(i, knot);
        }
    }

    void PlaceTurnMarkers()
    {
        if (turnMarkerPrefab == null)
        {
            Debug.LogWarning("Turn marker prefab is not assigned!");
            return;
        }

        Transform markersParent = transform.Find("TurnMarkers");
        if (markersParent == null)
        {
            GameObject markersObject = new GameObject("TurnMarkers");
            markersObject.transform.parent = transform;
            markersParent = markersObject.transform;
        }
        else
        {
            foreach (Transform child in markersParent)
            {
                Destroy(child.gameObject);
            }
        }

        foreach (Vector3 turnPoint in turnPoints)
        {
            GameObject marker = Instantiate(turnMarkerPrefab, turnPoint, Quaternion.identity, markersParent);
            marker.name = "TurnMarker";
        }
    }
}
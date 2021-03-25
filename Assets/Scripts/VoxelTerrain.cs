using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Terminology:
//  ChunkGroup -> a group of Chunks which can be saved & loaded together
//  Chunk -> a single Y x Y x Y chunk of voxels; the smallest unit we generate meshes for
//  Block -> a 2x2 patch of voxels, identified by the coordinate of the top left voxel

public class Voxel
{
    public float signedDistance;
    public int materials;
}

public class VoxelChunk
{
    public const int CHUNK_SIZE = 32;

    public Voxel[] voxels = new Voxel[CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE];

    public VoxelChunk()
    {
        for (int i = 0; i < CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE; i++)
        {
            voxels[i] = new Voxel();
            voxels[i].signedDistance = 1f;
        }
    }

    public Voxel VoxelAtPosition(int x, int y, int z)
    {
        return voxels[x + CHUNK_SIZE * (y + CHUNK_SIZE * z)];
    }

    public Voxel VoxelAtPosition(Vector3Int position)
    {
        return VoxelAtPosition(position.x, position.y, position.z);
    }
}

public struct EdgeOffset
{
    public Vector3Int p1;
    public Vector3Int p2;

    public EdgeOffset(Vector3Int p1, Vector3Int p2)
    {
        this.p1 = p1;
        this.p2 = p2;
    }
}

public class EdgeSet
{
    private Dictionary<Vector3Int, Dictionary<Vector3Int, Edge>> seenEdges = new Dictionary<Vector3Int, Dictionary<Vector3Int, Edge>>();
    private List<Edge> edges = new List<Edge>();

    public List<Edge> AllEdges
    {
        get { return edges; }
    }

    public Edge GetEdge(Vector3Int p1, Vector3Int p2)
    {
        if (seenEdges.ContainsKey(p1) && seenEdges[p1].ContainsKey(p2))
        {
            return seenEdges[p1][p2];
        }

        return null;
    }

    public void AddEdge(Edge edge)
    {
        if (!seenEdges.ContainsKey(edge.edgePos1))
            seenEdges.Add(edge.edgePos1, new Dictionary<Vector3Int, Edge>());

        seenEdges[edge.edgePos1][edge.edgePos2] = edge;

        edges.Add(edge);
    }
}

// Fully represents an edge, including:
//  Its two positions within a chunk
//  The two voxels that make it up
//  Enough information to find its block & neighboring blocks
public class Edge
{
    public Vector3Int edgePos1;
    public Vector3Int edgePos2;
    public Voxel voxel1;
    public Voxel voxel2;
    public List<Vector3Int> blocks;
}

public class VoxelTerrain : MonoBehaviour
{
    // Given an (x,y,z) position for a voxel, these are all the offsets
    // that need to be applied to generate the 12 possible edges
    private EdgeOffset[] EdgeOffsets = new EdgeOffset[]
    {
        // Front face
        new EdgeOffset(new Vector3Int(0,0,0), new Vector3Int(1,0,0)),
        new EdgeOffset(new Vector3Int(0,0,0), new Vector3Int(0,1,0)),
        new EdgeOffset(new Vector3Int(0,1,0), new Vector3Int(1,1,0)),
        new EdgeOffset(new Vector3Int(1,0,0), new Vector3Int(1,1,0)),

        // Back face
        new EdgeOffset(new Vector3Int(0,0,1), new Vector3Int(1,0,1)),
        new EdgeOffset(new Vector3Int(0,0,1), new Vector3Int(0,1,1)),
        new EdgeOffset(new Vector3Int(0,1,1), new Vector3Int(1,1,1)),
        new EdgeOffset(new Vector3Int(1,0,1), new Vector3Int(1,1,1)),

        // Left side
        new EdgeOffset(new Vector3Int(0,0,0), new Vector3Int(0,0,1)),
        new EdgeOffset(new Vector3Int(0,1,0), new Vector3Int(0,1,1)),

        // Right side
        new EdgeOffset(new Vector3Int(1,0,0), new Vector3Int(1,0,1)),
        new EdgeOffset(new Vector3Int(1,1,0), new Vector3Int(1,1,1)),
    };

    private void FindSurfaceEdgesAndVertices(VoxelChunk chunk, EdgeSet edgeSet, List<Vector3> vertices, List<Vector3> normals, Dictionary<Vector3Int, int> blockVertexMap)
    {
        for (int x = 0; x < VoxelChunk.CHUNK_SIZE; x+=1)
        {
            for (int y = 0; y < VoxelChunk.CHUNK_SIZE; y+=1)
            {
                for (int z = 0; z < VoxelChunk.CHUNK_SIZE; z+=1)
                {
                    Vector3Int p0 = new Vector3Int(x, y, z);
                    Voxel v0 = chunk.VoxelAtPosition(p0);

                    Vector3 vertex = Vector3.zero;
                    Vector3 normal = Vector3.zero;
                    int numIntersectingEdges = 0;
                    foreach (EdgeOffset offset in EdgeOffsets)
                    {
                        // P1 & P2 represent the actual edge
                        Vector3Int p1 = p0 + offset.p1;
                        Vector3Int p2 = p0 + offset.p2;

                        // Bounds check - can't be an edge if any of these exist outside the chunk
                        // TODO: In a real-world scenario, we would evaluate neighboring chunks
                        if (p1.x < 0 || p1.y < 0 || p1.z < 0 ||
                            p2.x < 0 || p2.y < 0 || p2.z < 0)
                            continue;

                        if (p1.x >= VoxelChunk.CHUNK_SIZE || p1.y >= VoxelChunk.CHUNK_SIZE || p1.z >= VoxelChunk.CHUNK_SIZE ||
                            p2.x >= VoxelChunk.CHUNK_SIZE || p2.y >= VoxelChunk.CHUNK_SIZE || p2.z >= VoxelChunk.CHUNK_SIZE)
                            continue;

                        Voxel v1 = chunk.VoxelAtPosition(p1);
                        Voxel v2 = chunk.VoxelAtPosition(p2);

                        // Sign change - this is an edge we're interested in.
                        if (!Mathf.Approximately(Mathf.Sign(v1.signedDistance), Mathf.Sign(v2.signedDistance)))
                        {
                            // Keep track of the vertex, we'll be adding to the vertex list while we're in here
                            vertex += DetermineEdgePoint(p1, p2, v1, v2);
                            ++numIntersectingEdges;

                            // Add to our edge list
                            AddEdge(p0, p1, p2, v1, v2, edgeSet);

                            // Normal is the sum of all the gradients
                            normal += DetermineGradient(p1, p2, v1, v2);
                        }
                    }

                    if (numIntersectingEdges != 0)
                    {
                        vertex /= numIntersectingEdges;
                        vertices.Add(vertex);
                        normals.Add(normal);
                        blockVertexMap[p0] = vertices.Count - 1;
                    }
                }
            }
        }
    }

    private Vector3 DetermineEdgePoint(Vector3Int p1, Vector3Int p2, Voxel v1, Voxel v2)
    {
        Vector3 p1v3 = p1;
        Vector3 p2v3 = p2;

        float interp = (v1.signedDistance / (v1.signedDistance - v2.signedDistance));
        return (1.0f - interp) * p1v3 + interp * p2v3;
    }

    private Vector3 DetermineGradient(Vector3Int p1, Vector3Int p2, Voxel v1, Voxel v2)
    {
        Vector3 gradient = Vector3.zero;

        if (p1.x != p2.x)
            gradient.x += v2.signedDistance - v1.signedDistance;
        if (p1.y != p2.y)
            gradient.y += v2.signedDistance - v1.signedDistance;
        if (p1.z != p2.z)
            gradient.z += v2.signedDistance - v1.signedDistance;

        return gradient;
    }

    private void AddEdge(Vector3Int p0, Vector3Int p1, Vector3Int p2, Voxel v1, Voxel v2, EdgeSet edges)
    {
        Edge edge = edges.GetEdge(p1, p2);
        if (edge != null)
        {
            edge.blocks.Add(p0);
        }
        else
        {
            edge = new Edge();

            edge.blocks = new List<Vector3Int>();
            edge.blocks.Add(p0);
            edge.edgePos1 = p1;
            edge.edgePos2 = p2;
            edge.voxel1 = v1;
            edge.voxel2 = v2;

            edges.AddEdge(edge);
        }
    }

    private void GenerateFaces(EdgeSet surfaceEdges, List<Vector3> vertices, List<Vector3> normals, Dictionary<Vector3Int, int> blockVertexMap, List<int> indices)
    {
        foreach(Edge edge in surfaceEdges.AllEdges)
        {
            // At this point, an edge in the list should reference the 4 blocks that share that edge
            // If it doesn't, something went wrong.
            if (edge.blocks.Count != 4)
            {
                continue;
            }

            int i1 = blockVertexMap[edge.blocks[0]];
            int i2 = blockVertexMap[edge.blocks[1]];
            int i3 = blockVertexMap[edge.blocks[2]];
            int i4 = blockVertexMap[edge.blocks[3]];

            // Determine winding order based on our estimated normals
            Vector3 p1 = vertices[i1];
            Vector3 p2 = vertices[i2];
            Vector3 p3 = vertices[i3];
            Vector3 p4 = vertices[i4];

            Vector3 p1p2 = p2 - p1;
            Vector3 p1p3 = p3 - p1;
            Vector3 cross = Vector3.Cross(p1p2, p1p3);
            float dotNormal = Vector3.Dot(cross, normals[i1]);

            if (dotNormal >= 0f)
            {
                indices.Add(i1);
                indices.Add(i2);
                indices.Add(i3);

                indices.Add(i2);
                indices.Add(i4);
                indices.Add(i3);
            }
            else
            {
                indices.Add(i2);
                indices.Add(i1);
                indices.Add(i3);

                indices.Add(i2);
                indices.Add(i3);
                indices.Add(i4);
            }
        }
    }

    private void SmoothNormals(List<Vector3> vertices, List<int> indices, List<Vector3> outNormals)
    {
        // TODO : Garbage generation, yay
        List<int> normalCounts = new List<int>();
        outNormals.Clear();

        for (int i = 0; i < vertices.Count; ++i)
        {
            normalCounts.Add(0);
            outNormals.Add(Vector3.zero);
        }

        // We approximate the normals of the SDF, but now that we have real triangles we want real, accurate normals.
        // To do this, we first need to calculate all the surface normals. Then for every vertex,
        // average all the surface normals connected to that vertex.
        for (int i = 0; i < indices.Count; i += 3)
        {
            int i1 = indices[i + 0];
            int i2 = indices[i + 1];
            int i3 = indices[i + 2];

            Vector3 p1 = vertices[i1];
            Vector3 p2 = vertices[i2];
            Vector3 p3 = vertices[i3];

            Vector3 p1p2 = p2 - p1;
            Vector3 p1p3 = p3 - p1;
            Vector3 normal = Vector3.Cross(p1p2, p1p3);

            outNormals[i1] += normal;
            outNormals[i2] += normal;
            outNormals[i3] += normal;

            normalCounts[i1]++;
            normalCounts[i2]++;
            normalCounts[i3]++;
        }

        for (int i = 0; i < outNormals.Count; i++)
        {
            if (normalCounts[i] != 0)
            {
                outNormals[i] /= normalCounts[i];
                outNormals[i].Normalize();
            }
        }
    }

    /// TEST CODE ///



    VoxelChunk testChunk;

    private void Start()
    {
        testChunk = GenerateTestChunk();
        UpdateTestChunk();
    }

    void UpdateTestChunk()
    {
        EdgeSet edgeSet = new EdgeSet();
        Dictionary<Vector3Int, int> blockVertexMap = new Dictionary<Vector3Int, int>();
        List<int> faces = new List<int>();
        List<Vector3> vertices = new List<Vector3>();
        List<Vector3> normals = new List<Vector3>();

        FindSurfaceEdgesAndVertices(testChunk, edgeSet, vertices, normals, blockVertexMap);
        GenerateFaces(edgeSet, vertices, normals, blockVertexMap, faces);
        SmoothNormals(vertices, faces, normals);

        Mesh mesh = new Mesh();
        mesh.SetVertices(vertices);
        mesh.SetIndices(faces, MeshTopology.Triangles, 0);
        mesh.SetNormals(normals);
        GetComponent<MeshFilter>().mesh = mesh;
        GetComponent<MeshCollider>().sharedMesh = mesh;
    }

    private void Update()
    {
        if (Input.GetMouseButton(1))
            UpdateTestChunk();

        if (Input.GetMouseButton(0))
        {
            Debug.Log("Mouse button detected");

            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;
            if (Physics.Raycast(ray, out hit))
            {
                Debug.Log("Raycast Hit");

                Vector3 pos = hit.point;
                StartCoroutine(AnimateBlobGrow(pos));
                //Vector3Int posInt = new Vector3Int(Mathf.CeilToInt(pos.x), Mathf.CeilToInt(pos.y), Mathf.CeilToInt(pos.z));
                //RenderSphereIntoChunk(posInt, 2f, testChunk);
                //UpdateTestChunk();
            }
        }
    }

    private IEnumerator AnimateBlobGrow(Vector3 pos)
    {
        Vector3Int posInt = new Vector3Int(Mathf.CeilToInt(pos.x), Mathf.CeilToInt(pos.y), Mathf.CeilToInt(pos.z));
        for (float i = 1.0f; i <= 1.5f; i += 0.2f)
        {
            RenderSphereIntoChunk(posInt, i, testChunk);
            UpdateTestChunk();
            yield return null;
        }
    }

    VoxelChunk GenerateTestChunk()
    {
        VoxelChunk testChunk = new VoxelChunk();
        RenderSphereIntoChunk(new Vector3Int(16, 16, 16), 5f, testChunk);
        return testChunk;
    }

    void RenderSphereIntoChunk(Vector3Int center, float radius, VoxelChunk testChunk)
    {
        for (int x = 1; x < VoxelChunk.CHUNK_SIZE - 1; x++)
        {
            for (int y = 1; y < VoxelChunk.CHUNK_SIZE - 1; y++)
            {
                for (int z = 1; z < VoxelChunk.CHUNK_SIZE - 1; z++)
                {
                    Voxel voxel = testChunk.VoxelAtPosition(x, y, z);
                    Vector3 pos = new Vector3(x, y, z);
                    float sdf = Vector3.Distance(pos, center) - radius;

                    voxel.signedDistance = Mathf.Min(voxel.signedDistance, sdf);
                }
            }
        }
    }
}

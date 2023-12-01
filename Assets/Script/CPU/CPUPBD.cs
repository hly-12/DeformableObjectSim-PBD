using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System;
using System.Linq;
using Assets.script;
using UnityEngine.Rendering;

public class CPUPBD : MonoBehaviour
{
    public enum MyModel
    {
        IcoSphere_low,
        Torus,
        Bunny,
        Armadillo,
    };

    [Header("3D model")]
    public MyModel model;
    [HideInInspector]
    private string modelName;


    [Header("Obj Parameters")]
    public float invMass = 1.0f;
    public float dt = 0.01f; // have to devide by 20
    public Vector3 gravity = new Vector3(0f, -9.81f, 0f);
    public int iteration = 1;

    [Header("Distance Constrinat Parameters")]
    public float stretchStiffness = 1.0f;
    public float compressStiffness = 1.0f;
    [Header("Bending Constrinat Parameters")]
    public float bendingStiffness = 1.0f;
    [Header("Volume  Constrinat Parameters")]
    public float volumeStiffness = 1.0f;

    [Header("Collision")]
    public GameObject[] collidableObjects;

    [HideInInspector]
    private int nodeCount;
    private int springCount;
    private int triCount; // size of triangle
    private int tetCount;
    private int bendingCount;
    private int numCollidableSpheres, numCollidableCubes;



    //main  property
    //list position
    Vector3[] Positions;
    Vector3[] ProjectPositions;
    Vector3[] WorldPositions;
    Vector3[] Velocities;
    Vector3[] Forces;
    Vector3[] Normals;
    List<Spring> distanceConstraints = new List<Spring>();
    List<Triangle> triangles = new List<Triangle>();
    List<Tetrahedron> tetrahedrons = new List<Tetrahedron>();
    List<Bending> bendingConstraints = new List<Bending>();


    Vector3[] DeltaPos;
    int[] deltaCounter;


    bool[] collidedNodes;

    //for render
    ComputeBuffer vertsBuff = null;
    ComputeBuffer triBuffer = null;

    [Header("Rendering Paramenter")]
    public Shader renderingShader;
    public Color matColor;

    [HideInInspector]
    private Material material;

    [Header("Label Data")]
    public bool renderVolumeText;
    public string Text;
    public int xOffset;
    public int yOffset;
    public int fontSize;
    public Color textColor = Color.white;
    private Rect rectPos;
    private Color color;

    //public Material material;

    struct vertData
    {
        public Vector3 pos;
        public Vector2 uvs;
        public Vector3 norms;
    };
    int[] triArray;
    vertData[] vDataArray;

    float totalVolume;
    void SelectModelName()
    {
        switch (model)
        {
            case MyModel.IcoSphere_low: modelName = "icosphere_low.1"; break;
            case MyModel.Torus: modelName = "torus.1"; break;
            case MyModel.Bunny: modelName = "bunny.1"; break;
            case MyModel.Armadillo: modelName = "Armadillo.1"; break;
        }
    }

    private void setupMeshData()
    {
        //print(Application.dataPath);
        string filePath = Application.dataPath + "/TetGen-Model/";
        LoadTetModel.LoadData(filePath + modelName, gameObject);

        Positions = LoadTetModel.positions.ToArray();
        triangles = LoadTetModel.triangles;
        distanceConstraints = LoadTetModel.springs;
        triArray = LoadTetModel.triangleArr.ToArray();
        //if (useTetVolumeConstraint)
        tetrahedrons = LoadTetModel.tetrahedrons;
        bendingConstraints = LoadTetModel.bendings;

        nodeCount = Positions.Length;
        springCount = distanceConstraints.Count;
        triCount = triangles.Count; //
        tetCount = tetrahedrons.Count;
        bendingCount = bendingConstraints.Count;

        WorldPositions = new Vector3[nodeCount];
        ProjectPositions = new Vector3[nodeCount];
        Velocities = new Vector3[nodeCount];
        Forces = new Vector3[nodeCount];
        DeltaPos = new Vector3[nodeCount];
        deltaCounter = new int[nodeCount];
        DeltaPos.Initialize();
        deltaCounter.Initialize();

        ProjectPositions = LoadTetModel.positions.ToArray();
        WorldPositions.Initialize();
        Velocities.Initialize();
        Forces.Initialize();

        vDataArray = new vertData[nodeCount];

        for (int i = 0; i < nodeCount; i++)
        {
            vDataArray[i] = new vertData();
            vDataArray[i].pos = Positions[i];
            vDataArray[i].norms = Vector3.zero;
            vDataArray[i].uvs = Vector3.zero;
        }

        int triBuffStride = sizeof(int);
        triBuffer = new ComputeBuffer(triArray.Length,
            triBuffStride, ComputeBufferType.Default);


        int vertsBuffstride = 8 * sizeof(float);
        vertsBuff = new ComputeBuffer(vDataArray.Length,
            vertsBuffstride, ComputeBufferType.Default);
        LoadTetModel.ClearData();

        print("node count: "+nodeCount);
        print("stretch constraint: " + springCount);
        print("bending constraint: "+bendingCount);
        print("volume constraint: "+tetCount);

    }
    private void setupShader()
    {
        material.SetBuffer(Shader.PropertyToID("vertsBuff"), vertsBuff);
        material.SetBuffer(Shader.PropertyToID("triBuff"), triBuffer);
    }
    private void setBuffData()
    {

        vertsBuff.SetData(vDataArray);
        triBuffer.SetData(triArray);
        //Quaternion rotate = new Quaternion(0, 0, 0, 0);
        //transform.Rotate(rotate.eulerAngles);
        //transform.
        Vector3 translation = transform.position;
        Vector3 scale = this.transform.localScale;
        Quaternion rotationeuler = transform.rotation;
        Matrix4x4 trs = Matrix4x4.TRS(translation, rotationeuler, scale);
        material.SetMatrix("TRSMatrix", trs);
        material.SetMatrix("invTRSMatrix", trs.inverse);
    }
    void setup()
    {
        //obj = gameObject;
        material = new Material(renderingShader); // new material for difference object
        material.color = matColor; //set color to material

        SelectModelName();
        setupMeshData();
        //setupCollisionConstraint();
        setupShader();
        setBuffData();

        totalVolume = computeObjectVolume();
    }

    void Start()
    {

        setup();
        //foreach (Tetrahedron t in tetrahedrons)
        //{
        //    totalVolume += t.RestVolume;
        //}

        //print(transform.localToWorldMatrix);
        //print(transform.worldToLocalMatrix);
    }
    void addExternalForce(Vector3 force)
    {
        for (int i = 0; i < nodeCount; i++)
        {
            Velocities[i] += (force / invMass) * dt;
            //cout << to_string(Velocities[i]) << endl;
        }
    }
    void addExplicitEuler()
    {
        for (int i = 0; i < nodeCount; i++)
        {
            ProjectPositions[i] = Positions[i] + Velocities[i] * dt;
            //cout << to_string(ProjectPositions[i]) << endl;
        }
    }

    void collisionDetectionAndRespone()
    {
        for (int i = 0; i < nodeCount; i++)
        {
            //floor position = Vector3(x,-2,z)
            //if (transform.TransformPoint(ProjectPositions[i]).y < -2.0f)
            //{

            //    ProjectPositions[i].y = transform.InverseTransformPoint(
            //        new Vector3(ProjectPositions[i].x, -2.0f, ProjectPositions[i].z)).y;
            //    Velocities[i] = Vector3.zero;
            //}
            if ((ProjectPositions[i]).y < -2.0f)
            {
                ProjectPositions[i].y = -2.0f+0.01f;
                Velocities[i] = Vector3.zero;
            }

        }
    }
    void satisfyDistanceConstraint()
    {
        for (int i = 0; i < springCount; i++)
        {
            Spring constraint = distanceConstraints[i];
            int i1 = constraint.i1;
            int i2 = constraint.i2;
            float restLength = constraint.RestLength;

            Vector3 pi = ProjectPositions[i1];
            Vector3 pj = ProjectPositions[i2];

            float d = Vector3.Distance(pi, pj);

            Vector3 n = (pi - pj).normalized;

            float wi = invMass; //inverse mass
            float wj = invMass; //inverse mass


            float stiffness = d < restLength ? compressStiffness : stretchStiffness;
            Vector3 deltaP1 = stiffness * wi / (wi + wj) * (d - restLength) * n;
            Vector3 deltaP2 = stiffness * wj / (wi + wj) * (d - restLength) * n;

            ProjectPositions[i1] -= deltaP1;
            ProjectPositions[i2] += deltaP2;
        }
    }

    void satisfyBendingConstraint()
    {
        for (int i = 0; i < bendingCount; i++)
        {
            Bending bending = bendingConstraints[i];

            Vector3 p0 = ProjectPositions[bending.index0];
            Vector3 p1 = ProjectPositions[bending.index1];
            Vector3 p2 = ProjectPositions[bending.index2];
            Vector3 p3 = ProjectPositions[bending.index3];

            //Vector3 n1 = (Vector3.Cross(p2 - p0, p3 - p0)).normalized;
            //Vector3 n2 = (Vector3.Cross(p3 - p1, p2 - p1)).normalized;

            //float d = Vector3.Dot(n1, n2);
            //d = Mathf.Clamp(d,- 1.0f, 1.0f);
            //float restAngle = Mathf.Acos(d);

            Vector3 wing = p3 - p2;
            float wingLength = wing.magnitude;

            if (wingLength >= 1e-7)
            {
                Vector3 n1 = Vector3.Cross(p2 - p0, p3 - p0);
                n1 /= n1.sqrMagnitude;

                Vector3 n2 = Vector3.Cross(p3 - p1, p2 - p1);
                n2 /= n2.sqrMagnitude;

                float invWingLength = 1.0f / wingLength;

                Vector3 q0 = wingLength * n1;
                Vector3 q1 = wingLength * n2;
                Vector3 q2 = Vector3.Dot(p0 - p3, wing) * invWingLength * n1
                            + Vector3.Dot(p1 - p3, wing) * invWingLength * n2;
                Vector3 q3 = Vector3.Dot(p2 - p0, wing) * invWingLength * n1
                            + Vector3.Dot(p2 - p1, wing) * invWingLength * n2;

                n1.Normalize();
                n2.Normalize();

                float d = Vector3.Dot(n1, n2);
                d = Mathf.Clamp(d, -1.0f, 1.0f);
                float currentAngle = Mathf.Acos(d);

                float lamda = 0;
                lamda += invMass * q0.sqrMagnitude;
                lamda += invMass * q1.sqrMagnitude;
                lamda += invMass * q2.sqrMagnitude;
                lamda += invMass * q3.sqrMagnitude;

                if (lamda != 0.0f)
                {
                    lamda = (currentAngle - bending.restAngle) / lamda * bendingStiffness;

                    if (Vector3.Dot(Vector3.Cross(n1, n2), wing) > 0.0f)
                    {
                        lamda = -lamda;
                    }

                    ProjectPositions[bending.index0] -= invMass * lamda * q0;
                    ProjectPositions[bending.index1] -= invMass * lamda * q1;
                    ProjectPositions[bending.index2] -= invMass * lamda * q2;
                    ProjectPositions[bending.index3] -= invMass * lamda * q3;
                }
            }
        }
    }
    void satisfyVolumeConstraint()
    {
        for (int i = 0; i < tetCount; i++)
        {

            Tetrahedron t = tetrahedrons[i];
            //cout << t.initVolume << endl;

            int idx1 = t.i1;
            int idx2 = t.i2;
            int idx3 = t.i3;
            int idx4 = t.i4;

            Vector3 p0 = ProjectPositions[idx1];
            Vector3 p1 = ProjectPositions[idx2];
            Vector3 p2 = ProjectPositions[idx3];
            Vector3 p3 = ProjectPositions[idx4];

            float volume = computeTetraVolume(p0, p1, p2, p3);

            float restVolume = t.RestVolume;

            Vector3 grad0 = Vector3.Cross(p1 - p2, p3 - p2);
            Vector3 grad1 = Vector3.Cross(p2 - p0, p3 - p0);
            Vector3 grad2 = Vector3.Cross(p0 - p1, p3 - p1);
            Vector3 grad3 = Vector3.Cross(p1 - p0, p2 - p0);

            float lambda = grad0.x * grad0.x + grad0.y * grad0.y + grad0.z * grad0.z +
                grad1.x * grad1.x + grad1.y * grad1.y + grad1.z * grad1.z +
                grad2.x * grad2.x + grad2.y * grad2.y + grad2.z * grad2.z +
                grad3.x * grad3.x + grad3.y * grad3.y + grad3.z * grad3.z;


            lambda = volumeStiffness * (volume - restVolume) / lambda;

            Vector3 deltaP1 = -lambda * grad0;
            Vector3 deltaP2 = -lambda * grad1;
            Vector3 deltaP3 = -lambda * grad2;
            Vector3 deltaP4 = -lambda * grad3;

            ProjectPositions[idx1] += deltaP1;
            ProjectPositions[idx2] += deltaP2;
            ProjectPositions[idx3] += deltaP3;
            ProjectPositions[idx4] += deltaP4;

        }
    }

    void updatePositions()
    {
        for (int i = 0; i < nodeCount; i++)
        {
            Velocities[i] = (ProjectPositions[i] - Positions[i]) / dt;
            Positions[i] = ProjectPositions[i];
            vDataArray[i].pos = Positions[i];
        }
    }

    void PBDSolving()
    {
        addExternalForce(gravity);
        //TODO: damp velocity 
        addExplicitEuler();

        for (int j = 0; j < iteration; j++)
        {
            satisfyDistanceConstraint();
            satisfyBendingConstraint();
            satisfyVolumeConstraint();
            collisionDetectionAndRespone();
        }
        updatePositions();
        //TODO: apply friction
    }


    void computeVertexNormal()
    {
        //

        for (int i = 0; i < triCount; i++)
        {
            //print(TriIndices[i * 3 + 0]+","+ TriIndices[i * 3 + 1] + "," +TriIndices[i * 3 + 2]);
            //
            Vector3 v1 = Positions[triArray[i * 3 + 0]];
            Vector3 v2 = Positions[triArray[i * 3 + 1]];
            Vector3 v3 = Positions[triArray[i * 3 + 2]];

            Vector3 N = (Vector3.Cross(v2 - v1, v3 - v1));

            vDataArray[triArray[i * 3 + 0]].norms += N;
            vDataArray[triArray[i * 3 + 1]].norms += N;
            vDataArray[triArray[i * 3 + 2]].norms += N;
        }
        for (int i = 0; i < nodeCount; i++)
        {
            vDataArray[i].norms = vDataArray[i].norms.normalized;
        }
    }
    void Update()
    {
        PBDSolving();
        computeVertexNormal();
        vertsBuff.SetData(vDataArray);

        Bounds bounds = new Bounds(Vector3.zero, Vector3.one * 100);

        material.SetPass(0);
        Graphics.DrawProcedural(
            material,
            bounds,
            MeshTopology.Triangles,
            triArray.Length,
            1,
            null,
            null,
            ShadowCastingMode.On,
            true,
            gameObject.layer
        );

    }
    private float computeTetraVolume(Vector3 i1, Vector3 i2, Vector3 i3, Vector3 i4)
    {
        float volume = 0.0f;

        volume = 1.0f / 6.0f
            * (i3.x * i2.y * i1.z - i4.x * i2.y * i1.z - i2.x * i3.y * i1.z
            + i4.x * i3.y * i1.z + i2.x * i4.y * i1.z - i3.x * i4.y * i1.z
            - i3.x * i1.y * i2.z + i4.x * i1.y * i2.z + i1.x * i3.y * i2.z
            - i4.x * i3.y * i2.z - i1.x * i4.y * i2.z + i3.x * i4.y * i2.z
            + i2.x * i1.y * i3.z - i4.x * i1.y * i3.z - i1.x * i2.y * i3.z
            + i4.x * i2.y * i3.z + i1.x * i4.y * i3.z - i2.x * i4.y * i3.z
            - i2.x * i1.y * i4.z + i3.x * i1.y * i4.z + i1.x * i2.y * i4.z
            - i3.x * i2.y * i4.z - i1.x * i3.y * i4.z + i2.x * i3.y * i4.z);

        return volume;
    }

    //private void OnGUI()
    //{
    //    int w = Screen.width, h = Screen.height;
    //    GUIStyle style = new GUIStyle();
    //    Rect rect = new Rect(0, 0, w, h * 2 / 100);
    //    style.alignment = TextAnchor.UpperLeft;
    //    style.fontSize = h * 2 / 50;
    //    style.normal.textColor = new Color(1, 1, 1, 1.0f);

    //    float currVolume = 0;
    //    foreach (Tetrahedron t in tetrahedrons)
    //    {
    //        //currVolume += t.restVolume;
    //        currVolume += computeTetraVolume(Positions[t.i1], Positions[t.i2],
    //            Positions[t.i3], Positions[t.i4]);
    //        //print(t.restVolume);
    //    }
    //    float vLost = (currVolume / totalVolume) * 100.0f;
    //    string text = string.Format("Volume: {0:0.00} %", vLost);
    //    GUI.Label(rect, text, style);


    //}
    float computeObjectVolume()
    {
        //made by sum of all tetra
        float volume = 0.0f;
        foreach (Tetrahedron tet in tetrahedrons)
        {
            Vector3 p0 = Positions[tet.i1];
            Vector3 p1 = Positions[tet.i2];
            Vector3 p2 = Positions[tet.i3];
            Vector3 p3 = Positions[tet.i4];
            volume += computeTetraVolume(p0, p1, p2, p3);
        }
        return volume;
    }
    private void OnGUI()
    {
        if (renderVolumeText)
        {
            int w = Screen.width, h = Screen.height;
            GUIStyle style = new GUIStyle();

            rectPos = new Rect(0 + xOffset, yOffset, w, h * 2 / 100);
            Rect rect = rectPos;
            style.alignment = TextAnchor.UpperLeft;
            style.fontSize = h * 2 / 50;
            Color col;
            string htmlValue = "#FFED00";
            if (ColorUtility.TryParseHtmlString(htmlValue, out col))
                style.normal.textColor = col;

            //get volume data;

            float currVolume = 0;
            currVolume = computeObjectVolume();

            float vLost;
            if (totalVolume == 0.0f)
            {
                vLost = 0.0f;
            }
            else
            {
                vLost = (currVolume / totalVolume) * 100.0f;
            }
            string text = string.Format("Volume: {0:0.00} %", vLost);
            GUI.Label(rect, text, style);
        }
    }



    //private void OnDrawGizmos()
    //{
    //    if (collidedNodes == null) return;

    //    Gizmos.color = Color.yellow;
    //    for(int i = 0;i<nodeCount;i++)
    //    {
    //        if (collidedNodes[i])
    //            Gizmos.DrawSphere(Positions[i], 0.1f);
    //    }
    //}

    private void OnDestroy()
    {
        if (this.enabled)
        {
            vertsBuff.Dispose();
            triBuffer.Dispose();
        }
    }
}

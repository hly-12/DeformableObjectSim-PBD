﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System;
using System.Linq;
using Assets.script;
using UnityEngine.Rendering;

public class GPUPBD : MonoBehaviour
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
    public int iteration = 5;
    public float convergence_factor = 1.5f;

    [Header("Distance Constrinat Parameters")]
    public float stretchStiffness = 1.0f;
    public float compressStiffness = 1.0f;
    [Header("Bending Constrinat Parameters")]
    public float bendingStiffness = 1.0f;
    [Header("Volume  Constrinat Parameters")]
    public float volumeStiffness = 1.0f;

    [Header("Collision")]
    public GameObject floor;

    [Header("Label Data")]
    public bool renderVolumeText;
    public string Text;
    public int xOffset;
    public int yOffset;
    public int fontSize;
    public Color textColor = Color.white;
    private Rect rectPos;
    private Color color;

    [Header("Volume Data")]
    public bool writeVolumeToFile;
    public bool writeImageFrame;
    public int maxFrameNum;
    public string directory = "";
    public string volumeFileName = "";

    [HideInInspector]
    private int nodeCount;
    private int springCount;
    private int triCount; // size of triangle
    private int tetCount;
    private int bendingCount;

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
    //for render
    ComputeBuffer vertsBuff = null;
    ComputeBuffer triBuffer = null;

    //for compute shader
    private ComputeBuffer positionsBuffer;
    private ComputeBuffer projectedPositionsBuffer;
    private ComputeBuffer velocitiesBuffer;

    private ComputeBuffer triangleBuffer;
    private ComputeBuffer triangleIndicesBuffer;

    private ComputeBuffer deltaPositionsBuffer;
    private ComputeBuffer deltaPositionsUIntBuffer;
    private ComputeBuffer deltaCounterBuffer;

    private ComputeBuffer objVolumeBuffer;

    private ComputeBuffer distanceConstraintsBuffer;
    private ComputeBuffer bendingConstraintsBuffer;
    private ComputeBuffer tetVolConstraintsBuffer;

    //kerne id (might not use all currently)
    //private int applyExternalForcesKernel;
    //private int dampVelocitiesKernel;
    private int applyExplicitEulerKernel;
    private int floorCollisionKernel;


    private int satisfyDistanceConstraintKernel;
    private int satisfyBendingConstraintKernel;
    private int satisfyTetVolConstraintKernel;

    private int averageConstraintDeltasKernel;
    private int updatePositionsKernel;

    private int computeObjVolumeKernel;   // for compute object's volume

    private int computeVerticesNormal; // for rendering purpose 

    [Header("Rendering Paramenter")]
    public ComputeShader computeShader;
    public Shader renderingShader;
    public Color matColor;

    [HideInInspector]
    private Material material;
    private ComputeShader computeShaderobj;

    struct vertData
    {
        public Vector3 pos;
        public Vector2 uvs;
        public Vector3 norms;
    };
    int[] triArray;
    vertData[] vDataArray;
    private static GameObject obj;

    float totalVolume; //
    int frame = 0; //number ot time frame
    float[] volumeDataGPU = new float[1]; //use to get data from GPU

    [HideInInspector]
    List<string[]> tableData = new List<string[]>();

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

    void addBendingConstraint()
    {
        Dictionary<Edge, List<Triangle>> wingEdges = new Dictionary<Edge, List<Triangle>>(new EdgeComparer());

        // map edges to all of the faces to which they are connected
        foreach (Triangle tri in triangles)
        {
            Edge e1 = new Edge(tri.vertices[0], tri.vertices[1]);
            if (wingEdges.ContainsKey(e1) && !wingEdges[e1].Contains(tri))
            {
                wingEdges[e1].Add(tri);
            }
            else
            {
                List<Triangle> tris = new List<Triangle>();
                tris.Add(tri);
                wingEdges.Add(e1, tris);
            }

            Edge e2 = new Edge(tri.vertices[0], tri.vertices[2]);
            if (wingEdges.ContainsKey(e2) && !wingEdges[e2].Contains(tri))
            {
                wingEdges[e2].Add(tri);
            }
            else
            {
                List<Triangle> tris = new List<Triangle>();
                tris.Add(tri);
                wingEdges.Add(e2, tris);
            }

            Edge e3 = new Edge(tri.vertices[1], tri.vertices[2]);
            if (wingEdges.ContainsKey(e3) && !wingEdges[e3].Contains(tri))
            {
                wingEdges[e3].Add(tri);
            }
            else
            {
                List<Triangle> tris = new List<Triangle>();
                tris.Add(tri);
                wingEdges.Add(e3, tris);
            }
        }

        // wingEdges are edges with 2 occurences,
        // so we need to remove the lower frequency ones
        List<Edge> keyList = wingEdges.Keys.ToList();
        foreach (Edge e in keyList)
        {
            if (wingEdges[e].Count < 2)
            {
                wingEdges.Remove(e);
            }
        }

        bendingCount = wingEdges.Count;

        foreach (Edge wingEdge in wingEdges.Keys)
        {
            /* wingEdges are indexed like in the Bridson,
                * Simulation of Clothing with Folds and Wrinkles paper
                *    3
                *    ^
                * 0  |  1
                *    2
                */

            int[] indices = new int[4];
            indices[2] = wingEdge.startIndex;
            indices[3] = wingEdge.endIndex;

            int b = 0;
            foreach (Triangle tri in wingEdges[wingEdge])
            {
                for (int i = 0; i < 3; i++)
                {
                    int point = tri.vertices[i];
                    if (point != indices[2] && point != indices[3])
                    {
                        //tri #1
                        if (b == 0)
                        {
                            indices[0] = point;
                            break;
                        }
                        //tri #2
                        else if (b == 1)
                        {
                            indices[1] = point;
                            break;
                        }
                    }
                }
                b++;
            }
            Vector3 p0 = Positions[indices[0]];
            Vector3 p1 = Positions[indices[1]];
            Vector3 p2 = Positions[indices[2]];
            Vector3 p3 = Positions[indices[3]];

            Vector3 n1 = (Vector3.Cross(p2 - p0, p3 - p0)).normalized;
            Vector3 n2 = (Vector3.Cross(p3 - p1, p2 - p1)).normalized;

            float d = Vector3.Dot(n1, n2);
            d = Mathf.Clamp(d, -1.0f, 1.0f);

            Bending bending = new Bending();
            bending.index0 = indices[0];
            bending.index1 = indices[1];
            bending.index2 = indices[2];
            bending.index3 = indices[3];
            bending.restAngle = Mathf.Acos(d);

            bendingConstraints.Add(bending);
        }
    }

    void setupMeshData()
    {
        //print(Application.dataPath);
        string filePath = Application.dataPath + "/TetGen-Model/";
        LoadTetModel.LoadData(filePath + modelName, gameObject);

        Positions = LoadTetModel.positions.ToArray();
        triangles = LoadTetModel.triangles;
        distanceConstraints = LoadTetModel.springs;
        triArray = LoadTetModel.triangleArr.ToArray();

        tetrahedrons = LoadTetModel.tetrahedrons;
        bendingConstraints = LoadTetModel.bendings;

        nodeCount = Positions.Length;
        springCount = distanceConstraints.Count;
        triCount = triangles.Count; //
        tetCount = tetrahedrons.Count;
        bendingCount = bendingConstraints.Count;

        print("node count: " + nodeCount);
        print("stretch constraint: " + springCount);
        print("bending constraint: " + bendingCount);
        print("volume constraint: " + tetCount);

        WorldPositions = new Vector3[nodeCount];
        ProjectPositions = new Vector3[nodeCount];
        Velocities = new Vector3[nodeCount];
        Forces = new Vector3[nodeCount];
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

        Vector3 translation = transform.position;
        Vector3 scale = this.transform.localScale;
        Quaternion rotationeuler = transform.rotation;
        Matrix4x4 trs = Matrix4x4.TRS(translation, rotationeuler, scale);
        material.SetMatrix("TRSMatrix", trs);
        material.SetMatrix("invTRSMatrix", trs.inverse);
    }

    private void setupComputeBuffer()
    {
        positionsBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        positionsBuffer.SetData(Positions);

        velocitiesBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        velocitiesBuffer.SetData(Velocities);

        projectedPositionsBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        projectedPositionsBuffer.SetData(Positions);

        UInt3Struct[] deltaPosUintArray = new UInt3Struct[nodeCount];
        deltaPosUintArray.Initialize();
        Vector3[] deltaPositionArray = new Vector3[nodeCount];
        deltaPositionArray.Initialize();

        int[] deltaCounterArray = new int[nodeCount];
        deltaCounterArray.Initialize();


        deltaPositionsBuffer = new ComputeBuffer(nodeCount, sizeof(float) * 3);
        deltaPositionsBuffer.SetData(deltaPositionArray);

        deltaPositionsUIntBuffer = new ComputeBuffer(nodeCount, sizeof(uint) * 3);
        deltaPositionsUIntBuffer.SetData(deltaPosUintArray);

        deltaCounterBuffer = new ComputeBuffer(nodeCount, sizeof(int));
        deltaCounterBuffer.SetData(deltaCounterArray);




        List<MTriangle> initTriangle = new List<MTriangle>();  //list of triangle cooresponding to node 
        List<int> initTrianglePtr = new List<int>(); //contain a group of affectd triangle to node
        initTrianglePtr.Add(0);

        for (int i = 0; i < nodeCount; i++)
        {
            foreach (Triangle tri in triangles)
            {
                if (tri.vertices[0] == i || tri.vertices[1] == i || tri.vertices[2] == i)
                {
                    MTriangle tmpTri = new MTriangle();
                    tmpTri.v0 = tri.vertices[0];
                    tmpTri.v1 = tri.vertices[1];
                    tmpTri.v2 = tri.vertices[2];
                    initTriangle.Add(tmpTri);
                }
            }
            initTrianglePtr.Add(initTriangle.Count);
        }

        print(initTrianglePtr.Count);

        triangleBuffer = new ComputeBuffer(initTriangle.Count, (sizeof(int) * 3));
        triangleBuffer.SetData(initTriangle.ToArray());

        triangleIndicesBuffer = new ComputeBuffer(initTrianglePtr.Count, sizeof(int));
        triangleIndicesBuffer.SetData(initTrianglePtr.ToArray());


        distanceConstraintsBuffer = new ComputeBuffer(springCount, sizeof(float) + sizeof(int) * 2);
        distanceConstraintsBuffer.SetData(distanceConstraints.ToArray());

        bendingConstraintsBuffer = new ComputeBuffer(bendingCount, sizeof(float) + sizeof(int) * 4);
        bendingConstraintsBuffer.SetData(bendingConstraints.ToArray());

        tetVolConstraintsBuffer = new ComputeBuffer(tetCount, sizeof(float) + sizeof(int) * 4);
        tetVolConstraintsBuffer.SetData(tetrahedrons.ToArray());

        uint[] initUint = new uint[1];
        initUint.Initialize();
        objVolumeBuffer = new ComputeBuffer(1, sizeof(uint));
        objVolumeBuffer.SetData(initUint);



    }


    float computeObjectVolume()
    {
        //made by sum of all tetra
        float volume = 0.0f;
        foreach (Tetrahedron tet in tetrahedrons)
        {
            volume += tet.RestVolume;
        }
        return volume;
    }
    private void setupKernel()
    {
        applyExplicitEulerKernel = computeShaderobj.FindKernel("applyExplicitEulerKernel");

        floorCollisionKernel = computeShaderobj.FindKernel("floorCollisionKernel");
        //for solving all constraint at once
        //satisfyPointConstraintsKernel = computeShaderobj.FindKernel("projectConstraintDeltasKernel");
        //satisfySphereCollisionsKernel = computeShaderobj.FindKernel("projectConstraintDeltasKernel");
        //satisfyCubeCollisionsKernel = computeShaderobj.FindKernel("projectConstraintDeltasKernel");
        //for solving constrint one-by-one
        satisfyDistanceConstraintKernel = computeShaderobj.FindKernel("satisfyDistanceConstraintKernel");
        averageConstraintDeltasKernel = computeShaderobj.FindKernel("averageConstraintDeltasKernel");

        satisfyBendingConstraintKernel = computeShaderobj.FindKernel("satisfyBendingConstraintKernel");
        satisfyTetVolConstraintKernel = computeShaderobj.FindKernel("satisfyTetVolConstraintKernel");

        //update position
        updatePositionsKernel = computeShaderobj.FindKernel("updatePositionsKernel");
        //object volume
        computeObjVolumeKernel = computeShaderobj.FindKernel("computeObjVolumeKernel");
        //for rendering
        computeVerticesNormal = computeShaderobj.FindKernel("computeVerticesNormal");

    }
    private void setupComputeShader()
    {
        //send uniform data for kernels in compute shader
        computeShaderobj.SetInt("nodeCount", nodeCount);
        computeShaderobj.SetInt("springCount", springCount);
        computeShaderobj.SetInt("triCount", triCount);
        computeShaderobj.SetInt("tetCount", tetCount);
        computeShaderobj.SetInt("bendingCount", bendingCount);

        computeShaderobj.SetFloat("dt", dt);
        computeShaderobj.SetFloat("invMass", invMass);
        computeShaderobj.SetFloat("stretchStiffness", stretchStiffness);
        computeShaderobj.SetFloat("compressStiffness", compressStiffness);
        computeShaderobj.SetFloat("bendingStiffness", bendingStiffness);
        computeShaderobj.SetFloat("tetVolStiffness", volumeStiffness);
        computeShaderobj.SetFloat("convergence_factor", convergence_factor);

        computeShaderobj.SetVector("gravity", gravity);

        // bind buffer data to each kernel

        //Kernel #1 add force & apply euler
        computeShaderobj.SetBuffer(applyExplicitEulerKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(applyExplicitEulerKernel, "Positions", positionsBuffer);
        computeShaderobj.SetBuffer(applyExplicitEulerKernel, "ProjectedPositions", projectedPositionsBuffer);

        //Kernel #2
        computeShaderobj.SetBuffer(satisfyDistanceConstraintKernel, "deltaPos", deltaPositionsBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(satisfyDistanceConstraintKernel, "deltaPosAsInt", deltaPositionsUIntBuffer);   //for find the correct project position
        computeShaderobj.SetBuffer(satisfyDistanceConstraintKernel, "deltaCount", deltaCounterBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(satisfyDistanceConstraintKernel, "ProjectedPositions", projectedPositionsBuffer);
        computeShaderobj.SetBuffer(satisfyDistanceConstraintKernel, "distanceConstraints", distanceConstraintsBuffer);
        computeShaderobj.SetBuffer(satisfyDistanceConstraintKernel, "Positions", positionsBuffer);

        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "deltaPos", deltaPositionsBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "deltaPosAsInt", deltaPositionsUIntBuffer);   //for find the correct project position
        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "deltaCount", deltaCounterBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "ProjectedPositions", projectedPositionsBuffer);
        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "bendingConstraints", bendingConstraintsBuffer);
        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(satisfyBendingConstraintKernel, "Positions", positionsBuffer);

        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "deltaPos", deltaPositionsBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "deltaPosAsInt", deltaPositionsUIntBuffer);   //for find the correct project position
        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "deltaCount", deltaCounterBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "ProjectedPositions", projectedPositionsBuffer);
        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "tetVolumeConstraints", tetVolConstraintsBuffer);
        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(satisfyTetVolConstraintKernel, "Positions", positionsBuffer);

        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "deltaPos", deltaPositionsBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "deltaPosAsInt", deltaPositionsUIntBuffer);   //for find the correct project position
        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "deltaCount", deltaCounterBuffer);            //for find the correct project position
        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "ProjectedPositions", projectedPositionsBuffer);
        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "distanceConstraints", distanceConstraintsBuffer);
        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(averageConstraintDeltasKernel, "Positions", positionsBuffer);

        computeShaderobj.SetBuffer(floorCollisionKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(floorCollisionKernel, "Positions", positionsBuffer);
        computeShaderobj.SetBuffer(floorCollisionKernel, "ProjectedPositions", projectedPositionsBuffer);
        //Kernel  update position
        //computeShaderobj.SetBuffer(applyExplicitEulerKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(updatePositionsKernel, "Velocities", velocitiesBuffer);
        computeShaderobj.SetBuffer(updatePositionsKernel, "Positions", positionsBuffer);
        computeShaderobj.SetBuffer(updatePositionsKernel, "ProjectedPositions", projectedPositionsBuffer);
        computeShaderobj.SetBuffer(updatePositionsKernel, "vertsBuff", vertsBuff); //passing to rendering

        computeShaderobj.SetBuffer(computeObjVolumeKernel, "objVolume", objVolumeBuffer);
        computeShaderobj.SetBuffer(computeObjVolumeKernel, "Positions", positionsBuffer);
        computeShaderobj.SetBuffer(computeObjVolumeKernel, "tetVolumeConstraints", tetVolConstraintsBuffer);

        //kernel compute vertices normal
        computeShaderobj.SetBuffer(computeVerticesNormal, "Positions", positionsBuffer);
        computeShaderobj.SetBuffer(computeVerticesNormal, "Triangles", triangleBuffer);
        computeShaderobj.SetBuffer(computeVerticesNormal, "TrianglePtr", triangleIndicesBuffer);
        computeShaderobj.SetBuffer(computeVerticesNormal, "vertsBuff", vertsBuff); //passing to rendering
        computeShaderobj.SetBuffer(computeVerticesNormal, "objVolume", objVolumeBuffer);
    }

    void setup()
    {
        material = new Material(renderingShader); // new material for difference object
        material.color = matColor; //set color to material
        computeShaderobj = Instantiate(computeShader); // to instantiate the compute shader to be use with multiple object

        SelectModelName();
        setupMeshData();
        setupShader();
        setBuffData();
        setupComputeBuffer();
        setupKernel();
        setupComputeShader();

        totalVolume = computeObjectVolume();
    }
    void Start()
    {
        setup();
    }
    // Update is called once per frame
    void dispatchComputeShader()
    {
        ////update uniform data and GPU buffer here
        ////PBD algorithm
        computeShaderobj.Dispatch(applyExplicitEulerKernel, (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);
        ////damp velocity() here
        for (int i = 0; i < iteration; i++)
        {
            //solving constraint using avaerage jacobi style
            //convergence rate slower that Gauss–Seidel method implement on CPU method
            computeShaderobj.Dispatch(satisfyDistanceConstraintKernel, (int)Mathf.Ceil(springCount / 1024.0f), 1, 1);
            computeShaderobj.Dispatch(satisfyBendingConstraintKernel, (int)Mathf.Ceil(bendingCount / 1024.0f), 1, 1);
            computeShaderobj.Dispatch(satisfyTetVolConstraintKernel, (int)Mathf.Ceil(tetCount / 1024.0f), 1, 1);
            computeShaderobj.Dispatch(averageConstraintDeltasKernel, (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);

            computeShaderobj.SetFloat("floorCoordY", (floor.transform.position).y);
            computeShaderobj.Dispatch(floorCollisionKernel, (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);
        }
        computeShaderobj.Dispatch(updatePositionsKernel, (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);

        //compute object volume
        if (renderVolumeText)
        {
            computeShaderobj.Dispatch(computeObjVolumeKernel, (int)Mathf.Ceil(tetCount / 1024.0f), 1, 1);
            objVolumeBuffer.GetData(volumeDataGPU);
        }
        //compute normal for rendering
        computeShaderobj.Dispatch(computeVerticesNormal, (int)Mathf.Ceil(nodeCount / 1024.0f), 1, 1);
    }
    void renderObject()
    {
        Bounds bounds = new Bounds(Vector3.zero, Vector3.one * 10000);
        material.SetPass(0);
        Graphics.DrawProcedural(material, bounds, MeshTopology.Triangles, triArray.Length,
            1, null, null, ShadowCastingMode.On, true, gameObject.layer);

    }
    void Update()
    {
        
        dispatchComputeShader();
        renderObject();

        if (writeVolumeToFile)
        {
            buildDataPerRow(frame, volumeDataGPU[0]);
            if (frame == maxFrameNum - 1)
            {
                //volume file name refer to prefix of the output file or method used
                writeTableData(volumeFileName + "_" + modelName);
                print("write Done");
                UnityEditor.EditorApplication.isPlaying = false;
            }
        }
        frame++;
    }
    void buildDataPerRow(int frame_num, float currVolume)
    {
        List<string> rowData = new List<string>();
        rowData.Add(frame.ToString());

        float Vpercentage = (currVolume / totalVolume) * 100.0f;
        rowData.Add(Vpercentage.ToString());
        tableData.Add(rowData.ToArray());
    }
    void writeTableData(string fileName)
    {
        string[][] output = new string[tableData.Count][];
        for (int i = 0; i < output.Length; i++)
        {
            output[i] = tableData[i];
        }
        int length = output.GetLength(0);
        string delimiter = ",";
        StringBuilder sb = new StringBuilder();

        for (int index = 0; index < length; index++)
            sb.AppendLine(string.Join(delimiter, output[index]));
        string filePath = directory + fileName + ".csv";



        StreamWriter outStream = System.IO.File.CreateText(filePath);
        outStream.WriteLine(sb);
        outStream.Close();
        tableData.Clear();
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

            float currVolume = volumeDataGPU[0];
            //currVolume = computeSurfaceVolume();

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
    private void OnDestroy()
    {
        if (this.enabled)
        {
            vertsBuff.Dispose();
            triBuffer.Dispose();

            triangleBuffer.Dispose();
            triangleIndicesBuffer.Dispose();

            positionsBuffer.Dispose();
            velocitiesBuffer.Dispose();
            projectedPositionsBuffer.Dispose();
            deltaPositionsBuffer.Dispose();
            deltaPositionsUIntBuffer.Dispose();
            distanceConstraintsBuffer.Dispose();
            bendingConstraintsBuffer.Dispose();
            tetVolConstraintsBuffer.Dispose();
            objVolumeBuffer.Dispose();
            deltaCounterBuffer.Dispose();
        }
    }
}

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace Assets.script
{

    public class SphereCollisionConstraint
    {
        public Vector3 sphereCenter;
        public float sphereRadius;
        //public Vector3 collisionPosition;
        //public Vector3 collisionNormal;
    }
    public class CubeCollisionConstraint
    {
        public Vector3 cubeExtent;
        public Transform cubeTransform;

        public bool IsPointInCube(Vector3 point)
        {
            return Mathf.Abs(point.x) < cubeExtent.x &&
                Mathf.Abs(point.y) < cubeExtent.y &&
                Mathf.Abs(point.z) < cubeExtent.z;
        }

    }
    public class BoxCollisionConstraint
    {
        public Vector3 MinPos;
        public Vector3 MaxPos;
        public Vector3 Center;

        public bool IsCollided(Vector3 Point)
        {
            return Point.x >= MinPos.x && Point.x <= MaxPos.x &&
                Point.y >= MinPos.y && Point.y <= MaxPos.y &&
                Point.z >= MinPos.z && Point.z <= MaxPos.z;
        }
    }
}

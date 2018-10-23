using System.Collections.Generic;
using Assets.Scripts.IAJ.Unity.Utils;
using RAIN.Navigation.Graph;
using UnityEngine;
using System;

namespace Assets.Scripts.IAJ.Unity.Pathfinding.Path
{
    public class GlobalPath : Path
    {
        public List<NavigationGraphNode> PathNodes { get; protected set; }
        public List<Vector3> PathPositions { get; protected set; } 
        public bool IsPartial { get; set; }
        public float Length { get; set; }
        public List<LocalPath> LocalPaths { get; protected set; } 

        public GlobalPath()
        {
            this.PathNodes = new List<NavigationGraphNode>();
            this.PathPositions = new List<Vector3>();
            this.LocalPaths = new List<LocalPath>();
        }

        public override float GetParam(Vector3 position, float previousParam)
        {
			//TODO: implement latter
			throw new NotImplementedException();
        }

        public override Vector3 GetPosition(float param)
        {
			//TODO: implement latter
			throw new NotImplementedException();
        }

        public override bool PathEnd(float param)
        {
			//TODO: implement latter
			throw new NotImplementedException();
        }
    }
}

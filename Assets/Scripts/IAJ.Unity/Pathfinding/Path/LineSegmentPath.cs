using Assets.Scripts.IAJ.Unity.Utils;
using UnityEngine;
using System;

namespace Assets.Scripts.IAJ.Unity.Pathfinding.Path
{
    public class LineSegmentPath : LocalPath
    {
        protected Vector3 LineVector;
        public LineSegmentPath(Vector3 start, Vector3 end)
        {
            this.StartPosition = start;
            this.EndPosition = end;
            this.LineVector = end - start;
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

        public override float GetParam(Vector3 position, float lastParam)
        {
			//TODO: implement latter
			throw new NotImplementedException();
        }
    }
}

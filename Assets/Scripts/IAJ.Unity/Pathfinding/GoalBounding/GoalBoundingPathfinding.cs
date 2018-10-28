using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures.GoalBounding;
using Assets.Scripts.IAJ.Unity.Pathfinding.Heuristics;
using System.Collections.Generic;
using RAIN.Navigation.NavMesh;
using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures;
using RAIN.Navigation.Graph;
using UnityEngine;

namespace Assets.Scripts.IAJ.Unity.Pathfinding.GoalBounding
{
    public class GoalBoundingPathfinding : NodeArrayAStarPathFinding
    {
        public GoalBoundingTable GoalBoundingTable { get; protected set;}
        public int DiscardedEdges { get; protected set; }
		public int TotalEdges { get; protected set; }

        public GoalBoundingPathfinding(NavMeshPathGraph graph, IHeuristic heuristic, GoalBoundingTable goalBoundsTable) : base(graph, heuristic)
        {
            this.GoalBoundingTable = goalBoundsTable;
        }

        public override void InitializePathfindingSearch(Vector3 startPosition, Vector3 goalPosition)
        {
            this.DiscardedEdges = 0;
			this.TotalEdges = 0;
            base.InitializePathfindingSearch(startPosition, goalPosition);
        }

        protected override void ProcessChildNode(NodeRecord parentNode, NavigationGraphEdge connectionEdge, int edgeIndex)
        {
            int ind = parentNode.node.NodeIndex;
            NodeGoalBounds goalBounds = this.GoalBoundingTable.table[parentNode.node.NodeIndex];
            if (goalBounds != null)
            {
                if (!goalBounds.connectionBounds[GetGoalBoundingBoxIndex(goalBounds.connectionBounds)].PositionInsideBounds(connectionEdge.ToNode.Position)) return;
            }

            base.ProcessChildNode(parentNode, connectionEdge, edgeIndex);
        }

        protected int GetGoalBoundingBoxIndex(IAJ.Unity.Pathfinding.DataStructures.GoalBounding.Bounds[] bounds)
        {
            List<int> goalIndices = new List<int>();

            for (int i = 0; i < bounds.Length; i++)
            {
                if (IsInBounds(this.GoalPosition, bounds[i])) goalIndices.Add(i);
            }

            if (goalIndices.Count == 1) return goalIndices[0];
            else
            {
                float bestArea = -1f;
                for (int j = 0; j < goalIndices.Count; j++)
                {
                    IAJ.Unity.Pathfinding.DataStructures.GoalBounding.Bounds boundingBox = bounds[goalIndices[j]];
                    float boxArea = (boundingBox.maxx - boundingBox.minx) * (boundingBox.maxz - boundingBox.minz);
                    if (bestArea < 0f || boxArea < bestArea)
                    {
                        bestArea = boxArea;
                        return goalIndices[j];
                    }
                }
            }
            return -1;
        }

        protected bool IsInBounds(Vector3 position, IAJ.Unity.Pathfinding.DataStructures.GoalBounding.Bounds bounds)
        {
            return (position.x <= bounds.maxx && position.x >= bounds.minx &&
                    position.z <= bounds.maxz && position.z >= bounds.minz);
        }
    }
}

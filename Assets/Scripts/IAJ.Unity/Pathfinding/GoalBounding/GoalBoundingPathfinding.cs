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
        private int GoalIndex;
        private NodeGoalBounds searchBounds;

        public GoalBoundingPathfinding(NavMeshPathGraph graph, IHeuristic heuristic, GoalBoundingTable goalBoundsTable) : base(graph, heuristic)
        {
            this.GoalBoundingTable = goalBoundsTable;
        }

        public override void InitializePathfindingSearch(Vector3 startPosition, Vector3 goalPosition)
        {
            this.DiscardedEdges = 0;
			this.TotalEdges = 0;
            List<int> goalIndices = new List<int>();

            this.searchBounds = this.GoalBoundingTable.table[this.Quantize(startPosition).NodeIndex];

            for (int i = 0; i < this.searchBounds.connectionBounds.Length; i++)
            {
                if (IsInBounds(goalPosition, this.searchBounds.connectionBounds[i])) goalIndices.Add(i);
            }

            if (goalIndices.Count == 1) this.GoalIndex = goalIndices[0];
            else
            {
                float bestArea = -1f;
                for (int j = 0; j < goalIndices.Count; j++)
                {
                    IAJ.Unity.Pathfinding.DataStructures.GoalBounding.Bounds boundingBox = this.searchBounds.connectionBounds[goalIndices[j]];
                    float boxArea = (boundingBox.maxx - boundingBox.minx) * (boundingBox.maxz - boundingBox.minz);
                    if (bestArea < 0f || boxArea < bestArea)
                    {
                        bestArea = boxArea;
                        this.GoalIndex = goalIndices[j];
                    }
                }
            }
            base.InitializePathfindingSearch(startPosition, goalPosition);
        }

        protected override void ProcessChildNode(NodeRecord parentNode, NavigationGraphEdge connectionEdge, int edgeIndex)
        {
            if (this.searchBounds != null)
            {
                if (!IsOverlappingGoalBox(parentNode.node.Position)) return;
            }
            else
            {
                this.searchBounds = this.GoalBoundingTable.table[connectionEdge.ToNode.NodeIndex];
            }

            base.ProcessChildNode(parentNode, connectionEdge, edgeIndex);
        }

        protected bool IsOverlappingGoalBox(Vector3 position)
        {
            return IsInBounds(position, this.searchBounds.connectionBounds[this.GoalIndex]);
        }

        protected bool IsInBounds(Vector3 position, IAJ.Unity.Pathfinding.DataStructures.GoalBounding.Bounds bounds)
        {
            return (position.x <= bounds.maxx && position.x >= bounds.minx &&
                    position.z <= bounds.maxz && position.z >= bounds.minz);
        }
    }
}

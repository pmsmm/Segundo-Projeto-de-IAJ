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
        private IAJ.Unity.Pathfinding.DataStructures.GoalBounding.Bounds[] searchBounds;

        public GoalBoundingPathfinding(NavMeshPathGraph graph, IHeuristic heuristic, GoalBoundingTable goalBoundsTable) : base(graph, heuristic)
        {
            this.GoalBoundingTable = goalBoundsTable;
        }

        public override void InitializePathfindingSearch(Vector3 startPosition, Vector3 goalPosition)
        {
            this.DiscardedEdges = 0;
			this.TotalEdges = 0;

            int quantized = this.Quantize(startPosition).NodeIndex;
            NodeGoalBounds goalBounds = this.GoalBoundingTable.table[0];
            searchBounds = goalBounds.connectionBounds;
            List<int> goalIndices = new List<int>();
            for (int i = 0; i < searchBounds.Length; i++)
            {
                if (IsInBounds(goalPosition, searchBounds[i])) goalIndices.Add(i);
            }

            if (goalIndices.Count == 1) this.GoalIndex = goalIndices[0];
            else
            {
                float bestArea = -1f;
                for (int j = 0; j < goalIndices.Count; j++)
                {
                    IAJ.Unity.Pathfinding.DataStructures.GoalBounding.Bounds boundingBox = searchBounds[goalIndices[j]];
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
            if (!IsOverlappingGoalBox(parentNode.node.Position)) return;

            //float f;
            //float g;
            //float h;

            var childNode = connectionEdge.ToNode;
            var childNodeRecord = this.NodeRecordArray.GetNodeRecord(childNode);

            if (childNodeRecord == null)
            {
                //this piece of code is used just because of the special start nodes and goal nodes added to the RAIN Navigation graph when a new search is performed.
                //Since these special goals were not in the original navigation graph, they will not be stored in the NodeRecordArray and we will have to add them
                //to a special structure
                //it's ok if you don't understand this, this is a hack and not part of the NodeArrayA* algorithm, just do NOT CHANGE THIS, or your algorithm will not work
                childNodeRecord = new NodeRecord
                {
                    node = childNode,
                    parent = parentNode,
                    status = NodeStatus.Unvisited
                };
                this.NodeRecordArray.AddSpecialCaseNode(childNodeRecord);
            }

            //TODO: Review
            childNodeRecord.gValue = childNodeRecord.gValue != 0f ? childNodeRecord.gValue : parentNode.gValue + (childNode.LocalPosition - parentNode.node.LocalPosition).magnitude;
            childNodeRecord.hValue = childNodeRecord.hValue != 0f ? childNodeRecord.hValue : this.Heuristic.H(childNode, this.GoalNode);
            childNodeRecord.fValue = childNodeRecord.fValue != 0f ? childNodeRecord.fValue : F(childNodeRecord);

            switch (childNodeRecord.status)
            {
                case NodeStatus.Unvisited:
                    this.Open.AddToOpen(childNodeRecord);
                    childNodeRecord.status = NodeStatus.Open;
                    break;
                case NodeStatus.Open:
                    if (childNodeRecord.fValue > parentNode.fValue) this.Open.Replace(parentNode, childNodeRecord);
                    break;
                case NodeStatus.Closed:
                    if (childNodeRecord.fValue > parentNode.fValue)
                    {
                        childNodeRecord.status = NodeStatus.Open;
                        this.Open.AddToOpen(childNodeRecord);
                    }
                    break;
            }
        }

        protected bool IsOverlappingGoalBox(Vector3 position)
        {
            List<int> boxIndices = new List<int>();
            for (int i = 0; i < this.searchBounds.Length; i++)
            {
                if (IsInBounds(position, this.searchBounds[i])) boxIndices.Add(i);
            }

            return boxIndices.Contains(this.GoalIndex);
        }

        protected bool IsInBounds(Vector3 position, IAJ.Unity.Pathfinding.DataStructures.GoalBounding.Bounds bounds)
        {
            return (position.x <= bounds.maxx && position.x >= bounds.minx &&
                    position.z <= bounds.maxz && position.z >= bounds.minz);
        }
    }
}

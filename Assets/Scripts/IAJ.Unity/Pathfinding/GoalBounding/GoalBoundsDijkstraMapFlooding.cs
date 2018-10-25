using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures;
using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures.GoalBounding;
using RAIN.Navigation.Graph;
using RAIN.Navigation.NavMesh;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Assets.Scripts.IAJ.Unity.Pathfinding.GoalBounding
{
    //The Dijkstra algorithm is similar to the A* but with a couple of differences
    //1) no heuristic function
    //2) it will not stop until the open list is empty
    //3) we dont need to execute the algorithm in multiple steps (because it will be executed offline)
    //4) we don't need to return any path (partial or complete)
    //5) we don't need to do anything when a node is already in closed
    public class GoalBoundsDijkstraMapFlooding
    {
        public NavMeshPathGraph NavMeshGraph { get; protected set; }
        public NavigationGraphNode StartNode { get; protected set; }
        public NodeGoalBounds NodeGoalBounds { get; protected set; }
        protected NodeRecordArray NodeRecordArray { get; set; }

        public IOpenSet Open { get; protected set; }
        public IClosedSet Closed { get; protected set; }
        
        public GoalBoundsDijkstraMapFlooding(NavMeshPathGraph graph)
        {
            this.NavMeshGraph = graph;
            //do not change this
            var nodes = this.GetNodesHack(graph);
            this.NodeRecordArray = new NodeRecordArray(nodes);
            this.Open = this.NodeRecordArray;
            this.Closed = this.NodeRecordArray;
        }

        public void Search(NavigationGraphNode startNode, NodeGoalBounds nodeGoalBounds)
        {
            Debug.Log("Initiated search");
            this.NodeGoalBounds = nodeGoalBounds;

            NodeRecord startNodeRecord = this.NodeRecordArray.GetNodeRecord(startNode);
            this.Open.AddToOpen(startNodeRecord);

            for (int j = 0; j < startNode.OutEdgeCount; j++) this.NodeRecordArray.GetNodeRecord(startNode.EdgeOut(j).ToNode).id = j;

            while (this.Open.CountOpen() > 0)
            {
                NodeRecord Node = this.Open.GetBestAndRemove();
                this.Closed.AddToClosed(Node);
                if (Node.id != -1) UpdateBoundingBox(Node.id, Node.node.Position);

                for (int i = 0; i < Node.node.OutEdgeCount; i++)
                {
                    ProcessChildNode(Node, Node.node.EdgeOut(i), i);
                }
            }

            List<NodeRecord> closed = this.Closed.All().ToList<NodeRecord>();
            for (int j = 0; j < closed.Count; j++)
            {
                closed[j].status = NodeStatus.Unvisited;
            }
            Debug.Log("Concluded search");
        }


        protected void ProcessChildNode(NodeRecord parent, NavigationGraphEdge connectionEdge, int connectionIndex)
        {
            NodeRecord node = this.NodeRecordArray.GetNodeRecord(connectionEdge.ToNode);

            float g = parent.gValue + (node.node.LocalPosition - parent.node.LocalPosition).magnitude;

            switch (node.status)
            {
                case NodeStatus.Unvisited:
                    UpdateNodeRecord(node, parent, g);
                    this.Open.AddToOpen(node);
                    node.status = NodeStatus.Open;
                    break;
                case NodeStatus.Open:
                    if (node.gValue > g)
                    {
                        UpdateNodeRecord(node, parent, g);
                        this.Open.Replace(parent, node);
                    }
                    break;
            }
        }

        protected void UpdateBoundingBox(int index, Vector3 position)
        {
            Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures.GoalBounding.Bounds bounds = this.NodeGoalBounds.connectionBounds[index];

            bounds.maxx = (position.x > bounds.maxx) ? position.x : bounds.maxx + 1f;
            bounds.minx = (position.x < bounds.minx) ? position.x : bounds.minx - 1f;
            bounds.maxz = (position.z > bounds.maxz) ? position.z : bounds.maxz + 1f;
            bounds.minz = (position.z < bounds.minz) ? position.z : bounds.minz - 1f;
        }

        protected void UpdateNodeRecord(NodeRecord node, NodeRecord parent, float g)
        {
            node.gValue = g;
            node.parent = parent;
            if (parent.id != -1) node.id = parent.id;
        }

        private List<NavigationGraphNode> GetNodesHack(NavMeshPathGraph graph)
        {
            //this hack is needed because in order to implement NodeArrayA* you need to have full acess to all the nodes in the navigation graph in the beginning of the search
            //unfortunately in RAINNavigationGraph class the field which contains the full List of Nodes is private
            //I cannot change the field to public, however there is a trick in C#. If you know the name of the field, you can access it using reflection (even if it is private)
            //using reflection is not very efficient, but it is ok because this is only called once in the creation of the class
            //by the way, NavMeshPathGraph is a derived class from RAINNavigationGraph class and the _pathNodes field is defined in the base class,
            //that's why we're using the type of the base class in the reflection call
            return (List<NavigationGraphNode>)Utils.Reflection.GetInstanceField(typeof(RAINNavigationGraph), graph, "_pathNodes");
        }

    }
}

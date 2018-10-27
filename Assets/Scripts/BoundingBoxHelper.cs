using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;
using Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures.GoalBounding;

namespace Assets.Scripts.IAJ.Unity.Pathfinding.GoalBounding {
    public class BoundingBoxHelper : MonoBehaviour {

        private int boundingBoxIndex = 0;
        private NodeGoalBounds[] table = null;

        // Use this for initialization
        void Start() {
            table = (AssetDatabase.LoadAssetAtPath<Assets.Scripts.IAJ.Unity.Pathfinding.DataStructures.GoalBounding.GoalBoundingTable>("Assets/Resources/GoalBoundingTable.asset")).table;
            IncrementIndex();
        }

        // Update is called once per frame
        void Update() {
            if (Input.GetKeyDown(KeyCode.P)) IncrementIndex();
            if (Input.GetKeyDown(KeyCode.O)) DecrementIndex();

            if (Input.GetKey(KeyCode.RightArrow)) IncrementIndex();
            if (Input.GetKey(KeyCode.LeftArrow)) DecrementIndex();

            DrawBoxes();
        }

        private void DrawBoxes()
        {
            IAJ.Unity.Pathfinding.DataStructures.GoalBounding.Bounds[] boxBounds = table[boundingBoxIndex].connectionBounds;
            foreach (IAJ.Unity.Pathfinding.DataStructures.GoalBounding.Bounds bound in boxBounds)
            {
                Debug.DrawLine(new Vector3(bound.maxx, 0, bound.maxz), new Vector3(bound.minx, 0, bound.maxz), Color.red);
                Debug.DrawLine(new Vector3(bound.maxx, 0, bound.maxz), new Vector3(bound.maxx, 0, bound.minz), Color.red);
                Debug.DrawLine(new Vector3(bound.minx, 0, bound.minz), new Vector3(bound.minx, 0, bound.maxz), Color.red);
                Debug.DrawLine(new Vector3(bound.minx, 0, bound.minz), new Vector3(bound.maxx, 0, bound.minz), Color.red);
            }
        }

        private void IncrementIndex()
        {
            boundingBoxIndex++;
            if (boundingBoxIndex >= table.Length) boundingBoxIndex = 0;
            else if (table[boundingBoxIndex] == null) IncrementIndex();
        }

        private void DecrementIndex()
        {
            boundingBoxIndex--;
            if (boundingBoxIndex < 0) boundingBoxIndex = table.Length - 1;
            else if (table[boundingBoxIndex] == null) DecrementIndex();
        }
    }
}
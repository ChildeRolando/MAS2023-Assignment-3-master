using Drone;
using System.Linq;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using System; // Add this at the top of your script



namespace UnityStandardAssets.Vehicles.Car
{
    public class DroneAgent : Agent
    {
        [HideInInspector]
        public Team team;
        [HideInInspector]
        public Position position;
        float m_Existential;
        float m_SmallReward = 0.001f;
        float m_ControllThreshold = 5f;
        PD controller;

        bool m_HasBall = false;
        bool m_HasBallEnemy = false;
        Vector3 LastBallPosition = Vector3.zero;
        Vector3[] LastFriendPosition = new Vector3[3];
        Vector3[] LastEnemyPosition = new Vector3[3];
        Vector3 LastMyPosition = Vector3.zero;

        int m_Score = 0;
        int m_ScoreEnemy = 0;
        int m_LastScore = 0;
        int m_LastScoreEnemy = 0;
        float m_ScoreReward = 10f;

        public GlobalVariables globalVariables;

        protected override void Awake()
        {
            base.Awake();
            // if Multiple instances of DroneAgent found
            int nums = FindObjectsOfType<DroneManagerSoccer>().Length;
            if (nums > 1)
            {
                Debug.Log( nums + " instances of DroneManagerSoccer found.");
            }
            
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            // Implement your heuristic logic here
            // This will be called when the Agent is set to Heuristic Only
            // The actionsOut parameter will be populated with the actions that the Agent will perform

            // --- INPUT VECTOR of length 8---
            /* globalVariables.moveDirection = new Vector3(Input.GetAxis("Horizontal"), 0f, Input.GetAxis("Vertical")).normalized;
            globalVariables.kickDirection = new Vector3(Input.GetAxis("Horizontal2"), 0f, Input.GetAxis("Vertical2")).normalized;
            globalVariables.wantToKick = Input.GetKey(KeyCode.Space);
            globalVariables.kickSpeed = 1f; */
            // ------------------------------------
        }

        public override void OnEpisodeBegin()
        {
            // Reset the environment
            controller = new PD(globalVariables.manager.drone.transform);
            team = globalVariables.manager.CompareTag("Blue") ? Team.Blue : Team.Red;
            // random position
            position = (Position)UnityEngine.Random.Range(0, 3);
            m_Existential = 1f / MaxStep;

            LastBallPosition = globalVariables.manager.soccer.ball.transform.position;
            foreach (GameObject friend in globalVariables.manager.soccer.friends)
            {
                int index = Array.IndexOf(globalVariables.manager.soccer.friends, friend);
                LastFriendPosition[index] = friend.transform.position;
            }
            for (int i = 0; i < globalVariables.manager.soccer.enemies.Length; i++)
            {
                LastEnemyPosition[i] = globalVariables.manager.soccer.enemies[i].transform.position;
            }
            LastMyPosition = globalVariables.manager.drone.transform.position;

        }

        public override void CollectObservations(VectorSensor sensor)
        {
            // Collect observations
            var pos_ball = globalVariables.manager.soccer.ball.transform.position;
            var vel_ball = pos_ball - LastBallPosition;
            var goal_own = globalVariables.manager.soccer.own_goal.transform.position;
            var goal_other = globalVariables.manager.soccer.other_goal.transform.position;
            var pos_self = globalVariables.manager.drone.transform.position;
            var vel_self = pos_self - LastMyPosition;


            foreach (GameObject friend in globalVariables.manager.soccer.friends)
            {
                // judge if the friend is agent itself
                if (friend.transform == globalVariables.manager.drone.transform)
                {
                    continue;
                }
                int index = Array.IndexOf(globalVariables.manager.soccer.friends, friend);
                Vector3 vel_friend = friend.transform.position - LastFriendPosition[index];
                sensor.AddObservation(getXZ(friend.transform.position-pos_self));
                sensor.AddObservation(getXZ(vel_friend));
            }


            foreach (GameObject enemy in globalVariables.manager.soccer.enemies)
            {
                int index = Array.IndexOf(globalVariables.manager.soccer.enemies, enemy);
                Vector3 vel_enemy = enemy.transform.position - LastEnemyPosition[index];
                sensor.AddObservation(getXZ(enemy.transform.position-pos_self));
                sensor.AddObservation(getXZ(vel_enemy));
            }
            bool can_kick = globalVariables.manager.soccer.CanKick();
            sensor.AddObservation(getXZ(pos_ball-pos_self));
            sensor.AddObservation(getXZ(vel_ball));
            sensor.AddObservation(getXZ(pos_self));
            sensor.AddObservation(getXZ(vel_self));
            sensor.AddObservation(getXZ(goal_other-pos_self));
            sensor.AddObservation(getXZ(goal_own-pos_self));
            sensor.AddObservation(can_kick);

            updateLastPosition();

            float[] getXZ(Vector3 v)
            {
                return new float[] { v.x, v.z };
            }
            void updateLastPosition()
            {
                LastBallPosition = globalVariables.manager.soccer.ball.transform.position;
                foreach (GameObject friend in globalVariables.manager.soccer.friends)
                {
                    int index = Array.IndexOf(globalVariables.manager.soccer.friends, friend);
                    LastFriendPosition[index] = friend.transform.position;
                }
                for (int i = 0; i < 3; i++)
                {
                    LastEnemyPosition[i] = globalVariables.manager.soccer.enemies[i].transform.position;
                }
                LastMyPosition = globalVariables.manager.drone.transform.position;
            }
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            // --- INPUT VECTOR of length 6---
            /* Vector3 targetPos = new Vector3(actions.ContinuousActions[0], 0f, actions.ContinuousActions[1]).normalized-globalVariables.manager.drone.transform.position;
            // clamp the target position to the circle with radius 1
            targetPos = Vector3.ClampMagnitude(targetPos, 10f);
            Vector3 vel_self = globalVariables.manager.drone.transform.position - LastMyPosition; */
            globalVariables.moveDirection = new Vector3(actions.ContinuousActions[0], 0f, actions.ContinuousActions[1]).normalized;
            globalVariables.kickDirection = new Vector3(actions.ContinuousActions[2], 0f, actions.ContinuousActions[3]).normalized;

            if (actions.DiscreteActions[0] == 0) 
            {
                globalVariables.wantToKick = false;
            } else {
                globalVariables.wantToKick = true;
            }

            globalVariables.kickSpeed = Mathf.Clamp(actions.ContinuousActions[4], 0f, 1f);

            // ------------------------------------
            
            
            ComputeRewardGeneral();
        }
        
        private void ComputeRewardGeneral()
        {
            GoBehindBallReward();
            //BallPositionReward();
            PassingReward();
            ControllReward();
            ShootingReward();
            ScoreReward();
            CloseToBallReward();
        }

        private void CloseToBallReward()
        {
            // if the agent is close to the ball, give a reward
            float closeToBallParameter = 100f;
            float distance = Vector3.Distance(globalVariables.manager.drone.transform.position, globalVariables.manager.soccer.ball.transform.position);
            float reward = 1f/distance;
            if(position == Position.Goalie)
            {
                closeToBallParameter = 0.5f;
            }
            AddReward(reward*m_SmallReward*closeToBallParameter);
          
        }
        
        private void GoBehindBallReward()
        {
            // if the agent is behind the ball, give a reward
            float goBehindBallParameter = 10f;
            float self_goal_z = globalVariables.manager.soccer.own_goal.transform.position.z;
            float ball_z = globalVariables.manager.soccer.ball.transform.position.z;
            float self_z = globalVariables.manager.drone.transform.position.z;
            float distance_selfGoal_self = Mathf.Abs(self_goal_z - self_z);
            float distance_selfGoal_ball = Mathf.Abs(self_goal_z - ball_z);
            if(distance_selfGoal_self < distance_selfGoal_ball)
            {
                AddReward(m_SmallReward*goBehindBallParameter);
            }
        }

        private void PassingReward()
        {
            float PassingParameter = 50f;
            // kick result decided by the sum of the ball's velocity and the kick direction
            // calculate the kick result
            GameObject[] friends = globalVariables.manager.soccer.friends;
            int friendNum = friends.Length;
            Vector3 kickResult = globalVariables.manager.soccer.ball.GetComponent<Rigidbody>().velocity + globalVariables.kickDirection;
            // calculate the reletive position of the ball and friends
            Vector3[] reletivePosition = new Vector3[friendNum-1];
            int j=0;
            for (int i = 0; i < friendNum; i++)
            {
                if (friends[i].transform == globalVariables.manager.drone.transform)
                {
                    continue;
                }
                Vector3 friendPosition = friends[i].transform.position;
                Vector3 ballPosition = globalVariables.manager.soccer.ball.transform.position;
                reletivePosition[j] = friendPosition - ballPosition;
                j++;
            }
            // calculate the angle between the kick result and the reletive position
            float[] angle = new float[reletivePosition.Length];
            for (int i = 0; i < reletivePosition.Length; i++)
            {
                angle[i] = Vector3.Angle(kickResult, reletivePosition[i]);
            }
            // find the smallest angle using the min function
            float minAngle = angle.Min();
            // smaller the angle, bigger the reward using the inverse function
            float reward = 1f / minAngle;
            // clamp the reward
            reward = Mathf.Clamp(reward, 0f, 1f)*m_SmallReward*PassingParameter;
            AddReward(reward);
        }

        private void ShootingReward()
        {
            // if the agent is shooting to the goal, give a reward
            float shootingParameter = 100f;
            Vector3 kickResult = globalVariables.manager.soccer.ball.GetComponent<Rigidbody>().velocity + globalVariables.kickDirection;
            Vector3 goalDirection = globalVariables.manager.soccer.other_goal.transform.position - globalVariables.manager.soccer.ball.transform.position;
            float angle = Vector3.Angle(kickResult, goalDirection);
            float reward = 1f / angle;
            reward = Mathf.Clamp(reward, 0f, 1f)*m_SmallReward*shootingParameter;
            AddReward(reward);
        }

        private void ControllReward()
        {
            float controllParameter = 10f;
            if (IsControllingBall()){
                AddReward(m_SmallReward*controllParameter);
            } else if (m_HasBallEnemy){
                AddReward(-m_SmallReward*controllParameter);
            }
        }

        private void ScoreReward()
        {
            m_LastScore = m_Score;
            m_LastScoreEnemy = m_ScoreEnemy;
            GameManagerSoccer gameManager = GameObject.FindObjectOfType<GameManagerSoccer>();
            m_Score = gameManager.blue_score;   
            m_ScoreEnemy = gameManager.red_score;
            if (m_Score > m_LastScore){
                AddReward(m_ScoreReward);
            } else if (m_ScoreEnemy > m_LastScoreEnemy){
                AddReward(-m_ScoreReward);
            }
        }

        private void BallPositionReward()
        {
            // if the ball is in the other half, give a reward to the striker and generic
            if (position == Position.Striker || position == Position.Generic && InOtherHalf(globalVariables.manager.soccer.ball.transform.position.z))
            {
                AddReward(m_SmallReward);
            }

            // if the ball is in the own half, give a punishment to the goalie and generic
            if (position == Position.Goalie || position == Position.Generic && !InOtherHalf(globalVariables.manager.soccer.ball.transform.position.z))
            {
                AddReward(-m_SmallReward);
            }
        }

        private void PlayerPositionReward()
        {
            // if the player is in the other half, give a reward to the striker
            if (position == Position.Striker && InOtherHalf(globalVariables.manager.drone.transform.position.z))
            {
                AddReward(m_SmallReward);
            }

            // if the player is in the own half, give a reward to the goalie
            if (position == Position.Goalie && !InOtherHalf(globalVariables.manager.drone.transform.position.z))
            {
                AddReward(m_SmallReward);
            }
        }

        private bool InOtherHalf(float position_z)
        {
            float self_goal_z = globalVariables.manager.soccer.own_goal.transform.position.z;
            float other_goal_z = globalVariables.manager.soccer.other_goal.transform.position.z;
            bool in_other_half = Mathf.Abs(self_goal_z - position_z) > Mathf.Abs(other_goal_z - position_z);

            return in_other_half;
        }

        private bool IsControllingBall()
        {
            // compute the distance between the ball and all the agents, 
            // and check if the closest one is a friend agent 
            // and check if the distance is less than a threshold
            float minDist = float.MaxValue;
            GameObject closestAgent = null;
            List<GameObject> allAgents = new List<GameObject>(globalVariables.manager.soccer.friends);
            allAgents.AddRange(globalVariables.manager.soccer.enemies);

            foreach (GameObject agent in allAgents)
            {
                float dist = Vector3.Distance(agent.transform.position, globalVariables.manager.soccer.ball.transform.position);
                if (dist < minDist)
                {
                    minDist = dist;
                    closestAgent = agent;
                }
            }

            if (closestAgent == null)
            {
                m_HasBall = false;
                m_HasBallEnemy = false;
                return false;
            }

            if ( globalVariables.manager.soccer.friends.Contains(closestAgent) && minDist < m_ControllThreshold )
            {
                m_HasBall = true;
                m_HasBallEnemy = false;
                return true;
            }
            else
            {
                m_HasBall = false;
                m_HasBallEnemy = true;
                return false;
            }
        }

    }
}
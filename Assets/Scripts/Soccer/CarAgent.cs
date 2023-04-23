using Drone;
using System.Linq;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;


namespace UnityStandardAssets.Vehicles.Car
{
    public class CarAgent : Agent
    {
        [HideInInspector]
        public Team team;
        [HideInInspector]
        public Position position;
        float m_Existential;
        float m_SmallReward = 0.001f;
        float m_ControllThreshold = 5f;

        float m_LastBallDistance = 0f;
        float m_CurrentBallDistance = 0f;
        bool m_HasBall = false;
        bool m_HasBallEnemy = false;
        Vector3 m_LastBallPosition = Vector3.zero;

        int m_Score = 0;
        int m_ScoreEnemy = 0;
        int m_LastScore = 0;
        int m_LastScoreEnemy = 0;
        float m_ScoreReward = 10f;

        public GlobalVariablesCar GlobalVariablesCar;

        protected override void Awake()
        {
            base.Awake();
            // if Multiple instances of CarAgent found
            int nums = FindObjectsOfType<CarManagerSoccer>().Length;
            if (nums > 1)
            {
                Debug.Log( nums + " instances of CarManagerSoccer found.");
            }
            
        }

        public override void Heuristic(in ActionBuffers actionsOut)
        {
            // Implement your heuristic logic here
            // This will be called when the Agent is set to Heuristic Only
            // The actionsOut parameter will be populated with the actions that the Agent will perform

            // --- INPUT VECTOR of length 8---
            /* GlobalVariablesCar.moveDirection = new Vector3(Input.GetAxis("Horizontal"), 0f, Input.GetAxis("Vertical")).normalized;
            GlobalVariablesCar.kickDirection = new Vector3(Input.GetAxis("Horizontal2"), 0f, Input.GetAxis("Vertical2")).normalized;
            GlobalVariablesCar.wantToKick = Input.GetKey(KeyCode.Space);
            GlobalVariablesCar.kickSpeed = 1f; */
            // ------------------------------------
        }

        public override void OnEpisodeBegin()
        {
            // Reset the environment
            team = GlobalVariablesCar.manager.CompareTag("Blue") ? Team.Blue : Team.Red;
            // random position
            position = (Position)Random.Range(0, 3);
            m_Existential = 1f / MaxStep;
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            // Collect observations
            var transform_ball = GlobalVariablesCar.manager.soccer.ball.transform;
            var transform_self = GlobalVariablesCar.manager.car.transform;
            var transform_goal_other = GlobalVariablesCar.manager.soccer.other_goal.transform;
            var transform_goal_own = GlobalVariablesCar.manager.soccer.own_goal.transform;

            var ridgidbody_ball = GlobalVariablesCar.manager.soccer.ball.GetComponent<Rigidbody>();
            var ridgidbody_self = GlobalVariablesCar.manager.car.GetComponent<Rigidbody>();

            var pos_self = transform_self.position;
            var vel_self = ridgidbody_self.velocity;

            foreach (GameObject friend in GlobalVariablesCar.manager.soccer.friends)
            {
                // judge if the friend is agent itself
                if (friend.transform == GlobalVariablesCar.manager.car.transform)
                {
                    continue;
                }
                var transform_friend = friend.transform;
                var ridgidbody_friend = friend.GetComponent<Rigidbody>();
                sensor.AddObservation(getXZ(transform_friend.position-pos_self));
                sensor.AddObservation(getXZ(ridgidbody_friend.velocity-vel_self));
            }

            foreach (GameObject enemy in GlobalVariablesCar.manager.soccer.enemies)
            {
                var transform_enemy = enemy.transform;
                var ridgidbody_enemy = enemy.GetComponent<Rigidbody>();
                sensor.AddObservation(getXZ(transform_enemy.position-pos_self));
                sensor.AddObservation(getXZ(ridgidbody_enemy.velocity-vel_self));
            }
            bool can_kick = GlobalVariablesCar.manager.soccer.CanKick();
            sensor.AddObservation(getXZ(transform_ball.position-pos_self));
            sensor.AddObservation(getXZ(ridgidbody_ball.velocity-vel_self));
            sensor.AddObservation(getXZ(transform_self.position-pos_self));
            sensor.AddObservation(getXZ(ridgidbody_self.velocity-vel_self));
            sensor.AddObservation(getXZ(transform_goal_other.position-pos_self));
            sensor.AddObservation(getXZ(transform_goal_own.position-pos_self));
            sensor.AddObservation(can_kick);

            float[] getXZ(Vector3 v)
            {
                return new float[] { v.x, v.z };
            }
        }

        public override void OnActionReceived(ActionBuffers actions)
        {
            // --- INPUT VECTOR of length 6---
            GlobalVariablesCar.steering = actions.ContinuousActions[0];
            GlobalVariablesCar.acceleration = actions.ContinuousActions[1];
            GlobalVariablesCar.kickDirection = new Vector3(actions.ContinuousActions[2], 0f, actions.ContinuousActions[3]).normalized;

            if (actions.DiscreteActions[0] == 0) 
            {
                GlobalVariablesCar.wantToKick = false;
            } else {
                GlobalVariablesCar.wantToKick = true;
            }

            GlobalVariablesCar.kickSpeed = Mathf.Clamp(actions.ContinuousActions[4], 0f, 1f);

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
            float distance = Vector3.Distance(GlobalVariablesCar.manager.car.transform.position, GlobalVariablesCar.manager.soccer.ball.transform.position);
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
            float self_goal_z = GlobalVariablesCar.manager.soccer.own_goal.transform.position.z;
            float ball_z = GlobalVariablesCar.manager.soccer.ball.transform.position.z;
            float self_z = GlobalVariablesCar.manager.car.transform.position.z;
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
            GameObject[] friends = GlobalVariablesCar.manager.soccer.friends;
            int friendNum = friends.Length;
            Vector3 kickResult = GlobalVariablesCar.manager.soccer.ball.GetComponent<Rigidbody>().velocity + GlobalVariablesCar.kickDirection;
            // calculate the reletive position of the ball and friends
            Vector3[] reletivePosition = new Vector3[friendNum-1];
            int j=0;
            for (int i = 0; i < friendNum; i++)
            {
                if (friends[i].transform == GlobalVariablesCar.manager.car.transform)
                {
                    continue;
                }
                Vector3 friendPosition = friends[i].transform.position;
                Vector3 ballPosition = GlobalVariablesCar.manager.soccer.ball.transform.position;
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
            Vector3 kickResult = GlobalVariablesCar.manager.soccer.ball.GetComponent<Rigidbody>().velocity + GlobalVariablesCar.kickDirection;
            Vector3 goalDirection = GlobalVariablesCar.manager.soccer.other_goal.transform.position - GlobalVariablesCar.manager.soccer.ball.transform.position;
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
            if (position == Position.Striker || position == Position.Generic && InOtherHalf(GlobalVariablesCar.manager.soccer.ball.transform.position.z))
            {
                AddReward(m_SmallReward);
            }

            // if the ball is in the own half, give a punishment to the goalie and generic
            if (position == Position.Goalie || position == Position.Generic && !InOtherHalf(GlobalVariablesCar.manager.soccer.ball.transform.position.z))
            {
                AddReward(-m_SmallReward);
            }
        }

        private void PlayerPositionReward()
        {
            // if the player is in the other half, give a reward to the striker
            if (position == Position.Striker && InOtherHalf(GlobalVariablesCar.manager.car.transform.position.z))
            {
                AddReward(m_SmallReward);
            }

            // if the player is in the own half, give a reward to the goalie
            if (position == Position.Goalie && !InOtherHalf(GlobalVariablesCar.manager.car.transform.position.z))
            {
                AddReward(m_SmallReward);
            }
        }

        private bool InOtherHalf(float position_z)
        {
            float self_goal_z = GlobalVariablesCar.manager.soccer.own_goal.transform.position.z;
            float other_goal_z = GlobalVariablesCar.manager.soccer.other_goal.transform.position.z;
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
            List<GameObject> allAgents = new List<GameObject>(GlobalVariablesCar.manager.soccer.friends);
            allAgents.AddRange(GlobalVariablesCar.manager.soccer.enemies);

            foreach (GameObject agent in allAgents)
            {
                float dist = Vector3.Distance(agent.transform.position, GlobalVariablesCar.manager.soccer.ball.transform.position);
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

            if ( GlobalVariablesCar.manager.soccer.friends.Contains(closestAgent) && minDist < m_ControllThreshold )
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
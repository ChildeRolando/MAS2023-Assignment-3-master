using Drone;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

namespace UnityStandardAssets.Vehicles.Car
{
    
    public enum Position
    {
        Striker,
        Goalie,
        Generic
    }

    public enum Team
    {
        Blue,
        Red
    }
    
    public class GlobalVariables
    {
        public DroneManagerSoccer manager;
        public Vector3 moveDirection = Vector3.zero;
        public Vector3 kickDirection = Vector3.zero;
        public float kickSpeed = 0f;
        public bool wantToKick = false;
    }

    public class DroneAISoccer : DroneAIBehavior
    {
        [HideInInspector]
        public Team team;
        [HideInInspector]
        public Position position;
        [HideInInspector]
        public GlobalVariables globalVariables;
        public DroneAgent agent;
        public void Plan()
        {
            // Plan your agent's actions here
            var droneManager = globalVariables.manager;
            agent = droneManager.GetComponent<DroneAgent>();
            agent.globalVariables = globalVariables;
        }

        public DroneAISoccer(DroneManagerSoccer droneManagerSoccer)
        {
            globalVariables = new GlobalVariables();
            globalVariables.manager = droneManagerSoccer;
        }

        public DroneAction Tick()
        {
            DroneAction action = new DroneAction();
            agent.RequestDecision();
            
            action.move_vector = globalVariables.moveDirection;

            if (globalVariables.wantToKick == true)
            {
                action.kickDirection = globalVariables.kickDirection * globalVariables.kickSpeed * globalVariables.manager.soccer.maxKickSpeed;
            } else {
                action.kickDirection = Vector3.zero;
            }
            
            return action;
        }

    }

}
using Drone;
using System.Linq;
using System.Collections.Generic;
using UnityEngine;

namespace UnityStandardAssets.Vehicles.Car
{
    public class GlobalVariablesCar
    {
        public CarManagerSoccer manager;
        public float steering = 0f;
        public float acceleration = 0f;
        public Vector3 kickDirection = Vector3.zero;
        public float kickSpeed = 0f;
        public bool wantToKick = false;
    }

    public class CarAISoccer : CarAIBehavior
    {
        [HideInInspector]
        public Team team;
        [HideInInspector]
        public Position position;
        [HideInInspector]
        public GlobalVariablesCar GlobalVariablesCar;
        public CarAgent agent;
        public void Plan()
        {
            // Plan your agent's actions here
            var carManager = GlobalVariablesCar.manager;
            agent = carManager.GetComponent<CarAgent>();
            agent.GlobalVariablesCar = GlobalVariablesCar;
        }

        public CarAISoccer(CarManagerSoccer carManagerSoccer)
        {
            GlobalVariablesCar = new GlobalVariablesCar();
            GlobalVariablesCar.manager = carManagerSoccer;
        }

        public CarAction Tick()
        {
            CarAction action = new CarAction();
            agent.RequestDecision();
            
            action.steering = GlobalVariablesCar.steering;
            action.acceleration = GlobalVariablesCar.acceleration;

            if (GlobalVariablesCar.wantToKick == true)
            {
                action.kickDirection = GlobalVariablesCar.kickDirection * GlobalVariablesCar.kickSpeed * GlobalVariablesCar.manager.soccer.maxKickSpeed;
            } else {
                action.kickDirection = Vector3.zero;
            }
            
            return action;
        }

    }

}
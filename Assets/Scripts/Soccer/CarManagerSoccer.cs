using System;
using Unity.Collections;
using Unity.Netcode;
using UnityEngine;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(SoccerComponent))]
    public class CarManagerSoccer : CarManager
    {
        public SoccerComponent soccer;
        public NetworkVariable<FixedString64Bytes> synctag = new NetworkVariable<FixedString64Bytes>();

        public override void Initialize()
        {
            soccer = GetComponent<SoccerComponent>();
            tag = synctag.Value.ToString();
            base.Initialize();
        }

        private void Start()
        {
            GameObject car_sphere = transform.Find("Sphere").gameObject;

            if (CompareTag("Blue"))
            {
                soccer.own_goal = GameObject.FindWithTag("BlueGoal");
                soccer.other_goal = GameObject.FindWithTag("RedGoal");
                car_sphere.GetComponent<Renderer>().material.SetColor("_Color", Color.blue);
            }
            else
            {
                soccer.own_goal = GameObject.FindWithTag("RedGoal");
                soccer.other_goal = GameObject.FindWithTag("BlueGoal");
                car_sphere.GetComponent<Renderer>().material.SetColor("_Color", Color.red);
            }

            soccer.Initialize();
        }

        protected override CarAIBehavior CreateAIBehavior()
        {
            return new CarAISoccer(this);
        }

        protected override void DoServer(CarAction carAction)
        {
            if (IsServer)
            {
                car.Move(carAction.steering, carAction.acceleration, carAction.acceleration, 0f);
                if (carAction.kickDirection != Vector3.zero)
                    soccer.KickBall(carAction.kickDirection);
            }
        }
    }
}
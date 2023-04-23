using Drone;
using Unity.Collections;
using Unity.Netcode;
using UnityEngine;

namespace UnityStandardAssets.Vehicles.Car
{
    public class DroneManagerSoccer : DroneManager
    {
        public SoccerComponent soccer;
        public NetworkVariable<FixedString64Bytes> synctag = new NetworkVariable<FixedString64Bytes>();

        public override void Initialize()
        {
            base.Initialize();
            soccer = GetComponent<SoccerComponent>();
            tag = synctag.Value.ToString();
        }

        private void Start()
        {
            GameObject go = transform.Find("Sphere").gameObject;

            if (CompareTag("Blue"))
            {
                soccer.own_goal = GameObject.FindWithTag("BlueGoal");
                soccer.other_goal = GameObject.FindWithTag("RedGoal");
                go.GetComponent<Renderer>().material.SetColor("_Color", Color.blue);
            }
            else
            {
                soccer.own_goal = GameObject.FindWithTag("RedGoal");
                soccer.other_goal = GameObject.FindWithTag("BlueGoal");
                go.GetComponent<Renderer>().material.SetColor("_Color", Color.red);
            }

            soccer.Initialize();
        }

        protected override DroneAIBehavior CreateAIBehavior()
        {
            return new DroneAISoccer(this);
        }

        protected override void DoServer(DroneAction action)
        {
            drone.Move_vect(action.move_vector);
            if (action.kickDirection != Vector3.zero)
                soccer.KickBall(action.kickDirection);
        }
    }
}
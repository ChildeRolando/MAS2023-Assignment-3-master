using System.Collections;
using System.Collections.Generic;
using UnityEngine; // to use random
using System;
namespace UnityStandardAssets.Vehicles.Car
{
    public class PD{
        public float k_p = 2f; // proportional gain
        public float k_d = 0.5f; // derivative gain
        Transform vehicle;
        public PD(Transform vehicle)
        {
            this.vehicle = vehicle;
        }

        public (float steering, float acceleration)PDCar(Vector3 targetPos,Vector3 carVelocity)
        {
            Vector3 positionDiff = targetPos - vehicle.position;
            Vector3 targetVelocity = positionDiff / Time.fixedDeltaTime;
            Vector3 velocityDiff = targetVelocity - carVelocity;

            Vector3 desireAcceleration = k_p * positionDiff + k_d * velocityDiff;

            float steering = Vector3.Dot(desireAcceleration, vehicle.right);
            float acceleration = Vector3.Dot(desireAcceleration, vehicle.forward);

            if (acceleration < 0f){
                acceleration = -acceleration;}

            return (steering, acceleration);
        }
        public Vector3 PDDrone(Vector3 targetPos, Vector3 droneVelocity)
        {
            Vector3 positionDiff = targetPos - vehicle.position;
            Vector3 targetVelocity = positionDiff / Time.fixedDeltaTime;
            Vector3 velocityDiff = targetVelocity - droneVelocity;

            Vector3 desireAcceleration = k_p * positionDiff + k_d * velocityDiff;
            return desireAcceleration;
        }
    }
}
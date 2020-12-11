﻿
using System;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = System.Random;

public class PositionAgent : Agent
{
    private Vector3 g = new Vector3(0, 9.80665f, 0);
    
    public float speed = 1;
    public float rotationSpeed = 180f;

    private bool _arrived;
    private float _randX, _randY, _randZ;

    private Vector3 _newPosition;
    
    private Quaternion _oldRotation, _rotation;
    
    private float stddev = 0.1f;
    private float _x , _y, _z;
    private float _magnitude;
    private Vector3 _axis, _bodyAcc, _accWithNoise, _angularVelocityWithNoise, _oldPosition, _oldVelocity, _velocity;
    private Vector3 AngularVelocity => (_axis * _magnitude) / Time.deltaTime;
    
    private Position _position;
    public override void Initialize()
    {
        ResetEnvironment();
    }
    
    private void ResetEnvironment()
    {
        _oldVelocity = Vector3.zero;
        
        _randX = UnityEngine.Random.Range(-10, 10);
        _randY = UnityEngine.Random.Range(-10, 10);
        _randZ = UnityEngine.Random.Range(-10, 10);
        _newPosition = new Vector3(_randX, _randY, _randZ);
        
        _oldPosition = Vector3.zero;
        _oldRotation = Quaternion.identity;
        
        transform.position = Vector3.zero;
        transform.rotation = Quaternion.identity;
        
        _position = new Position();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
       // acc
        _velocity = (transform.position - _oldPosition) / Time.deltaTime;
        Vector3 acc = (_velocity - _oldVelocity) / Time.deltaTime + g;
        Quaternion q = transform.rotation;
        _bodyAcc = q * acc;
    
        //Omega
        _rotation = transform.rotation;
        Quaternion deltaRotation = _rotation * Quaternion.Inverse(_oldRotation);
        deltaRotation.ToAngleAxis(out _magnitude, out _axis);
    
        _accWithNoise = new Vector3(
            (float) (_bodyAcc.x + SampleGaussian(0.0, stddev, new Random())),
            (float) (_bodyAcc.y + SampleGaussian(0.0, stddev, new Random())),
            (float) (_bodyAcc.z + SampleGaussian(0.0, stddev, new Random())));
    
        _angularVelocityWithNoise = new Vector3(
            (float) (AngularVelocity.x + SampleGaussian(0.0, 0.1, new Random())),
            (float) (AngularVelocity.y + SampleGaussian(0.0, 0.1, new Random())),
            (float) (AngularVelocity.z + SampleGaussian(0.0, 0.1, new Random())));
    
        if (!float.IsNaN(_accWithNoise.x))
        {
            sensor.AddObservation(_accWithNoise);
            sensor.AddObservation(_angularVelocityWithNoise);
        }
        else
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(Vector3.zero);
        }
    
        _oldPosition = transform.position;
        _oldVelocity = _velocity;
        _oldRotation = _rotation;
    }

    public override void OnActionReceived(float[] vectorAction)
    {
        var estimateBodyAcc = new Vector3(_accWithNoise.x * (vectorAction[0]+1), 
            _accWithNoise.y * (vectorAction[1]+1), 
            _accWithNoise.z * (vectorAction[2]+1));
        var estimateAngularVelocity = new Vector3(_angularVelocityWithNoise.x * (vectorAction[3]+1), 
            _angularVelocityWithNoise.y * (vectorAction[4]+1), 
            _angularVelocityWithNoise.z * (vectorAction[5]+1));
        
        AddReward(-Vector3.Distance(_bodyAcc, estimateBodyAcc));
        AddReward(-Vector3.Distance(AngularVelocity, estimateAngularVelocity));
        
        
        // calculating position
        _position.EstimateAngularVelocity = estimateAngularVelocity;
        _position.EstimateBodyAcc = estimateBodyAcc;
        
        _position.CalculatePosition();
        
    }

    private void FixedUpdate()
    {
        // ControlAgent();
        MoveAgent();
        RequestDecision();
        RequestAction();
    }

    /**
     *  Used for tests where the agent is under human control.
     */
    private void ControlAgent()
    {
        var pitch = Input.GetKey(KeyCode.W) ? -1.0f : 0.0f;
        pitch += Input.GetKey(KeyCode.S) ? 1.0f : 0.0f;
        
        var yaw = Input.GetKey(KeyCode.Q) ? -1.0f : 0.0f;
        yaw += Input.GetKey(KeyCode.E) ? 1.0f : 0.0f;
        
        var roll = Input.GetKey(KeyCode.A) ? -1.0f : 0.0f;
        roll += Input.GetKey(KeyCode.D) ? 1.0f : 0.0f;
        
        
        _x += Input.GetKey(KeyCode.J) ? -1.0f : 0.0f
            + (Input.GetKey(KeyCode.L) ? 1.0f : 0.0f);
        
        _y += Input.GetKey(KeyCode.I) ? 1.0f : 0.0f 
            + (Input.GetKey(KeyCode.K) ? -1.0f : 0.0f);
        
        _z += Input.GetKey(KeyCode.U) ? -1.0f : 0.0f 
            + (Input.GetKey(KeyCode.O) ? 1.0f : 0.0f);
        
        if (Input.GetKey(KeyCode.B))
        {
            _x *= 0.3f * Time.fixedDeltaTime;
            _y *= 0.3f * Time.fixedDeltaTime;
            _z *= 0.3f * Time.fixedDeltaTime;
        }
        
        transform.Rotate(new Vector3(0,0,1) * (yaw * rotationSpeed * Time.fixedDeltaTime));
        transform.Rotate(new Vector3(0,1,0) * (roll * rotationSpeed * Time.fixedDeltaTime));
        transform.Rotate(new Vector3(1,0,0) * (pitch * rotationSpeed * Time.fixedDeltaTime));
        
        transform.Translate(_x * Time.fixedDeltaTime * 0.1f, 0, 0);
        transform.Translate(0, _y * Time.fixedDeltaTime * 0.1f, 0);
        transform.Translate(0, 0, _z  * Time.fixedDeltaTime * 0.1f);
    }

    private void MoveAgent()
    {
        var direction = (_newPosition - transform.position ).normalized;
        
        transform.position += direction * speed * Time.deltaTime;
        if (Vector3.Distance(transform.position, _newPosition) <= 0.1f)
        {
            _randX = UnityEngine.Random.Range(-10, 10);
            _randY = UnityEngine.Random.Range(-10, 10);
            _randZ = UnityEngine.Random.Range(-10, 10);
            _newPosition = new Vector3(_randX, _randY, _randZ);
        }

        Random random = new Random();
        float xSpin = random.Next(-1,2);
        float ySpin = random.Next(-1,2);
        float zSpin = random.Next(-1,2);
        
        transform.Rotate(transform.up * (xSpin * rotationSpeed * Time.fixedDeltaTime));
        transform.Rotate(transform.forward * (ySpin * rotationSpeed * Time.fixedDeltaTime));
        transform.Rotate(transform.right * (zSpin * rotationSpeed * Time.fixedDeltaTime));
    }

    private static double SampleGaussian(double mean, double stddev, Random random)
    {
        double x1 = 1 - random.NextDouble();
        double x2 = 1 - random.NextDouble();

        double y1 = Math.Sqrt(-2.0 * Math.Log(x1)) * Math.Cos(2.0 * Math.PI * x2);
        return y1 * stddev + mean; 
    }

    public override void Heuristic(float[] actionsOut)
    {
    }

    public override void OnEpisodeBegin()
    {
        ResetEnvironment();
    }
}

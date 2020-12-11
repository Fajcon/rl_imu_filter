using System;
using UnityEngine;

public class Position
{

    private Vector3 _estimateVelocity, _calculatedEstimatePosition;
    public Vector3 EstimateBodyAcc, EstimateAngularVelocity;
    
    private Vector3 g = new Vector3(0, 0, 9.8f);

    private Quaternion _estimateRotation;

    private const float DT = 0.005f;
    
    private string _output;

    public void CalculatePosition()
    {
        _estimateRotation *= Quaternion.Euler(EstimateAngularVelocity * DT);
        
        _estimateVelocity += ((_estimateRotation * EstimateBodyAcc)-g) * DT;
        _calculatedEstimatePosition += _estimateVelocity * DT;

        _output += _calculatedEstimatePosition.x.ToString("0.0000000000", System.Globalization.CultureInfo.InvariantCulture) + "," + 
                   _calculatedEstimatePosition.y.ToString("0.0000000000", System.Globalization.CultureInfo.InvariantCulture) + "," + 
                   _calculatedEstimatePosition.z.ToString("0.0000000000", System.Globalization.CultureInfo.InvariantCulture) + "," + 
                   _estimateRotation.w.ToString("0.0000000000", System.Globalization.CultureInfo.InvariantCulture) + "," + 
                   _estimateRotation.x.ToString("0.0000000000", System.Globalization.CultureInfo.InvariantCulture) + "," + 
                   _estimateRotation.y.ToString("0.0000000000", System.Globalization.CultureInfo.InvariantCulture) + "," +
                   _estimateRotation.z.ToString("0.0000000000", System.Globalization.CultureInfo.InvariantCulture) + "\n";
    }

    public void SavePositionToFile()
    {
        System.IO.File.WriteAllText("estimate_position_and_rotation.csv", _output);
    }

    public Position()
    {
        EstimateBodyAcc = Vector3.zero;
        EstimateAngularVelocity = Vector3.zero;
        _estimateVelocity = Vector3.zero;
        _calculatedEstimatePosition = Vector3.zero;
        _estimateRotation = Quaternion.identity;
        _output = "";
    }
}

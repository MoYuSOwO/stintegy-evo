using Xunit;

namespace StintegyEVO.Drivers.Tests;

public sealed class LineActuatorsTests
{
    [Fact]
    public void LeftOfTheLineTurnsRight()
    {
        LineActuators.Steer cmd = LineActuators.CurvatureForLine(
            n: 2f,
            nLine: 0f,
            headingError: 0f,
            speed: 40f,
            pathKappa: 0f,
            halfWidth: 7f,
            carHalfWidth: 0.95f,
            maxKappa: 0.2f
        );
        Assert.True(cmd.Curvature < 0f, $"expected a right turn, k={cmd.Curvature}");
        Assert.InRange(cmd.PredictedN, -2.5f, 2.5f);
    }

    [Fact]
    public void DoesNotAskToCrossTheInsideEdge()
    {
        LineActuators.Steer cmd = LineActuators.CurvatureForLine(
            n: -5.4f,
            nLine: -5.4f,
            headingError: 0.05f,
            speed: 60f,
            pathKappa: -0.008f,
            halfWidth: 7f,
            carHalfWidth: 0.95f,
            maxKappa: 0.2f
        );
        Assert.InRange(cmd.PredictedN, cmd.NLow, cmd.NHigh);
    }
}

using System;
using StintegyEVO.Core.Cars;
using Xunit;
using Xunit.Abstractions;
namespace StintegyEVO.Core.Tests;
public sealed class TmpSpinCost
{
    private readonly ITestOutputHelper _o;
    public TmpSpinCost(ITestOutputHelper o) => _o = o;
    [Fact]
    public void OneSpin()
    {
        CarConfig car = new();
        TireConfig tires = new() { StartingSurfaceTempC = 90f, StartingCoreTempC = 90f };
        CarState s = new() { Speed = 70f, SideslipAngleRadians = 0.65f, YawRateRadiansPerSecond = -1.5f, Energy = PowertrainState.Filled(0.8f) };
        s.InstallFreshTires(tires);
        var input = new CarPhysicsStepInput(new DriverInput(0f, 0f), CarStrategy.Default, 25f, 35f);
        for (int i = 0; i < 30; i++) { s.SideslipAngleRadians = 0.65f; CarPhysics.Step(s, car, tires, input, 1f / 60f); }
        float w0 = (s.FrontLeft.Wear + s.FrontRight.Wear + s.RearLeft.Wear + s.RearRight.Wear) * 25f;
        float t0 = (s.FrontLeft.SurfaceTempC + s.RearLeft.SurfaceTempC) * 0.5f;
        int steps = 0;
        while (s.Spinning && steps < 40 * 60) { CarPhysics.Step(s, car, tires, input, 1f / 60f); steps++; }
        float w1 = (s.FrontLeft.Wear + s.FrontRight.Wear + s.RearLeft.Wear + s.RearRight.Wear) * 25f;
        float t1 = (s.FrontLeft.SurfaceTempC + s.RearLeft.SurfaceTempC) * 0.5f;
        _o.WriteLine($"一次旋转 {steps / 60f:0.00} 秒：胎耗 {w0:0.000}% → {w1:0.000}% (+{w1 - w0:0.000})  胎面 {t0:0.0}C → {t1:0.0}C");
    }
}

// using Godot;
// using System.Collections.Generic;
// using PloyRacing.Core.Track;
// using PloyRacing.Util;
// using PloyRacing.Core.Car.Configs;
// using PloyRacing.Core.Car.Components;

// namespace PloyRacing.Core.Car.Controllers;

// public class AIController : IController
// {
//     // Stanley 软化系数：防止分母为0，保持低速稳定
//     private const float SoftFactor = 0.1f;
//     // 平滑系数：余弦平滑防止刚开始打方向盘时候的抖动
//     private const float BlendFactor = 0.25f;

//     public float Throttle { get; private set; } = 0.0f;
//     public float Brake { get; private set; } = 0.0f;
//     public float SteeringAngle { get; private set; } = 0.0f;

//     public float FuelSaveFactor { get; set; } = 0.0f;
//     public float TireSaveFactor { get; set; } = 0.0f;

//     public readonly List<Vector2> path = [];
//     public readonly List<float> vel = [];
//     public float TargetSpeed { get; private set; }

//     // 记忆上一帧的踏板状态，用来做人类脚踝的平滑过渡
//     private float _lastThrottle = 0.0f;
//     private float _lastBrake = 0.0f;

//     // P 控制器系数：踩踏板的“果断程度”
//     private const float KpAccel = 2.0f;  
//     private const float KpBrake = 5.0f;

//     public float CalculateSteering(CarSensor carSensor, CarLogic carLogic, TrackData track) {
//         if (path.Count < 2) return 0.0f;

//         // 获取前轴位置
//         Vector2 pos = carSensor.Position;
//         float carHeading = carSensor.Rotation;
//         float halfWheelBase = carLogic.Config.Chassis.HalfWheelBase;

//         // 前轴坐标
//         Vector2 offset = new(Mathf.Cos(carHeading) * halfWheelBase, Mathf.Sin(carHeading) * halfWheelBase);
//         Vector2 frontAxlePos = pos + offset;

//         // 找到距离前轴最近的参考点
//         int targetIdx = 0;
//         float minDistSq = float.MaxValue;

//         int searchLimit = Mathf.Min(path.Count, 10);
//         for (int i = 0; i < searchLimit; i++) {
//             float distSq = path[i].DistanceSquaredTo(frontAxlePos);
//             if (distSq < minDistSq) {
//                 minDistSq = distSq;
//                 targetIdx = i;
//             }
//         }

//         // 拿到参考点和下一个点 (用于算切线)
//         if (targetIdx >= path.Count - 1) targetIdx = path.Count - 2;
//         Vector2 pRef = path[targetIdx];
//         Vector2 pNext = path[targetIdx + 1];

//         // 只计算航向误差
//         Vector2 v = pNext - pRef;
//         float pathHeading = (float) Mathf.Atan2(v.Y, v.X);
//         float finalSteer = GeomUtil.NormalizeAngle(pathHeading - carHeading);

//         // 归一化输出
//         return Mathf.Clamp(finalSteer / carLogic.Config.Chassis.MaxSteerAngle, -1, 1);
//     }

//     private void GenerateRecoveryPath(CarSensor carSensor, CarLogic carLogic, TrackData track)
//     {
//         path.Clear();
//         int nextIdx = track.FindNearestIndex(carSensor.Position) + 1;
//         Vector2 optimalLineWorld = track[nextIdx].Optimal;
//         Vector2 tangent = track[nextIdx].Tangent;
//         Vector2 pos = carSensor.Position;
//         float speed = carSensor.LinearVelocity.Length();

//         Vector2 leftNormal = new(tangent.Y, -tangent.X);

//         // 叉乘算垂直距离 (d0 > 0: 车在赛道左边; d0 < 0: 车在赛道右边)
//         float d0 = (pos - optimalLineWorld).Dot(leftNormal);

//         // 回归参数
//         float mergeDistance = Mathf.Max(20.0f, speed * 0.5f);
//         float currentDist = 0.0f;
//         int idx = nextIdx;
//         float totalGenDist = mergeDistance + 400.0f;

//         // 余弦回归路线
//         while (currentDist < totalGenDist)
//         {
//             Vector2 nodeOptimal = track[idx].Optimal;
//             Vector2 nodeLeft = track[idx].LeftEdge;
//             Vector2 nodeRight = track[idx].RightEdge;
//             Vector2 nodeTan = track[idx].Tangent;
//             Vector2 nextNodeTan = track[idx + 1].Tangent;

//             float cross = nodeTan.Cross(nextNodeTan);
//             float dot = nodeTan.Dot(nextNodeTan);
//             float angleDiff = Mathf.Abs(Mathf.Atan2(cross, dot));

//             float trackCurvature = angleDiff / TrackData.StepLength;
//             float baseMargin = carLogic.Config.Chassis.Width * 0.5f + 0.2f;
//             float yawFactor = carLogic.Config.Chassis.Length * 0.5f;
//             float attitudeComp = trackCurvature * yawFactor;
//             float dynamicSafeMargin = baseMargin + attitudeComp;

//             // 计算当前点的目标偏移量
//             float targetOffset = 0.0f;
//             if (currentDist < mergeDistance) 
//             {
//                 // 计算归一化进度
//                 float progress = currentDist / mergeDistance;
//                 // 线性分量
//                 float linearDecay = 1.0f - progress;
//                 // 余弦分量
//                 float cosineDecay = 0.5f * (1.0f + Mathf.Cos(progress * Mathf.Pi));

//                 float decayFactor = cosineDecay * (1.0f - BlendFactor) + linearDecay * BlendFactor;
//                 targetOffset = d0 * decayFactor;
//             }

//             // 相对于赛车线，左右两边还能往外偏多少米
//             float maxLeftOffset = Mathf.Max(0.0f, nodeLeft.DistanceTo(nodeOptimal) - dynamicSafeMargin);
//             float maxRightOffset = -Mathf.Max(0.0f, nodeRight.DistanceTo(nodeOptimal) - dynamicSafeMargin);

//             float effectiveMaxLeft = (d0 > 0) ? Mathf.Max(maxLeftOffset, targetOffset) : maxLeftOffset;
//             float effectiveMaxRight = (d0 < 0) ? Mathf.Min(maxRightOffset, targetOffset) : maxRightOffset;

//             if (targetOffset > effectiveMaxLeft)
//             {
//                 targetOffset = effectiveMaxLeft;
//             }
//             else if (targetOffset < effectiveMaxRight)
//             {
//                 targetOffset = effectiveMaxRight;
//             }

//             Vector2 currentLeftNormal = new(nodeTan.Y, -nodeTan.X);
//             currentLeftNormal *= targetOffset;
//             path.Add(nodeOptimal + currentLeftNormal);

//             idx++;
//             currentDist += TrackData.StepLength;
//         }
//     }

//     public void CalculateTargetSpeed(CarSensor carSensor, CarLogic carLogic, TrackData track)
//     {
//         vel.Clear();
//         vel.AddRange([0.0f, 0.0f, 0.0f]);

//         float mass = carSensor.Mass;
//         float maxSpeed = carLogic.Config.Power.GearboxConfig[^1].MaxSpeed;
        
//         // 最小横向摩擦
//         float latMiu = float.MaxValue;
//         foreach (var tire in carLogic.Tires)
//         {
//             latMiu = Mathf.Min(latMiu, tire.CurrLatPeakFriction);
//         }
//         latMiu *= track.Friction;

//         // 空气动力学下压力系数
//         AeroOutput aeroAt1 = carLogic.Aero.CalculateAero(1.0f, 0f);
//         float dfCoef = aeroAt1.DownforceFront + aeroAt1.DownforceRear;

//         for (int i = 3; i < path.Count - 3; i++)
//         {
//             Vector2 pPrev = path[i - 3], pCurr = path[i], pNext = path[i + 3];
//             float curvature = GeomUtil.Curvature(pPrev, pCurr, pNext);
            
//             if (curvature > 0.0001f)
//             {
//                 // V^2 * k = (m * g + V^2 * dfCoef) * LatMiu / m
//                 float denom = mass * curvature - dfCoef * latMiu;
                
//                 if (denom > 0) {
//                     vel.Add(Mathf.Sqrt((mass * GeomUtil.g * latMiu) / denom));
//                 } else {
//                     vel.Add(maxSpeed);
//                 }
//             }
//             else
//             {
//                 vel.Add(maxSpeed);
//             }
//         }

//         vel.Add(vel[path.Count - 4]);
//     }

//     private float CalculatePreviewDistance(CarSensor carSensor, CarLogic carLogic, TrackData track)
//     {
//         float v0 = carSensor.LinearVelocity.Length();
//         if (v0 < 0.1f) return 10f;

//         float currentLatAccel = Mathf.Abs(carSensor.LocalAccel.X);
//         float mass = carSensor.Mass;

//         float latMiu = float.MaxValue, longMiu = float.MaxValue;
//         foreach (var tire in carLogic.Tires)
//         {
//             latMiu = Mathf.Min(latMiu, tire.CurrLatPeakFriction);
//             longMiu = Mathf.Min(longMiu, tire.CurrLongPeakFriction);
//         }
//         latMiu *= track.Friction;
//         longMiu *= track.Friction;

//         AeroOutput aero = carLogic.Aero.CalculateAero(v0, 0f);
//         float currentDownforce = aero.DownforceFront + aero.DownforceRear;
//         float currentDrag = aero.DragForce;

//         // 系数 k_df, k_drag，使得下压力 = k_df * v^2，阻力 = k_drag * v^2
//         float v0Sq = v0 * v0;
//         float k_df = v0Sq > 0.01f ? currentDownforce / v0Sq : 0f;
//         float k_drag = v0Sq > 0.01f ? currentDrag / v0Sq : 0f;

//         // 包含下压力的真实最大横纵抓地力
//         float maxVerticalForce = mass * GeomUtil.g + currentDownforce;
//         float maxLatAccel = latMiu * maxVerticalForce / mass;
//         float latUsage = Mathf.Clamp(currentLatAccel / maxLatAccel, 0f, 1f);

//         // 摩擦椭圆给纵向预留的系数
//         float longMiuEff = longMiu * Mathf.Sqrt(1f - latUsage * latUsage);

//         // 预留的坡度信息
//         float slopeAngle = 0f;
//         float cosSlope = Mathf.Cos(slopeAngle);
//         float sinSlope = Mathf.Sin(slopeAngle);

//         // 纵向减速度公式 a = A + B v^2 中的系数
//         // A: 与速度无关的部分（重力 + 摩擦力在坡道上的投影）
//         // B: 与 v^2 成正比的部分（下压力带来的额外抓地力 + 空气阻力）
//         float A = longMiuEff * GeomUtil.g * cosSlope + GeomUtil.g * sinSlope;
//         float B = (longMiuEff * k_df + k_drag) / mass;

//         //寻找前方第一个低于当前速度的限速点
//         float vTarget = v0;
//         bool foundTarget = false;
//         for (int i = 0; i < vel.Count; i++)
//         {
//             if (vel[i] < v0 - 0.1f)
//             {
//                 vTarget = vel[i];
//                 foundTarget = true;
//                 break;
//             }
//         }

//         if (!foundTarget)
//         {
//             return 800f;
//         }

//         // 计算精确刹车距离（解析解）
//         float d_brake;
//         if (A <= 0f)
//         {
//             // 极端情况：重力下坡分量太大，即使全力制动都无法减速
//             d_brake = 800f;
//         }
//         else if (Mathf.Abs(B) < 1e-6f)
//         {
//             // 无空气动力影响的匀减速
//             d_brake = (v0 * v0 - vTarget * vTarget) / (2f * A);
//         }
//         else
//         {
//             // 有空气动力学效应，精确积分结果
//             float numerator = A + B * v0Sq;
//             float denominator = A + B * vTarget * vTarget;
//             if (denominator <= 0f) denominator = 1e-10f;
//             d_brake = Mathf.Log(numerator / denominator) / (2f * B);
//         }

//         // 响应时间补偿
//         float responseTime = 0.3f;
//         // 最终预瞄距离
//         float finalPreview = d_brake + v0 * responseTime;
//         return Mathf.Max(finalPreview, 10f);
//     }

//     private float FindSpeedLimitInPreview(CarSensor carSensor, TrackData track, float previewDist)
//     {
//         float currentSpeed = carSensor.LinearVelocity.Length();
//         Vector2 carPos = carSensor.Position;

//         float accumulatedDist = 0f;
//         for (int i = 3; i < path.Count - 3; i++)
//         {
//             accumulatedDist += (path[i + 1] - path[i]).Length();
//             if (accumulatedDist > previewDist) break;
//             if (vel[i] < currentSpeed - 0.1f)
//             {
//                 return vel[i];
//             }
//         }
//         return 9999f;
//     }

//     private float SolveBrakePedalPrecise(CarLogic carLogic, float targetForce)
//     {
//         if (targetForce <= 0f) return 0f;

//         var brake = carLogic.Brake;
//         var envTemp = carLogic.Environment.EnvTemp;

//         float maxTheoretical = brake.Config.MaxBrakeForce;
//         float biasF = brake.BiasFront;
//         float biasR = 1f - biasF;

//         // 获取四个轮子的当前效率
//         float effFL = brake.CalculateEfficiency(brake.Temp.FrontLeft, envTemp);
//         float effFR = brake.CalculateEfficiency(brake.Temp.FrontRight, envTemp);
//         float effRL = brake.CalculateEfficiency(brake.Temp.RearLeft, envTemp);
//         float effRR = brake.CalculateEfficiency(brake.Temp.RearRight, envTemp);

//         // 在当前效率下，前后轴各自能产生的最大制动力（当踏板=1.0时）
//         float maxFrontActual = 0.5f * biasF * maxTheoretical * (effFL + effFR);
//         float maxRearActual = 0.5f * biasR * maxTheoretical * (effRL + effRR);
//         float maxTotalActual = maxFrontActual + maxRearActual;

//         float pedal = targetForce / maxTotalActual;
//         return Mathf.Clamp(pedal, 0f, 1f);
//     }

//     private float GetDriveTrainLongLimit(CarLogic carLogic, CarSensor carSensor, TrackData track, float longBudget)
//     {
//         float limit = 0f;
//         float[] longMius = new float[4];

//         var load = carSensor.Params.Load;
//         var trackFriction = track.Friction;

//         for (int i = 0; i < 4; i++)
//         {
//             longMius[i] = carLogic.Tires[i].CurrLongPeakFriction * trackFriction;
//         }
//         if (carLogic.Config.DriveType == CarDriveType.FrontDrive)
//         {
//             limit += load.FrontLeft * longMius[0] + load.FrontRight * longMius[1];
//         }
//         else if (carLogic.Config.DriveType == CarDriveType.RearDrive)
//         {
//             limit += load.RearLeft * longMius[2] + load.RearRight * longMius[3];
//         }
//         else
//         {
//             limit += load.FrontLeft * longMius[0] + load.FrontRight * longMius[1];
//             limit += load.RearLeft * longMius[2] + load.RearRight * longMius[3];
//         }
//         return limit * longBudget;
//     }

//     public (float Throttle, float Brake) CalculatePedals(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
//     {
//         float currentSpeed = carSensor.LinearVelocity.Length();
//         float currentLatAccel = carSensor.LocalAccel.X;
//         float mass = carSensor.Mass;

//         float latMiu = float.MaxValue;
//         float longMiu = float.MaxValue;
//         foreach (var tire in carLogic.Tires)
//         {
//             latMiu = Mathf.Min(latMiu, tire.CurrLatPeakFriction);
//             longMiu = Mathf.Min(longMiu, tire.CurrLongPeakFriction);
//         }
//         latMiu *= track.Friction;
//         longMiu *= track.Friction;

//         float previewDist = CalculatePreviewDistance(carSensor, carLogic, track);
//         float vLimit = FindSpeedLimitInPreview(carSensor, track, previewDist); 

//         GD.Print("previewDist: ", previewDist);

//         TargetSpeed = vLimit;

//         AeroOutput aero = carLogic.Aero.CalculateAero(currentSpeed, 0f);
//         float downforce = aero.DownforceFront + aero.DownforceRear;

//         float maxLatAccel = latMiu * (mass * GeomUtil.g + downforce) / mass;
//         float latUsage = Mathf.Clamp(currentLatAccel / maxLatAccel, 0f, 1f);
//         float longBudget = Mathf.Sqrt(1f - latUsage * latUsage);

//         float maxTireLongForce = longBudget * longMiu * (mass * GeomUtil.g + downforce);

//         float targetThrottle = 0f, targetBrake = 0f;
//         if (currentSpeed > vLimit + 1.0f)
//         {
//             float brakeMaxForce = carLogic.Brake.Config.MaxBrakeForce;
//             float brakeForceNeeded = Mathf.Min(maxTireLongForce, brakeMaxForce);

//             targetBrake = SolveBrakePedalPrecise(carLogic, brakeForceNeeded);
//         }
//         else
//         {
//             float speedError = vLimit - currentSpeed;
//             if (speedError > 0)
//             {
//                 PowerComponent power = carLogic.Power;
//                 float engineMaxForce = power.Config.BaseForce 
//                                     * power.Config.GetForceRatioAtRPMRatio(power.RPMRatio) 
//                                     * power.Config.GetMultiplierAtGear(power.CurrentGear);
//                 float driveTrainLimit = GetDriveTrainLongLimit(carLogic, carSensor, track, longBudget);
//                 float driveForceNeeded = Mathf.Min(driveTrainLimit, engineMaxForce);
//                 targetThrottle = driveForceNeeded / engineMaxForce;
//             }
//         }

//         // 滤波输出
//         _lastThrottle = Mathf.Lerp(_lastThrottle, targetThrottle, 12.0f * dt);
//         _lastBrake = Mathf.Lerp(_lastBrake, targetBrake, 18.0f * dt); 

//         return (_lastThrottle, _lastBrake);
//     }

//     public void ThinkTick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
//     {
//         GenerateRecoveryPath(carSensor, carLogic, track);
//         SteeringAngle = CalculateSteering(carSensor, carLogic, track);
//         CalculateTargetSpeed(carSensor, carLogic, track);
//         (Throttle, Brake) = CalculatePedals(dt, carSensor, carLogic, track);
//         GD.Print(Throttle);
//     }

//     public void Tick(float dt, CarSensor carSensor, CarLogic carLogic, TrackData track)
//     {
//     }
// }
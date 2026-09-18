using System.Reflection;
using System.Text.Json;
using StintegyEVO.Core.Cars;
using StintegyEVO.Core.Drivers;
using StintegyEVO.Core.Racing;
using StintegyEVO.Core.Track;
using StintegyEVO.Core.Tests;
class Program {
static void Emit(object o)=>Console.WriteLine(JsonSerializer.Serialize(o));
static TireState[] Wheels(CarState s)=>new[]{s.FrontLeft,s.FrontRight,s.RearLeft,s.RearRight};
static object Temps(CarState s)=>new {surface=Wheels(s).Select(t=>t.SurfaceTempC).ToArray(),core=Wheels(s).Select(t=>t.CoreTempC).ToArray(),wear=Wheels(s).Select(t=>t.Wear).ToArray()};
static object Call(string name,params object[] args)=>typeof(CarPhysicsTests).GetMethods(BindingFlags.NonPublic|BindingFlags.Static).Single(m=>m.Name==name && m.GetParameters().Length==args.Length).Invoke(null,args)!;
static void Main(string[] args){
if(args.Length==0 || args[0]=="thermal") {
 foreach(float use in new[]{.6f,.7f,.8f,.85f,.9f,.955f}) {var s=(CarState)Call("RunRepresentativeCoreDutyCycle",use); Emit(new {kind="calibration",use,temps=Temps(s)});}
 foreach(float ambient in new[]{25f,70f}) {
 var c=new CarConfig(); var t=new TireConfig{StartingSurfaceTempC=70f,StartingCoreTempC=70f}; var s=new CarState{Speed=36f,Energy=PowertrainState.Filled(.8f)};s.InstallFreshTires(t);
 float k=(float)Call("CurvatureForGripShare",s,c,t,CarStrategy.Default,1f);
 for(int i=0;i<120;i++) {CarPhysics.Step(s,c,t,new CarPhysicsStepInput(new DriverInput(k,0),CarStrategy.Default,ambient,ambient==25?35:70),1f/60); if(i%30==29) Emit(new{kind="thermal_response",ambient,time=(i+1)/60f,s.Speed,slip=s.SideslipAngleRadians,temps=Temps(s)});}
 }
 foreach(string name in new[]{"AxleLateralComplianceRedistributesHeatAndWearWithoutChangingTotalWork","CoreTemperatureRespondsMoreSlowlyThanTreadSurface"}){try{typeof(CarPhysicsTests).GetMethod(name)!.Invoke(new CarPhysicsTests(),null);Emit(new{kind="test",name,passed=true});}catch(TargetInvocationException e){Emit(new{kind="test",name,passed=false,error=e.InnerException!.Message});}}
 return;
}
string mode=args[0];var track=TrackFactory.SimpleTestTrack();var start=track.Sample(track.Grids[1].S);var tires=new TireConfig{StartingSurfaceTempC=90f,StartingCoreTempC=90f,ColdGripLossPerCSquared=mode=="no_temp_grip"?0f:.00060f,HotGripLossPerCSquared=mode=="no_temp_grip"?0f:.00070f};
var driver=new ReferenceLineDriver();var car=new RaceCar("lap-test",new CarConfig(),tires,driver,new CarState{Position=start.RefPosition,Heading=start.RefHeading,Speed=8f,Energy=PowertrainState.Filled(.9f)});var sim=new RaceSimulation(track);sim.AddCar(car); int walls=0;float first=-1;
for(int i=0;i<60*150;i++) {
 sim.Step(1f/60); if(car.LastBoundaryContact.HasValue){walls++;if(first<0)first=(i+1)/60f;}
 var s=car.State;var d=driver.LastTelemetry;var p=s.Telemetry;
 if(mode=="probe" && i==4199){
 foreach(float steering in new[]{-.32f,0f,.32f}){
 var q=s.Clone();var input=new CarPhysicsStepInput(new DriverInput(steering,p.Input.DesiredAccel),car.Strategy,sim.Environment.AirTempC,sim.Environment.TrackTempC);
 CarPhysics.Step(q,car.CarConfig,car.TireConfig,input,1f/120);
 var freePosition=q.Position;float freeHeading=q.Heading;
 var free=q.Clone();
 foreach(string pose in new[]{"full","translation_only","rotation_only"}){
 q=free.Clone();if(pose=="translation_only")q.Heading=s.Heading;if(pose=="rotation_only")q.Position=s.Position;
 var hit=TrackBoundaryResolver.ResolveSweep(track,s,q,car.Collision);
 Emit(new{kind="boundary_probe",pose,steering,hit=hit.HasValue,fraction=hit?.ImpactFraction,freeDistance=System.Numerics.Vector2.Distance(freePosition,s.Position),resolvedDistance=System.Numerics.Vector2.Distance(q.Position,s.Position),freeRotation=freeHeading-s.Heading,resolvedRotation=q.Heading-s.Heading,normalSpeed=hit.HasValue?System.Numerics.Vector2.Dot(q.Velocity,hit.Value.Normal):0,Speed=q.Speed,IsRecovering=d.IsRecovering,control=d.ControlSeverity});
 }
 }
 return;
 }

 if(i%6==5 && i<60*20 || i%600==599 || first==(i+1)/60f) Emit(new{kind="driver",mode,time=(i+1)/60f,s=car.Progress.CurrentS,dist=car.Progress.TotalDistance,d=car.Progress.CurrentD,s.Speed,heading=s.Heading,slip=s.SideslipAngleRadians,yaw=s.YawRateRadiansPerSecond,error=d.LateralErrorMeters,target=d.TargetSpeed,k=d.DesiredCurvature,actualK=p.ActualCurvature,frontGrip=p.FrontGripAccel,rearGrip=p.RearGripAccel,frontUse=p.FrontLateralUse,rearUse=p.RearLateralUse,over=p.OverLimit,accel=p.ActualLongitudinalAccel,requestedAccel=p.RequestedLongitudinalAccel,controlSeverity=d.ControlSeverity,d.IsRecovering,temps=Temps(s),walls});
}
Emit(new{kind="driver_result",mode,walls,first,dist=car.Progress.TotalDistance,length=track.LengthMeters});
}
}

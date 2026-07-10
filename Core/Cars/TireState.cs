namespace TheStint.Core.Cars;

public sealed class TireState
{
    public TireState() : this(TireConfig.Default)
    {
    }

    public TireState(TireConfig config)
    {
        Reset(config);
    }

    public void Reset(TireConfig config)
    {
        SurfaceTempC = config.StartingSurfaceTempC;
        CoreTempC = config.StartingCoreTempC;
        Wear = 0f;
        LoadN = 0f;
    }

    public float SurfaceTempC { get; set; }
    public float CoreTempC { get; set; }
    public float Wear { get; set; }
    public float LoadN { get; set; }

    public TireState Clone()
    {
        return new TireState
        {
            SurfaceTempC = SurfaceTempC,
            CoreTempC = CoreTempC,
            Wear = Wear,
            LoadN = LoadN
        };
    }

    public void CopyFrom(TireState other)
    {
        SurfaceTempC = other.SurfaceTempC;
        CoreTempC = other.CoreTempC;
        Wear = other.Wear;
        LoadN = other.LoadN;
    }
}

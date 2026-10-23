namespace StintegyEVO.Core.Cars;

public sealed class TireState
{
    /// <summary>A new tyre, off the blankets.</summary>
    public TireState()
    {
        Reset(Tire.BlanketTempC, Tire.BlanketTempC);
    }

    /// <summary>A new tyre, at these temperatures.</summary>
    public void Reset(float surfaceTempC, float coreTempC)
    {
        SurfaceTempC = surfaceTempC;
        CoreTempC = coreTempC;
        Wear = 0f;
        LoadN = 0f;
    }

    public float SurfaceTempC { get; set; }
    public float CoreTempC { get; set; }
    public float Wear { get; set; }
    public float LoadN { get; set; }

    /// <summary>
    /// What the road under this wheel is worth, as a multiplier on clean
    /// racing surface. One on tarmac, less on a kerb, much less past the
    /// white line.
    ///
    /// Per wheel and not per car, because that is the only way a car with
    /// two wheels on the grass is a different car from one with four on
    /// the road - and cutting a corner is then priced by how much of the
    /// car went, continuously, instead of by a flag that trips somewhere
    /// nobody can see.
    /// </summary>
    public float SurfaceGrip { get; set; } = 1f;

    /// <summary>How hard the road under it is on rubber: a multiplier on wear, one on an average circuit.</summary>
    public float SurfaceAbrasion { get; set; } = 1f;

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
        SurfaceGrip = other.SurfaceGrip;
        SurfaceAbrasion = other.SurfaceAbrasion;
        SurfaceTempC = other.SurfaceTempC;
        CoreTempC = other.CoreTempC;
        Wear = other.Wear;
        LoadN = other.LoadN;
    }
}

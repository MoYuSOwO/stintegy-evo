namespace StintegyEVO.Core.Car.Controllers.V1.VelocityProfiles;

public interface IVelocityProfileSolver
{
    VelocityProfile Solve(VelocityProfileRequest request);
}

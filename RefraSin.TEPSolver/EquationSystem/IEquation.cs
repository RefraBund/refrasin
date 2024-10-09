namespace RefraSin.TEPSolver.EquationSystem;

public interface IEquation
{
    public double Value();

    public IEnumerable<(int, double)> Derivative();
}

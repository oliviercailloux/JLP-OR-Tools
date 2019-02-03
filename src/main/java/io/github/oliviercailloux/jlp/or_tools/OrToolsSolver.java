package io.github.oliviercailloux.jlp.or_tools;

import java.time.Duration;
import java.time.temporal.ChronoUnit;
import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.common.collect.ImmutableBiMap;
import com.google.common.collect.ImmutableMap;
import com.google.common.collect.Range;
import com.google.ortools.linearsolver.MPConstraint;
import com.google.ortools.linearsolver.MPObjective;
import com.google.ortools.linearsolver.MPSolver;
import com.google.ortools.linearsolver.MPSolver.OptimizationProblemType;
import com.google.ortools.linearsolver.MPVariable;

import io.github.oliviercailloux.jlp.elements.Constraint;
import io.github.oliviercailloux.jlp.elements.FiniteRange;
import io.github.oliviercailloux.jlp.elements.Objective;
import io.github.oliviercailloux.jlp.elements.Sense;
import io.github.oliviercailloux.jlp.elements.SumTerms;
import io.github.oliviercailloux.jlp.elements.Term;
import io.github.oliviercailloux.jlp.elements.Variable;
import io.github.oliviercailloux.jlp.elements.VariableKind;
import io.github.oliviercailloux.jlp.mp.IMP;
import io.github.oliviercailloux.jlp.parameters.Configuration;
import io.github.oliviercailloux.jlp.result.ComputationTime;
import io.github.oliviercailloux.jlp.result.Result;
import io.github.oliviercailloux.jlp.result.ResultStatus;
import io.github.oliviercailloux.jlp.result.Solution;
import io.github.oliviercailloux.jlp.solve.Solver;
import io.github.oliviercailloux.jlp.solve.SolverException;

public class OrToolsSolver implements Solver {
	@SuppressWarnings("unused")
	private static final Logger LOGGER = LoggerFactory.getLogger(OrToolsSolver.class);

	public static Range<Double> getBounds(Constraint constraint) {
		switch (constraint.getOperator()) {
		case EQ:
			return FiniteRange.closed(constraint.getRhs(), constraint.getRhs());
		case GE:
			return FiniteRange.atLeast(constraint.getRhs());
		case LE:
			return FiniteRange.atMost(constraint.getRhs());
		default:
			throw new AssertionError();
		}
	}

	private MPSolver solver;

	private ImmutableBiMap<Variable, MPVariable> varsToOr;

	private Configuration configuration;

	public OrToolsSolver() {
		solver = null;
		varsToOr = null;
		configuration = Configuration.defaultConfiguration();
	}

	@Override
	public void setConfiguration(Configuration configuration) {
		this.configuration = configuration;
		if (configuration.getForceDeterministic()) {
			throw new UnsupportedOperationException();
		}
		if (configuration.getMaxCpuTime().compareTo(Configuration.ENOUGH) < 0) {
			throw new UnsupportedOperationException();
		}
	}

	@Override
	public Result solve(IMP mp) {
		init(mp);
		final List<Variable> variables = mp.getVariables();
		setVariables(variables);
		final List<Constraint> constraints = mp.getConstraints();
		setConstraints(constraints);
		final Objective objective = mp.getObjective();
		setObjective(objective);

		if (configuration.getMaxWallTime().compareTo(Configuration.ENOUGH) < 0) {
			solver.setTimeLimit(configuration.getMaxWallTime().get(ChronoUnit.MILLIS));
		}

		final long start = System.nanoTime();
		final com.google.ortools.linearsolver.MPSolver.ResultStatus orResult = solver.solve();
		final long end = System.nanoTime();
		final long lengthNs = end - start;
		final long wallTimeSinceStartedSolverMs = solver.wallTime();
		assert wallTimeSinceStartedSolverMs * 1_000_000 > lengthNs : String
				.format("Wall time (ms): %d, length (ns): %d", wallTimeSinceStartedSolverMs, lengthNs);
		final ComputationTime time = ComputationTime.ofWallTime(Duration.of(lengthNs, ChronoUnit.NANOS));

		final ResultStatus resultStatus = transformResultStatus(orResult);
		final Result result;
		if (resultStatus == ResultStatus.OPTIMAL) {
			final ImmutableMap<Variable, Double> values = getSolutionValues();
			final double objectiveValue = solver.objective().value();
			final Solution solution = Solution.of(mp, objectiveValue, values);
			result = Result.withSolution(resultStatus, time, configuration, solution);
		} else {
			result = Result.noSolution(resultStatus, time, configuration);
		}
		return result;
	}

	private void init(IMP mp) {
		LOGGER.info("Loading native library jniortools (using {}).", System.getProperty("java.library.path"));
		System.loadLibrary("jniortools");
	
		final OptimizationProblemType type;
		if (mp.getDimension().getIntegerDomainsCount() >= 1) {
			type = OptimizationProblemType.CBC_MIXED_INTEGER_PROGRAMMING;
		} else {
			type = OptimizationProblemType.GLOP_LINEAR_PROGRAMMING;
		}
		solver = new MPSolver(mp.getName(), type);
	}

	private void setVariables(List<Variable> variables) {
		final ImmutableBiMap.Builder<Variable, MPVariable> varsToOrBuilder = ImmutableBiMap.builder();
		for (Variable variable : variables) {
			final Range<Double> bounds = variable.getBounds();
			final VariableKind kind = variable.getKind();
			final MPVariable x;
			switch (kind) {
			case BOOL_KIND:
				x = solver.makeBoolVar(variable.getDescription());
				break;
			case INT_KIND:
				x = solver.makeIntVar(bounds.lowerEndpoint(), bounds.upperEndpoint(), variable.getDescription());
				break;
			case REAL_KIND:
				x = solver.makeNumVar(bounds.lowerEndpoint(), bounds.upperEndpoint(), variable.getDescription());
				break;
			default:
				throw new AssertionError();
			}
			varsToOrBuilder.put(variable, x);
		}
		varsToOr = varsToOrBuilder.build();
	}

	private void setConstraints(List<Constraint> constraints) {
		for (Constraint constraint : constraints) {
			final Range<Double> bounds = getBounds(constraint);
			final MPConstraint c = solver.makeConstraint(bounds.lowerEndpoint(), bounds.upperEndpoint(),
					constraint.getDescription());
			final SumTerms lhs = constraint.getLhs();
			for (Term term : lhs) {
				final Variable variable = term.getVariable();
				final MPVariable orVar = varsToOr.get(variable);
				c.setCoefficient(orVar, term.getCoefficient());
			}
		}
	}

	private void setObjective(Objective objective) {
		if (!objective.isZero()) {
			final MPObjective orObjective = solver.objective();
	
			final Sense sense = objective.getSense();
			switch (sense) {
			case MAX:
				orObjective.setMaximization();
				break;
			case MIN:
				orObjective.setMinimization();
				break;
			default:
				throw new AssertionError();
			}
	
			final SumTerms function = objective.getFunction();
			for (Term term : function) {
				final Variable variable = term.getVariable();
				final MPVariable orVar = varsToOr.get(variable);
				orObjective.setCoefficient(orVar, term.getCoefficient());
			}
		}
	
	}

	private ResultStatus transformResultStatus(com.google.ortools.linearsolver.MPSolver.ResultStatus orResult) {
		final ResultStatus rs;
		switch (orResult) {
		case ABNORMAL:
		case NOT_SOLVED:
		case FEASIBLE:
			throw new SolverException();
		case OPTIMAL:
			rs = ResultStatus.OPTIMAL;
			break;
		case INFEASIBLE:
			rs = ResultStatus.INFEASIBLE;
			break;
		case UNBOUNDED:
			rs = ResultStatus.UNBOUNDED;
			break;
		default:
			throw new AssertionError();
		}
		return rs;
	}

	private ImmutableMap<Variable, Double> getSolutionValues() {
		final ImmutableMap.Builder<Variable, Double> valuesBuilder = ImmutableMap.builder();
		for (Variable variable : varsToOr.keySet()) {
			final MPVariable orVariable = varsToOr.get(variable);
			final double value = orVariable.solutionValue();
			valuesBuilder.put(variable, value);
		}
		final ImmutableMap<Variable, Double> values = valuesBuilder.build();
		return values;
	}

}
